// ma4::PwmDecoder (lib/Actuators/PwmDecoder.h): MA4 encoder PWM -> shaft angle.
//
// The first tests are pid-tuner's own decoder checks (tests/control_test.cpp,
// commit d7b3440), with its millisecond times written in microseconds. The
// rest pin down each validity rule separately.
#include <unity.h>

#include <cstdint>

#include "PwmDecoder.h"

void setUp() {}
void tearDown() {}

namespace {

constexpr float kDegPerCount = 360.0f / 4096.0f;
constexpr uint16_t kLowBandPeriod = 43510;  // 4.351 ms, the MA4's slower PWM

// Drives one decoder edge by edge, keeping a capture counter and a clock that
// move together (one capture tick is 0.1 us).
struct EdgeFeeder {
  ma4::PwmDecoder decoder;
  uint16_t ticks = 0;
  uint32_t timeUs = 100000;

  void rising() { decoder.recordEdge(true, ticks, timeUs); }
  void falling() { decoder.recordEdge(false, ticks, timeUs); }
  void advance(uint32_t captureTicks) {
    ticks = uint16_t(ticks + captureTicks);
    timeUs += captureTicks / 10;
  }
  // The rest of a period that began with a rising edge: falling, then rising.
  void period(uint32_t highTicks, uint32_t periodTicks) {
    advance(highTicks);
    falling();
    advance(periodTicks - highTicks);
    rising();
  }
};

// High time for a given MA4 count in a period of kLowBandPeriod ticks.
uint32_t highTicksForCount(uint32_t count) { return (128 + count) * 10; }

}  // namespace

// --- decodeAngle ------------------------------------------------------------

void test_tuner_endpoints_decode_to_zero_and_full_scale() {
  float angle = -1.0f;
  TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle(128, 4351, angle));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, angle);
  TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle(4223, 4351, angle));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4095 * kDegPerCount, angle);
}

void test_tuner_rejects_an_empty_period_and_a_too_short_pulse() {
  float angle = 0.0f;
  TEST_ASSERT_FALSE(ma4::PwmDecoder::decodeAngle(0, 0, angle));
  TEST_ASSERT_FALSE(ma4::PwmDecoder::decodeAngle(10, 4351, angle));
}

void test_half_and_quarter_turns_decode_exactly() {
  float angle = 0.0f;
  TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle(128 + 2048, 4351, angle));
  TEST_ASSERT_EQUAL_FLOAT(180.0f, angle);
  TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle(128 + 1024, 4351, angle));
  TEST_ASSERT_EQUAL_FLOAT(90.0f, angle);
}

void test_duty_not_period_sets_the_angle() {
  // The same duty over a period five times shorter gives the same angle.
  float slow = 0.0f;
  float fast = 0.0f;
  TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle(highTicksForCount(1000), kLowBandPeriod, slow));
  TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle((128 + 1000) * 2, kLowBandPeriod / 5, fast));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000 * kDegPerCount, slow);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, slow, fast);
}

void test_high_time_of_a_whole_period_or_more_is_rejected() {
  float angle = 0.0f;
  TEST_ASSERT_FALSE(ma4::PwmDecoder::decodeAngle(4351, 4351, angle));
  TEST_ASSERT_FALSE(ma4::PwmDecoder::decodeAngle(5000, 4351, angle));
}

void test_two_counts_of_quantisation_are_allowed_at_each_end_and_clamped() {
  // A period of 2 * 4351 ticks puts counts on half-integers.
  float angle = -1.0f;
  TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle(253, 8702, angle));  // -1.5 counts
  TEST_ASSERT_EQUAL_FLOAT(0.0f, angle);
  TEST_ASSERT_FALSE(ma4::PwmDecoder::decodeAngle(251, 8702, angle));  // -2.5 counts
  TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle(8449, 8702, angle));  // 4096.5 counts
  TEST_ASSERT_EQUAL_FLOAT(4095 * kDegPerCount, angle);
  TEST_ASSERT_FALSE(ma4::PwmDecoder::decodeAngle(8451, 8702, angle));  // 4097.5 counts
}

void test_a_rejected_decode_leaves_the_angle_untouched() {
  float angle = 42.0f;
  TEST_ASSERT_FALSE(ma4::PwmDecoder::decodeAngle(10, 4351, angle));
  TEST_ASSERT_EQUAL_FLOAT(42.0f, angle);
}

void test_every_angle_is_below_a_full_turn() {
  float angle = 0.0f;
  for (uint32_t high = 0; high < 4351; ++high) {
    if (ma4::PwmDecoder::decodeAngle(high, 4351, angle)) {
      TEST_ASSERT_TRUE(angle >= 0.0f);
      TEST_ASSERT_TRUE(angle < 360.0f);
    }
  }
}

// --- recordEdge / PwmSample ---------------------------------------------------

void test_tuner_sequence_across_a_capture_counter_wrap() {
  ma4::PwmDecoder decoder;
  decoder.recordEdge(true, 65000, 10000);
  decoder.recordEdge(false, 1000, 11000);
  decoder.recordEdge(true, 42974, 14000);
  TEST_ASSERT_EQUAL_UINT32(43510, decoder.getSample().periodTicks);
  TEST_ASSERT_EQUAL_UINT32(1536, decoder.getSample().highTicks);
  TEST_ASSERT_TRUE(decoder.getSample().valid);
  // The next falling edge must not touch the completed sample.
  decoder.recordEdge(false, 44974, 15000);
  TEST_ASSERT_EQUAL_UINT32(1536, decoder.getSample().highTicks);
  TEST_ASSERT_TRUE(decoder.getSample().isFresh(15000));
  TEST_ASSERT_FALSE(decoder.getSample().isFresh(100000));
}

void test_tuner_freshness_across_the_clock_wrap() {
  ma4::PwmDecoder decoder;
  decoder.recordEdge(true, 10, UINT32_MAX - 4000);
  decoder.recordEdge(false, 500, UINT32_MAX - 3000);
  decoder.recordEdge(true, 11010, UINT32_MAX - 2000);
  TEST_ASSERT_TRUE(decoder.getSample().valid);
  TEST_ASSERT_TRUE(decoder.getSample().isFresh(3000));
}

void test_nothing_is_valid_before_the_first_edge() {
  ma4::PwmDecoder decoder;
  TEST_ASSERT_FALSE(decoder.getSample().valid);
  TEST_ASSERT_FALSE(decoder.getSample().isFresh(0));
}

void test_the_first_full_period_is_valid_and_carries_the_angle() {
  EdgeFeeder f;
  f.rising();
  f.period(highTicksForCount(1024), kLowBandPeriod);
  const ma4::PwmSample& s = f.decoder.getSample();
  TEST_ASSERT_TRUE(s.valid);
  TEST_ASSERT_EQUAL_UINT32(kLowBandPeriod, s.periodTicks);
  TEST_ASSERT_EQUAL_UINT32(f.timeUs, s.timestampUs);
  float angle = 0.0f;
  TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle(s.highTicks, s.periodTicks, angle));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, angle);
}

void test_a_falling_edge_before_any_rising_edge_is_ignored() {
  EdgeFeeder f;
  f.falling();
  f.advance(1000);
  f.rising();  // first rising edge: no period yet
  TEST_ASSERT_FALSE(f.decoder.getSample().valid);
  f.period(highTicksForCount(0), kLowBandPeriod);
  TEST_ASSERT_TRUE(f.decoder.getSample().valid);
}

void test_a_period_without_a_falling_edge_is_invalid() {
  EdgeFeeder f;
  f.rising();
  f.advance(kLowBandPeriod);
  f.rising();
  TEST_ASSERT_FALSE(f.decoder.getSample().valid);
  // And the decoder recovers on the next complete period.
  f.period(highTicksForCount(2048), kLowBandPeriod);
  TEST_ASSERT_TRUE(f.decoder.getSample().valid);
}

void test_rising_edges_more_than_6_ms_apart_are_invalid() {
  ma4::PwmDecoder decoder;
  decoder.recordEdge(true, 0, 10000);
  decoder.recordEdge(false, 2000, 10200);
  decoder.recordEdge(true, kLowBandPeriod, 16001);
  TEST_ASSERT_FALSE(decoder.getSample().valid);

  ma4::PwmDecoder onTheLimit;
  onTheLimit.recordEdge(true, 0, 10000);
  onTheLimit.recordEdge(false, 2000, 10200);
  onTheLimit.recordEdge(true, kLowBandPeriod, 16000);
  TEST_ASSERT_TRUE(onTheLimit.getSample().valid);
}

void test_period_must_be_in_one_of_the_two_ma4_bands() {
  struct Case {
    uint32_t periodTicks;
    bool valid;
  };
  const Case cases[] = {
      {10299, false}, {10300, true}, {11500, true}, {11501, false},
      {30000, false}, {41299, false}, {41300, true}, {46000, true}, {46001, false},
  };
  for (const Case& c : cases) {
    EdgeFeeder f;
    f.rising();
    f.period(c.periodTicks / 2, c.periodTicks);  // mid-scale duty
    TEST_ASSERT_EQUAL_MESSAGE(c.valid, f.decoder.getSample().valid, "period band");
  }
}

void test_an_undecodable_duty_makes_the_period_invalid() {
  EdgeFeeder f;
  f.rising();
  f.period(100, kLowBandPeriod);  // far below the minimum high time
  TEST_ASSERT_FALSE(f.decoder.getSample().valid);
  TEST_ASSERT_FALSE(f.decoder.getSample().isFresh(f.timeUs));
}

void test_a_sample_is_fresh_for_exactly_30_ms() {
  EdgeFeeder f;
  f.rising();
  f.period(highTicksForCount(100), kLowBandPeriod);
  const uint32_t t = f.decoder.getSample().timestampUs;
  TEST_ASSERT_TRUE(f.decoder.getSample().isFresh(t));
  TEST_ASSERT_TRUE(f.decoder.getSample().isFresh(t + 30000));
  TEST_ASSERT_FALSE(f.decoder.getSample().isFresh(t + 30001));
}

void test_a_bad_period_replaces_a_good_one() {
  EdgeFeeder f;
  f.rising();
  f.period(highTicksForCount(100), kLowBandPeriod);
  TEST_ASSERT_TRUE(f.decoder.getSample().valid);
  f.period(highTicksForCount(100), 30000);  // out of band
  TEST_ASSERT_FALSE(f.decoder.getSample().valid);
  TEST_ASSERT_FALSE(f.decoder.getSample().isFresh(f.timeUs));
}

void test_a_steady_signal_stays_valid_period_after_period() {
  EdgeFeeder f;
  f.rising();
  for (uint32_t count = 0; count < 4096; count += 37) {
    f.period(highTicksForCount(count), kLowBandPeriod);
    const ma4::PwmSample& s = f.decoder.getSample();
    TEST_ASSERT_TRUE(s.valid);
    float angle = -1.0f;
    TEST_ASSERT_TRUE(ma4::PwmDecoder::decodeAngle(s.highTicks, s.periodTicks, angle));
    TEST_ASSERT_FLOAT_WITHIN(0.5f * kDegPerCount, count * kDegPerCount, angle);
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_tuner_endpoints_decode_to_zero_and_full_scale);
  RUN_TEST(test_tuner_rejects_an_empty_period_and_a_too_short_pulse);
  RUN_TEST(test_half_and_quarter_turns_decode_exactly);
  RUN_TEST(test_duty_not_period_sets_the_angle);
  RUN_TEST(test_high_time_of_a_whole_period_or_more_is_rejected);
  RUN_TEST(test_two_counts_of_quantisation_are_allowed_at_each_end_and_clamped);
  RUN_TEST(test_a_rejected_decode_leaves_the_angle_untouched);
  RUN_TEST(test_every_angle_is_below_a_full_turn);
  RUN_TEST(test_tuner_sequence_across_a_capture_counter_wrap);
  RUN_TEST(test_tuner_freshness_across_the_clock_wrap);
  RUN_TEST(test_nothing_is_valid_before_the_first_edge);
  RUN_TEST(test_the_first_full_period_is_valid_and_carries_the_angle);
  RUN_TEST(test_a_falling_edge_before_any_rising_edge_is_ignored);
  RUN_TEST(test_a_period_without_a_falling_edge_is_invalid);
  RUN_TEST(test_rising_edges_more_than_6_ms_apart_are_invalid);
  RUN_TEST(test_period_must_be_in_one_of_the_two_ma4_bands);
  RUN_TEST(test_an_undecodable_duty_makes_the_period_invalid);
  RUN_TEST(test_a_sample_is_fresh_for_exactly_30_ms);
  RUN_TEST(test_a_bad_period_replaces_a_good_one);
  RUN_TEST(test_a_steady_signal_stays_valid_period_after_period);
  return UNITY_END();
}
