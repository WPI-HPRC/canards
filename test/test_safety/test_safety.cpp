// SafetyMonitor and makeTimeStep
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.4 and step 4 of
// section 10).
#include <unity.h>

#include <cstdint>
#include <limits>

#include "FlightConfig.h"
#include "Frames.h"
#include "Safety/SafetyMonitor.h"
#include "SensorSource.h"

void setUp() {}
void tearDown() {}

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

constexpr uint64_t kT0 = 10 * FlightConfig::kUsPerSecond;

constexpr uint16_t kAllFaults = SafetyFault::kImuGap | SafetyFault::kTimeBackwards |
                                SafetyFault::kImuReadFailed | SafetyFault::kImuNonFinite |
                                SafetyFault::kTransitionRefused;

// A healthy IMU sample.
SensorFrame goodFrame(uint64_t timeUs) {
  SensorFrame f;
  f.timeUs = timeUs;
  f.imuOk = true;
  f.accel = Vec3{0.0f, 0.0f, FlightConfig::kGravity};
  f.gyro = Vec3{0.01f, -0.02f, 0.03f};
  return f;
}

// A normal step: not the first, a gap well inside the limit.
TimeStep normalStep() {
  const uint64_t gapUs = FlightConfig::kMaxImuGapUs / 2 + 1;
  return makeTimeStep(true, kT0, kT0 + gapUs);
}

void expectHex16At(int line, uint16_t expected, uint16_t actual, const char* message) {
  UNITY_TEST_ASSERT_EQUAL_HEX16(expected, actual, line, message);
}

#define EXPECT_ACTIVE(expected, monitor) \
  expectHex16At(__LINE__, (expected), (monitor).active(), "active fault bits")
#define EXPECT_STICKY(expected, monitor) \
  expectHex16At(__LINE__, (expected), (monitor).sticky(), "sticky fault bits")

}  // namespace

// ---------------------------------------------------------------------------
// makeTimeStep
// ---------------------------------------------------------------------------

static void test_time_step_default_is_a_first_step() {
  const TimeStep step;
  TEST_ASSERT_TRUE(step.first);
  TEST_ASSERT_FALSE(step.backwards);
  TEST_ASSERT_EQUAL_UINT64(0, step.gapUs);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, step.dt);
}

static void test_time_step_first_sample() {
  // The "previous time" argument means nothing on the first sample, whatever
  // it holds.
  const uint64_t previousValues[] = {0, kT0 - 1, kT0, kT0 + 1, ~0ull};
  for (uint64_t previous : previousValues) {
    const TimeStep step = makeTimeStep(false, previous, kT0);
    TEST_ASSERT_TRUE(step.first);
    TEST_ASSERT_FALSE(step.backwards);
    TEST_ASSERT_EQUAL_UINT64(0, step.gapUs);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, step.dt);
  }
}

static void test_time_step_normal_sample() {
  const TimeStep step = makeTimeStep(true, kT0, kT0 + 5000);
  TEST_ASSERT_FALSE(step.first);
  TEST_ASSERT_FALSE(step.backwards);
  TEST_ASSERT_EQUAL_UINT64(5000, step.gapUs);
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.005f, step.dt);
}

static void test_time_step_dt_is_the_gap_in_seconds() {
  const uint64_t gaps[] = {1, 100, 4807, 10000, 1000000, 3600ull * 1000000ull};
  for (uint64_t gap : gaps) {
    const TimeStep step = makeTimeStep(true, kT0, kT0 + gap);
    TEST_ASSERT_EQUAL_UINT64(gap, step.gapUs);
    const float expected = static_cast<float>(static_cast<double>(gap) * 1e-6);
    TEST_ASSERT_FLOAT_WITHIN(expected * 1e-6f, expected, step.dt);
    TEST_ASSERT_TRUE(step.dt > 0.0f);
  }
}

static void test_time_step_backwards() {
  const TimeStep step = makeTimeStep(true, kT0, kT0 - 1);
  TEST_ASSERT_FALSE(step.first);
  TEST_ASSERT_TRUE(step.backwards);
  // Not the huge number an unsigned subtraction would give.
  TEST_ASSERT_EQUAL_UINT64(0, step.gapUs);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, step.dt);

  const TimeStep farBack = makeTimeStep(true, kT0, 0);
  TEST_ASSERT_TRUE(farBack.backwards);
  TEST_ASSERT_EQUAL_UINT64(0, farBack.gapUs);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, farBack.dt);
}

static void test_time_step_repeated_timestamp_counts_as_backwards() {
  const TimeStep step = makeTimeStep(true, kT0, kT0);
  TEST_ASSERT_FALSE(step.first);
  TEST_ASSERT_TRUE(step.backwards);
  TEST_ASSERT_EQUAL_UINT64(0, step.gapUs);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, step.dt);
}

static void test_time_step_works_above_32_bits_of_microseconds() {
  // Crossing 2^32 us (71.6 minutes): a 32-bit time would look like it went
  // backwards here.
  const uint64_t before = (1ull << 32) - 2000;
  const TimeStep crossing = makeTimeStep(true, before, before + 5000);
  TEST_ASSERT_FALSE(crossing.backwards);
  TEST_ASSERT_EQUAL_UINT64(5000, crossing.gapUs);
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.005f, crossing.dt);

  // Times that differ only above bit 32.
  const TimeStep high = makeTimeStep(true, 7ull << 32, (7ull << 32) + 5000);
  TEST_ASSERT_EQUAL_UINT64(5000, high.gapUs);
  const TimeStep highBack = makeTimeStep(true, 8ull << 32, (7ull << 32) + 5000);
  TEST_ASSERT_TRUE(highBack.backwards);
  TEST_ASSERT_EQUAL_UINT64(0, highBack.gapUs);
}

// ---------------------------------------------------------------------------
// SafetyMonitor
// ---------------------------------------------------------------------------

static void test_monitor_starts_clean() {
  SafetyMonitor monitor;
  EXPECT_ACTIVE(0, monitor);
  EXPECT_STICKY(0, monitor);
  TEST_ASSERT_FALSE(monitor.anyActive());
}

static void test_healthy_cycles_raise_nothing() {
  SafetyMonitor monitor;
  monitor.check(goodFrame(kT0), makeTimeStep(false, 0, kT0), 0);
  EXPECT_ACTIVE(0, monitor);

  uint64_t previous = kT0;
  for (int i = 0; i < 100; ++i) {
    const uint64_t now = previous + FlightConfig::kMaxImuGapUs / 2 + 1;
    monitor.check(goodFrame(now), makeTimeStep(true, previous, now), 0);
    previous = now;
    EXPECT_ACTIVE(0, monitor);
    TEST_ASSERT_FALSE(monitor.anyActive());
  }
  EXPECT_STICKY(0, monitor);
}

static void test_imu_gap_boundary() {
  // Exactly the limit is still fine.
  SafetyMonitor atLimit;
  atLimit.check(goodFrame(kT0 + FlightConfig::kMaxImuGapUs),
                makeTimeStep(true, kT0, kT0 + FlightConfig::kMaxImuGapUs), 0);
  EXPECT_ACTIVE(0, atLimit);
  EXPECT_STICKY(0, atLimit);

  // One microsecond more is a fault.
  SafetyMonitor overLimit;
  overLimit.check(goodFrame(kT0 + FlightConfig::kMaxImuGapUs + 1),
                  makeTimeStep(true, kT0, kT0 + FlightConfig::kMaxImuGapUs + 1), 0);
  EXPECT_ACTIVE(SafetyFault::kImuGap, overLimit);
  EXPECT_STICKY(SafetyFault::kImuGap, overLimit);
  TEST_ASSERT_TRUE(overLimit.anyActive());
}

static void test_a_long_gap_is_a_fault() {
  SafetyMonitor monitor;
  const uint64_t now = kT0 + 100 * FlightConfig::kMaxImuGapUs;
  monitor.check(goodFrame(now), makeTimeStep(true, kT0, now), 0);
  EXPECT_ACTIVE(SafetyFault::kImuGap, monitor);
}

static void test_the_first_sample_is_never_a_gap() {
  // The board has been up for an hour before the first cycle: there is no
  // previous sample to be late against.
  SafetyMonitor monitor;
  const uint64_t now = 3600ull * FlightConfig::kUsPerSecond;
  monitor.check(goodFrame(now), makeTimeStep(false, 0, now), 0);
  EXPECT_ACTIVE(0, monitor);

  // Even if a caller hands over a first step with a gap filled in.
  TimeStep odd;
  odd.first = true;
  odd.gapUs = FlightConfig::kMaxImuGapUs + 1;
  SafetyMonitor other;
  other.check(goodFrame(now), odd, 0);
  EXPECT_ACTIVE(0, other);
}

static void test_time_going_backwards_is_a_fault() {
  SafetyMonitor monitor;
  monitor.check(goodFrame(kT0 - 1), makeTimeStep(true, kT0, kT0 - 1), 0);
  EXPECT_ACTIVE(SafetyFault::kTimeBackwards, monitor);
  EXPECT_STICKY(SafetyFault::kTimeBackwards, monitor);
}

static void test_a_repeated_timestamp_is_a_fault() {
  SafetyMonitor monitor;
  monitor.check(goodFrame(kT0), makeTimeStep(true, kT0, kT0), 0);
  EXPECT_ACTIVE(SafetyFault::kTimeBackwards, monitor);
}

static void test_failed_imu_read_is_a_fault() {
  SafetyMonitor monitor;
  SensorFrame frame = goodFrame(kT0);
  frame.imuOk = false;
  monitor.check(frame, normalStep(), 0);
  EXPECT_ACTIVE(SafetyFault::kImuReadFailed, monitor);
  EXPECT_STICKY(SafetyFault::kImuReadFailed, monitor);
}

static void test_non_finite_imu_value_is_a_fault() {
  const float badValues[] = {kNaN, kInf, -kInf};
  for (float bad : badValues) {
    for (int field = 0; field < 6; ++field) {
      SafetyMonitor monitor;
      SensorFrame frame = goodFrame(kT0);
      float* fields[6] = {&frame.accel.x, &frame.accel.y, &frame.accel.z,
                          &frame.gyro.x,  &frame.gyro.y,  &frame.gyro.z};
      *fields[field] = bad;
      monitor.check(frame, normalStep(), 0);
      EXPECT_ACTIVE(SafetyFault::kImuNonFinite, monitor);
      EXPECT_STICKY(SafetyFault::kImuNonFinite, monitor);
    }
  }
}

static void test_slow_sensors_do_not_raise_imu_faults() {
  // The magnetometer and barometer are not the IMU. Garbage there is the
  // estimator's problem and must not force the canards to neutral.
  SafetyMonitor monitor;
  SensorFrame frame = goodFrame(kT0);
  frame.magFresh = true;
  frame.mag = Vec3{kNaN, kInf, 0.0f};
  frame.baroFresh = true;
  frame.pressurePa = kNaN;
  frame.temperatureC = kNaN;
  monitor.check(frame, normalStep(), 0);
  EXPECT_ACTIVE(0, monitor);
}

static void test_refused_transition_is_active_for_one_check() {
  SafetyMonitor monitor;
  monitor.check(goodFrame(kT0), normalStep(), 0);
  EXPECT_ACTIVE(0, monitor);

  // The state machine refused a transition during the previous cycle.
  monitor.check(goodFrame(kT0), normalStep(), 1);
  EXPECT_ACTIVE(SafetyFault::kTransitionRefused, monitor);
  EXPECT_STICKY(SafetyFault::kTransitionRefused, monitor);

  // Same count: nothing new was refused.
  monitor.check(goodFrame(kT0), normalStep(), 1);
  EXPECT_ACTIVE(0, monitor);
  EXPECT_STICKY(SafetyFault::kTransitionRefused, monitor);
  monitor.check(goodFrame(kT0), normalStep(), 1);
  EXPECT_ACTIVE(0, monitor);
}

static void test_refused_transition_every_cycle_stays_active() {
  SafetyMonitor monitor;
  monitor.check(goodFrame(kT0), normalStep(), 0);
  for (uint32_t count = 1; count <= 10; ++count) {
    monitor.check(goodFrame(kT0), normalStep(), count);
    EXPECT_ACTIVE(SafetyFault::kTransitionRefused, monitor);
  }
  monitor.check(goodFrame(kT0), normalStep(), 10);
  EXPECT_ACTIVE(0, monitor);
}

static void test_refused_count_jumping_by_more_than_one_is_one_fault_cycle() {
  SafetyMonitor monitor;
  monitor.check(goodFrame(kT0), normalStep(), 0);
  monitor.check(goodFrame(kT0), normalStep(), 7);
  EXPECT_ACTIVE(SafetyFault::kTransitionRefused, monitor);
  monitor.check(goodFrame(kT0), normalStep(), 7);
  EXPECT_ACTIVE(0, monitor);
}

static void test_refusal_before_the_first_check_is_still_reported() {
  // The monitor starts from a count of 0, so a nonzero count on its very
  // first check is a refusal it has not reported yet.
  SafetyMonitor monitor;
  monitor.check(goodFrame(kT0), makeTimeStep(false, 0, kT0), 3);
  EXPECT_ACTIVE(SafetyFault::kTransitionRefused, monitor);
}

static void test_faults_clear_from_active_but_stay_sticky() {
  SafetyMonitor monitor;

  // Cycle 1: a gap.
  monitor.check(goodFrame(kT0), makeTimeStep(true, 0, kT0), 0);
  EXPECT_ACTIVE(SafetyFault::kImuGap, monitor);
  EXPECT_STICKY(SafetyFault::kImuGap, monitor);

  // Cycle 2: healthy. Active clears at once; sticky remembers.
  monitor.check(goodFrame(kT0), normalStep(), 0);
  EXPECT_ACTIVE(0, monitor);
  TEST_ASSERT_FALSE(monitor.anyActive());
  EXPECT_STICKY(SafetyFault::kImuGap, monitor);

  // Cycle 3: a different fault. Active shows only this cycle's.
  SensorFrame failed = goodFrame(kT0);
  failed.imuOk = false;
  monitor.check(failed, normalStep(), 0);
  EXPECT_ACTIVE(SafetyFault::kImuReadFailed, monitor);
  EXPECT_STICKY(SafetyFault::kImuGap | SafetyFault::kImuReadFailed, monitor);

  // Cycle 4: healthy again.
  monitor.check(goodFrame(kT0), normalStep(), 0);
  EXPECT_ACTIVE(0, monitor);
  EXPECT_STICKY(SafetyFault::kImuGap | SafetyFault::kImuReadFailed, monitor);

  // Many healthy cycles never clear sticky.
  for (int i = 0; i < 1000; ++i) monitor.check(goodFrame(kT0), normalStep(), 0);
  EXPECT_ACTIVE(0, monitor);
  EXPECT_STICKY(SafetyFault::kImuGap | SafetyFault::kImuReadFailed, monitor);
}

static void test_several_faults_in_one_cycle_are_all_reported() {
  SafetyMonitor monitor;
  monitor.check(goodFrame(kT0), normalStep(), 0);

  SensorFrame frame = goodFrame(kT0);
  frame.imuOk = false;
  frame.accel.x = kNaN;
  const uint64_t now = kT0 + FlightConfig::kMaxImuGapUs + 1;
  monitor.check(frame, makeTimeStep(true, kT0, now), 1);

  const uint16_t expected = SafetyFault::kImuGap | SafetyFault::kImuReadFailed |
                            SafetyFault::kImuNonFinite | SafetyFault::kTransitionRefused;
  EXPECT_ACTIVE(expected, monitor);
  EXPECT_STICKY(expected, monitor);

  // Backwards time on a later cycle fills in the last bit.
  monitor.check(goodFrame(kT0), makeTimeStep(true, kT0, kT0 - 5), 1);
  EXPECT_ACTIVE(SafetyFault::kTimeBackwards, monitor);
  EXPECT_STICKY(kAllFaults, monitor);
}

static void test_fault_bits_are_distinct() {
  const uint16_t bits[] = {SafetyFault::kImuGap, SafetyFault::kTimeBackwards,
                           SafetyFault::kImuReadFailed, SafetyFault::kImuNonFinite,
                           SafetyFault::kTransitionRefused};
  uint16_t seen = 0;
  for (uint16_t bit : bits) {
    TEST_ASSERT_TRUE(bit != 0);
    TEST_ASSERT_EQUAL_HEX16(0, bit & (bit - 1));  // exactly one bit set
    TEST_ASSERT_EQUAL_HEX16(0, seen & bit);       // not used by another fault
    seen |= bit;
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_time_step_default_is_a_first_step);
  RUN_TEST(test_time_step_first_sample);
  RUN_TEST(test_time_step_normal_sample);
  RUN_TEST(test_time_step_dt_is_the_gap_in_seconds);
  RUN_TEST(test_time_step_backwards);
  RUN_TEST(test_time_step_repeated_timestamp_counts_as_backwards);
  RUN_TEST(test_time_step_works_above_32_bits_of_microseconds);
  RUN_TEST(test_monitor_starts_clean);
  RUN_TEST(test_healthy_cycles_raise_nothing);
  RUN_TEST(test_imu_gap_boundary);
  RUN_TEST(test_a_long_gap_is_a_fault);
  RUN_TEST(test_the_first_sample_is_never_a_gap);
  RUN_TEST(test_time_going_backwards_is_a_fault);
  RUN_TEST(test_a_repeated_timestamp_is_a_fault);
  RUN_TEST(test_failed_imu_read_is_a_fault);
  RUN_TEST(test_non_finite_imu_value_is_a_fault);
  RUN_TEST(test_slow_sensors_do_not_raise_imu_faults);
  RUN_TEST(test_refused_transition_is_active_for_one_check);
  RUN_TEST(test_refused_transition_every_cycle_stays_active);
  RUN_TEST(test_refused_count_jumping_by_more_than_one_is_one_fault_cycle);
  RUN_TEST(test_refusal_before_the_first_check_is_still_reported);
  RUN_TEST(test_faults_clear_from_active_but_stay_sticky);
  RUN_TEST(test_several_faults_in_one_cycle_are_all_reported);
  RUN_TEST(test_fault_bits_are_distinct);
  return UNITY_END();
}
