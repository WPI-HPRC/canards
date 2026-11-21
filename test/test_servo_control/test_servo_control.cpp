// ServoControl: canard angles <-> servo pulses, and the bench tuner's PID
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 8.2 and step 6).
//
// Three groups of tests:
//   1. The ported tuner code (PidController, ControlMath). The first of these
//      are pid-tuner's own tests/control_test.cpp, so a change that would make
//      bench-tuned gains invalid fails here.
//   2. ServoControl against the tuner's control step and against
//      Canards-Rocket's desktop tests (test/test_desktop/test_canards.cpp).
//   3. The configuration as shipped.
//
// Every table here is local to the test, apart from group 3, which reads
// VehicleConfig and never copies a number from it.

#include <unity.h>

#include <cmath>
#include <limits>

#include "Actuators.h"
#include "ServoControl/ControlMath.h"
#include "ServoControl/PidController.h"
#include "ServoControl/ServoConfig.h"
#include "ServoControl/ServoControl.h"
#include "VehicleConfig.h"

void setUp() {}
void tearDown() {}

namespace {

// Equal to the last bit. TEST_ASSERT_EQUAL_FLOAT allows a relative error,
// which is not good enough for "exactly the centre pulse".
#define TEST_ASSERT_EXACT_FLOAT(expected, actual) \
  TEST_ASSERT_FLOAT_WITHIN(0.0f, (expected), (actual))
#define TEST_ASSERT_EXACT_FLOAT_MESSAGE(expected, actual, message) \
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0f, (expected), (actual), (message))

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg = kPi / 180.0f;  // radians in one degree
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// Tolerances. A pulse is about 1500 us in float, so 1e-3 us is a few bits.
constexpr float kPulseTol = 1e-3f;
// An encoder angle goes rad -> deg -> rad through float.
constexpr float kAngleTol = 5e-6f;

// The servo-lag constant hard-coded in the reference controller
// (Canards-Rocket src/boilerplate/controller/controller.cpp:17): the share of
// the gap left after one 5 ms step with a 30 ms lag.
constexpr float kReferenceServoLag = 0.846482f;

// The airframe's canard -> channel map, written out so a test does not depend
// on VehicleConfig. Its inverse is different from itself, so a map applied
// backwards is caught.
constexpr int kChannelOfCanard[kNumCanards] = {0, 2, 3, 1};
constexpr int kCanardOfChannel[kNumCanards] = {0, 3, 1, 2};

// Every channel the same: +10 us per degree, zero at 100 deg absolute, +/-8 deg
// of travel, 1400..1600 us, no gains. Position loop at 100 Hz, 30 ms servo lag.
// (Canards-Rocket's test table, without its reversed channel.)
ServoConfig plainConfig() {
  return {
      {
          {0.0f, 0.0f, 0.0f, 100.0f, 1.0f, 1500.0f, 10.0f, -8.0f, 8.0f, 1400.0f, 1600.0f, true},
          {0.0f, 0.0f, 0.0f, 100.0f, 1.0f, 1500.0f, 10.0f, -8.0f, 8.0f, 1400.0f, 1600.0f, true},
          {0.0f, 0.0f, 0.0f, 100.0f, 1.0f, 1500.0f, 10.0f, -8.0f, 8.0f, 1400.0f, 1600.0f, true},
          {0.0f, 0.0f, 0.0f, 100.0f, 1.0f, 1500.0f, 10.0f, -8.0f, 8.0f, 1400.0f, 1600.0f, true},
      },
      {0, 2, 3, 1},
      {1.0f, 1.0f, 1.0f, 1.0f},
      0.010f,
      0.030f,
  };
}

// Every channel different (zero, centre, slope), so a pulse or an encoder that
// lands on the wrong channel shows up as a wrong number.
ServoConfig distinctConfig() {
  return {
      {
          {0.0f, 0.0f, 0.0f, 10.0f, 1.0f, 1500.0f, 10.0f, -8.0f, 8.0f, 1300.0f, 1700.0f, true},
          {0.0f, 0.0f, 0.0f, 100.0f, 1.0f, 1510.0f, 11.0f, -8.0f, 8.0f, 1300.0f, 1700.0f, true},
          {0.0f, 0.0f, 0.0f, 200.0f, 1.0f, 1520.0f, 12.0f, -8.0f, 8.0f, 1300.0f, 1700.0f, true},
          {0.0f, 0.0f, 0.0f, 300.0f, 1.0f, 1530.0f, 13.0f, -8.0f, 8.0f, 1300.0f, 1700.0f, true},
      },
      {0, 2, 3, 1},
      {1.0f, 1.0f, 1.0f, 1.0f},
      0.010f,
      0.030f,
  };
}

// The 2026-11-20 bench rows (Canards-Rocket src/canards/CanardConfig.h),
// marked calibrated. A copy on purpose: these tests are about the arithmetic
// on real-sized numbers, not about what VehicleConfig holds today.
ServoConfig benchConfig() {
  return {
      {
          {0.0f, 0.0f, 0.0f, 105.835f, 1.0f, 1477.3f, -17.18f, -10.0f, 10.0f, 1305.0f, 1649.0f, true},
          {0.0f, 0.0f, 0.0f, 24.207f, 1.0f, 1386.3f, -18.17f, -10.0f, 10.0f, 1205.0f, 1568.0f, true},
          {0.0f, 0.0f, 0.0f, 289.523f, 1.0f, 1451.3f, -22.10f, -10.0f, 10.0f, 1230.0f, 1672.0f, true},
          {0.0f, 0.0f, 0.0f, 342.260f, 1.0f, 1487.3f, -18.97f, -10.0f, 10.0f, 1298.0f, 1677.0f, true},
      },
      {0, 2, 3, 1},
      {1.0f, 1.0f, 1.0f, 1.0f},
      0.010f,
      0.030f,
  };
}

// The absolute encoder angle, rad in [0, 2*pi), that a channel reads when its
// shaft is `shaftDeg` away from neutral.
float encoderRad(const ServoChannel& channel, float shaftDeg) {
  float deg = std::fmod(channel.zeroAngle + channel.direction * shaftDeg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg * kDeg;
}

// Valid feedback with each shaft at the given angle [deg], by servo channel.
ActuatorFeedback feedbackAt(const ServoConfig& config, float ch0, float ch1, float ch2, float ch3) {
  const float shaftDeg[kNumCanards] = {ch0, ch1, ch2, ch3};
  ActuatorFeedback feedback;
  for (int ch = 0; ch < kNumCanards; ++ch) {
    feedback.encoderAngleRad[ch] = encoderRad(config.channels[ch], shaftDeg[ch]);
    feedback.encoderValid[ch] = true;
  }
  return feedback;
}

ActuatorFeedback feedbackAtAll(const ServoConfig& config, float shaftDeg) {
  return feedbackAt(config, shaftDeg, shaftDeg, shaftDeg, shaftDeg);
}

// No encoder is usable: measure() then reports the servo model.
ActuatorFeedback noFeedback() { return ActuatorFeedback{}; }

// One ServoControl driven the way the Canards subsystem drives it: measure(),
// then command(), every cycle.
struct Rig {
  explicit Rig(const ServoConfig& config) : servo(config) {}

  void cycle(const float canardRad[kNumCanards], float dt, const ActuatorFeedback& feedback) {
    servo.measure(feedback, angles);
    servo.command(canardRad, dt, feedback, pulseUs);
  }

  ServoControl servo;
  CanardAngles angles;
  float pulseUs[kNumCanards] = {};
};

// What the servo model holds, read the only way it can be: with no encoder.
void readModel(ServoControl& servo, float modelRad[kNumCanards]) {
  CanardAngles angles;
  servo.measure(noFeedback(), angles);
  TEST_ASSERT_FALSE(angles.measured);
  for (int i = 0; i < kNumCanards; ++i) modelRad[i] = angles.rad[i];
}

// ---------------------------------------------------------------------------
// 1. The ported tuner code
// ---------------------------------------------------------------------------

// pid-tuner tests/control_test.cpp, lines 15-16.
void test_wrapped_error_takes_the_short_way_round() {
  TEST_ASSERT_EXACT_FLOAT(2.0f, tuning::wrappedError(1.0f, 359.0f));
  TEST_ASSERT_EXACT_FLOAT(-2.0f, tuning::wrappedError(359.0f, 1.0f));
  // Either side of half a turn.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 179.0f, tuning::wrappedError(279.0f, 100.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -179.0f, tuning::wrappedError(281.0f, 100.0f));
}

void test_clamp_limits_a_number_and_passes_nan() {
  TEST_ASSERT_EXACT_FLOAT(3.0f, tuning::clamp(5.0f, 0.0f, 3.0f));
  TEST_ASSERT_EXACT_FLOAT(0.0f, tuning::clamp(-1.0f, 0.0f, 3.0f));
  TEST_ASSERT_EXACT_FLOAT(2.0f, tuning::clamp(2.0f, 0.0f, 3.0f));
  // This is why ServoControl checks its results itself.
  TEST_ASSERT_TRUE(std::isnan(tuning::clamp(kNaN, 0.0f, 3.0f)));
}

// pid-tuner tests/control_test.cpp, lines 17-22.
void test_pid_ignores_a_zero_time_step_then_adds_p_and_i() {
  tuning::PidController controller;
  TEST_ASSERT_TRUE(controller.setGains(1.0f, 1.0f, 0.0f));
  TEST_ASSERT_EXACT_FLOAT(0.0f, controller.update(10.0f, 0.0f, 0.0f, -20.0f, 20.0f));
  // P = 1 * 10, I = 1 * 10 * 0.1 s.
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 11.0f, controller.update(10.0f, 0.0f, 0.1f, -20.0f, 20.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, controller.getTerms().integral);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, controller.getIntegralState());

  controller.reset();
  TEST_ASSERT_EXACT_FLOAT(0.0f, controller.getTerms().integral);
  TEST_ASSERT_EXACT_FLOAT(0.0f, controller.getTerms().output);
}

// pid-tuner tests/control_test.cpp, lines 23-27.
void test_pid_saturated_output_does_not_wind_up() {
  tuning::PidController controller;
  TEST_ASSERT_TRUE(controller.setGains(100.0f, 1.0f, 0.0f));
  TEST_ASSERT_EXACT_FLOAT(20.0f, controller.update(10.0f, 0.0f, 0.01f, -20.0f, 20.0f));
  TEST_ASSERT_EXACT_FLOAT(0.0f, controller.getTerms().integral);
  TEST_ASSERT_TRUE(std::isfinite(controller.update(10.0f, 0.0f, 0.0f, -20.0f, 20.0f)));
  TEST_ASSERT_EXACT_FLOAT(20.0f, controller.update(kNaN, 0.0f, 0.1f, -20.0f, 20.0f));
}

void test_pid_integral_freezes_while_saturated_and_resumes_after() {
  tuning::PidController controller;
  TEST_ASSERT_TRUE(controller.setGains(1.0f, 10.0f, 0.0f));
  // Inside the limits: P 4.5 + I 10 * 4.5 * 0.01.
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.95f, controller.update(4.5f, 0.0f, 0.01f, -5.0f, 5.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.45f, controller.getTerms().integral);
  // Saturated high and still pushing up: the integral must not move.
  TEST_ASSERT_EXACT_FLOAT(5.0f, controller.update(10.0f, 0.0f, 0.01f, -5.0f, 5.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.45f, controller.getTerms().integral);
  // Back inside: it integrates again, from where it stopped.
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.55f, controller.update(1.0f, 0.0f, 0.01f, -5.0f, 5.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.55f, controller.getTerms().integral);
}

void test_pid_holds_its_last_output_on_a_bad_call() {
  tuning::PidController controller;
  TEST_ASSERT_TRUE(controller.setGains(0.0f, 1.0f, 0.0f));
  // 0.1 s is the longest step the tuner accepts.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, controller.update(1.0f, 0.0f, 0.1f, -20.0f, 20.0f));
  // Each of these is refused: same output, and the integral does not move.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, controller.update(1.0f, 0.0f, 0.1001f, -20.0f, 20.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, controller.update(1.0f, 0.0f, -0.01f, -20.0f, 20.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, controller.update(1.0f, 0.0f, kNaN, -20.0f, 20.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, controller.update(1.0f, 0.0f, kInf, -20.0f, 20.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, controller.update(1.0f, kNaN, 0.01f, -20.0f, 20.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, controller.update(kInf, 0.0f, 0.01f, -20.0f, 20.0f));
  // A good call carries on from 0.1.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, controller.update(1.0f, 0.0f, 0.1f, -20.0f, 20.0f));
}

// pid-tuner tests/control_test.cpp, lines 28-29, plus the upper cap.
void test_pid_refuses_bad_gains_and_keeps_the_old_ones() {
  tuning::PidController controller;
  TEST_ASSERT_TRUE(controller.setGains(2.0f, 0.0f, 0.0f));
  TEST_ASSERT_FALSE(controller.setGains(kNaN, 0.0f, 0.0f));
  TEST_ASSERT_FALSE(controller.setGains(-1.0f, 0.0f, 0.0f));
  TEST_ASSERT_FALSE(controller.setGains(0.0f, kInf, 0.0f));
  TEST_ASSERT_FALSE(controller.setGains(0.0f, 0.0f, 10001.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, controller.update(1.0f, 0.0f, 0.01f, -20.0f, 20.0f));
  TEST_ASSERT_TRUE(controller.setGains(10000.0f, 0.0f, 0.0f));
}

void test_pid_derivative_is_on_the_measurement_and_filtered() {
  tuning::PidController controller;
  TEST_ASSERT_TRUE(controller.setGains(0.0f, 0.0f, 1.0f));
  // First call after a reset: no derivative yet.
  TEST_ASSERT_EXACT_FLOAT(0.0f, controller.update(0.0f, 0.0f, 0.01f, -1000.0f, 1000.0f));
  // A step in the TARGET gives no kick.
  TEST_ASSERT_EXACT_FLOAT(0.0f, controller.update(5.0f, 0.0f, 0.01f, -1000.0f, 1000.0f));
  // The measurement rises 1 deg in 10 ms: -100 deg/s, of which the filter
  // (0.2 per sample) lets 20 % through. Rising measurement -> negative output.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -20.0f, controller.update(5.0f, 1.0f, 0.01f, -1000.0f, 1000.0f));
  // Then it stops moving, and the filter decays by 0.8 per sample.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -16.0f, controller.update(5.0f, 1.0f, 0.01f, -1000.0f, 1000.0f));
}

// ---------------------------------------------------------------------------
// 2. ServoControl: the pulse formula
// ---------------------------------------------------------------------------

void test_pulse_is_centre_plus_slope_times_angle() {
  const ServoConfig config = plainConfig();
  Rig rig(config);
  const float command[kNumCanards] = {2.0f * kDeg, -3.0f * kDeg, 0.0f, 5.0f * kDeg};
  rig.cycle(command, 0.005f, feedbackAtAll(config, 0.0f));
  // Canards 0, 1, 2, 3 are channels 0, 2, 3, 1.
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[0]);
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1470.0f, rig.pulseUs[2]);
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1500.0f, rig.pulseUs[3]);
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1550.0f, rig.pulseUs[1]);
}

void test_bench_rows_give_the_expected_pulses() {
  // centre + us/deg * angle for each bench row, worked out by hand. On every
  // channel a positive canard angle is a SHORTER pulse.
  struct Case {
    float canardDeg;
    float pulseUs[kNumCanards];  // by servo channel, CH1..CH4
  };
  const Case cases[] = {
      {4.0f, {1408.58f, 1313.62f, 1362.90f, 1411.42f}},
      {-4.0f, {1546.02f, 1458.98f, 1539.70f, 1563.18f}},
      {7.5f, {1348.45f, 1250.025f, 1285.55f, 1345.025f}},
      {-7.5f, {1606.15f, 1522.575f, 1617.05f, 1629.575f}},
  };
  const ServoConfig config = benchConfig();
  for (const Case& c : cases) {
    Rig rig(config);
    const float angle = c.canardDeg * kDeg;
    const float command[kNumCanards] = {angle, angle, angle, angle};
    // Zero gains, so where the encoders sit makes no difference.
    rig.cycle(command, 0.005f, feedbackAtAll(config, 0.0f));
    for (int ch = 0; ch < kNumCanards; ++ch) {
      TEST_ASSERT_FLOAT_WITHIN(0.01f, c.pulseUs[ch], rig.pulseUs[ch]);
      // Inside the row's own pulse limits, so nothing here was clamped.
      TEST_ASSERT_TRUE(rig.pulseUs[ch] > config.channels[ch].minPulseUs);
      TEST_ASSERT_TRUE(rig.pulseUs[ch] < config.channels[ch].maxPulseUs);
    }
  }
}

// ---------------------------------------------------------------------------
// 2. ServoControl: which canard is which channel
// ---------------------------------------------------------------------------

void test_each_canard_drives_its_own_channel() {
  const ServoConfig config = distinctConfig();
  // 2 degrees on each channel's own calibration line.
  const float moved[kNumCanards] = {1520.0f, 1532.0f, 1544.0f, 1556.0f};
  for (int canard = 0; canard < kNumCanards; ++canard) {
    Rig rig(config);
    float command[kNumCanards] = {};
    command[canard] = 2.0f * kDeg;
    rig.cycle(command, 0.005f, feedbackAtAll(config, 0.0f));
    for (int ch = 0; ch < kNumCanards; ++ch) {
      if (ch == kChannelOfCanard[canard]) {
        TEST_ASSERT_FLOAT_WITHIN(kPulseTol, moved[ch], rig.pulseUs[ch]);
      } else {
        TEST_ASSERT_EXACT_FLOAT(config.channels[ch].centerPulseUs, rig.pulseUs[ch]);
      }
    }
  }
}

void test_each_encoder_is_reported_on_its_own_canard() {
  const ServoConfig config = distinctConfig();
  for (int ch = 0; ch < kNumCanards; ++ch) {
    ServoControl servo(config);
    float shaftDeg[kNumCanards] = {};
    shaftDeg[ch] = 3.0f;
    CanardAngles angles;
    servo.measure(feedbackAt(config, shaftDeg[0], shaftDeg[1], shaftDeg[2], shaftDeg[3]), angles);
    TEST_ASSERT_TRUE(angles.measured);
    for (int canard = 0; canard < kNumCanards; ++canard) {
      const float want = canard == kCanardOfChannel[ch] ? 3.0f * kDeg : 0.0f;
      TEST_ASSERT_FLOAT_WITHIN(kAngleTol, want, angles.rad[canard]);
    }
  }
}

void test_the_channel_map_comes_from_the_config() {
  // A different airframe: nothing may be wired to a global table.
  ServoConfig config = distinctConfig();
  const int map[kNumCanards] = {3, 0, 1, 2};
  for (int i = 0; i < kNumCanards; ++i) config.channelOfCanard[i] = map[i];
  Rig rig(config);

  // Canard 0 (channel 3) is commanded to +2 deg; canard 2 (channel 1) sits at
  // -4 deg.
  const float command[kNumCanards] = {2.0f * kDeg, 0.0f, 0.0f, 0.0f};
  rig.cycle(command, 0.005f, feedbackAt(config, 0.0f, -4.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(rig.angles.measured);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, -4.0f * kDeg, rig.angles.rad[2]);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, rig.angles.rad[0]);
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1556.0f, rig.pulseUs[3]);
  TEST_ASSERT_EXACT_FLOAT(1500.0f, rig.pulseUs[0]);
  TEST_ASSERT_EXACT_FLOAT(1510.0f, rig.pulseUs[1]);
  TEST_ASSERT_EXACT_FLOAT(1520.0f, rig.pulseUs[2]);
}

void test_a_broken_channel_map_drives_nothing() {
  const int broken[][kNumCanards] = {
      {0, 2, 3, 3},   // one channel named twice, one never
      {0, 2, 3, 4},   // a channel that does not exist
      {-1, 2, 3, 1},
  };
  for (const auto& map : broken) {
    ServoConfig config = distinctConfig();
    for (int i = 0; i < kNumCanards; ++i) config.channelOfCanard[i] = map[i];
    ServoControl servo(config);
    TEST_ASSERT_FALSE(servo.allCalibrated());

    const ActuatorFeedback feedback = feedbackAtAll(config, 3.0f);
    CanardAngles angles;
    servo.measure(feedback, angles);
    TEST_ASSERT_FALSE(angles.measured);

    const float command[kNumCanards] = {4.0f * kDeg, 4.0f * kDeg, 4.0f * kDeg, 4.0f * kDeg};
    float pulseUs[kNumCanards] = {9999.0f, 9999.0f, 9999.0f, 9999.0f};
    servo.command(command, 0.005f, feedback, pulseUs);
    for (int ch = 0; ch < kNumCanards; ++ch) {
      TEST_ASSERT_EXACT_FLOAT(config.channels[ch].centerPulseUs, pulseUs[ch]);
    }
    servo.measure(feedback, angles);
    for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_EXACT_FLOAT(0.0f, angles.rad[i]);
  }
}

// ---------------------------------------------------------------------------
// 2. ServoControl: signs and the encoder wrap
// ---------------------------------------------------------------------------

void test_reversed_encoder_direction() {
  // Canard 1 is channel 2. Its encoder counts DOWN as the canard goes positive.
  ServoConfig config = plainConfig();
  config.channels[2].direction = -1.0f;
  config.channels[2].kp = 0.5f;
  Rig rig(config);

  ActuatorFeedback feedback = feedbackAtAll(config, 0.0f);
  feedback.encoderAngleRad[2] = 98.0f * kDeg;  // 2 deg below the zero angle
  rig.servo.measure(feedback, rig.angles);
  TEST_ASSERT_TRUE(rig.angles.measured);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 2.0f * kDeg, rig.angles.rad[1]);
  feedback.encoderAngleRad[2] = 102.0f * kDeg;
  rig.servo.measure(feedback, rig.angles);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, -2.0f * kDeg, rig.angles.rad[1]);

  // The pulse does not care about the encoder direction: the slope's own sign
  // says which way the servo turns.
  const float command[kNumCanards] = {0.0f, 2.0f * kDeg, 0.0f, 0.0f};
  feedback.encoderAngleRad[2] = 99.0f * kDeg;  // the shaft is at +1 deg of the 2 wanted
  rig.cycle(command, 0.010f, feedback);
  // 1 deg short: the loop adds kp * 1 deg. Read the wrong way round, the
  // shaft would seem 3 deg short and this would be 1535.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[2]);
}

void test_canard_sign_flips_command_and_measurement() {
  // Canard 1 (channel 2) is mounted so controller-positive is shaft-negative.
  ServoConfig config = plainConfig();
  config.canardSign[1] = -1.0f;
  {
    Rig rig(config);
    const float command[kNumCanards] = {3.0f * kDeg, 3.0f * kDeg, 0.0f, 0.0f};
    // Shafts of channels 0 and 2 both at +3 deg.
    rig.cycle(command, 0.005f, feedbackAt(config, 3.0f, 0.0f, 3.0f, 0.0f));
    TEST_ASSERT_TRUE(rig.angles.measured);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 3.0f * kDeg, rig.angles.rad[0]);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, -3.0f * kDeg, rig.angles.rad[1]);
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1530.0f, rig.pulseUs[0]);
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1470.0f, rig.pulseUs[2]);
  }
  {
    // Closed loop stays negative feedback: canard +2 deg wanted (shaft -2),
    // canard at +1 deg (shaft -1). The shaft must go 0.5 deg further negative.
    config.channels[2].kp = 0.5f;
    Rig rig(config);
    const float command[kNumCanards] = {0.0f, 2.0f * kDeg, 0.0f, 0.0f};
    rig.cycle(command, 0.010f, feedbackAt(config, 0.0f, 0.0f, -1.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 1.0f * kDeg, rig.angles.rad[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1475.0f, rig.pulseUs[2]);
  }
}

void test_encoder_wrap_across_zero_is_measured_about_the_zero_angle() {
  ServoConfig config = plainConfig();
  config.channels[0].zeroAngle = 359.0f;  // canard 0
  config.channels[2].zeroAngle = 1.0f;    // canard 1
  ServoControl servo(config);
  CanardAngles angles;

  ActuatorFeedback feedback = feedbackAtAll(config, 0.0f);
  feedback.encoderAngleRad[0] = 1.0f * kDeg;    // 2 deg past a zero at 359
  feedback.encoderAngleRad[2] = 359.0f * kDeg;  // 2 deg before a zero at 1
  servo.measure(feedback, angles);
  TEST_ASSERT_TRUE(angles.measured);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 2.0f * kDeg, angles.rad[0]);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, -2.0f * kDeg, angles.rad[1]);

  // The ends of the encoder's range, and the same side of the wrap.
  feedback.encoderAngleRad[0] = 0.0f;
  feedback.encoderAngleRad[2] = 4095.0f / 4096.0f * 2.0f * kPi;  // the encoder's last count
  servo.measure(feedback, angles);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 1.0f * kDeg, angles.rad[0]);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, (4095.0f / 4096.0f * 360.0f - 361.0f) * kDeg, angles.rad[1]);
  feedback.encoderAngleRad[0] = 357.0f * kDeg;
  feedback.encoderAngleRad[2] = 3.0f * kDeg;
  servo.measure(feedback, angles);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, -2.0f * kDeg, angles.rad[0]);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 2.0f * kDeg, angles.rad[1]);
}

void test_bench_zero_near_the_wrap_reads_small_angles_either_side() {
  // CH4's zero is at 342.26 deg, so +7.5 deg is still below 360 but +20 deg
  // is past the wrap. CH4 is canard 2.
  const ServoConfig config = benchConfig();
  ServoControl servo(config);
  CanardAngles angles;
  servo.measure(feedbackAt(config, 0.0f, 0.0f, 0.0f, 7.5f), angles);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 7.5f * kDeg, angles.rad[2]);
  servo.measure(feedbackAt(config, 0.0f, 0.0f, 0.0f, 20.0f), angles);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 20.0f * kDeg, angles.rad[2]);
  servo.measure(feedbackAt(config, 0.0f, 0.0f, 0.0f, -7.5f), angles);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, -7.5f * kDeg, angles.rad[2]);
}

// ---------------------------------------------------------------------------
// 2. ServoControl: what never moves a servo off centre
// ---------------------------------------------------------------------------

void test_uncalibrated_channel_gets_its_centre_pulse_and_the_rest_still_move() {
  ServoConfig config = distinctConfig();
  config.channels[2].calibrated = false;  // canard 1
  Rig rig(config);
  TEST_ASSERT_FALSE(rig.servo.allCalibrated());

  const float command[kNumCanards] = {2.0f * kDeg, 2.0f * kDeg, 2.0f * kDeg, 2.0f * kDeg};
  for (int n = 0; n < 6; ++n) {
    rig.cycle(command, 0.005f, feedbackAtAll(config, 2.0f));
    TEST_ASSERT_EXACT_FLOAT(1520.0f, rig.pulseUs[2]);  // its centre, untouched
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[0]);
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1532.0f, rig.pulseUs[1]);
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1556.0f, rig.pulseUs[3]);
    // One channel without a known zero: nothing is reported as measured.
    TEST_ASSERT_FALSE(rig.angles.measured);
  }
}

// The intent of Canards-Rocket's test_uncalibrated_channels_only_send_centre_pulses,
// with a table of its own (the reference test reads the global config).
void test_uncalibrated_table_only_sends_centre_pulses() {
  ServoConfig config = distinctConfig();
  for (int ch = 0; ch < kNumCanards; ++ch) {
    config.channels[ch].calibrated = false;
    config.channels[ch].kp = 0.5f;
    config.channels[ch].ki = 10.0f;
    config.channels[ch].kd = 0.01f;
  }
  Rig rig(config);
  TEST_ASSERT_FALSE(rig.servo.allCalibrated());
  for (int n = 0; n < 60; ++n) {
    const float swing = 7.5f * kDeg * std::sin(0.3f * static_cast<float>(n));
    const float command[kNumCanards] = {swing, -swing, 7.5f * kDeg, -7.5f * kDeg};
    rig.cycle(command, 0.005f, feedbackAtAll(config, static_cast<float>(n % 7) - 3.0f));
    TEST_ASSERT_FALSE(rig.angles.measured);
    for (int ch = 0; ch < kNumCanards; ++ch) {
      TEST_ASSERT_EXACT_FLOAT(config.channels[ch].centerPulseUs, rig.pulseUs[ch]);
    }
  }
}

void test_non_finite_command_gives_centre_on_that_channel_only() {
  const ServoConfig config = plainConfig();
  const float bad[] = {kNaN, kInf, -kInf};
  for (float value : bad) {
    Rig rig(config);
    // Canard 1 is channel 2.
    const float command[kNumCanards] = {2.0f * kDeg, value, -2.0f * kDeg, 1.0f * kDeg};
    for (int n = 0; n < 4; ++n) {
      rig.cycle(command, 0.005f, feedbackAtAll(config, 0.0f));
      TEST_ASSERT_EXACT_FLOAT(1500.0f, rig.pulseUs[2]);
      TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[0]);
      TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1480.0f, rig.pulseUs[3]);
      TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1510.0f, rig.pulseUs[1]);
    }
    // The model must not be poisoned: it is what the attitude controller is
    // given when an encoder drops out. That canard's servo is at centre, so
    // its model stays at neutral.
    float model[kNumCanards];
    readModel(rig.servo, model);
    for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_TRUE(std::isfinite(model[i]));
    // Not exactly zero: the model was seeded from a measured zero, which
    // comes back as about 1e-7 rad after the degree/radian round trip.
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, model[1]);
    TEST_ASSERT_TRUE(model[0] > 0.0f);
  }
}

void test_non_finite_command_restarts_that_channels_loop() {
  ServoConfig config = plainConfig();
  config.channels[0].ki = 10.0f;
  Rig rig(config);
  const ActuatorFeedback feedback = feedbackAtAll(config, 0.0f);
  const float command[kNumCanards] = {2.0f * kDeg, 0.0f, 0.0f, 0.0f};
  // 2 deg of error, ki 10, 10 ms per run: 0.2 deg (2 us) more each cycle.
  for (int n = 0; n < 5; ++n) rig.cycle(command, 0.010f, feedback);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1530.0f, rig.pulseUs[0]);

  const float nanCommand[kNumCanards] = {kNaN, 0.0f, 0.0f, 0.0f};
  rig.cycle(nanCommand, 0.010f, feedback);
  TEST_ASSERT_EXACT_FLOAT(1500.0f, rig.pulseUs[0]);

  // The 1 deg that had built up is gone.
  rig.cycle(command, 0.010f, feedback);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1522.0f, rig.pulseUs[0]);
}

void test_bad_calibration_numbers_hold_that_channel_at_centre() {
  // Each of these makes channel 0 (canard 0) impossible to drive safely.
  // Channel 2 (canard 1) must carry on.
  const float command[kNumCanards] = {2.0f * kDeg, 2.0f * kDeg, 0.0f, 0.0f};
  for (int variant = 0; variant < 6; ++variant) {
    ServoConfig config = plainConfig();
    switch (variant) {
      case 0: config.channels[0].pulseUsPerDegree = kNaN; break;
      case 1: config.channels[0].pulseUsPerDegree = kInf; break;
      case 2: config.channels[0].maxPulseUs = kNaN; break;
      case 3: config.channels[0].minPulseUs = 1600.0f; config.channels[0].maxPulseUs = 1400.0f; break;
      case 4: config.channels[0].minAngle = kNaN; break;
      case 5: config.canardSign[0] = kNaN; break;
    }
    Rig rig(config);
    for (int n = 0; n < 3; ++n) {
      rig.cycle(command, 0.005f, feedbackAtAll(config, 0.0f));
      TEST_ASSERT_EXACT_FLOAT_MESSAGE(1500.0f, rig.pulseUs[0], "channel 0 must stay at centre");
      TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[2]);
    }
  }
}

// ---------------------------------------------------------------------------
// 2. ServoControl: limits
// ---------------------------------------------------------------------------

// Canards-Rocket test_pulses_stay_inside_travel_and_pulse_limits.
void test_pulses_stay_inside_travel_and_pulse_limits() {
  // Steep calibration: 30 us/deg, so the 1400..1600 pulse limits are reached
  // at 3.3 deg. Travel is only +/-2 deg.
  ServoConfig config = plainConfig();
  for (int ch = 0; ch < kNumCanards; ++ch) {
    config.channels[ch].pulseUsPerDegree = 30.0f;
    config.channels[ch].minAngle = -2.0f;
    config.channels[ch].maxAngle = 2.0f;
  }
  const float command[kNumCanards] = {7.0f * kDeg, -7.0f * kDeg, 1.0f * kDeg, 0.0f};
  {
    Rig rig(config);
    rig.cycle(command, 0.005f, feedbackAtAll(config, 0.0f));
    TEST_ASSERT_EXACT_FLOAT(1560.0f, rig.pulseUs[0]);  // canard 0 stopped at +2 deg
    TEST_ASSERT_EXACT_FLOAT(1440.0f, rig.pulseUs[2]);  // canard 1 stopped at -2 deg
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1530.0f, rig.pulseUs[3]);
    TEST_ASSERT_EXACT_FLOAT(1500.0f, rig.pulseUs[1]);
  }
  {
    // With the travel widened, the pulse limits are what stops it.
    config.channels[0].maxAngle = 8.0f;
    config.channels[2].minAngle = -8.0f;
    Rig rig(config);
    rig.cycle(command, 0.005f, feedbackAtAll(config, 0.0f));
    TEST_ASSERT_EXACT_FLOAT(1600.0f, rig.pulseUs[0]);
    TEST_ASSERT_EXACT_FLOAT(1400.0f, rig.pulseUs[2]);
  }
}

void test_loop_correction_cannot_push_the_shaft_past_its_travel() {
  // A strong loop and a shaft that will not move: target 7 deg, travel 8 deg.
  ServoConfig config = plainConfig();
  config.channels[0].kp = 5.0f;
  config.channels[0].ki = 50.0f;
  Rig rig(config);
  const float command[kNumCanards] = {7.0f * kDeg, 0.0f, 0.0f, 0.0f};
  for (int n = 0; n < 50; ++n) {
    rig.cycle(command, 0.010f, feedbackAtAll(config, 0.0f));
    // Never more than centre + 10 us/deg * 8 deg.
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1580.0f, rig.pulseUs[0]);
  }
}

// ---------------------------------------------------------------------------
// 2. ServoControl: open loop, closed loop, cadence
// ---------------------------------------------------------------------------

void test_zero_gains_run_open_loop_on_the_calibration() {
  // The encoders say the shafts are nowhere near the target. With no gains,
  // that must not change a single pulse.
  const ServoConfig config = benchConfig();
  Rig rig(config);
  const float command[kNumCanards] = {4.0f * kDeg, 4.0f * kDeg, 4.0f * kDeg, 4.0f * kDeg};
  const float want[kNumCanards] = {1408.58f, 1313.62f, 1362.90f, 1411.42f};
  for (int n = 0; n < 20; ++n) {
    rig.cycle(command, 0.005f, feedbackAtAll(config, -5.0f + 0.4f * static_cast<float>(n)));
    TEST_ASSERT_TRUE(rig.angles.measured);
    for (int ch = 0; ch < kNumCanards; ++ch) {
      TEST_ASSERT_FLOAT_WITHIN(0.01f, want[ch], rig.pulseUs[ch]);
    }
  }
}

void test_invalid_gains_leave_the_channel_open_loop() {
  // The tuner refuses the whole gain set if one gain is bad.
  ServoConfig config = plainConfig();
  config.channels[0].kp = -1.0f;
  config.channels[0].ki = 10.0f;
  config.channels[2].kp = kNaN;
  Rig rig(config);
  const float command[kNumCanards] = {2.0f * kDeg, 2.0f * kDeg, 0.0f, 0.0f};
  for (int n = 0; n < 5; ++n) {
    rig.cycle(command, 0.010f, feedbackAtAll(config, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[0]);
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[2]);
  }
}

// Canards-Rocket test_servo_loop_corrects_a_position_error_at_100_hz, extended
// to an error of the other sign.
void test_position_error_is_corrected_with_negative_feedback() {
  ServoConfig config = plainConfig();
  for (int ch = 0; ch < kNumCanards; ++ch) config.channels[ch].kp = 0.5f;
  Rig rig(config);
  const float command[kNumCanards] = {2.0f * kDeg, 0.0f, 0.0f, 0.0f};

  // The shaft sits at 1 deg while 2 are wanted.
  const ActuatorFeedback low = feedbackAt(config, 1.0f, 0.0f, 0.0f, 0.0f);
  rig.cycle(command, 0.005f, low);  // 5 ms: the 100 Hz loop has not run yet
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1520.0f, rig.pulseUs[0]);
  rig.cycle(command, 0.005f, low);  // 10 ms: it runs, and asks for 0.5 deg more
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, rig.pulseUs[1]);

  // Now the shaft overshoots to 3 deg.
  const ActuatorFeedback high = feedbackAt(config, 3.0f, 0.0f, 0.0f, 0.0f);
  rig.cycle(command, 0.005f, high);  // held between runs
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[0]);
  rig.cycle(command, 0.005f, high);  // 0.5 deg less
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1515.0f, rig.pulseUs[0]);
}

void test_negative_feedback_holds_with_the_bench_slope_sign() {
  // On the airframe us/deg is negative. A canard short of its positive target
  // must be driven further positive, which is a SHORTER pulse than open loop.
  ServoConfig config = benchConfig();
  for (int ch = 0; ch < kNumCanards; ++ch) config.channels[ch].kp = 0.5f;
  Rig rig(config);
  const float command[kNumCanards] = {4.0f * kDeg, 4.0f * kDeg, 4.0f * kDeg, 4.0f * kDeg};
  const float openLoop[kNumCanards] = {1408.58f, 1313.62f, 1362.90f, 1411.42f};

  rig.cycle(command, 0.010f, feedbackAtAll(config, 3.0f));  // 1 deg short
  for (int ch = 0; ch < kNumCanards; ++ch) {
    // Half a degree more, on each row's own slope.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, openLoop[ch] + 0.5f * config.channels[ch].pulseUsPerDegree,
                             rig.pulseUs[ch]);
    TEST_ASSERT_TRUE(rig.pulseUs[ch] < openLoop[ch]);
  }
  rig.cycle(command, 0.010f, feedbackAtAll(config, 5.0f));  // 1 deg over
  for (int ch = 0; ch < kNumCanards; ++ch) TEST_ASSERT_TRUE(rig.pulseUs[ch] > openLoop[ch]);
}

// Drives a pure-integral loop with a constant 2 deg error. Every loop run adds
// ki * error * (time handed to the PID) to the correction, so the pulse shows
// both WHEN the loop ran and WHAT elapsed time it was given.
void checkCadence(float rateHz, int everyCycles, int cycles) {
  ServoConfig config = plainConfig();
  for (int ch = 0; ch < kNumCanards; ++ch) config.channels[ch].ki = 10.0f;
  Rig rig(config);
  const float dt = 1.0f / rateHz;
  const float command[kNumCanards] = {2.0f * kDeg, 2.0f * kDeg, 2.0f * kDeg, 2.0f * kDeg};
  const ActuatorFeedback feedback = feedbackAtAll(config, 0.0f);
  // Degrees of correction added by one run: ki * error * accumulated time.
  const float perRunDeg = 10.0f * 2.0f * static_cast<float>(everyCycles) * dt;
  for (int n = 1; n <= cycles; ++n) {
    rig.cycle(command, dt, feedback);
    const float runs = static_cast<float>(n / everyCycles);
    const float want = 1500.0f + 10.0f * (2.0f + perRunDeg * runs);
    // One run is worth about 2 us, so 0.02 us tells a missed or extra run
    // apart from float rounding.
    for (int ch = 0; ch < kNumCanards; ++ch) {
      TEST_ASSERT_FLOAT_WITHIN(0.02f, want, rig.pulseUs[ch]);
    }
  }
}

void test_loop_runs_every_second_cycle_at_200_hz() { checkCadence(200.0f, 2, 24); }
void test_loop_runs_every_second_cycle_at_208_hz() { checkCadence(208.0f, 2, 24); }
void test_loop_runs_every_cycle_at_104_hz() { checkCadence(104.0f, 1, 24); }
void test_loop_runs_every_cycle_at_100_hz() { checkCadence(100.0f, 1, 24); }
void test_loop_runs_every_tenth_cycle_at_1_khz() { checkCadence(1000.0f, 10, 45); }

void test_loop_period_comes_from_the_config() {
  // A 50 Hz loop on a 200 Hz cycle: every 4th cycle, with 20 ms as its step.
  ServoConfig config = plainConfig();
  config.servoLoopPeriodS = 0.020f;
  config.channels[0].ki = 10.0f;
  Rig rig(config);
  const float command[kNumCanards] = {2.0f * kDeg, 0.0f, 0.0f, 0.0f};
  for (int n = 1; n <= 12; ++n) {
    rig.cycle(command, 0.005f, feedbackAtAll(config, 0.0f));
    const float runs = static_cast<float>(n / 4);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1520.0f + 10.0f * (10.0f * 2.0f * 0.020f) * runs,
                             rig.pulseUs[0]);
  }
}

// The tuner's control step (pid-tuner include/Tuner.h, Tuner::update):
//   correction = pid.update(target, measured, dt, minAngle - target, maxAngle - target)
//   pulse      = clamp(centre + us_per_deg * (target + correction), minPulse, maxPulse)
// ServoControl must give the same pulse, run for run, for the same shaft motion.
void checkAgainstTheTuner(float dt, int cyclesPerRun) {
  ServoConfig config = benchConfig();
  ServoChannel& row = config.channels[0];  // CH1, canard 0
  row.kp = 0.4f;
  row.ki = 2.0f;
  row.kd = 0.01f;
  Rig rig(config);

  tuning::PidController tuner;
  TEST_ASSERT_TRUE(tuner.setGains(row.kp, row.ki, row.kd));
  const float targetDeg = 4.0f;
  const float command[kNumCanards] = {targetDeg * kDeg, 0.0f, 0.0f, 0.0f};
  float tunerPulse = row.centerPulseUs + row.pulseUsPerDegree * targetDeg;  // before the first run

  float shaftDeg = 0.0f;
  bool sawACorrection = false;
  for (int n = 1; n <= 80; ++n) {
    // A shaft that creeps towards the target and wobbles: enough to exercise
    // all three terms.
    shaftDeg += 0.08f * (targetDeg - shaftDeg) + 0.05f * std::sin(0.7f * static_cast<float>(n));
    rig.cycle(command, dt, feedbackAt(config, shaftDeg, 0.0f, 0.0f, 0.0f));
    if (n % cyclesPerRun == 0) {
      const float correction =
          tuner.update(targetDeg, shaftDeg, dt * static_cast<float>(cyclesPerRun),
                       row.minAngle - targetDeg, row.maxAngle - targetDeg);
      tunerPulse = tuning::clamp(row.centerPulseUs + row.pulseUsPerDegree * (targetDeg + correction),
                                 row.minPulseUs, row.maxPulseUs);
      if (std::fabs(correction) > 0.1f) sawACorrection = true;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, tunerPulse, rig.pulseUs[0]);
  }
  TEST_ASSERT_TRUE(sawACorrection);  // otherwise this test compared nothing
}

void test_closed_loop_matches_the_tuner_at_100_hz() { checkAgainstTheTuner(0.010f, 1); }
void test_closed_loop_matches_the_tuner_on_a_200_hz_cycle() { checkAgainstTheTuner(0.005f, 2); }

// ---------------------------------------------------------------------------
// 2. ServoControl: lost encoders
// ---------------------------------------------------------------------------

void test_measured_flag_is_all_or_nothing_but_the_loop_is_per_channel() {
  ServoConfig config = plainConfig();
  for (int ch = 0; ch < kNumCanards; ++ch) config.channels[ch].kp = 0.5f;
  Rig rig(config);
  const float command[kNumCanards] = {2.0f * kDeg, 2.0f * kDeg, 2.0f * kDeg, 2.0f * kDeg};

  // Every shaft at 1 deg, every encoder good.
  ActuatorFeedback feedback = feedbackAtAll(config, 1.0f);
  rig.cycle(command, 0.010f, feedback);
  TEST_ASSERT_TRUE(rig.angles.measured);
  for (int i = 0; i < kNumCanards; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 1.0f * kDeg, rig.angles.rad[i]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[i]);
  }

  // Channel 1's encoder drops out.
  feedback.encoderValid[1] = false;
  rig.cycle(command, 0.010f, feedback);
  TEST_ASSERT_FALSE(rig.angles.measured);
  // All four angles are now the model, including the three still measured.
  // It started from the measured 1 deg and took one 10 ms step towards 2 deg.
  const float modelDeg = 2.0f - std::exp(-0.010f / 0.030f);
  for (int i = 0; i < kNumCanards; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, modelDeg * kDeg, rig.angles.rad[i]);
  }
  // But only channel 1 loses its position loop.
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[3]);

  // It comes back by itself.
  feedback.encoderValid[1] = true;
  rig.cycle(command, 0.010f, feedback);
  TEST_ASSERT_TRUE(rig.angles.measured);
  TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 1.0f * kDeg, rig.angles.rad[kCanardOfChannel[1]]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[1]);
}

void test_lost_encoder_restarts_that_channels_loop() {
  ServoConfig config = plainConfig();
  config.channels[0].ki = 10.0f;
  Rig rig(config);
  ActuatorFeedback feedback = feedbackAtAll(config, 0.0f);
  const float command[kNumCanards] = {2.0f * kDeg, 0.0f, 0.0f, 0.0f};
  for (int n = 0; n < 5; ++n) rig.cycle(command, 0.010f, feedback);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1530.0f, rig.pulseUs[0]);  // 1 deg built up

  feedback.encoderValid[0] = false;
  rig.cycle(command, 0.010f, feedback);
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[0]);  // calibration alone

  feedback.encoderValid[0] = true;
  rig.cycle(command, 0.010f, feedback);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1522.0f, rig.pulseUs[0]);  // from nothing again
}

void test_non_finite_encoder_angle_counts_as_no_encoder() {
  ServoConfig config = plainConfig();
  for (int ch = 0; ch < kNumCanards; ++ch) config.channels[ch].kp = 0.5f;
  const float bad[] = {kNaN, kInf, -kInf};
  for (float value : bad) {
    Rig rig(config);
    ActuatorFeedback feedback = feedbackAtAll(config, 1.0f);
    feedback.encoderAngleRad[0] = value;  // flagged valid, but not a number
    const float command[kNumCanards] = {2.0f * kDeg, 2.0f * kDeg, 0.0f, 0.0f};
    rig.cycle(command, 0.010f, feedback);
    TEST_ASSERT_FALSE(rig.angles.measured);
    for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_TRUE(std::isfinite(rig.angles.rad[i]));
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[0]);  // open loop
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[2]);      // canard 1 still closed loop
  }
}

// ---------------------------------------------------------------------------
// 2. ServoControl: the servo model
// ---------------------------------------------------------------------------

void test_servo_model_matches_the_reference_constant_at_5_ms() {
  const ServoConfig config = plainConfig();  // 30 ms lag
  // The measured zero the model starts from is about 1e-7 rad, not exactly
  // 0 (degree/radian round trip), and exp(-5/30) differs from 0.846482 in the
  // seventh digit; 1e-6 rad is still a thousand times finer than the encoder.
  constexpr float kModelTol = 1e-6f;
  Rig rig(config);
  const float command[kNumCanards] = {3.0f * kDeg, -2.0f * kDeg, 4.0f * kDeg, 1.0f * kDeg};

  // Good feedback at neutral, so the model starts from a measured zero.
  rig.cycle(command, 0.005f, feedbackAtAll(config, 0.0f));
  TEST_ASSERT_TRUE(rig.angles.measured);
  for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_FLOAT_WITHIN(kAngleTol, 0.0f, rig.angles.rad[i]);

  // The reference's servoModelStep, twice.
  float reference[kNumCanards] = {};
  for (int i = 0; i < kNumCanards; ++i) {
    reference[i] = kReferenceServoLag * reference[i] + (1.0f - kReferenceServoLag) * command[i];
  }
  float model[kNumCanards];
  readModel(rig.servo, model);
  for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_FLOAT_WITHIN(kModelTol, reference[i], model[i]);

  rig.servo.command(command, 0.005f, noFeedback(), rig.pulseUs);
  for (int i = 0; i < kNumCanards; ++i) {
    reference[i] = kReferenceServoLag * reference[i] + (1.0f - kReferenceServoLag) * command[i];
  }
  readModel(rig.servo, model);
  for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_FLOAT_WITHIN(2e-7f, reference[i], model[i]);
}

void test_servo_model_follows_the_real_time_step_and_time_constant() {
  const float command[kNumCanards] = {4.0f * kDeg, 4.0f * kDeg, 4.0f * kDeg, 4.0f * kDeg};
  float pulseUs[kNumCanards];
  float model[kNumCanards];

  // One 10 ms step closes 1 - exp(-1/3) of the gap with a 30 ms lag.
  ServoConfig config = plainConfig();
  ServoControl tenMs(config);
  tenMs.command(command, 0.010f, noFeedback(), pulseUs);
  readModel(tenMs, model);
  TEST_ASSERT_FLOAT_WITHIN(1e-7f, (1.0f - std::exp(-1.0f / 3.0f)) * command[0], model[0]);

  // Two 5 ms steps end in the same place as one 10 ms step.
  ServoControl twoFiveMs(config);
  twoFiveMs.command(command, 0.005f, noFeedback(), pulseUs);
  twoFiveMs.command(command, 0.005f, noFeedback(), pulseUs);
  float modelTwo[kNumCanards];
  readModel(twoFiveMs, modelTwo);
  for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_FLOAT_WITHIN(1e-7f, model[i], modelTwo[i]);

  // A slower servo: 60 ms.
  ServoConfig slowConfig = plainConfig();
  slowConfig.servoModelTauS = 0.060f;
  ServoControl slow(slowConfig);
  slow.command(command, 0.010f, noFeedback(), pulseUs);
  readModel(slow, model);
  TEST_ASSERT_FLOAT_WITHIN(1e-7f, (1.0f - std::exp(-1.0f / 6.0f)) * command[0], model[0]);

  // Left running, it settles on the command.
  for (int n = 0; n < 400; ++n) slow.command(command, 0.005f, noFeedback(), pulseUs);
  readModel(slow, model);
  for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_FLOAT_WITHIN(1e-6f, command[i], model[i]);
}

void test_servo_model_ignores_a_bad_time_step() {
  const ServoConfig config = plainConfig();
  ServoControl servo(config);
  const float command[kNumCanards] = {4.0f * kDeg, -4.0f * kDeg, 2.0f * kDeg, 1.0f * kDeg};
  float pulseUs[kNumCanards];
  servo.command(command, 0.005f, noFeedback(), pulseUs);
  float before[kNumCanards];
  readModel(servo, before);
  TEST_ASSERT_TRUE(before[0] > 0.0f);

  const float badDt[] = {0.0f, -0.005f, kNaN, kInf, -kInf};
  for (float dt : badDt) {
    servo.command(command, dt, noFeedback(), pulseUs);
    float after[kNumCanards];
    readModel(servo, after);
    for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_EXACT_FLOAT(before[i], after[i]);
    // The pulses still follow the calibration.
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1540.0f, pulseUs[0]);
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1460.0f, pulseUs[2]);
  }
}

// Canards-Rocket test_lost_encoder_falls_back_to_the_servo_model.
void test_model_starts_from_the_last_measurement() {
  const ServoConfig config = plainConfig();
  Rig rig(config);
  const float neutral[kNumCanards] = {};
  // Shafts by channel 1, 0.5, 2, -1 deg, so canards 0..3 are at 1, 2, -1, 0.5.
  ActuatorFeedback feedback = feedbackAt(config, 1.0f, 0.5f, 2.0f, -1.0f);
  rig.cycle(neutral, 0.005f, feedback);
  TEST_ASSERT_TRUE(rig.angles.measured);
  const float measuredDeg[kNumCanards] = {1.0f, 2.0f, -1.0f, 0.5f};
  for (int i = 0; i < kNumCanards; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, measuredDeg[i] * kDeg, rig.angles.rad[i]);
  }

  feedback.encoderValid[2] = false;  // one encoder drops out
  rig.servo.measure(feedback, rig.angles);
  TEST_ASSERT_FALSE(rig.angles.measured);
  // One 5 ms lag step from the measured angles towards the neutral command.
  for (int i = 0; i < kNumCanards; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(kAngleTol, kReferenceServoLag * measuredDeg[i] * kDeg,
                             rig.angles.rad[i]);
  }
}

// ---------------------------------------------------------------------------
// 2. ServoControl: time steps, reset, the small accessors
// ---------------------------------------------------------------------------

void test_bad_time_step_does_not_disturb_the_cadence() {
  ServoConfig config = plainConfig();
  config.channels[0].kp = 0.5f;
  Rig rig(config);
  const float command[kNumCanards] = {2.0f * kDeg, 0.0f, 0.0f, 0.0f};
  const ActuatorFeedback feedback = feedbackAt(config, 1.0f, 0.0f, 0.0f, 0.0f);

  rig.cycle(command, 0.0f, feedback);  // the first cycle of a flight has dt = 0
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[0]);
  rig.cycle(command, 0.005f, feedback);  // 5 ms so far
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[0]);
  rig.cycle(command, kNaN, feedback);  // adds no time and runs nothing
  TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, rig.pulseUs[0]);
  rig.cycle(command, 0.005f, feedback);  // 10 ms: the loop runs
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[0]);
  rig.cycle(command, kInf, feedback);  // the correction is held
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1525.0f, rig.pulseUs[0]);
}

void test_reset_forgets_the_loop_the_cadence_and_the_model() {
  ServoConfig config = plainConfig();
  for (int ch = 0; ch < kNumCanards; ++ch) {
    config.channels[ch].kp = 0.5f;
    config.channels[ch].ki = 10.0f;
    config.channels[ch].kd = 0.02f;
  }
  const float command[kNumCanards] = {2.0f * kDeg, -1.0f * kDeg, 3.0f * kDeg, 0.5f * kDeg};

  // Seven 5 ms cycles: integrators wound up, a correction held, the model
  // away from zero, and half a loop period (5 ms) already counted.
  Rig used(config);
  for (int n = 0; n < 7; ++n) {
    used.cycle(command, 0.005f, feedbackAtAll(config, 0.2f * static_cast<float>(n)));
  }
  TEST_ASSERT_TRUE(std::fabs(used.pulseUs[0] - 1520.0f) > 1.0f);  // the loop really was active
  used.servo.reset();

  float model[kNumCanards];
  readModel(used.servo, model);
  for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_EXACT_FLOAT(0.0f, model[i]);

  // From here on it must be indistinguishable from a new object.
  Rig fresh(config);
  for (int n = 1; n <= 9; ++n) {
    ActuatorFeedback feedback = feedbackAtAll(config, 1.0f - 0.1f * static_cast<float>(n));
    feedback.encoderValid[3] = n < 6;  // later cycles expose the model through measure()
    used.cycle(command, 0.005f, feedback);
    fresh.cycle(command, 0.005f, feedback);
    TEST_ASSERT_EQUAL(fresh.angles.measured, used.angles.measured);
    for (int i = 0; i < kNumCanards; ++i) {
      TEST_ASSERT_EXACT_FLOAT(fresh.pulseUs[i], used.pulseUs[i]);
      TEST_ASSERT_EXACT_FLOAT(fresh.angles.rad[i], used.angles.rad[i]);
    }
    if (n == 1) {
      // 5 ms after a reset the 100 Hz loop has not run. Had the 5 ms counted
      // before the reset survived, it would have.
      TEST_ASSERT_FLOAT_WITHIN(kPulseTol, 1520.0f, used.pulseUs[0]);
    }
    if (n == 2) {
      // Shaft at 0.8 deg, 2 wanted: P 0.6 + I 0.12, and no derivative on the
      // first run after a reset.
      TEST_ASSERT_FLOAT_WITHIN(0.01f, 1527.2f, used.pulseUs[0]);
    }
  }
}

void test_center_pulses_and_all_calibrated() {
  ServoConfig config = distinctConfig();
  {
    const ServoControl servo(config);
    float pulseUs[kNumCanards] = {};
    servo.centerPulses(pulseUs);
    TEST_ASSERT_EXACT_FLOAT(1500.0f, pulseUs[0]);
    TEST_ASSERT_EXACT_FLOAT(1510.0f, pulseUs[1]);
    TEST_ASSERT_EXACT_FLOAT(1520.0f, pulseUs[2]);
    TEST_ASSERT_EXACT_FLOAT(1530.0f, pulseUs[3]);
    TEST_ASSERT_TRUE(servo.allCalibrated());
  }
  for (int ch = 0; ch < kNumCanards; ++ch) {
    ServoConfig oneMissing = config;
    oneMissing.channels[ch].calibrated = false;
    const ServoControl servo(oneMissing);
    TEST_ASSERT_FALSE(servo.allCalibrated());
  }
}

void test_neutral_command_is_the_centre_pulse() {
  const ServoConfig config = benchConfig();
  Rig rig(config);
  const float neutral[kNumCanards] = {};
  float centre[kNumCanards];
  rig.servo.centerPulses(centre);
  rig.cycle(neutral, 0.005f, feedbackAtAll(config, 0.0f));
  for (int ch = 0; ch < kNumCanards; ++ch) {
    TEST_ASSERT_FLOAT_WITHIN(kPulseTol, centre[ch], rig.pulseUs[ch]);
  }
}

// ---------------------------------------------------------------------------
// 3. The configuration as shipped
// ---------------------------------------------------------------------------

// VehicleConfig marks every servo row calibrated now that the canard
// handedness is confirmed (2026-11-21), so the flight program really moves the
// canards. On every channel a longer pulse turns the canard anticlockwise seen
// from the tail, so a POSITIVE canard command must give a SHORTER pulse, and
// the pulse limits must hold whatever is commanded.
void test_shipped_config_is_calibrated_and_a_positive_canard_shortens_its_pulse() {
  const ServoConfig& config = VehicleConfig::kServoConfig;
  for (int ch = 0; ch < kNumCanards; ++ch) TEST_ASSERT_TRUE(config.channels[ch].calibrated);
  {
    const ServoControl servo(config);
    TEST_ASSERT_TRUE(servo.allCalibrated());
  }

  // Each canard alone, both ways: only its own channel moves, the right way.
  const float small = 2.0f * kDeg;
  const float signs[] = {1.0f, -1.0f};
  for (int canard = 0; canard < kNumCanards; ++canard) {
    for (float sign : signs) {
      Rig rig(config);
      float command[kNumCanards] = {};
      command[canard] = sign * small;
      rig.cycle(command, 0.005f, noFeedback());
      const int moved = config.channelOfCanard[canard];
      for (int ch = 0; ch < kNumCanards; ++ch) {
        const float centre = config.channels[ch].centerPulseUs;
        if (ch != moved) {
          TEST_ASSERT_EXACT_FLOAT(centre, rig.pulseUs[ch]);
        } else if (sign > 0.0f) {
          TEST_ASSERT_TRUE_MESSAGE(rig.pulseUs[ch] < centre, "positive canard: shorter pulse");
        } else {
          TEST_ASSERT_TRUE_MESSAGE(rig.pulseUs[ch] > centre, "negative canard: longer pulse");
        }
      }
    }
  }

  // Anything commanded, with or without encoders: inside the pulse limits.
  for (int n = 0; n < 80; ++n) {
    Rig rig(config);
    const float swing = 30.0f * kDeg * std::sin(0.4f * static_cast<float>(n));
    const float command[kNumCanards] = {swing, -swing, 90.0f * kDeg, -90.0f * kDeg};
    ActuatorFeedback feedback;
    for (int ch = 0; ch < kNumCanards; ++ch) {
      feedback.encoderAngleRad[ch] = std::fmod(0.3f * static_cast<float>(n + ch), 2.0f * kPi);
      feedback.encoderValid[ch] = n % 2 == 0;
    }
    for (int k = 0; k < 5; ++k) rig.cycle(command, 0.005f, feedback);
    for (int ch = 0; ch < kNumCanards; ++ch) {
      TEST_ASSERT_TRUE(rig.pulseUs[ch] >= config.channels[ch].minPulseUs);
      TEST_ASSERT_TRUE(rig.pulseUs[ch] <= config.channels[ch].maxPulseUs);
    }
  }
}

void test_shipped_channel_map_is_usable() {
  // If the map were not four different channels, ServoControl would refuse to
  // drive anything even after calibration. Check it with the rows marked
  // calibrated in a local copy.
  ServoConfig config = VehicleConfig::kServoConfig;
  for (int ch = 0; ch < kNumCanards; ++ch) config.channels[ch].calibrated = true;
  const ServoControl servo(config);
  TEST_ASSERT_TRUE(servo.allCalibrated());
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();

  RUN_TEST(test_wrapped_error_takes_the_short_way_round);
  RUN_TEST(test_clamp_limits_a_number_and_passes_nan);
  RUN_TEST(test_pid_ignores_a_zero_time_step_then_adds_p_and_i);
  RUN_TEST(test_pid_saturated_output_does_not_wind_up);
  RUN_TEST(test_pid_integral_freezes_while_saturated_and_resumes_after);
  RUN_TEST(test_pid_holds_its_last_output_on_a_bad_call);
  RUN_TEST(test_pid_refuses_bad_gains_and_keeps_the_old_ones);
  RUN_TEST(test_pid_derivative_is_on_the_measurement_and_filtered);

  RUN_TEST(test_pulse_is_centre_plus_slope_times_angle);
  RUN_TEST(test_bench_rows_give_the_expected_pulses);

  RUN_TEST(test_each_canard_drives_its_own_channel);
  RUN_TEST(test_each_encoder_is_reported_on_its_own_canard);
  RUN_TEST(test_the_channel_map_comes_from_the_config);
  RUN_TEST(test_a_broken_channel_map_drives_nothing);

  RUN_TEST(test_reversed_encoder_direction);
  RUN_TEST(test_canard_sign_flips_command_and_measurement);
  RUN_TEST(test_encoder_wrap_across_zero_is_measured_about_the_zero_angle);
  RUN_TEST(test_bench_zero_near_the_wrap_reads_small_angles_either_side);

  RUN_TEST(test_uncalibrated_channel_gets_its_centre_pulse_and_the_rest_still_move);
  RUN_TEST(test_uncalibrated_table_only_sends_centre_pulses);
  RUN_TEST(test_non_finite_command_gives_centre_on_that_channel_only);
  RUN_TEST(test_non_finite_command_restarts_that_channels_loop);
  RUN_TEST(test_bad_calibration_numbers_hold_that_channel_at_centre);

  RUN_TEST(test_pulses_stay_inside_travel_and_pulse_limits);
  RUN_TEST(test_loop_correction_cannot_push_the_shaft_past_its_travel);

  RUN_TEST(test_zero_gains_run_open_loop_on_the_calibration);
  RUN_TEST(test_invalid_gains_leave_the_channel_open_loop);
  RUN_TEST(test_position_error_is_corrected_with_negative_feedback);
  RUN_TEST(test_negative_feedback_holds_with_the_bench_slope_sign);
  RUN_TEST(test_loop_runs_every_second_cycle_at_200_hz);
  RUN_TEST(test_loop_runs_every_second_cycle_at_208_hz);
  RUN_TEST(test_loop_runs_every_cycle_at_104_hz);
  RUN_TEST(test_loop_runs_every_cycle_at_100_hz);
  RUN_TEST(test_loop_runs_every_tenth_cycle_at_1_khz);
  RUN_TEST(test_loop_period_comes_from_the_config);
  RUN_TEST(test_closed_loop_matches_the_tuner_at_100_hz);
  RUN_TEST(test_closed_loop_matches_the_tuner_on_a_200_hz_cycle);

  RUN_TEST(test_measured_flag_is_all_or_nothing_but_the_loop_is_per_channel);
  RUN_TEST(test_lost_encoder_restarts_that_channels_loop);
  RUN_TEST(test_non_finite_encoder_angle_counts_as_no_encoder);

  RUN_TEST(test_servo_model_matches_the_reference_constant_at_5_ms);
  RUN_TEST(test_servo_model_follows_the_real_time_step_and_time_constant);
  RUN_TEST(test_servo_model_ignores_a_bad_time_step);
  RUN_TEST(test_model_starts_from_the_last_measurement);

  RUN_TEST(test_bad_time_step_does_not_disturb_the_cadence);
  RUN_TEST(test_reset_forgets_the_loop_the_cadence_and_the_model);
  RUN_TEST(test_center_pulses_and_all_calibrated);
  RUN_TEST(test_neutral_command_is_the_centre_pulse);

  RUN_TEST(test_shipped_config_is_calibrated_and_a_positive_canard_shortens_its_pulse);
  RUN_TEST(test_shipped_channel_map_is_usable);

  return UNITY_END();
}
