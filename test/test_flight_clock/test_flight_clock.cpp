// The flight clock and the launch-cycle rules in FlightComputer:
//   - one bad IMU timestamp cannot complete a debounce or fire a timeout;
//   - a CanardsEnable that arrives on the launch cycle is refused;
//   - the estimator is told "not on the pad" as soon as the IMU reads
//     launch-level specific force, before the launch debounce completes.

#include <unity.h>

#include <cstdint>

#include "AttitudeControl/LqrAttitudeController.h"
#include "FakeActuators.h"
#include "FakeCommandSource.h"
#include "FakeEstimator.h"
#include "FakeSensorSource.h"
#include "FlightComputer.h"
#include "FlightConfig.h"
#include "RecordingLogSink.h"
#include "VehicleConfig.h"

void setUp() {}
void tearDown() {}

namespace {

namespace fc = FlightConfig;

constexpr uint64_t kPeriodUs = 5000;
constexpr uint64_t kStartUs = 1 * fc::kUsPerSecond;
static_assert(kPeriodUs <= fc::kMaxImuGapUs, "the nominal period must not be an IMU gap");

// Samples until a hold of `us` that starts on a sample is met.
constexpr uint64_t samplesFor(uint64_t us) { return (us + kPeriodUs - 1) / kPeriodUs; }

// A rocket that sits at 1 g until `high` is set, then reads 6 g. The
// estimator reports nothing valid, so only the raw IMU and the timeouts matter.
struct Rig {
  bool high = false;
  FakeEstimator estimator;
  FakeSensorSource sensors;
  FakeCommandSource commands;
  FakeActuators actuators;
  RecordingLogSink log;
  LqrAttitudeController controller{fc::kCanardLimitRad};
  FlightComputer computer;

  Rig()
      : sensors(kStartUs, kPeriodUs,
                [this](uint64_t) {
                  SensorFrame f;
                  f.imuOk = true;
                  f.accel = Vec3{0.0f, 0.0f, (high ? 6.0f : 1.0f) * fc::kGravity};
                  return f;
                }),
        computer(FlightIo{sensors, commands, actuators, estimator, controller, log}, VehicleConfig::kServoConfig) {
    estimator.setInvalid();
  }

  void cycle() { TEST_ASSERT_TRUE(computer.cycle()); }
  StateId state() const { return computer.state(); }
  const CycleRecord& last() const { return log.records.back(); }

  // Init -> PreFlight, then a few quiet samples.
  void toPreFlight() {
    commands.push(makeCommand(CommandType::EnterPreFlight));
    cycle();
    TEST_ASSERT_EQUAL(StateId::PreFlight, state());
    for (int i = 0; i < 5; ++i) cycle();
  }
};

void test_flight_clock_follows_normal_sample_time() {
  Rig rig;
  for (int i = 0; i < 10; ++i) {
    rig.cycle();
    TEST_ASSERT_EQUAL_UINT64(rig.last().frame.timeUs, rig.last().flightTimeUs);
    TEST_ASSERT_EQUAL_UINT64(rig.computer.flightTimeUs(), rig.last().flightTimeUs);
  }
}

void test_flight_clock_never_goes_back_and_caps_a_forward_jump() {
  Rig rig;
  rig.cycle();
  rig.cycle();
  const uint64_t before = rig.computer.flightTimeUs();

  rig.sensors.setNextTimeUs(0);  // backwards
  rig.cycle();
  TEST_ASSERT_EQUAL_UINT64(before, rig.computer.flightTimeUs());

  rig.sensors.setNextTimeUs(before + 10 * fc::kUsPerSecond);  // far forward
  rig.cycle();
  TEST_ASSERT_EQUAL_UINT64(before + fc::kMaxImuGapUs, rig.computer.flightTimeUs());
  TEST_ASSERT_TRUE((rig.last().safetyActive & SafetyFault::kImuGap) != 0);
}

void test_one_bad_timestamp_cannot_complete_the_launch_debounce() {
  Rig rig;
  rig.toPreFlight();

  // The first launch-level sample...
  rig.high = true;
  rig.cycle();
  const uint64_t firstHighUs = rig.last().flightTimeUs;
  // ...then one sample stamped far in the past, then normal samples again.
  // With raw sample time the run would restart at 0 and the next sample would
  // count as seconds of hold.
  rig.sensors.setNextTimeUs(0);
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::PreFlight, rig.state());
  rig.sensors.setNextTimeUs(rig.sensors.frames()[rig.sensors.frames().size() - 2].timeUs + kPeriodUs);
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::PreFlight, rig.state());

  // Launch is detected only once the hold has really passed on the flight clock.
  while (rig.state() == StateId::PreFlight) {
    TEST_ASSERT_TRUE_MESSAGE(rig.last().flightTimeUs - firstHighUs < fc::kLaunchHoldUs + kPeriodUs,
                             "launch was never detected");
    rig.cycle();
  }
  TEST_ASSERT_EQUAL(StateId::Boost, rig.state());
  TEST_ASSERT_TRUE(rig.last().flightTimeUs - firstHighUs >= fc::kLaunchHoldUs);
  TEST_ASSERT_TRUE(rig.last().flightTimeUs - firstHighUs < fc::kLaunchHoldUs + kPeriodUs);
}

void test_a_forward_timestamp_jump_does_not_fire_a_timeout() {
  Rig rig;
  rig.toPreFlight();
  rig.high = true;
  while (rig.state() != StateId::Boost) rig.cycle();
  const uint64_t boostEntryUs = rig.last().flightTimeUs;
  rig.high = false;
  rig.cycle();

  // One sample stamped 10 s ahead, far beyond kBoostMaxUs, then back to normal.
  const uint64_t normalNextUs = rig.sensors.nextTimeUs();
  rig.sensors.setNextTimeUs(normalNextUs + 10 * fc::kUsPerSecond);
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::Boost, rig.state());
  rig.sensors.setNextTimeUs(normalNextUs + kPeriodUs);
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::Boost, rig.state());

  // Boost still ends on its timeout (the estimator gives no altitude), at
  // kBoostMaxUs of flight-clock time; the jump counted as at most kMaxImuGapUs.
  while (rig.state() == StateId::Boost) rig.cycle();
  TEST_ASSERT_EQUAL(StateId::Control, rig.state());
  const uint64_t inBoostUs = rig.last().flightTimeUs - boostEntryUs;
  TEST_ASSERT_TRUE(inBoostUs >= fc::kBoostMaxUs);
  TEST_ASSERT_TRUE(inBoostUs < fc::kBoostMaxUs + kPeriodUs);
}

void test_a_backward_timestamp_jump_does_not_stretch_a_ground_test() {
  Rig rig;
  rig.toPreFlight();
  rig.commands.push(makeCommand(CommandType::CanardsEnable));
  rig.commands.push(makeTestCommand(-1, 0.05f));
  rig.cycle();
  const uint64_t testStartUs = rig.last().flightTimeUs;
  TEST_ASSERT_EQUAL(CanardMode::Testing, rig.last().canards.mode);

  // A sample stamped far in the past: the hold must not restart or stretch.
  rig.sensors.setNextTimeUs(0);
  rig.cycle();
  rig.sensors.setNextTimeUs(rig.sensors.frames()[rig.sensors.frames().size() - 2].timeUs + kPeriodUs);

  uint64_t lastTestingUs = testStartUs;
  for (uint64_t i = 0; i < samplesFor(fc::kGroundTestHoldUs) + 20; ++i) {
    rig.cycle();
    if (rig.last().canards.mode == CanardMode::Testing) lastTestingUs = rig.last().flightTimeUs;
  }
  TEST_ASSERT_TRUE(lastTestingUs - testStartUs < fc::kGroundTestHoldUs);
  TEST_ASSERT_EQUAL(CanardMode::Neutral, rig.last().canards.mode);
}

void test_enable_arriving_on_the_launch_cycle_is_refused() {
  Rig rig;
  rig.toPreFlight();
  rig.high = true;
  // Run up to the cycle before launch completes, then send the enable.
  for (uint64_t i = 0; i < samplesFor(fc::kLaunchHoldUs); ++i) {
    rig.cycle();
    TEST_ASSERT_EQUAL(StateId::PreFlight, rig.state());
  }
  rig.commands.push(makeCommand(CommandType::CanardsEnable));
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::Boost, rig.state());
  TEST_ASSERT_TRUE(rig.last().transitioned);
  TEST_ASSERT_FALSE(rig.last().canards.enabled);
  TEST_ASSERT_FALSE(rig.last().actuators.servoPower);
  TEST_ASSERT_EQUAL_UINT32(1, rig.last().canards.rejectedCommandCount);
}

void test_enable_one_cycle_before_launch_is_kept() {
  Rig rig;
  rig.toPreFlight();
  rig.high = true;
  for (uint64_t i = 0; i < samplesFor(fc::kLaunchHoldUs) - 1; ++i) rig.cycle();
  rig.commands.push(makeCommand(CommandType::CanardsEnable));
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::PreFlight, rig.state());
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::Boost, rig.state());
  TEST_ASSERT_TRUE(rig.last().canards.enabled);
  TEST_ASSERT_TRUE(rig.last().actuators.servoPower);
  TEST_ASSERT_EQUAL_UINT32(0, rig.last().canards.rejectedCommandCount);
}

void test_an_enable_repeated_on_the_launch_cycle_keeps_an_earlier_enable() {
  Rig rig;
  rig.toPreFlight();
  rig.commands.push(makeCommand(CommandType::CanardsEnable));
  rig.cycle();
  rig.high = true;
  for (uint64_t i = 0; i < samplesFor(fc::kLaunchHoldUs); ++i) rig.cycle();
  rig.commands.push(makeCommand(CommandType::CanardsEnable));  // a repeat, on the launch cycle
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::Boost, rig.state());
  TEST_ASSERT_TRUE(rig.last().canards.enabled);
  TEST_ASSERT_TRUE(rig.last().actuators.servoPower);
}

void test_estimator_is_off_the_pad_from_the_first_launch_level_sample() {
  Rig rig;
  rig.toPreFlight();
  TEST_ASSERT_TRUE(rig.estimator.lastInput().onPad);
  rig.high = true;
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::PreFlight, rig.state());  // the debounce has not completed
  TEST_ASSERT_FALSE(rig.estimator.lastInput().onPad);
  rig.high = false;
  rig.cycle();
  TEST_ASSERT_TRUE(rig.estimator.lastInput().onPad);  // a bump on the pad: back at rest
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_flight_clock_follows_normal_sample_time);
  RUN_TEST(test_flight_clock_never_goes_back_and_caps_a_forward_jump);
  RUN_TEST(test_one_bad_timestamp_cannot_complete_the_launch_debounce);
  RUN_TEST(test_a_forward_timestamp_jump_does_not_fire_a_timeout);
  RUN_TEST(test_a_backward_timestamp_jump_does_not_stretch_a_ground_test);
  RUN_TEST(test_enable_arriving_on_the_launch_cycle_is_refused);
  RUN_TEST(test_enable_one_cycle_before_launch_is_kept);
  RUN_TEST(test_an_enable_repeated_on_the_launch_cycle_keeps_an_earlier_enable);
  RUN_TEST(test_estimator_is_off_the_pad_from_the_first_launch_level_sample);
  return UNITY_END();
}
