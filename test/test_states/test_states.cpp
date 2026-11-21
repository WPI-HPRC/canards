// The seven flight states, LaunchDetector and GroundTest, each on its own
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 6.1, 6.2, 6.4 and step
// 5 of section 10).
//
// Each test drives ONE state the way the StateMachine would (init() on entry,
// then execute() and next() every cycle) with a real Navigation fed by a
// FakeEstimator, and a real Canards subsystem. No StateMachine: next()'s answer
// is checked, not acted on.
//
// Every threshold, hold and timeout comes from FlightConfig, never a copy of
// its value, so these tests keep passing when a placeholder changes.
#include <unity.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>

#include "Actuators.h"
#include "AttitudeControl/LqrAttitudeController.h"
#include "FakeCommandSource.h"
#include "FakeEstimator.h"
#include "FlightConfig.h"
#include "Frames.h"
#include "SensorSource.h"
#include "ServoControl/ServoControl.h"
#include "StateMachine/GroundTest.h"
#include "StateMachine/State.h"
#include "StateMachine/StateId.h"
#include "StateMachine/states/BoostState.h"
#include "StateMachine/states/ChuteState.h"
#include "StateMachine/states/CoastState.h"
#include "StateMachine/states/ControlState.h"
#include "StateMachine/states/InitState.h"
#include "StateMachine/states/LandState.h"
#include "StateMachine/states/LaunchDetector.h"
#include "StateMachine/states/PreFlightState.h"
#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Navigation/Navigation.h"
#include "VehicleConfig.h"

void setUp() {}
void tearDown() {}

namespace {

namespace fc = FlightConfig;

// The sample period of every test in this file. Defined once.
constexpr uint64_t kStepUs = 5000;
// Start late enough that nothing ever needs a negative time.
constexpr uint64_t kT0 = 10 * fc::kUsPerSecond;
constexpr uint64_t kOneDayUs = 24ull * 3600ull * fc::kUsPerSecond;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kHalfPi = 1.57079632679489661923f;

// Rounds a duration up to a whole number of sample periods: a hold of `us`
// is first satisfied this long after the first true sample.
constexpr uint64_t ceilToStep(uint64_t us) { return (us + kStepUs - 1) / kStepUs * kStepUs; }
// How many consecutive true samples a Persistence of `holdUs` needs.
constexpr uint64_t samplesToHold(uint64_t holdUs) { return ceilToStep(holdUs) / kStepUs + 1; }

float justBelow(float x) { return std::nextafter(x, -std::numeric_limits<float>::infinity()); }

// The VehicleConfig servo rows, all marked calibrated, so a test can see a
// canard really move.
ServoConfig makeCalibratedServos() {
  ServoConfig config = VehicleConfig::kServoConfig;
  for (ServoChannel& channel : config.channels) channel.calibrated = true;
  return config;
}
const ServoConfig kCalibratedServos = makeCalibratedServos();

// What the IMU reads, in its own axes, for a body-axis vector. The mount map
// is a rotation, so its inverse is its transpose.
Vec3 sensorAxes(const Vec3& body) {
  Mat3 t;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) t.m[r][c] = VehicleConfig::kImuToBody.m[c][r];
  }
  return t * body;
}

// Body -> NED with the nose `tiltRad` away from straight up, rolled `rollRad`
// about the nose.
Quat noseUpTilted(float tiltRad, float rollRad = 0.0f) {
  return fromAxisAngle(Vec3{0.0f, 1.0f, 0.0f}, kHalfPi - tiltRad) *
         fromAxisAngle(Vec3{1.0f, 0.0f, 0.0f}, rollRad);
}

#define EXPECT_STATE(expected, actual) TEST_ASSERT_EQUAL_STRING(stateName(expected), stateName(actual))
#define EXPECT_REQUEST(expected, actual) \
  TEST_ASSERT_EQUAL_INT(static_cast<int>(expected), static_cast<int>(actual))

// ---------------------------------------------------------------------------
// The rig: one of every state, on a real Navigation and Canards.
// ---------------------------------------------------------------------------

struct Rig {
  FakeEstimator estimator;
  Navigation nav{estimator};
  ServoControl servos{kCalibratedServos};
  LqrAttitudeController controller{fc::kCanardLimitRad};
  Canards canards{controller, servos};

  InitState init{canards};
  PreFlightState preFlight{canards, nav};
  BoostState boost{canards};
  ControlState control{canards};
  CoastState coast{canards};
  ChuteState chute{canards};
  LandState land{canards};

  // --- What the IMU and the estimator report on the next cycle -------------
  bool imuOk = true;
  Vec3 accelBody{fc::kGravity, 0.0f, 0.0f};  // at rest, nose up
  bool navValid = true;
  Quat attitude = noseUpTilted(0.0f);
  Vec3 bodyRate;
  float speed = 0.0f;
  float altitude = 0.0f;

  CommandList commands;  // the next cycle's commands; cleared after each cycle

  StateId current = StateId::Init;  // the state being driven
  uint64_t timeUs = kT0;            // the last sample time
  uint64_t enteredUs = kT0;
  bool sensed = false;
  ActuatorCommand out;

  State& state(StateId id) {
    switch (id) {
      case StateId::Init: return init;
      case StateId::PreFlight: return preFlight;
      case StateId::Boost: return boost;
      case StateId::Control: return control;
      case StateId::Coast: return coast;
      case StateId::Chute: return chute;
      case StateId::Land: return land;
    }
    return land;
  }

  // Enters `id` at the last sample time, as the StateMachine does on the
  // cycle of a transition.
  void enter(StateId id) {
    current = id;
    enteredUs = timeUs;
    Cycle cycle{timeUs, 0.0f, 0, nav, commands};
    state(id).init(cycle);
  }

  // One cycle at sample time `t`, in the FlightComputer's order: sensors and
  // navigation, then `body` (the state), then the canards. Returns `body`'s
  // answer.
  StateId runCycle(uint64_t t, const std::function<StateId(const Cycle&)>& body) {
    const float dt = (sensed && t > timeUs) ? static_cast<float>(t - timeUs) * 1e-6f : 0.0f;
    SensorFrame frame;
    frame.timeUs = t;
    frame.imuOk = imuOk;
    frame.accel = sensorAxes(accelBody);
    if (navValid) {
      estimator.setValid(attitude, bodyRate, Vec3{0.0f, 0.0f, -speed}, altitude);
    } else {
      estimator.setInvalid();
    }
    nav.update(frame, dt, isGroundState(current));
    timeUs = t;
    sensed = true;

    canards.beginCycle();
    Cycle cycle{t, dt, t >= enteredUs ? t - enteredUs : 0, nav, commands};
    const StateId wanted = body(cycle);
    ActuatorFeedback feedback;
    canards.periodic(cycle, current, false, feedback, out);
    commands.clear();
    return wanted;
  }

  // One cycle of the current state at sample time `t`; returns next().
  StateId stepAt(uint64_t t) {
    return runCycle(t, [this](const Cycle& cycle) {
      State& s = state(current);
      s.execute(cycle);
      return s.next(cycle);
    });
  }
  StateId step() { return stepAt(timeUs + kStepUs); }

  // Enables the canards on the ground and lets them see PreFlight, so a
  // flight state entered afterwards is not locked out.
  void enableFromPreFlight() {
    current = StateId::PreFlight;
    canards.handleCommand(makeCommand(CommandType::CanardsEnable), StateId::PreFlight);
    runCycle(timeUs + kStepUs, [this](const Cycle&) { return current; });
  }

  void push(const Command& command) { commands.push(command); }

  const CanardsStatus& status() const { return canards.status(); }
};

// ---------------------------------------------------------------------------
// Generic checks
// ---------------------------------------------------------------------------

using Setter = void (*)(Rig&, bool);
using Prepare = void (*)(Rig&);

// A debounced condition of `from` that leads to `to`:
//   - one sample short of the hold does not transition, the next one does,
//     at the first true sample + the hold (rounded up to a sample);
//   - an interruption restarts the hold.
// `prepare` runs once right after entry (e.g. to arm Control's speed exit).
void expectDebounced(StateId from, StateId to, uint64_t holdUs, Setter set, Prepare prepare = nullptr) {
  const uint64_t n = samplesToHold(holdUs);

  {  // True path, and one sample short.
    Rig rig;
    rig.enter(from);
    if (prepare) prepare(rig);
    set(rig, false);
    EXPECT_STATE(from, rig.step());
    set(rig, true);
    const uint64_t firstTrueUs = rig.timeUs + kStepUs;
    for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(from, rig.step());
    EXPECT_STATE(to, rig.step());
    TEST_ASSERT_EQUAL_UINT64(firstTrueUs + ceilToStep(holdUs), rig.timeUs);
  }

  {  // An interruption restarts the hold.
    Rig rig;
    rig.enter(from);
    if (prepare) prepare(rig);
    set(rig, true);
    for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(from, rig.step());
    set(rig, false);
    EXPECT_STATE(from, rig.step());
    set(rig, true);
    for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(from, rig.step());
    EXPECT_STATE(to, rig.step());
  }
}

// A timeout of `limitUs` in `from`: stays at limit - 1 us, leaves at exactly
// the limit. `setup` makes every other exit condition false (and, for the
// navigation states, makes the estimate INVALID while its values would
// otherwise satisfy the condition: an invalid value must never count).
void expectTimeout(StateId from, StateId to, uint64_t limitUs, Prepare setup) {
  Rig rig;
  rig.enter(from);
  setup(rig);
  for (int i = 0; i < 10; ++i) EXPECT_STATE(from, rig.step());
  EXPECT_STATE(from, rig.stepAt(rig.enteredUs + limitUs - 1));
  EXPECT_STATE(to, rig.stepAt(rig.enteredUs + limitUs));
}

void setLaunchAccel(Rig& rig, bool on) {
  rig.accelBody = Vec3{on ? fc::kLaunchAccel : fc::kGravity, 0.0f, 0.0f};
}

// ===========================================================================
// Init
// ===========================================================================

void test_init_goes_to_preflight_on_the_command_cycle() {
  Rig rig;
  rig.enter(StateId::Init);
  EXPECT_STATE(StateId::Init, rig.step());
  rig.push(makeCommand(CommandType::EnterPreFlight));
  EXPECT_STATE(StateId::PreFlight, rig.step());
}

void test_init_ignores_every_other_command() {
  Rig rig;
  rig.enter(StateId::Init);
  const CommandType others[] = {CommandType::StandDown, CommandType::CanardsEnable,
                                CommandType::CanardsDisable, CommandType::CanardTest,
                                CommandType::CanardTestStop};
  for (CommandType type : others) {
    rig.push(makeCommand(type));
    EXPECT_STATE(StateId::Init, rig.step());
  }
}

void test_init_has_no_timeout() {
  Rig rig;
  rig.enter(StateId::Init);
  EXPECT_STATE(StateId::Init, rig.step());
  EXPECT_STATE(StateId::Init, rig.stepAt(rig.enteredUs + kOneDayUs));
  EXPECT_STATE(StateId::Init, rig.step());
}

void test_init_requests_neutral_without_a_ground_test() {
  Rig rig;
  rig.enter(StateId::Init);
  rig.step();
  EXPECT_REQUEST(CanardRequest::Neutral, rig.status().request);
}

void test_launch_from_init_is_debounced() {
  expectDebounced(StateId::Init, StateId::Boost, fc::kLaunchHoldUs, setLaunchAccel);
}

void test_launch_wins_over_enter_preflight_in_the_same_cycle() {
  Rig rig;
  rig.enter(StateId::Init);
  setLaunchAccel(rig, true);
  const uint64_t n = samplesToHold(fc::kLaunchHoldUs);
  for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(StateId::Init, rig.step());
  rig.push(makeCommand(CommandType::EnterPreFlight));
  EXPECT_STATE(StateId::Boost, rig.step());
}

void test_enter_preflight_during_an_unfinished_launch_hold_goes_to_preflight() {
  if (samplesToHold(fc::kLaunchHoldUs) < 3) return;  // no unfinished hold to have
  Rig rig;
  rig.enter(StateId::Init);
  setLaunchAccel(rig, true);
  EXPECT_STATE(StateId::Init, rig.step());
  rig.push(makeCommand(CommandType::EnterPreFlight));
  EXPECT_STATE(StateId::PreFlight, rig.step());
}

// ===========================================================================
// LaunchDetector (through Init and PreFlight)
// ===========================================================================

void test_launch_from_preflight_is_debounced() {
  expectDebounced(StateId::PreFlight, StateId::Boost, fc::kLaunchHoldUs, setLaunchAccel);
}

void test_launch_threshold_is_inclusive_and_any_direction_counts() {
  const uint64_t n = samplesToHold(fc::kLaunchHoldUs);
  {  // Just below the threshold never launches.
    Rig rig;
    rig.enter(StateId::PreFlight);
    rig.accelBody = Vec3{justBelow(fc::kLaunchAccel), 0.0f, 0.0f};
    for (uint64_t i = 0; i < 4 * n; ++i) EXPECT_STATE(StateId::PreFlight, rig.step());
  }
  {  // Exactly the threshold, along a side axis: it is the magnitude that counts.
    Rig rig;
    rig.enter(StateId::PreFlight);
    rig.accelBody = Vec3{0.0f, -fc::kLaunchAccel, 0.0f};
    for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(StateId::PreFlight, rig.step());
    EXPECT_STATE(StateId::Boost, rig.step());
  }
}

void test_launch_ignores_the_estimator() {
  const uint64_t n = samplesToHold(fc::kLaunchHoldUs);
  {  // No estimate at all: the IMU alone detects the launch.
    Rig rig;
    rig.navValid = false;
    rig.enter(StateId::PreFlight);
    setLaunchAccel(rig, true);
    for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(StateId::PreFlight, rig.step());
    EXPECT_STATE(StateId::Boost, rig.step());
  }
  {  // An estimate that says "flying fast and high" does not launch a
     // rocket whose IMU reads 1 g.
    Rig rig;
    rig.speed = 10.0f * fc::kControlEndSpeed;
    rig.altitude = 10.0f * fc::kControlStartAltitudeM;
    rig.enter(StateId::Init);
    for (uint64_t i = 0; i < 4 * n; ++i) EXPECT_STATE(StateId::Init, rig.step());
    rig.enter(StateId::PreFlight);
    for (uint64_t i = 0; i < 4 * n; ++i) EXPECT_STATE(StateId::PreFlight, rig.step());
  }
}

void test_launch_fails_on_an_invalid_imu() {
  const uint64_t n = samplesToHold(fc::kLaunchHoldUs);
  {  // A failed read with launch-sized numbers in it.
    Rig rig;
    rig.enter(StateId::Init);
    setLaunchAccel(rig, true);
    rig.imuOk = false;
    for (uint64_t i = 0; i < 4 * n; ++i) EXPECT_STATE(StateId::Init, rig.step());
  }
  {  // A non-finite component.
    Rig rig;
    rig.enter(StateId::PreFlight);
    rig.accelBody = Vec3{fc::kLaunchAccel, kNaN, 0.0f};
    for (uint64_t i = 0; i < 4 * n; ++i) EXPECT_STATE(StateId::PreFlight, rig.step());
  }
  if (n >= 2) {  // One bad sample in the middle of the hold restarts it.
    Rig rig;
    rig.enter(StateId::Init);
    setLaunchAccel(rig, true);
    for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(StateId::Init, rig.step());
    rig.imuOk = false;
    EXPECT_STATE(StateId::Init, rig.step());
    rig.imuOk = true;
    for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(StateId::Init, rig.step());
    EXPECT_STATE(StateId::Boost, rig.step());
  }
}

void test_launch_hold_starts_over_on_reentry() {
  // A launch hold half done in PreFlight does not carry over a stand-down and
  // a new entry.
  const uint64_t n = samplesToHold(fc::kLaunchHoldUs);
  if (n < 3) return;
  Rig rig;
  rig.enter(StateId::PreFlight);
  setLaunchAccel(rig, true);
  for (uint64_t i = 0; i + 2 < n; ++i) EXPECT_STATE(StateId::PreFlight, rig.step());
  rig.enter(StateId::PreFlight);  // re-entry resets the detector
  EXPECT_STATE(StateId::PreFlight, rig.step());
  EXPECT_STATE(StateId::PreFlight, rig.step());
}

// ===========================================================================
// PreFlight
// ===========================================================================

void test_preflight_reinitialises_the_estimator_on_entry() {
  Rig rig;
  rig.enter(StateId::Init);
  rig.step();
  rig.step();
  TEST_ASSERT_EQUAL_UINT32(0, rig.estimator.reinitializeCount());

  rig.enter(StateId::PreFlight);
  rig.step();
  TEST_ASSERT_EQUAL_UINT32(1, rig.estimator.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(1, rig.nav.reinitializeCount());
  // It ran before this cycle's update, not after it.
  const size_t calls = rig.estimator.loggedCallCount();
  TEST_ASSERT_TRUE(rig.estimator.callAt(calls - 2) == FakeEstimator::Call::Reinitialize);
  TEST_ASSERT_TRUE(rig.estimator.callAt(calls - 1) == FakeEstimator::Call::Update);

  for (int i = 0; i < 10; ++i) rig.step();
  TEST_ASSERT_EQUAL_UINT32(1, rig.estimator.reinitializeCount());
}

void test_preflight_reinitialises_again_on_a_repeated_enter_preflight() {
  Rig rig;
  rig.enter(StateId::PreFlight);
  rig.step();
  TEST_ASSERT_EQUAL_UINT32(1, rig.estimator.reinitializeCount());

  rig.push(makeCommand(CommandType::EnterPreFlight));
  EXPECT_STATE(StateId::PreFlight, rig.step());
  rig.step();
  TEST_ASSERT_EQUAL_UINT32(2, rig.estimator.reinitializeCount());

  // Two in one cycle: one re-initialisation.
  rig.push(makeCommand(CommandType::EnterPreFlight));
  rig.push(makeCommand(CommandType::EnterPreFlight));
  rig.step();
  rig.step();
  TEST_ASSERT_EQUAL_UINT32(3, rig.estimator.reinitializeCount());

  // Any other command: none.
  const CommandType others[] = {CommandType::CanardsEnable, CommandType::CanardsDisable,
                                CommandType::CanardTest, CommandType::CanardTestStop};
  for (CommandType type : others) {
    rig.push(makeCommand(type));
    EXPECT_STATE(StateId::PreFlight, rig.step());
  }
  rig.step();
  TEST_ASSERT_EQUAL_UINT32(3, rig.estimator.reinitializeCount());
}

void test_preflight_stand_down_goes_to_init_on_the_command_cycle() {
  Rig rig;
  rig.enter(StateId::PreFlight);
  EXPECT_STATE(StateId::PreFlight, rig.step());
  rig.push(makeCommand(CommandType::StandDown));
  EXPECT_STATE(StateId::Init, rig.step());
}

void test_launch_wins_over_stand_down_in_the_same_cycle() {
  Rig rig;
  rig.enter(StateId::PreFlight);
  setLaunchAccel(rig, true);
  const uint64_t n = samplesToHold(fc::kLaunchHoldUs);
  for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(StateId::PreFlight, rig.step());
  rig.push(makeCommand(CommandType::StandDown));
  EXPECT_STATE(StateId::Boost, rig.step());
}

void test_preflight_has_no_timeout() {
  Rig rig;
  rig.enter(StateId::PreFlight);
  EXPECT_STATE(StateId::PreFlight, rig.step());
  EXPECT_STATE(StateId::PreFlight, rig.stepAt(rig.enteredUs + kOneDayUs));
  EXPECT_STATE(StateId::PreFlight, rig.step());
}

// ===========================================================================
// Boost
// ===========================================================================

void setHigh(Rig& rig, bool on) {
  rig.altitude = on ? fc::kControlStartAltitudeM : justBelow(fc::kControlStartAltitudeM);
}

void test_boost_altitude_exit_is_debounced() {
  expectDebounced(StateId::Boost, StateId::Control, fc::kControlStartHoldUs, setHigh);
}

void test_boost_timeout_at_exactly_its_limit() {
  expectTimeout(StateId::Boost, StateId::Control, fc::kBoostMaxUs, [](Rig& rig) {
    rig.navValid = false;
    rig.altitude = 10.0f * fc::kControlStartAltitudeM;
  });
}

// ===========================================================================
// Control
// ===========================================================================

void test_control_does_not_capture_a_reference_before_a_valid_attitude() {
  Rig rig;
  rig.navValid = false;
  rig.enter(StateId::Control);
  for (int i = 0; i < 5; ++i) {
    EXPECT_STATE(StateId::Control, rig.step());
    TEST_ASSERT_FALSE(rig.control.haveReference());
    EXPECT_REQUEST(CanardRequest::Neutral, rig.status().request);
  }
}

void test_control_captures_the_vertical_reference_of_that_cycles_attitude_once() {
  Rig rig;
  rig.navValid = false;
  rig.enter(StateId::Control);
  rig.step();
  TEST_ASSERT_FALSE(rig.control.haveReference());

  const Quat first = noseUpTilted(0.1f, 0.4f);
  rig.navValid = true;
  rig.attitude = first;
  rig.step();
  TEST_ASSERT_TRUE(rig.control.haveReference());
  const Quat expected = verticalReference(first);
  TEST_ASSERT_EQUAL_FLOAT(expected.w, rig.control.reference().w);
  TEST_ASSERT_EQUAL_FLOAT(expected.x, rig.control.reference().x);
  TEST_ASSERT_EQUAL_FLOAT(expected.y, rig.control.reference().y);
  TEST_ASSERT_EQUAL_FLOAT(expected.z, rig.control.reference().z);
  EXPECT_REQUEST(CanardRequest::AttitudeHold, rig.status().request);

  // A different attitude later, valid or not: never recaptured, still held.
  rig.attitude = noseUpTilted(0.2f, -1.0f);
  for (int i = 0; i < 5; ++i) {
    rig.step();
    TEST_ASSERT_EQUAL_FLOAT(expected.w, rig.control.reference().w);
    TEST_ASSERT_EQUAL_FLOAT(expected.x, rig.control.reference().x);
    TEST_ASSERT_EQUAL_FLOAT(expected.y, rig.control.reference().y);
    TEST_ASSERT_EQUAL_FLOAT(expected.z, rig.control.reference().z);
    EXPECT_REQUEST(CanardRequest::AttitudeHold, rig.status().request);
  }
  rig.navValid = false;
  rig.step();
  TEST_ASSERT_EQUAL_FLOAT(expected.x, rig.control.reference().x);
  EXPECT_REQUEST(CanardRequest::AttitudeHold, rig.status().request);
}

void test_control_entry_forgets_the_old_reference() {
  Rig rig;
  rig.enter(StateId::Control);
  rig.step();
  TEST_ASSERT_TRUE(rig.control.haveReference());
  rig.navValid = false;
  rig.enter(StateId::Control);
  TEST_ASSERT_FALSE(rig.control.haveReference());
  TEST_ASSERT_FALSE(rig.control.speedExitArmed());
}

void test_control_speed_exit_is_not_armed_when_entered_slow_and_accelerating() {
  Rig rig;
  const uint64_t samples = 4 * samplesToHold(fc::kControlEndHoldUs);
  TEST_ASSERT_TRUE_MESSAGE(samples * kStepUs < fc::kControlMaxUs, "test assumes the climb fits in the timeout");
  rig.speed = 0.5f * fc::kControlEndSpeed;
  rig.enter(StateId::Control);
  const float below = justBelow(fc::kControlEndSpeed);
  for (uint64_t i = 0; i < samples; ++i) {
    // From half the end speed up to just under it: below the end speed all
    // the time, but never "falling below" it.
    const float f = static_cast<float>(i) / static_cast<float>(samples - 1);
    rig.speed = 0.5f * fc::kControlEndSpeed + f * (below - 0.5f * fc::kControlEndSpeed);
    EXPECT_STATE(StateId::Control, rig.step());
    TEST_ASSERT_FALSE(rig.control.speedExitArmed());
  }
  // Up to the end speed: armed, still in Control.
  rig.speed = fc::kControlEndSpeed;
  EXPECT_STATE(StateId::Control, rig.step());
  TEST_ASSERT_TRUE(rig.control.speedExitArmed());
  // Then below it for the hold: Coast.
  rig.speed = 0.5f * fc::kControlEndSpeed;
  const uint64_t n = samplesToHold(fc::kControlEndHoldUs);
  for (uint64_t i = 0; i + 1 < n; ++i) EXPECT_STATE(StateId::Control, rig.step());
  EXPECT_STATE(StateId::Coast, rig.step());
}

void armSpeedExit(Rig& rig) {
  rig.speed = 2.0f * fc::kControlEndSpeed;
  EXPECT_STATE(StateId::Control, rig.step());
  TEST_ASSERT_TRUE(rig.control.speedExitArmed());
}

void setSlowForControl(Rig& rig, bool on) {
  // Exactly the end speed is not "below" it.
  rig.speed = on ? justBelow(fc::kControlEndSpeed) : fc::kControlEndSpeed;
}

void test_control_speed_exit_is_debounced_once_armed() {
  expectDebounced(StateId::Control, StateId::Coast, fc::kControlEndHoldUs, setSlowForControl, armSpeedExit);
}

void test_control_speed_exit_ignores_an_invalid_speed() {
  Rig rig;
  rig.enter(StateId::Control);
  armSpeedExit(rig);
  rig.navValid = false;
  rig.speed = 0.0f;
  for (uint64_t i = 0; i < 4 * samplesToHold(fc::kControlEndHoldUs); ++i) {
    EXPECT_STATE(StateId::Control, rig.step());
  }
}

void test_control_leaves_on_the_cutoff_latch_from_tilt() {
  Rig rig;
  rig.enableFromPreFlight();
  rig.speed = 2.0f * fc::kControlEndSpeed;
  rig.enter(StateId::Control);
  EXPECT_STATE(StateId::Control, rig.step());
  TEST_ASSERT_FALSE(rig.status().cutoffLatched);
  TEST_ASSERT_TRUE(rig.status().mode == CanardMode::Controlling);

  // The latch trips in this cycle's periodic(), after next() has run.
  rig.attitude = noseUpTilted(1.5f * fc::kMaxTiltRad);
  EXPECT_STATE(StateId::Control, rig.step());
  TEST_ASSERT_TRUE(rig.status().cutoffLatched);
  // So Control leaves on the next cycle, with no hold, even with the tilt gone.
  rig.attitude = noseUpTilted(0.0f);
  EXPECT_STATE(StateId::Coast, rig.step());
}

void test_control_leaves_on_the_cutoff_latch_from_rate() {
  Rig rig;
  rig.enableFromPreFlight();
  rig.speed = 2.0f * fc::kControlEndSpeed;
  rig.enter(StateId::Control);
  EXPECT_STATE(StateId::Control, rig.step());
  rig.bodyRate = Vec3{0.0f, 0.0f, 1.5f * fc::kMaxPitchYawRate};
  EXPECT_STATE(StateId::Control, rig.step());
  TEST_ASSERT_TRUE(rig.status().cutoffLatched);
  rig.bodyRate = Vec3{};
  EXPECT_STATE(StateId::Coast, rig.step());
}

void test_control_timeout_at_exactly_its_limit() {
  // Armed (speed at the end speed) but never below it.
  expectTimeout(StateId::Control, StateId::Coast, fc::kControlMaxUs,
                [](Rig& rig) { rig.speed = fc::kControlEndSpeed; });
  // No estimate at all.
  expectTimeout(StateId::Control, StateId::Coast, fc::kControlMaxUs, [](Rig& rig) {
    rig.navValid = false;
    rig.speed = 0.0f;
  });
}

// ===========================================================================
// Coast
// ===========================================================================

void setSlowForCoast(Rig& rig, bool on) {
  rig.speed = on ? justBelow(fc::kChuteSpeed) : fc::kChuteSpeed;
}

void test_coast_speed_exit_is_debounced() {
  expectDebounced(StateId::Coast, StateId::Chute, fc::kChuteHoldUs, setSlowForCoast);
}

void test_coast_timeout_at_exactly_its_limit() {
  expectTimeout(StateId::Coast, StateId::Chute, fc::kCoastMaxUs, [](Rig& rig) {
    rig.navValid = false;
    rig.speed = 0.0f;
  });
}

// ===========================================================================
// Chute
// ===========================================================================

void setLow(Rig& rig, bool on) {
  rig.altitude = on ? justBelow(fc::kLandAltitudeM) : fc::kLandAltitudeM;
}

void test_chute_altitude_exit_is_debounced() {
  expectDebounced(StateId::Chute, StateId::Land, fc::kLandHoldUs, setLow);
}

void test_chute_timeout_at_exactly_its_limit() {
  expectTimeout(StateId::Chute, StateId::Land, fc::kChuteMaxUs, [](Rig& rig) {
    rig.navValid = false;
    rig.altitude = 0.0f;
  });
}

// ===========================================================================
// Land, and what every flight state does with ground commands
// ===========================================================================

void test_land_never_leaves() {
  Rig rig;
  rig.enter(StateId::Land);
  const CommandType all[] = {CommandType::EnterPreFlight, CommandType::StandDown,
                             CommandType::CanardsEnable,  CommandType::CanardsDisable,
                             CommandType::CanardTest,     CommandType::CanardTestStop};
  for (CommandType type : all) {
    rig.push(makeCommand(type));
    EXPECT_STATE(StateId::Land, rig.step());
  }
  // Every condition any other state leaves on, and a launch.
  rig.altitude = 0.0f;
  rig.speed = 0.0f;
  setLaunchAccel(rig, true);
  for (uint64_t i = 0; i < samplesToHold(fc::kLandHoldUs) + samplesToHold(fc::kLaunchHoldUs); ++i) {
    EXPECT_STATE(StateId::Land, rig.step());
  }
  rig.navValid = false;
  EXPECT_STATE(StateId::Land, rig.stepAt(rig.enteredUs + kOneDayUs));
  EXPECT_STATE(StateId::Land, rig.step());
}

void test_flight_states_request_neutral_and_ignore_ground_commands() {
  const StateId neutralStates[] = {StateId::Boost, StateId::Coast, StateId::Chute, StateId::Land};
  const Command all[] = {makeCommand(CommandType::EnterPreFlight), makeCommand(CommandType::StandDown),
                         makeCommand(CommandType::CanardsEnable), makeCommand(CommandType::CanardsDisable),
                         makeTestCommand(-1, 0.05f), makeCommand(CommandType::CanardTestStop)};
  for (StateId id : neutralStates) {
    Rig rig;
    // Values that satisfy no exit condition of these states.
    rig.altitude = id == StateId::Boost ? 0.0f : fc::kLandAltitudeM;
    rig.speed = fc::kChuteSpeed;
    rig.enter(id);
    for (const Command& command : all) {
      rig.push(command);
      EXPECT_STATE(id, rig.step());
      EXPECT_REQUEST(CanardRequest::Neutral, rig.status().request);
    }
  }
  {  // Control keeps holding attitude whatever the ground says.
    Rig rig;
    rig.speed = fc::kControlEndSpeed;
    rig.enter(StateId::Control);
    for (const Command& command : all) {
      rig.push(command);
      EXPECT_STATE(StateId::Control, rig.step());
      EXPECT_REQUEST(CanardRequest::AttitudeHold, rig.status().request);
    }
  }
}

// ===========================================================================
// GroundTest
// ===========================================================================

// A rig in Init with the canards enabled, driving one GroundTest directly.
struct GroundRig {
  Rig rig;
  GroundTest test{rig.canards};
  bool lastActive = false;

  GroundRig() {
    rig.canards.handleCommand(makeCommand(CommandType::CanardsEnable), StateId::Init);
    rig.enter(StateId::Init);
  }

  // One cycle at `t` in which only the ground test runs. Returns update().
  bool stepAt(uint64_t t) {
    rig.runCycle(t, [this](const Cycle& cycle) {
      lastActive = test.update(cycle);
      return rig.current;
    });
    TEST_ASSERT_EQUAL(lastActive, test.active());
    return lastActive;
  }
  bool step() { return stepAt(rig.timeUs + kStepUs); }
};

constexpr float kAngle = 0.5f * fc::kGroundTestLimitRad;

void expectAngles(const GroundRig& g, float a0, float a1, float a2, float a3) {
  TEST_ASSERT_EQUAL_FLOAT(a0, g.test.angles()[0]);
  TEST_ASSERT_EQUAL_FLOAT(a1, g.test.angles()[1]);
  TEST_ASSERT_EQUAL_FLOAT(a2, g.test.angles()[2]);
  TEST_ASSERT_EQUAL_FLOAT(a3, g.test.angles()[3]);
}

void test_ground_test_moves_all_canards() {
  GroundRig g;
  TEST_ASSERT_FALSE(g.step());
  g.rig.push(makeTestCommand(-1, kAngle));
  TEST_ASSERT_TRUE(g.step());
  expectAngles(g, kAngle, kAngle, kAngle, kAngle);
  EXPECT_REQUEST(CanardRequest::TestAngles, g.rig.status().request);
  TEST_ASSERT_TRUE(g.rig.status().mode == CanardMode::Testing);
  for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_EQUAL_FLOAT(kAngle, g.rig.status().commandRad[i]);
}

void test_ground_test_moves_one_canard() {
  GroundRig g;
  g.rig.push(makeTestCommand(2, kAngle));
  TEST_ASSERT_TRUE(g.step());
  expectAngles(g, 0.0f, 0.0f, kAngle, 0.0f);
  // A second canard joins the first; the first keeps its angle.
  g.rig.push(makeTestCommand(0, -kAngle));
  TEST_ASSERT_TRUE(g.step());
  expectAngles(g, -kAngle, 0.0f, kAngle, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(-kAngle, g.rig.status().commandRad[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g.rig.status().commandRad[1]);
  TEST_ASSERT_EQUAL_FLOAT(kAngle, g.rig.status().commandRad[2]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g.rig.status().commandRad[3]);
}

void test_ground_test_ignores_a_bad_canard_index() {
  GroundRig g;
  const int8_t bad[] = {kNumCanards, -2, 127, -128};
  for (int8_t index : bad) {
    g.rig.push(makeTestCommand(index, kAngle));
    TEST_ASSERT_FALSE(g.step());
    expectAngles(g, 0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_REQUEST(CanardRequest::None, g.rig.status().request);
  }
  // A bad index during a test changes nothing and does not extend the hold.
  g.rig.push(makeTestCommand(1, kAngle));
  TEST_ASSERT_TRUE(g.step());
  const uint64_t startedUs = g.rig.timeUs;
  g.rig.push(makeTestCommand(kNumCanards, -kAngle));
  TEST_ASSERT_TRUE(g.stepAt(startedUs + fc::kGroundTestHoldUs - 1));
  expectAngles(g, 0.0f, kAngle, 0.0f, 0.0f);
  TEST_ASSERT_FALSE(g.stepAt(startedUs + fc::kGroundTestHoldUs));
}

void test_ground_test_ignores_a_non_finite_angle() {
  GroundRig g;
  const float bad[] = {kNaN, std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()};
  for (float angle : bad) {
    g.rig.push(makeTestCommand(-1, angle));
    TEST_ASSERT_FALSE(g.step());
    expectAngles(g, 0.0f, 0.0f, 0.0f, 0.0f);
  }
  // During a test: the angles and the hold are unchanged.
  g.rig.push(makeTestCommand(3, kAngle));
  TEST_ASSERT_TRUE(g.step());
  const uint64_t startedUs = g.rig.timeUs;
  g.rig.push(makeTestCommand(3, kNaN));
  TEST_ASSERT_TRUE(g.stepAt(startedUs + fc::kGroundTestHoldUs - 1));
  expectAngles(g, 0.0f, 0.0f, 0.0f, kAngle);
  TEST_ASSERT_FALSE(g.stepAt(startedUs + fc::kGroundTestHoldUs));
}

void test_ground_test_expires_after_its_hold_in_sample_time() {
  GroundRig g;
  g.rig.push(makeTestCommand(-1, kAngle));
  TEST_ASSERT_TRUE(g.step());
  const uint64_t startedUs = g.rig.timeUs;
  TEST_ASSERT_TRUE(g.stepAt(startedUs + fc::kGroundTestHoldUs - 1));
  TEST_ASSERT_TRUE(g.rig.status().mode == CanardMode::Testing);
  TEST_ASSERT_FALSE(g.stepAt(startedUs + fc::kGroundTestHoldUs));
  EXPECT_REQUEST(CanardRequest::None, g.rig.status().request);
  TEST_ASSERT_TRUE(g.rig.status().mode == CanardMode::Neutral);
  expectAngles(g, 0.0f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FALSE(g.step());
}

void test_ground_test_stop() {
  GroundRig g;
  g.rig.push(makeTestCommand(-1, kAngle));
  TEST_ASSERT_TRUE(g.step());
  g.rig.push(makeCommand(CommandType::CanardTestStop));
  TEST_ASSERT_FALSE(g.step());
  EXPECT_REQUEST(CanardRequest::None, g.rig.status().request);
  expectAngles(g, 0.0f, 0.0f, 0.0f, 0.0f);
  // A stop with nothing running is harmless.
  g.rig.push(makeCommand(CommandType::CanardTestStop));
  TEST_ASSERT_FALSE(g.step());
}

void test_ground_test_restart_extends_the_hold() {
  GroundRig g;
  g.rig.push(makeTestCommand(-1, kAngle));
  TEST_ASSERT_TRUE(g.step());
  const uint64_t firstUs = g.rig.timeUs;
  const uint64_t againUs = firstUs + fc::kGroundTestHoldUs / 2;
  g.rig.push(makeTestCommand(-1, -kAngle));
  TEST_ASSERT_TRUE(g.stepAt(againUs));
  expectAngles(g, -kAngle, -kAngle, -kAngle, -kAngle);
  TEST_ASSERT_TRUE(g.stepAt(firstUs + fc::kGroundTestHoldUs));
  TEST_ASSERT_TRUE(g.stepAt(againUs + fc::kGroundTestHoldUs - 1));
  TEST_ASSERT_FALSE(g.stepAt(againUs + fc::kGroundTestHoldUs));
}

void test_ground_test_applies_commands_in_arrival_order() {
  GroundRig g;
  g.rig.push(makeTestCommand(-1, kAngle));
  g.rig.push(makeCommand(CommandType::CanardTestStop));
  TEST_ASSERT_FALSE(g.step());

  g.rig.push(makeCommand(CommandType::CanardTestStop));
  g.rig.push(makeTestCommand(0, kAngle));
  TEST_ASSERT_TRUE(g.step());
  expectAngles(g, kAngle, 0.0f, 0.0f, 0.0f);
}

void test_ground_test_requests_its_angles_every_cycle_while_active() {
  GroundRig g;
  g.rig.push(makeTestCommand(-1, kAngle));
  TEST_ASSERT_TRUE(g.step());
  const uint64_t startedUs = g.rig.timeUs;
  // No more commands: the request is made again on every cycle of the hold.
  while (g.rig.timeUs + kStepUs < startedUs + fc::kGroundTestHoldUs) {
    TEST_ASSERT_TRUE(g.step());
    EXPECT_REQUEST(CanardRequest::TestAngles, g.rig.status().request);
    TEST_ASSERT_TRUE(g.rig.status().mode == CanardMode::Testing);
  }
}

void test_init_and_preflight_run_the_ground_test_without_a_conflict() {
  const StateId ground[] = {StateId::Init, StateId::PreFlight};
  for (StateId id : ground) {
    Rig rig;
    rig.canards.handleCommand(makeCommand(CommandType::CanardsEnable), id);
    rig.enter(id);
    rig.push(makeTestCommand(-1, kAngle));
    EXPECT_STATE(id, rig.step());
    for (int i = 0; i < 5; ++i) {
      EXPECT_REQUEST(CanardRequest::TestAngles, rig.status().request);
      TEST_ASSERT_TRUE(rig.status().mode == CanardMode::Testing);
      TEST_ASSERT_EQUAL_UINT32(0, rig.status().deniedCount);
      EXPECT_STATE(id, rig.step());
    }
    rig.push(makeCommand(CommandType::CanardTestStop));
    rig.step();
    EXPECT_REQUEST(CanardRequest::Neutral, rig.status().request);
  }
}

void test_ground_test_is_cleared_when_init_or_preflight_is_left() {
  const StateId ground[] = {StateId::Init, StateId::PreFlight};
  for (StateId id : ground) {
    Rig rig;
    rig.enter(id);
    rig.push(makeTestCommand(-1, kAngle));
    rig.step();
    EXPECT_REQUEST(CanardRequest::TestAngles, rig.status().request);

    // end(), then (only to observe it) one more execute() well inside the
    // old hold: the test is gone.
    Cycle cycle{rig.timeUs, 0.0f, 0, rig.nav, rig.commands};
    rig.state(id).end(cycle);
    rig.runCycle(rig.timeUs + kStepUs, [&rig, id](const Cycle& c) {
      rig.state(id).execute(c);
      return id;
    });
    EXPECT_REQUEST(CanardRequest::Neutral, rig.status().request);
  }
  {  // The real path: Init (testing) -> PreFlight -> stand down -> Init.
    Rig rig;
    rig.enter(StateId::Init);
    rig.push(makeTestCommand(-1, kAngle));
    rig.step();
    rig.push(makeCommand(CommandType::EnterPreFlight));
    EXPECT_STATE(StateId::PreFlight, rig.step());
    Cycle cycle{rig.timeUs, 0.0f, 0, rig.nav, rig.commands};
    rig.init.end(cycle);
    rig.enter(StateId::PreFlight);
    rig.step();
    EXPECT_REQUEST(CanardRequest::Neutral, rig.status().request);
    rig.push(makeTestCommand(-1, kAngle));
    rig.step();
    rig.push(makeCommand(CommandType::StandDown));
    EXPECT_STATE(StateId::Init, rig.step());
    rig.preFlight.end(cycle);
    rig.enter(StateId::Init);
    rig.step();
    EXPECT_REQUEST(CanardRequest::Neutral, rig.status().request);
  }
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_init_goes_to_preflight_on_the_command_cycle);
  RUN_TEST(test_init_ignores_every_other_command);
  RUN_TEST(test_init_has_no_timeout);
  RUN_TEST(test_init_requests_neutral_without_a_ground_test);
  RUN_TEST(test_launch_from_init_is_debounced);
  RUN_TEST(test_launch_wins_over_enter_preflight_in_the_same_cycle);
  RUN_TEST(test_enter_preflight_during_an_unfinished_launch_hold_goes_to_preflight);
  RUN_TEST(test_launch_from_preflight_is_debounced);
  RUN_TEST(test_launch_threshold_is_inclusive_and_any_direction_counts);
  RUN_TEST(test_launch_ignores_the_estimator);
  RUN_TEST(test_launch_fails_on_an_invalid_imu);
  RUN_TEST(test_launch_hold_starts_over_on_reentry);
  RUN_TEST(test_preflight_reinitialises_the_estimator_on_entry);
  RUN_TEST(test_preflight_reinitialises_again_on_a_repeated_enter_preflight);
  RUN_TEST(test_preflight_stand_down_goes_to_init_on_the_command_cycle);
  RUN_TEST(test_launch_wins_over_stand_down_in_the_same_cycle);
  RUN_TEST(test_preflight_has_no_timeout);
  RUN_TEST(test_boost_altitude_exit_is_debounced);
  RUN_TEST(test_boost_timeout_at_exactly_its_limit);
  RUN_TEST(test_control_does_not_capture_a_reference_before_a_valid_attitude);
  RUN_TEST(test_control_captures_the_vertical_reference_of_that_cycles_attitude_once);
  RUN_TEST(test_control_entry_forgets_the_old_reference);
  RUN_TEST(test_control_speed_exit_is_not_armed_when_entered_slow_and_accelerating);
  RUN_TEST(test_control_speed_exit_is_debounced_once_armed);
  RUN_TEST(test_control_speed_exit_ignores_an_invalid_speed);
  RUN_TEST(test_control_leaves_on_the_cutoff_latch_from_tilt);
  RUN_TEST(test_control_leaves_on_the_cutoff_latch_from_rate);
  RUN_TEST(test_control_timeout_at_exactly_its_limit);
  RUN_TEST(test_coast_speed_exit_is_debounced);
  RUN_TEST(test_coast_timeout_at_exactly_its_limit);
  RUN_TEST(test_chute_altitude_exit_is_debounced);
  RUN_TEST(test_chute_timeout_at_exactly_its_limit);
  RUN_TEST(test_land_never_leaves);
  RUN_TEST(test_flight_states_request_neutral_and_ignore_ground_commands);
  RUN_TEST(test_ground_test_moves_all_canards);
  RUN_TEST(test_ground_test_moves_one_canard);
  RUN_TEST(test_ground_test_ignores_a_bad_canard_index);
  RUN_TEST(test_ground_test_ignores_a_non_finite_angle);
  RUN_TEST(test_ground_test_expires_after_its_hold_in_sample_time);
  RUN_TEST(test_ground_test_stop);
  RUN_TEST(test_ground_test_restart_extends_the_hold);
  RUN_TEST(test_ground_test_applies_commands_in_arrival_order);
  RUN_TEST(test_ground_test_requests_its_angles_every_cycle_while_active);
  RUN_TEST(test_init_and_preflight_run_the_ground_test_without_a_conflict);
  RUN_TEST(test_ground_test_is_cleared_when_init_or_preflight_is_left);
  return UNITY_END();
}
