// The Canards subsystem
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.4 and step 4 of
// section 10).
//
// The rig is a real ServoControl on a calibrated test table, a real Navigation
// fed by a FakeEstimator, and a FakeAttitudeController. "Neutral" means every
// commanded angle is zero AND every pulse is that channel's centre pulse.
#include <unity.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "Actuators.h"
#include "AttitudeControl/AttitudeController.h"
#include "FakeAttitudeController.h"
#include "FakeEstimator.h"
#include "FlightConfig.h"
#include "Frames.h"
#include "SensorSource.h"
#include "ServoControl/ServoConfig.h"
#include "ServoControl/ServoControl.h"
#include "StateMachine/State.h"
#include "StateMachine/StateId.h"
#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Navigation/Navigation.h"

void setUp() {}
void tearDown() {}

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kPi = 3.14159265358979323846f;
constexpr float kHalfPi = 0.5f * kPi;
constexpr float kRadToDeg = 180.0f / kPi;

constexpr uint64_t kStepUs = 5000;
constexpr float kStepS = 0.005f;
// Late enough that "one stale time ago" is still a positive time.
constexpr uint64_t kT0 = FlightConfig::kNavStaleUs + 10 * FlightConfig::kUsPerSecond;

// The four rows of include/VehicleConfig.h, marked calibrated so a commanded
// angle really moves a pulse.
constexpr ServoConfig kTestServoConfig = {
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

const StateId kAllStates[] = {StateId::Init,  StateId::PreFlight, StateId::Boost, StateId::Control,
                              StateId::Coast, StateId::Chute,     StateId::Land};
const StateId kFlightStates[] = {StateId::Boost, StateId::Control, StateId::Coast, StateId::Chute,
                                 StateId::Land};

// Inside every limit, four different values so a swapped canard shows.
const float kTestAngles[kNumCanards] = {0.02f, -0.03f, 0.04f, -0.05f};
const float kControllerCommand[kNumCanards] = {0.01f, -0.02f, 0.03f, -0.04f};

// Body -> NED attitude with the nose `tiltRad` away from straight up.
Quat noseUpTilted(float tiltRad) { return fromAxisAngle(Vec3{0.0f, 1.0f, 0.0f}, kHalfPi - tiltRad); }

// What the states ask to hold in these tests: straight up.
Quat reference() { return noseUpTilted(0.0f); }

SensorFrame imuFrame(uint64_t timeUs) {
  SensorFrame f;
  f.timeUs = timeUs;
  f.imuOk = true;
  f.accel = Vec3{0.0f, 0.0f, -FlightConfig::kGravity};
  f.gyro = Vec3{0.0f, 0.0f, 0.0f};
  return f;
}

// A good in-flight estimate: a little tilt, gentle rates, plenty of speed.
void scriptGoodFlight(FakeEstimator& estimator) {
  estimator.setValid(noseUpTilted(0.1f), Vec3{0.4f, 0.3f, -0.2f},
                     Vec3{5.0f, 0.0f, -2.0f * FlightConfig::kMinControlSpeed}, 500.0f);
}

uint8_t u8(CanardMode m) { return static_cast<uint8_t>(m); }
uint8_t u8(CanardRequest r) { return static_cast<uint8_t>(r); }

struct Rig {
  FakeEstimator estimator;
  Navigation nav;
  FakeAttitudeController controller;
  ServoControl servo;
  Canards canards;
  CommandList commands;
  ActuatorFeedback feedback;  // no encoder valid unless a test says so
  ActuatorCommand out;
  uint64_t timeUs = kT0;
  bool safetyActive = false;

  Rig() : nav(estimator), servo(kTestServoConfig), canards(controller, servo) {
    scriptGoodFlight(estimator);
    controller.setCommand(kControllerCommand);
  }

  // One cycle: fresh sensor frame, beginCycle(), the requests, periodic().
  template <typename Requests>
  void stepWith(StateId state, Requests requests) {
    timeUs += kStepUs;
    nav.update(imuFrame(timeUs), kStepS, isGroundState(state));
    canards.beginCycle();
    requests(canards);
    const Cycle cycle{timeUs, kStepS, 0, nav, commands};
    canards.periodic(cycle, state, safetyActive, feedback, out);
  }

  void step(StateId state) {
    stepWith(state, [](Canards&) {});
  }
  void stepNeutral(StateId state) {
    stepWith(state, [](Canards& c) { c.requestNeutral(); });
  }
  void stepHold(StateId state, const Quat& ref) {
    stepWith(state, [&ref](Canards& c) { c.requestAttitudeHold(ref); });
  }
  void stepHold(StateId state) { stepHold(state, reference()); }
  void stepTest(StateId state, const float rad[kNumCanards]) {
    stepWith(state, [rad](Canards& c) { c.requestTestAngles(rad); });
  }
  void stepTest(StateId state) { stepTest(state, kTestAngles); }

  void command(CommandType type, StateId state) {
    Command c;
    c.type = type;
    canards.handleCommand(c, state);
  }
  void enable(StateId state) { command(CommandType::CanardsEnable, state); }
  void disable(StateId state) { command(CommandType::CanardsDisable, state); }

  // Enabled on the pad, through PreFlight, into Boost: the next Control cycle
  // may control.
  void readyToControl() {
    enable(StateId::PreFlight);
    step(StateId::PreFlight);
    step(StateId::Boost);
  }

  // Into `state` the normal way (via PreFlight, so never locked out), enabled.
  void enterEnabled(StateId state) {
    enable(StateId::PreFlight);
    step(StateId::PreFlight);
    if (state != StateId::PreFlight) step(state);
  }

  const CanardsStatus& status() const { return canards.status(); }
};

// The pulse the servo model gives canard `canard` commanded to `rad`, open
// loop on its calibration (no encoder, zero PID gains).
float expectedPulse(int canard, float rad, int& channel) {
  channel = kTestServoConfig.channelOfCanard[canard];
  const ServoChannel& ch = kTestServoConfig.channels[channel];
  float deg = kTestServoConfig.canardSign[canard] * rad * kRadToDeg;
  deg = deg < ch.minAngle ? ch.minAngle : (deg > ch.maxAngle ? ch.maxAngle : deg);
  float pulse = ch.centerPulseUs + ch.pulseUsPerDegree * deg;
  pulse = pulse < ch.minPulseUs ? ch.minPulseUs : (pulse > ch.maxPulseUs ? ch.maxPulseUs : pulse);
  return pulse;
}

void expectNeutralAt(int line, const Rig& rig, const char* what) {
  float centre[kNumCanards];
  rig.servo.centerPulses(centre);
  for (int i = 0; i < kNumCanards; ++i) {
    UNITY_TEST_ASSERT(rig.status().commandRad[i] == 0.0f, line, what);
    UNITY_TEST_ASSERT(rig.out.pulseUs[i] == centre[i], line, what);
  }
}

// Commanded angles are `expected` and every pulse follows from them.
void expectCommandAt(int line, const Rig& rig, const float expected[kNumCanards], const char* what) {
  for (int canard = 0; canard < kNumCanards; ++canard) {
    UNITY_TEST_ASSERT_FLOAT_WITHIN(1e-7f, expected[canard], rig.status().commandRad[canard], line,
                                   what);
    int ch = 0;
    const float pulse = expectedPulse(canard, expected[canard], ch);
    UNITY_TEST_ASSERT_FLOAT_WITHIN(0.01f, pulse, rig.out.pulseUs[ch], line, what);
    if (expected[canard] != 0.0f) {
      UNITY_TEST_ASSERT(rig.out.pulseUs[ch] != kTestServoConfig.channels[ch].centerPulseUs, line,
                        what);
    }
  }
  UNITY_TEST_ASSERT(rig.out.servoPower, line, what);
}

void expectOffAt(int line, const Rig& rig, const char* what) {
  UNITY_TEST_ASSERT_EQUAL_UINT8(u8(CanardMode::Off), u8(rig.status().mode), line, what);
  UNITY_TEST_ASSERT(!rig.status().powered, line, what);
  UNITY_TEST_ASSERT(!rig.out.servoPower, line, what);
  expectNeutralAt(line, rig, what);
}

#define EXPECT_NEUTRAL(rig, what) expectNeutralAt(__LINE__, (rig), (what))
#define EXPECT_COMMAND(rig, expected, what) expectCommandAt(__LINE__, (rig), (expected), (what))
#define EXPECT_OFF(rig, what) expectOffAt(__LINE__, (rig), (what))
#define EXPECT_MODE(expected, rig, what) \
  UNITY_TEST_ASSERT_EQUAL_UINT8(u8(expected), u8((rig).status().mode), __LINE__, (what))
#define EXPECT_LAST_DENIED(expected, rig, what) \
  UNITY_TEST_ASSERT_EQUAL_UINT8(u8(expected), u8((rig).status().lastDenied), __LINE__, (what))

// Powered neutral, nothing denied, no gate closed.
void expectPoweredNeutralAt(int line, const Rig& rig, const char* what) {
  UNITY_TEST_ASSERT_EQUAL_UINT8(u8(CanardMode::Neutral), u8(rig.status().mode), line, what);
  UNITY_TEST_ASSERT(rig.status().powered, line, what);
  UNITY_TEST_ASSERT(rig.out.servoPower, line, what);
  expectNeutralAt(line, rig, what);
}
#define EXPECT_POWERED_NEUTRAL(rig, what) expectPoweredNeutralAt(__LINE__, (rig), (what))

// Attitude hold in Control was refused by gate(s) `gates` alone.
void expectGatedAt(int line, const Rig& rig, uint16_t gates, uint32_t controllerCalls) {
  UNITY_TEST_ASSERT_EQUAL_HEX16(gates, rig.status().gates, line, "gates");
  UNITY_TEST_ASSERT_EQUAL_UINT8(u8(CanardRequest::AttitudeHold), u8(rig.status().request), line,
                                "request");
  UNITY_TEST_ASSERT_EQUAL_UINT32(controllerCalls, rig.controller.callCount(), line,
                                 "controller calls");
  UNITY_TEST_ASSERT_EQUAL_UINT32(0, rig.status().deniedCount, line, "a gate is not a denial");
  expectNeutralAt(line, rig, "neutral");
}
#define EXPECT_GATED(rig, gates, calls) expectGatedAt(__LINE__, (rig), (gates), (calls))

}  // namespace

// ---------------------------------------------------------------------------
// Requests
// ---------------------------------------------------------------------------

static void test_no_request_is_neutral() {
  for (StateId s : kAllStates) {
    Rig rig;
    rig.enterEnabled(s);
    rig.step(s);
    const CanardsStatus& st = rig.status();
    UNITY_TEST_ASSERT_EQUAL_UINT8(u8(CanardRequest::None), u8(st.request), __LINE__, stateName(s));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, st.deniedCount, stateName(s));
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(0, st.gates, stateName(s));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, rig.controller.callCount(), stateName(s));
    if (s == StateId::Chute || s == StateId::Land) {
      EXPECT_OFF(rig, stateName(s));
    } else {
      EXPECT_POWERED_NEUTRAL(rig, stateName(s));
    }
  }
}

static void test_permission_table() {
  enum Req { kNeutral, kTest, kHold };
  for (StateId s : kAllStates) {
    for (int r = kNeutral; r <= kHold; ++r) {
      Rig rig;
      rig.enterEnabled(s);
      if (r == kNeutral) rig.stepNeutral(s);
      if (r == kTest) rig.stepTest(s);
      if (r == kHold) rig.stepHold(s);

      const bool unpowered = s == StateId::Chute || s == StateId::Land;
      const bool allowed = r == kNeutral || (r == kTest && isGroundState(s)) ||
                           (r == kHold && s == StateId::Control);
      const char* what = stateName(s);
      const CanardsStatus& st = rig.status();

      if (allowed) {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, st.deniedCount, what);
        EXPECT_LAST_DENIED(CanardRequest::None, rig, what);
      } else {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, st.deniedCount, what);
        EXPECT_LAST_DENIED(r == kTest ? CanardRequest::TestAngles : CanardRequest::AttitudeHold, rig,
                           what);
        // A refused request is not attitude hold: no gate is reported.
        TEST_ASSERT_EQUAL_HEX16_MESSAGE(0, st.gates, what);
      }

      if (unpowered) {
        EXPECT_OFF(rig, what);
      } else if (!allowed || r == kNeutral) {
        EXPECT_POWERED_NEUTRAL(rig, what);
      } else if (r == kTest) {
        EXPECT_MODE(CanardMode::Testing, rig, what);
        EXPECT_COMMAND(rig, kTestAngles, what);
      } else {
        EXPECT_MODE(CanardMode::Controlling, rig, what);
        EXPECT_COMMAND(rig, kControllerCommand, what);
      }
      const uint32_t expectedCalls = (allowed && r == kHold) ? 1 : 0;
      TEST_ASSERT_EQUAL_UINT32_MESSAGE(expectedCalls, rig.controller.callCount(), what);
    }
  }
}

static void test_conflicting_requests_in_either_order_are_neutral_and_denied() {
  // Each pair, both orders, in a state where both halves would be allowed
  // (Control for attitude hold, PreFlight for test angles).
  for (int pair = 0; pair < 6; ++pair) {
    Rig rig;
    const StateId s = (pair < 4) ? StateId::Control : StateId::PreFlight;
    if (s == StateId::Control) {
      rig.readyToControl();
    } else {
      rig.enterEnabled(s);
    }
    rig.stepWith(s, [pair](Canards& c) {
      switch (pair) {
        case 0: c.requestNeutral(); c.requestAttitudeHold(reference()); break;
        case 1: c.requestAttitudeHold(reference()); c.requestNeutral(); break;
        case 2: c.requestTestAngles(kTestAngles); c.requestAttitudeHold(reference()); break;
        case 3: c.requestAttitudeHold(reference()); c.requestTestAngles(kTestAngles); break;
        case 4: c.requestNeutral(); c.requestTestAngles(kTestAngles); break;
        default: c.requestTestAngles(kTestAngles); c.requestNeutral(); break;
      }
    });
    const CanardsStatus& st = rig.status();
    TEST_ASSERT_EQUAL_UINT8(u8(CanardRequest::Conflict), u8(st.request));
    TEST_ASSERT_EQUAL_UINT32(1, st.deniedCount);
    EXPECT_LAST_DENIED(CanardRequest::Conflict, rig, "pair");
    TEST_ASSERT_EQUAL_HEX16(0, st.gates);
    TEST_ASSERT_EQUAL_UINT32(0, rig.controller.callCount());
    EXPECT_POWERED_NEUTRAL(rig, "conflict");
  }
}

static void test_a_conflict_is_not_undone_by_a_third_request() {
  Rig rig;
  rig.readyToControl();
  rig.stepWith(StateId::Control, [](Canards& c) {
    c.requestAttitudeHold(reference());
    c.requestNeutral();
    c.requestAttitudeHold(reference());
  });
  TEST_ASSERT_EQUAL_UINT8(u8(CanardRequest::Conflict), u8(rig.status().request));
  EXPECT_LAST_DENIED(CanardRequest::Conflict, rig, "conflict");
  TEST_ASSERT_EQUAL_UINT32(0, rig.controller.callCount());
  EXPECT_POWERED_NEUTRAL(rig, "conflict");
}

static void test_same_test_request_twice_last_one_wins() {
  Rig rig;
  rig.enterEnabled(StateId::Init);
  const float second[kNumCanards] = {-0.01f, 0.06f, -0.07f, 0.03f};
  rig.stepWith(StateId::PreFlight, [&second](Canards& c) {
    c.requestTestAngles(kTestAngles);
    c.requestTestAngles(second);
  });
  TEST_ASSERT_EQUAL_UINT32(0, rig.status().deniedCount);
  EXPECT_MODE(CanardMode::Testing, rig, "testing");
  EXPECT_COMMAND(rig, second, "second request");
}

static void test_same_hold_request_twice_last_reference_wins() {
  Rig rig;
  rig.readyToControl();
  const Quat first = noseUpTilted(0.3f);
  const Quat second = fromAxisAngle(Vec3{1.0f, 0.0f, 0.0f}, 0.2f) * reference();
  rig.stepWith(StateId::Control, [&](Canards& c) {
    c.requestAttitudeHold(first);
    c.requestAttitudeHold(second);
  });
  TEST_ASSERT_EQUAL_UINT32(0, rig.status().deniedCount);
  EXPECT_MODE(CanardMode::Controlling, rig, "controlling");
  TEST_ASSERT_EQUAL_UINT32(1, rig.controller.callCount());
  const Vec3 expected = attitudeError(second, rig.nav.attitude());
  TEST_ASSERT_EQUAL_FLOAT(expected.x, rig.controller.lastInput().attitudeError.x);
  TEST_ASSERT_EQUAL_FLOAT(expected.y, rig.controller.lastInput().attitudeError.y);
  TEST_ASSERT_EQUAL_FLOAT(expected.z, rig.controller.lastInput().attitudeError.z);
}

static void test_neutral_twice_is_not_a_conflict() {
  Rig rig;
  rig.enterEnabled(StateId::PreFlight);
  rig.stepWith(StateId::PreFlight, [](Canards& c) {
    c.requestNeutral();
    c.requestNeutral();
  });
  TEST_ASSERT_EQUAL_UINT8(u8(CanardRequest::Neutral), u8(rig.status().request));
  TEST_ASSERT_EQUAL_UINT32(0, rig.status().deniedCount);
  EXPECT_POWERED_NEUTRAL(rig, "neutral");
}

static void test_denials_accumulate_and_last_denied_follows_the_latest() {
  Rig rig;
  rig.readyToControl();
  rig.stepTest(StateId::Control);
  TEST_ASSERT_EQUAL_UINT32(1, rig.status().deniedCount);
  EXPECT_LAST_DENIED(CanardRequest::TestAngles, rig, "1");

  rig.stepHold(StateId::Coast);
  TEST_ASSERT_EQUAL_UINT32(2, rig.status().deniedCount);
  EXPECT_LAST_DENIED(CanardRequest::AttitudeHold, rig, "2");

  // Allowed and empty cycles leave both alone.
  rig.stepNeutral(StateId::Coast);
  rig.step(StateId::Coast);
  TEST_ASSERT_EQUAL_UINT32(2, rig.status().deniedCount);
  EXPECT_LAST_DENIED(CanardRequest::AttitudeHold, rig, "kept");
}

static void test_a_request_lasts_one_cycle() {
  Rig rig;
  rig.enterEnabled(StateId::PreFlight);
  rig.stepTest(StateId::PreFlight);
  EXPECT_MODE(CanardMode::Testing, rig, "testing");
  rig.step(StateId::PreFlight);
  EXPECT_POWERED_NEUTRAL(rig, "after test");
  TEST_ASSERT_EQUAL_UINT8(u8(CanardRequest::None), u8(rig.status().request));

  rig.step(StateId::Boost);
  rig.stepHold(StateId::Control);
  EXPECT_MODE(CanardMode::Controlling, rig, "controlling");
  TEST_ASSERT_EQUAL_UINT32(1, rig.controller.callCount());
  rig.step(StateId::Control);
  EXPECT_POWERED_NEUTRAL(rig, "after hold");
  TEST_ASSERT_EQUAL_UINT32(1, rig.controller.callCount());
  TEST_ASSERT_EQUAL_HEX16(0, rig.status().gates);
}

static void test_test_angles_on_the_launch_cycle_are_refused() {
  Rig rig;
  rig.enterEnabled(StateId::PreFlight);
  rig.stepTest(StateId::PreFlight);
  EXPECT_MODE(CanardMode::Testing, rig, "testing");
  // PreFlight asked again, but launch was detected this cycle: the state in
  // force is Boost.
  rig.stepTest(StateId::Boost);
  TEST_ASSERT_EQUAL_UINT32(1, rig.status().deniedCount);
  EXPECT_LAST_DENIED(CanardRequest::TestAngles, rig, "launch cycle");
  EXPECT_POWERED_NEUTRAL(rig, "launch cycle");
}

// ---------------------------------------------------------------------------
// Attitude hold and its gates
// ---------------------------------------------------------------------------

static void test_attitude_hold_acts_with_every_gate_open() {
  Rig rig;
  rig.readyToControl();
  rig.stepHold(StateId::Control);
  const CanardsStatus& st = rig.status();
  EXPECT_MODE(CanardMode::Controlling, rig, "controlling");
  TEST_ASSERT_TRUE(st.enabled);
  TEST_ASSERT_TRUE(st.powered);
  TEST_ASSERT_FALSE(st.lockedOut);
  TEST_ASSERT_FALSE(st.cutoffLatched);
  TEST_ASSERT_EQUAL_HEX16(0, st.gates);
  TEST_ASSERT_EQUAL_UINT32(0, st.deniedCount);
  TEST_ASSERT_EQUAL_UINT8(u8(CanardRequest::AttitudeHold), u8(st.request));
  TEST_ASSERT_EQUAL_UINT32(1, rig.controller.callCount());
  EXPECT_COMMAND(rig, kControllerCommand, "controller output");
}

static void test_controller_receives_error_rate_angles_and_speed() {
  Rig rig;
  rig.readyToControl();
  // Encoders on every channel, so the angles are measured and all different.
  const float shaftDeg[kNumCanards] = {1.5f, -0.5f, 2.0f, -1.0f};  // by canard
  for (int canard = 0; canard < kNumCanards; ++canard) {
    const int ch = kTestServoConfig.channelOfCanard[canard];
    const ServoChannel& row = kTestServoConfig.channels[ch];
    rig.feedback.encoderAngleRad[ch] = (row.zeroAngle + row.direction * shaftDeg[canard]) / kRadToDeg;
    rig.feedback.encoderValid[ch] = true;
  }
  const Quat ref = fromAxisAngle(Vec3{0.0f, 0.0f, 1.0f}, 0.25f) * reference();
  rig.stepHold(StateId::Control, ref);

  TEST_ASSERT_EQUAL_UINT32(1, rig.controller.callCount());
  const AttitudeControlInput& in = rig.controller.lastInput();
  const Vec3 error = attitudeError(ref, rig.nav.attitude());
  TEST_ASSERT_TRUE(norm(error) > 0.1f);  // a real error, not zero
  TEST_ASSERT_EQUAL_FLOAT(error.x, in.attitudeError.x);
  TEST_ASSERT_EQUAL_FLOAT(error.y, in.attitudeError.y);
  TEST_ASSERT_EQUAL_FLOAT(error.z, in.attitudeError.z);
  TEST_ASSERT_EQUAL_FLOAT(rig.nav.bodyRate().x, in.bodyRate.x);
  TEST_ASSERT_EQUAL_FLOAT(rig.nav.bodyRate().y, in.bodyRate.y);
  TEST_ASSERT_EQUAL_FLOAT(rig.nav.bodyRate().z, in.bodyRate.z);
  TEST_ASSERT_EQUAL_FLOAT(0.4f, in.bodyRate.x);
  TEST_ASSERT_EQUAL_FLOAT(rig.nav.speed(), in.speed);
  TEST_ASSERT_TRUE(in.speed > FlightConfig::kMinControlSpeed);

  TEST_ASSERT_TRUE(rig.status().angles.measured);
  for (int canard = 0; canard < kNumCanards; ++canard) {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, shaftDeg[canard] / kRadToDeg, rig.status().angles.rad[canard]);
    TEST_ASSERT_EQUAL_FLOAT(rig.status().angles.rad[canard], in.canardAngle[canard]);
  }
}

static void test_gate_not_enabled() {
  Rig rig;
  rig.step(StateId::PreFlight);
  rig.step(StateId::Boost);
  rig.stepHold(StateId::Control);
  EXPECT_GATED(rig, CanardGate::kNotEnabled, 0);
  EXPECT_OFF(rig, "never enabled");
  TEST_ASSERT_FALSE(rig.status().enabled);
}

static void test_gate_not_enabled_after_disable_in_flight() {
  Rig rig;
  rig.readyToControl();
  rig.stepHold(StateId::Control);
  EXPECT_MODE(CanardMode::Controlling, rig, "before");
  rig.controller.clearLog();
  rig.disable(StateId::Control);
  rig.stepHold(StateId::Control);
  EXPECT_GATED(rig, CanardGate::kNotEnabled, 0);
  EXPECT_OFF(rig, "disabled");
}

static void test_gate_locked_out() {
  Rig rig;
  rig.enable(StateId::Init);
  rig.step(StateId::Init);
  rig.step(StateId::Boost);
  rig.stepHold(StateId::Control);
  EXPECT_GATED(rig, CanardGate::kLockedOut, 0);
  EXPECT_MODE(CanardMode::Neutral, rig, "locked out");
  TEST_ASSERT_TRUE(rig.status().lockedOut);
  TEST_ASSERT_TRUE(rig.out.servoPower);
}

static void test_gate_safety_fault() {
  Rig rig;
  rig.readyToControl();
  rig.safetyActive = true;
  rig.stepHold(StateId::Control);
  EXPECT_GATED(rig, CanardGate::kSafetyFault, 0);
  EXPECT_MODE(CanardMode::Neutral, rig, "safety");

  // Active bits only: control comes back when the fault clears.
  rig.safetyActive = false;
  rig.stepHold(StateId::Control);
  EXPECT_MODE(CanardMode::Controlling, rig, "cleared");
  TEST_ASSERT_EQUAL_HEX16(0, rig.status().gates);
}

static void test_gate_attitude_invalid() {
  Rig rig;
  rig.readyToControl();
  rig.estimator.output.attitudeValid = false;
  rig.stepHold(StateId::Control);
  TEST_ASSERT_FALSE(rig.nav.attitudeValid());
  TEST_ASSERT_TRUE(rig.nav.speedValid());
  EXPECT_GATED(rig, CanardGate::kNavInvalid, 0);
  EXPECT_MODE(CanardMode::Neutral, rig, "attitude invalid");
}

static void test_gate_speed_invalid() {
  Rig rig;
  rig.readyToControl();
  rig.estimator.output.velocityValid = false;
  rig.stepHold(StateId::Control);
  TEST_ASSERT_TRUE(rig.nav.attitudeValid());
  TEST_ASSERT_FALSE(rig.nav.speedValid());
  EXPECT_GATED(rig, CanardGate::kNavInvalid, 0);
  EXPECT_MODE(CanardMode::Neutral, rig, "speed invalid");
}

static void test_gate_stale_navigation() {
  Rig rig;
  rig.readyToControl();
  rig.estimator.stampTime = false;
  rig.estimator.output.timeUs = rig.timeUs + kStepUs - FlightConfig::kNavStaleUs - 1;
  rig.stepHold(StateId::Control);
  EXPECT_GATED(rig, CanardGate::kNavInvalid, 0);
  EXPECT_MODE(CanardMode::Neutral, rig, "stale");

  // Exactly at the limit it is still fresh.
  rig.estimator.output.timeUs = rig.timeUs + kStepUs - FlightConfig::kNavStaleUs;
  rig.stepHold(StateId::Control);
  EXPECT_MODE(CanardMode::Controlling, rig, "at the limit");
}

static void test_gate_speed_just_below_the_minimum() {
  Rig rig;
  rig.readyToControl();
  rig.estimator.output.velocityNed =
      Vec3{0.0f, 0.0f, -std::nextafter(FlightConfig::kMinControlSpeed, 0.0f)};
  rig.stepHold(StateId::Control);
  TEST_ASSERT_TRUE(rig.nav.speed() < FlightConfig::kMinControlSpeed);
  EXPECT_GATED(rig, CanardGate::kSpeedLow, 0);
  EXPECT_MODE(CanardMode::Neutral, rig, "slow");
}

static void test_acts_at_exactly_the_minimum_speed() {
  Rig rig;
  rig.readyToControl();
  rig.estimator.output.velocityNed = Vec3{0.0f, 0.0f, -FlightConfig::kMinControlSpeed};
  rig.stepHold(StateId::Control);
  TEST_ASSERT_EQUAL_FLOAT(FlightConfig::kMinControlSpeed, rig.nav.speed());
  TEST_ASSERT_EQUAL_HEX16(0, rig.status().gates);
  EXPECT_MODE(CanardMode::Controlling, rig, "at the minimum");
  EXPECT_COMMAND(rig, kControllerCommand, "at the minimum");
}

static void test_gate_controller_failure() {
  Rig rig;
  rig.readyToControl();
  rig.controller.setFailure();
  rig.stepHold(StateId::Control);
  EXPECT_GATED(rig, CanardGate::kControllerFailed, 1);
  EXPECT_MODE(CanardMode::Neutral, rig, "failed");
}

static void test_gate_controller_non_finite_output() {
  const float badValues[] = {kNaN, kInf, -kInf};
  for (float bad : badValues) {
    for (int canard = 0; canard < kNumCanards; ++canard) {
      Rig rig;
      rig.readyToControl();
      rig.controller.setNonFiniteOutput(canard, bad);
      rig.stepHold(StateId::Control);
      EXPECT_GATED(rig, CanardGate::kNonFinite, 1);
      EXPECT_MODE(CanardMode::Neutral, rig, "non-finite output");
    }
  }
}

static void test_gate_non_finite_or_non_unit_reference() {
  const Quat good = reference();
  const Quat bad[] = {
      Quat{kNaN, good.x, good.y, good.z},
      Quat{good.w, kInf, good.y, good.z},
      Quat{good.w, good.x, -kInf, good.z},
      Quat{good.w, good.x, good.y, kNaN},
      Quat{0.0f, 0.0f, 0.0f, 0.0f},
      Quat{0.5f * good.w, 0.5f * good.x, 0.5f * good.y, 0.5f * good.z},
      Quat{2.0f * good.w, 2.0f * good.x, 2.0f * good.y, 2.0f * good.z},
  };
  for (const Quat& ref : bad) {
    Rig rig;
    rig.readyToControl();
    rig.stepHold(StateId::Control, ref);
    EXPECT_GATED(rig, CanardGate::kNonFinite, 0);
    EXPECT_MODE(CanardMode::Neutral, rig, "bad reference");
  }
}

static void test_output_is_clamped_to_the_canard_limit() {
  Rig rig;
  rig.readyToControl();
  const float lim = FlightConfig::kCanardLimitRad;
  rig.controller.setCommand(10.0f * lim, -10.0f * lim, 0.5f * lim, -std::nextafter(lim, 1.0f));
  rig.stepHold(StateId::Control);
  EXPECT_MODE(CanardMode::Controlling, rig, "clamped");
  const float expected[kNumCanards] = {lim, -lim, 0.5f * lim, -lim};
  EXPECT_COMMAND(rig, expected, "clamped");
  for (int i = 0; i < kNumCanards; ++i) TEST_ASSERT_TRUE(std::fabs(rig.status().commandRad[i]) <= lim);
}

// ---------------------------------------------------------------------------
// Test angles
// ---------------------------------------------------------------------------

static void test_test_angles_are_clamped_to_the_ground_test_limit() {
  const StateId ground[] = {StateId::Init, StateId::PreFlight};
  for (StateId s : ground) {
    Rig rig;
    rig.enterEnabled(s);
    const float lim = FlightConfig::kGroundTestLimitRad;
    const float asked[kNumCanards] = {1.0f, -1.0f, 0.5f * lim, -std::nextafter(lim, 1.0f)};
    rig.stepTest(s, asked);
    EXPECT_MODE(CanardMode::Testing, rig, stateName(s));
    const float expected[kNumCanards] = {lim, -lim, 0.5f * lim, -lim};
    EXPECT_COMMAND(rig, expected, stateName(s));
  }
}

static void test_a_non_finite_test_angle_makes_the_request_neutral() {
  const float badValues[] = {kNaN, kInf, -kInf};
  for (float bad : badValues) {
    for (int canard = 0; canard < kNumCanards; ++canard) {
      Rig rig;
      rig.enterEnabled(StateId::PreFlight);
      float asked[kNumCanards];
      for (int i = 0; i < kNumCanards; ++i) asked[i] = kTestAngles[i];
      asked[canard] = bad;
      rig.stepTest(StateId::PreFlight, asked);
      EXPECT_POWERED_NEUTRAL(rig, "non-finite test angle");
      TEST_ASSERT_EQUAL_UINT8(u8(CanardRequest::TestAngles), u8(rig.status().request));
    }
  }
}

static void test_a_safety_fault_forces_test_angles_to_neutral() {
  Rig rig;
  rig.enterEnabled(StateId::Init);
  rig.safetyActive = true;
  rig.stepTest(StateId::Init);
  EXPECT_POWERED_NEUTRAL(rig, "safety");
  TEST_ASSERT_EQUAL_UINT32(0, rig.status().deniedCount);

  rig.safetyActive = false;
  rig.stepTest(StateId::Init);
  EXPECT_MODE(CanardMode::Testing, rig, "cleared");
  EXPECT_COMMAND(rig, kTestAngles, "cleared");
}

// ---------------------------------------------------------------------------
// Envelope cut-off
// ---------------------------------------------------------------------------

static void test_cutoff_latches_on_tilt() {
  Rig rig;
  rig.readyToControl();
  rig.estimator.output.attitude = noseUpTilted(FlightConfig::kMaxTiltRad + 0.05f);
  rig.stepHold(StateId::Control);
  TEST_ASSERT_TRUE(rig.status().cutoffLatched);
  EXPECT_GATED(rig, CanardGate::kCutoff, 0);
  EXPECT_MODE(CanardMode::Neutral, rig, "tilt");
}

static void test_cutoff_latches_on_pitch_rate_either_sign() {
  const float signs[] = {1.0f, -1.0f};
  for (float sign : signs) {
    Rig rig;
    rig.readyToControl();
    rig.estimator.output.bodyRate = Vec3{0.0f, sign * (FlightConfig::kMaxPitchYawRate + 0.1f), 0.0f};
    rig.stepHold(StateId::Control);
    TEST_ASSERT_TRUE(rig.status().cutoffLatched);
    EXPECT_GATED(rig, CanardGate::kCutoff, 0);
  }
}

static void test_cutoff_latches_on_yaw_rate_either_sign() {
  const float signs[] = {1.0f, -1.0f};
  for (float sign : signs) {
    Rig rig;
    rig.readyToControl();
    rig.estimator.output.bodyRate = Vec3{0.0f, 0.0f, sign * (FlightConfig::kMaxPitchYawRate + 0.1f)};
    rig.stepHold(StateId::Control);
    TEST_ASSERT_TRUE(rig.status().cutoffLatched);
    EXPECT_GATED(rig, CanardGate::kCutoff, 0);
  }
}

static void test_cutoff_rates_exactly_at_the_limit_do_not_trip() {
  Rig rig;
  rig.readyToControl();
  const float r = FlightConfig::kMaxPitchYawRate;
  rig.estimator.output.bodyRate = Vec3{0.0f, -r, r};
  rig.stepHold(StateId::Control);
  TEST_ASSERT_FALSE(rig.status().cutoffLatched);
  EXPECT_MODE(CanardMode::Controlling, rig, "at the limit");
}

static void test_roll_rate_does_not_trip_the_cutoff() {
  Rig rig;
  rig.readyToControl();
  rig.estimator.output.bodyRate = Vec3{10.0f * FlightConfig::kMaxPitchYawRate, 0.0f, 0.0f};
  rig.stepHold(StateId::Control);
  TEST_ASSERT_FALSE(rig.status().cutoffLatched);
  TEST_ASSERT_EQUAL_HEX16(0, rig.status().gates);
  EXPECT_MODE(CanardMode::Controlling, rig, "roll");
}

static void test_cutoff_stays_latched_after_the_condition_clears() {
  Rig rig;
  rig.readyToControl();
  rig.estimator.output.bodyRate = Vec3{0.0f, FlightConfig::kMaxPitchYawRate + 1.0f, 0.0f};
  rig.stepHold(StateId::Control);
  TEST_ASSERT_TRUE(rig.status().cutoffLatched);

  scriptGoodFlight(rig.estimator);
  for (int i = 0; i < 20; ++i) {
    rig.stepHold(StateId::Control);
    TEST_ASSERT_TRUE(rig.status().cutoffLatched);
    EXPECT_GATED(rig, CanardGate::kCutoff, 0);
    EXPECT_MODE(CanardMode::Neutral, rig, "latched");
  }
  // Still visible to the log after Control.
  rig.step(StateId::Coast);
  TEST_ASSERT_TRUE(rig.status().cutoffLatched);
  rig.step(StateId::Chute);
  TEST_ASSERT_TRUE(rig.status().cutoffLatched);
}

static void test_cutoff_is_not_evaluated_without_a_valid_attitude() {
  Rig rig;
  rig.readyToControl();
  // The estimator says the attitude is invalid, but sends wild numbers.
  rig.estimator.output.attitudeValid = false;
  rig.estimator.output.attitude = noseUpTilted(FlightConfig::kMaxTiltRad + 0.5f);
  rig.estimator.output.bodyRate = Vec3{0.0f, 3.0f * FlightConfig::kMaxPitchYawRate,
                                       -3.0f * FlightConfig::kMaxPitchYawRate};
  rig.stepHold(StateId::Control);
  TEST_ASSERT_FALSE(rig.status().cutoffLatched);
  EXPECT_GATED(rig, CanardGate::kNavInvalid, 0);

  scriptGoodFlight(rig.estimator);
  rig.stepHold(StateId::Control);
  TEST_ASSERT_FALSE(rig.status().cutoffLatched);
  EXPECT_MODE(CanardMode::Controlling, rig, "valid again");
}

static void test_a_rocket_on_its_side_on_the_pad_does_not_trip_the_cutoff() {
  Rig rig;
  rig.enable(StateId::Init);
  // The identity attitude: nose pointing north, lying on its side, and rolling
  // about in someone's hands.
  rig.estimator.output.attitude = kIdentityQuat;
  rig.estimator.output.bodyRate = Vec3{0.0f, 2.0f * FlightConfig::kMaxPitchYawRate, 0.0f};
  const StateId ground[] = {StateId::Init, StateId::PreFlight};
  for (StateId s : ground) {
    rig.step(s);
    rig.stepNeutral(s);
    rig.stepTest(s);
    rig.stepHold(s);  // refused by the table, so never evaluated
    TEST_ASSERT_TRUE(rig.nav.attitudeValid());
    TEST_ASSERT_TRUE(rig.nav.tilt() > FlightConfig::kMaxTiltRad);
    TEST_ASSERT_FALSE(rig.status().cutoffLatched);
  }

  // Stood up and launched from PreFlight: it controls.
  scriptGoodFlight(rig.estimator);
  rig.step(StateId::PreFlight);
  rig.step(StateId::Boost);
  rig.stepHold(StateId::Control);
  TEST_ASSERT_FALSE(rig.status().cutoffLatched);
  EXPECT_MODE(CanardMode::Controlling, rig, "after the pad");
}

// ---------------------------------------------------------------------------
// Lock-out
// ---------------------------------------------------------------------------

static void test_launch_from_init_locks_out_for_the_whole_flight() {
  Rig rig;
  rig.enable(StateId::Init);
  rig.step(StateId::Init);
  TEST_ASSERT_FALSE(rig.status().lockedOut);
  rig.step(StateId::Boost);
  TEST_ASSERT_TRUE(rig.status().lockedOut);
  EXPECT_POWERED_NEUTRAL(rig, "boost");
  for (int i = 0; i < 50; ++i) {
    rig.stepHold(StateId::Control);
    TEST_ASSERT_TRUE(rig.status().lockedOut);
    EXPECT_GATED(rig, CanardGate::kLockedOut, 0);
    EXPECT_MODE(CanardMode::Neutral, rig, "locked out");
  }
  rig.step(StateId::Coast);
  TEST_ASSERT_TRUE(rig.status().lockedOut);
  rig.step(StateId::Chute);
  rig.step(StateId::Land);
  TEST_ASSERT_TRUE(rig.status().lockedOut);
}

static void test_launch_from_preflight_controls() {
  Rig rig;
  rig.enable(StateId::Init);
  rig.step(StateId::Init);
  rig.step(StateId::PreFlight);
  rig.step(StateId::Boost);
  TEST_ASSERT_FALSE(rig.status().lockedOut);
  for (int i = 0; i < 50; ++i) {
    rig.stepHold(StateId::Control);
    TEST_ASSERT_FALSE(rig.status().lockedOut);
    EXPECT_MODE(CanardMode::Controlling, rig, "controlling");
  }
  TEST_ASSERT_EQUAL_UINT32(50, rig.controller.callCount());
}

static void test_preflight_then_init_then_launch_locks_out() {
  Rig rig;
  rig.enable(StateId::PreFlight);
  rig.step(StateId::PreFlight);
  rig.step(StateId::Init);  // a scrub
  rig.step(StateId::Boost);
  TEST_ASSERT_TRUE(rig.status().lockedOut);
  rig.stepHold(StateId::Control);
  EXPECT_GATED(rig, CanardGate::kLockedOut, 0);
  TEST_ASSERT_TRUE(rig.status().enabled);  // the scrub did not disable
}

// ---------------------------------------------------------------------------
// Power, enable and disable
// ---------------------------------------------------------------------------

static void test_canards_are_disabled_at_boot() {
  Rig rig;
  TEST_ASSERT_FALSE(rig.status().enabled);
  TEST_ASSERT_FALSE(rig.status().powered);
  TEST_ASSERT_EQUAL_UINT8(u8(CanardMode::Off), u8(rig.status().mode));

  rig.stepTest(StateId::Init);
  TEST_ASSERT_FALSE(rig.status().enabled);
  EXPECT_OFF(rig, "boot");
  // An allowed request, merely unpowered: not a denial.
  TEST_ASSERT_EQUAL_UINT32(0, rig.status().deniedCount);
}

static void test_enable_is_accepted_on_the_ground() {
  const StateId ground[] = {StateId::Init, StateId::PreFlight};
  for (StateId s : ground) {
    Rig rig;
    rig.enable(s);
    rig.step(s);
    TEST_ASSERT_TRUE_MESSAGE(rig.status().enabled, stateName(s));
    TEST_ASSERT_TRUE_MESSAGE(rig.status().powered, stateName(s));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, rig.status().rejectedCommandCount, stateName(s));
    EXPECT_POWERED_NEUTRAL(rig, stateName(s));
  }
}

static void test_enable_is_rejected_and_counted_in_every_flight_state() {
  for (StateId s : kFlightStates) {
    Rig rig;
    rig.step(StateId::PreFlight);
    rig.enable(s);
    rig.step(s);
    TEST_ASSERT_FALSE_MESSAGE(rig.status().enabled, stateName(s));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, rig.status().rejectedCommandCount, stateName(s));
    EXPECT_OFF(rig, stateName(s));
    rig.enable(s);
    rig.step(s);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, rig.status().rejectedCommandCount, stateName(s));
    TEST_ASSERT_FALSE_MESSAGE(rig.status().enabled, stateName(s));
  }
}

static void test_disable_is_accepted_in_every_state() {
  for (StateId s : kAllStates) {
    Rig rig;
    rig.enterEnabled(s);
    rig.step(s);
    TEST_ASSERT_TRUE_MESSAGE(rig.status().enabled, stateName(s));
    rig.disable(s);
    rig.step(s);
    TEST_ASSERT_FALSE_MESSAGE(rig.status().enabled, stateName(s));
    EXPECT_OFF(rig, stateName(s));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, rig.status().rejectedCommandCount, stateName(s));
  }
}

static void test_disable_in_flight_drops_power_for_good() {
  Rig rig;
  rig.readyToControl();
  TEST_ASSERT_TRUE(rig.out.servoPower);
  rig.disable(StateId::Boost);
  rig.step(StateId::Boost);
  EXPECT_OFF(rig, "disabled in boost");
  // And it cannot be turned back on in flight.
  rig.enable(StateId::Control);
  rig.stepHold(StateId::Control);
  EXPECT_OFF(rig, "re-enable refused");
  TEST_ASSERT_EQUAL_UINT32(1, rig.status().rejectedCommandCount);
  TEST_ASSERT_EQUAL_UINT32(0, rig.controller.callCount());
}

static void test_no_servo_power_in_chute_and_land_even_when_enabled() {
  Rig rig;
  rig.readyToControl();
  rig.stepHold(StateId::Control);
  rig.step(StateId::Coast);
  EXPECT_POWERED_NEUTRAL(rig, "coast");
  const StateId descent[] = {StateId::Chute, StateId::Land};
  for (StateId s : descent) {
    rig.step(s);
    TEST_ASSERT_TRUE_MESSAGE(rig.status().enabled, stateName(s));
    EXPECT_OFF(rig, stateName(s));
    rig.stepNeutral(s);
    EXPECT_OFF(rig, stateName(s));
  }
}

static void test_other_commands_are_ignored() {
  const CommandType others[] = {CommandType::EnterPreFlight, CommandType::StandDown,
                                CommandType::CanardTest, CommandType::CanardTestStop};
  for (CommandType type : others) {
    Rig rig;
    rig.command(type, StateId::Init);
    rig.step(StateId::Init);
    TEST_ASSERT_FALSE(rig.status().enabled);
    TEST_ASSERT_EQUAL_UINT32(0, rig.status().rejectedCommandCount);

    rig.enable(StateId::Init);
    rig.command(type, StateId::Boost);
    rig.step(StateId::Init);
    TEST_ASSERT_TRUE(rig.status().enabled);
    TEST_ASSERT_EQUAL_UINT32(0, rig.status().rejectedCommandCount);
  }
}

static void test_a_forbidden_hold_reports_no_gate_powered_or_not() {
  // Disabled: refused by the table, not by the gates.
  Rig rig;
  rig.stepHold(StateId::Init);
  TEST_ASSERT_EQUAL_UINT32(1, rig.status().deniedCount);
  EXPECT_LAST_DENIED(CanardRequest::AttitudeHold, rig, "disabled");
  TEST_ASSERT_EQUAL_HEX16(0, rig.status().gates);
  EXPECT_OFF(rig, "disabled");

  rig.enable(StateId::Init);
  rig.stepHold(StateId::Init);
  TEST_ASSERT_EQUAL_UINT32(2, rig.status().deniedCount);
  TEST_ASSERT_EQUAL_HEX16(0, rig.status().gates);
  EXPECT_POWERED_NEUTRAL(rig, "enabled");
}

// ---------------------------------------------------------------------------
// Servo loop reset
// ---------------------------------------------------------------------------

static bool allZero(const CanardAngles& a) {
  for (int i = 0; i < kNumCanards; ++i) {
    if (a.rad[i] != 0.0f) return false;
  }
  return true;
}

static void test_servo_model_is_reset_on_power_down() {
  Rig rig;
  rig.enterEnabled(StateId::Init);
  for (int i = 0; i < 20; ++i) rig.stepTest(StateId::Init);
  TEST_ASSERT_FALSE(rig.status().angles.measured);  // no encoders: the model
  TEST_ASSERT_FLOAT_WITHIN(0.01f, kTestAngles[3], rig.status().angles.rad[3]);

  rig.disable(StateId::Init);
  rig.step(StateId::Init);
  // Unpowered, the model is not advanced, so only a reset brings it to zero.
  rig.step(StateId::Init);
  TEST_ASSERT_TRUE(allZero(rig.status().angles));
}

static void test_servo_model_is_reset_on_entering_chute() {
  Rig rig;
  rig.enterEnabled(StateId::PreFlight);
  for (int i = 0; i < 20; ++i) rig.stepTest(StateId::PreFlight);
  rig.step(StateId::Boost);
  TEST_ASSERT_FALSE(allZero(rig.status().angles));
  rig.step(StateId::Chute);
  rig.step(StateId::Chute);
  TEST_ASSERT_TRUE(allZero(rig.status().angles));
}

static void test_servo_model_is_reset_on_power_up() {
  Rig rig;
  // While the canards are off, something leaves the servo model off zero.
  float pulses[kNumCanards];
  for (int i = 0; i < 20; ++i) rig.servo.command(kTestAngles, kStepS, rig.feedback, pulses);
  CanardAngles before;
  rig.servo.measure(rig.feedback, before);
  TEST_ASSERT_FALSE(allZero(before));

  rig.enable(StateId::Init);
  rig.step(StateId::Init);  // power-up, neutral
  TEST_ASSERT_TRUE(rig.status().powered);
  // Without a reset the neutral command would only have moved the model one
  // lag step towards zero.
  rig.step(StateId::Init);
  TEST_ASSERT_TRUE(allZero(rig.status().angles));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_no_request_is_neutral);
  RUN_TEST(test_permission_table);
  RUN_TEST(test_conflicting_requests_in_either_order_are_neutral_and_denied);
  RUN_TEST(test_a_conflict_is_not_undone_by_a_third_request);
  RUN_TEST(test_same_test_request_twice_last_one_wins);
  RUN_TEST(test_same_hold_request_twice_last_reference_wins);
  RUN_TEST(test_neutral_twice_is_not_a_conflict);
  RUN_TEST(test_denials_accumulate_and_last_denied_follows_the_latest);
  RUN_TEST(test_a_request_lasts_one_cycle);
  RUN_TEST(test_test_angles_on_the_launch_cycle_are_refused);
  RUN_TEST(test_attitude_hold_acts_with_every_gate_open);
  RUN_TEST(test_controller_receives_error_rate_angles_and_speed);
  RUN_TEST(test_gate_not_enabled);
  RUN_TEST(test_gate_not_enabled_after_disable_in_flight);
  RUN_TEST(test_gate_locked_out);
  RUN_TEST(test_gate_safety_fault);
  RUN_TEST(test_gate_attitude_invalid);
  RUN_TEST(test_gate_speed_invalid);
  RUN_TEST(test_gate_stale_navigation);
  RUN_TEST(test_gate_speed_just_below_the_minimum);
  RUN_TEST(test_acts_at_exactly_the_minimum_speed);
  RUN_TEST(test_gate_controller_failure);
  RUN_TEST(test_gate_controller_non_finite_output);
  RUN_TEST(test_gate_non_finite_or_non_unit_reference);
  RUN_TEST(test_output_is_clamped_to_the_canard_limit);
  RUN_TEST(test_test_angles_are_clamped_to_the_ground_test_limit);
  RUN_TEST(test_a_non_finite_test_angle_makes_the_request_neutral);
  RUN_TEST(test_a_safety_fault_forces_test_angles_to_neutral);
  RUN_TEST(test_cutoff_latches_on_tilt);
  RUN_TEST(test_cutoff_latches_on_pitch_rate_either_sign);
  RUN_TEST(test_cutoff_latches_on_yaw_rate_either_sign);
  RUN_TEST(test_cutoff_rates_exactly_at_the_limit_do_not_trip);
  RUN_TEST(test_roll_rate_does_not_trip_the_cutoff);
  RUN_TEST(test_cutoff_stays_latched_after_the_condition_clears);
  RUN_TEST(test_cutoff_is_not_evaluated_without_a_valid_attitude);
  RUN_TEST(test_a_rocket_on_its_side_on_the_pad_does_not_trip_the_cutoff);
  RUN_TEST(test_launch_from_init_locks_out_for_the_whole_flight);
  RUN_TEST(test_launch_from_preflight_controls);
  RUN_TEST(test_preflight_then_init_then_launch_locks_out);
  RUN_TEST(test_canards_are_disabled_at_boot);
  RUN_TEST(test_enable_is_accepted_on_the_ground);
  RUN_TEST(test_enable_is_rejected_and_counted_in_every_flight_state);
  RUN_TEST(test_disable_is_accepted_in_every_state);
  RUN_TEST(test_disable_in_flight_drops_power_for_good);
  RUN_TEST(test_no_servo_power_in_chute_and_land_even_when_enabled);
  RUN_TEST(test_other_commands_are_ignored);
  RUN_TEST(test_a_forbidden_hold_reports_no_gate_powered_or_not);
  RUN_TEST(test_servo_model_is_reset_on_power_down);
  RUN_TEST(test_servo_model_is_reset_on_entering_chute);
  RUN_TEST(test_servo_model_is_reset_on_power_up);
  return UNITY_END();
}
