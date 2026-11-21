// The whole flight program through FlightComputer::cycle(), on a scripted
// flight (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 6.1, 6.2, 6.4
// and the done-criteria of step 5 of section 10).
//
// The harness: a FakeSensorSource producing one frame per call from a script
// that is a function of sample time, at ONE fixed sample period (kPeriodUs); a
// FakeEstimator fed the truth from the same script; a FakeCommandSource, a
// FakeActuators and a RecordingLogSink; the REAL LqrAttitudeController; and the
// VehicleConfig servo rows marked calibrated, so a canard that is driven
// visibly moves off its centre pulse.
//
// Every expected transition time is worked out here from the script's event
// times and the FlightConfig holds and timeouts, never hard-coded, so the tests
// keep passing when a placeholder changes. If a change makes the script itself
// impossible (e.g. a hold longer than the phase it sits in), checkScript()
// fails with a message saying which assumption broke.
#include <unity.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "Actuators.h"
#include "AttitudeControl/LqrAttitudeController.h"
#include "Estimator/NullEstimator.h"
#include "FakeActuators.h"
#include "FakeCommandSource.h"
#include "FakeEstimator.h"
#include "FakeSensorSource.h"
#include "FlightComputer.h"
#include "FlightConfig.h"
#include "Frames.h"
#include "RecordingLogSink.h"
#include "Safety/SafetyMonitor.h"
#include "SensorSource.h"
#include "StateMachine/StateId.h"
#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Logger/LogSink.h"
#include "VehicleConfig.h"

void setUp() {}
void tearDown() {}

namespace {

namespace fc = FlightConfig;

// THE sample period of every flight here (200 Hz, what the LQR gains assume).
constexpr uint64_t kPeriodUs = 5000;
static_assert(kPeriodUs <= fc::kMaxImuGapUs, "the nominal period must not be an IMU gap");

constexpr float kHalfPi = 1.57079632679489661923f;

// A duration rounded up to whole sample periods: a hold or timeout of `us`
// that starts on a sample is first met this long after it.
constexpr uint64_t ceilToSample(uint64_t us) { return (us + kPeriodUs - 1) / kPeriodUs * kPeriodUs; }
constexpr uint64_t samples(uint64_t n) { return n * kPeriodUs; }

#define EXPECT_STATE(expected, actual) TEST_ASSERT_EQUAL_STRING(stateName(expected), stateName(actual))
#define EXPECT_MODE(expected, actual) TEST_ASSERT_EQUAL_INT(static_cast<int>(expected), static_cast<int>(actual))

ServoConfig makeCalibratedServos() {
  ServoConfig config = VehicleConfig::kServoConfig;
  for (ServoChannel& channel : config.channels) channel.calibrated = true;
  return config;
}
const ServoConfig kCalibratedServos = makeCalibratedServos();

bool atCentre(const ActuatorCommand& out) {
  for (int ch = 0; ch < kNumCanards; ++ch) {
    if (out.pulseUs[ch] != kCalibratedServos.channels[ch].centerPulseUs) return false;
  }
  return true;
}

Vec3 sensorAxes(const Vec3& body) {
  Mat3 t;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) t.m[r][c] = VehicleConfig::kImuToBody.m[c][r];
  }
  return t * body;
}

float lerp(float a, float b, uint64_t t, uint64_t t0, uint64_t t1) {
  const float f = static_cast<float>(t - t0) / static_cast<float>(t1 - t0);
  return a + (b - a) * f;
}

// ---------------------------------------------------------------------------
// The scripted flight
// ---------------------------------------------------------------------------

// Event times, each a sample time. Where a threshold is crossed the script
// switches exactly at the event, so "the first sample past the threshold" is
// the event itself.
struct Timeline {
  uint64_t start;      // first sample: Init entered
  uint64_t command;    // ground commands sent (EnterPreFlight and/or CanardsEnable)
  uint64_t launch;     // first sample with |specific force| >= kLaunchAccel
  uint64_t boost;      // launch held kLaunchHoldUs
  uint64_t altitude;   // first sample with altitude >= kControlStartAltitudeM
  uint64_t control;    // that held kControlStartHoldUs
  uint64_t peak;       // burnout, top speed
  uint64_t slow;       // first sample with speed < kControlEndSpeed
  uint64_t coast;      // that held kControlEndHoldUs
  uint64_t chuteSlow;  // first sample with speed < kChuteSpeed (apogee)
  uint64_t chute;      // that held kChuteHoldUs
  uint64_t low;        // first sample with altitude < kLandAltitudeM
  uint64_t land;       // that held kLandHoldUs
  uint64_t end;        // the script runs to here
};

Timeline makeTimeline() {
  Timeline t;
  t.start = 1 * fc::kUsPerSecond;
  t.command = t.start + samples(20);
  t.launch = t.command + samples(200);
  t.boost = t.launch + ceilToSample(fc::kLaunchHoldUs);
  t.altitude = t.boost + samples(10);
  t.control = t.altitude + ceilToSample(fc::kControlStartHoldUs);
  t.peak = t.control + samples(200);
  t.slow = t.peak + samples(400);
  t.coast = t.slow + ceilToSample(fc::kControlEndHoldUs);
  t.chuteSlow = t.coast + samples(400);
  t.chute = t.chuteSlow + ceilToSample(fc::kChuteHoldUs);
  t.low = t.chute + samples(1000);
  t.land = t.low + ceilToSample(fc::kLandHoldUs);
  t.end = t.land + samples(100);
  return t;
}

// The script only works if every navigation exit happens before its timeout.
void checkScript(const Timeline& t) {
  TEST_ASSERT_TRUE_MESSAGE(t.control - t.boost < fc::kBoostMaxUs, "script: Boost would time out first");
  TEST_ASSERT_TRUE_MESSAGE(t.coast - t.control < fc::kControlMaxUs, "script: Control would time out first");
  TEST_ASSERT_TRUE_MESSAGE(t.chute - t.coast < fc::kCoastMaxUs, "script: Coast would time out first");
  TEST_ASSERT_TRUE_MESSAGE(t.land - t.chute < fc::kChuteMaxUs, "script: Chute would time out first");
  TEST_ASSERT_TRUE_MESSAGE(t.boost <= t.peak, "script: the motor burns out before launch is detected");
  TEST_ASSERT_TRUE_MESSAGE((t.command - t.start) / kPeriodUs + 2 < FakeEstimator::kLogCapacity,
                           "script: the PreFlight re-initialisation falls outside the estimator's call log");
}

// The flight, as a function of sample time.
struct Script {
  Timeline t = makeTimeline();
  float vPeak = 2.0f * fc::kControlEndSpeed;
  float apogeeM = 10.0f * (fc::kControlStartAltitudeM + fc::kLandAltitudeM) + 100.0f;
  float tiltRad = 0.25f * fc::kMaxTiltRad;  // tilted, well inside the envelope

  float speed(uint64_t time) const {
    const float between = fc::kControlEndSpeed - fc::kChuteSpeed;
    if (time < t.launch) return 0.0f;
    if (time < t.peak) return lerp(0.0f, vPeak, time, t.launch, t.peak);
    if (time < t.slow) return lerp(vPeak, fc::kControlEndSpeed, time, t.peak, t.slow);  // > end speed
    if (time < t.chuteSlow) {  // strictly between the Chute and the end speed
      return lerp(fc::kChuteSpeed + 0.9f * between, fc::kChuteSpeed + 0.1f * between, time, t.slow,
                  t.chuteSlow);
    }
    return 0.5f * fc::kChuteSpeed;
  }

  float altitude(uint64_t time) const {
    if (time < t.launch) return 0.0f;
    if (time < t.altitude) return lerp(0.0f, 0.5f * fc::kControlStartAltitudeM, time, t.launch, t.altitude);
    if (time < t.chuteSlow) return lerp(fc::kControlStartAltitudeM, apogeeM, time, t.altitude, t.chuteSlow);
    if (time < t.low) return lerp(apogeeM, fc::kLandAltitudeM + 1.0f, time, t.chuteSlow, t.low);
    if (time < t.end) return lerp(0.9f * fc::kLandAltitudeM, 0.0f, time, t.low, t.end);
    return 0.0f;
  }

  // Body -> NED. Nose up on the pad; tilted and rolled from launch on.
  Quat attitude(uint64_t time) const {
    const float tilt = time < t.launch ? 0.0f : tiltRad;
    const float roll = time < t.launch ? 0.0f : 0.3f;
    return fromAxisAngle(Vec3{0.0f, 1.0f, 0.0f}, kHalfPi - tilt) * fromAxisAngle(Vec3{1.0f, 0.0f, 0.0f}, roll);
  }

  // Specific force along the nose, m/s^2.
  float thrust(uint64_t time) const {
    if (time < t.launch) return fc::kGravity;             // at rest
    if (time < t.peak) return 2.0f * fc::kLaunchAccel;    // motor burning
    if (time < t.chuteSlow) return -0.5f * fc::kGravity;  // drag
    return fc::kGravity;                                  // under the chute
  }

  SensorFrame frame(uint64_t time) const {
    SensorFrame f;
    f.timeUs = time;
    f.imuOk = true;
    f.accel = sensorAxes(Vec3{thrust(time), 0.0f, 0.0f});
    f.gyro = Vec3{0.0f, 0.0f, 0.0f};
    const uint64_t n = (time - t.start) / kPeriodUs;
    f.magFresh = n % 10 == 0;
    f.mag = Vec3{20.0f, 1.0f, 45.0f};
    f.baroFresh = n % 2 == 0;
    f.pressurePa = 101325.0f - 12.0f * altitude(time);
    f.temperatureC = 20.0f;
    return f;
  }

  NavState truth(uint64_t time) const {
    NavState s;
    s.timeUs = time;
    s.attitude = attitude(time);
    const float v = speed(time);
    s.velocityNed = Vec3{0.0f, 0.0f, time < t.chuteSlow ? -v : v};
    s.altitudeAgl = altitude(time);
    s.attitudeValid = true;
    s.velocityValid = true;
    s.altitudeValid = true;
    return s;
  }
};

// ---------------------------------------------------------------------------
// The harness
// ---------------------------------------------------------------------------

struct Harness {
  Script script;
  FakeEstimator fakeEstimator;
  NullEstimator nullEstimator;
  FakeSensorSource sensors;
  FakeCommandSource commands;
  FakeActuators actuators;
  RecordingLogSink log;
  LqrAttitudeController controller{fc::kCanardLimitRad};
  std::unique_ptr<FlightComputer> computer;

  // Commands to hand to the source just before the cycle of that sample time.
  std::vector<std::pair<uint64_t, Command>> scheduled;

  explicit Harness(bool useNullEstimator = false)
      : sensors(script.t.start, kPeriodUs, [this](uint64_t time) {
          fakeEstimator.output = script.truth(time);  // the same script feeds the estimator
          return script.frame(time);
        }) {
    Estimator& estimator = useNullEstimator ? static_cast<Estimator&>(nullEstimator) : fakeEstimator;
    const FlightIo io{sensors, commands, actuators, estimator, controller, log};
    computer.reset(new FlightComputer(io, kCalibratedServos));
  }

  FlightComputer& fc() { return *computer; }

  void schedule(uint64_t time, const Command& command) { scheduled.emplace_back(time, command); }

  // One cycle. Hands the scheduled commands over first, then checks the log
  // record against what the fakes supplied and what the accessors say.
  bool cycle() {
    const uint64_t time = sensors.nextTimeUs();
    for (const auto& entry : scheduled) {
      if (entry.first == time) commands.push(entry.second);
    }
    const size_t deliveredBefore = commands.delivered().size();
    const size_t recordsBefore = log.records.size();
    const bool ran = fc().cycle();
    if (ran) {
      TEST_ASSERT_EQUAL_UINT32(recordsBefore + 1, log.records.size());
      checkRecord(log.records.back(), deliveredBefore);
    } else {
      TEST_ASSERT_EQUAL_UINT32(recordsBefore, log.records.size());
    }
    return ran;
  }

  // Runs every sample up to and including `time`.
  void runThrough(uint64_t time) {
    while (sensors.nextTimeUs() <= time) TEST_ASSERT_TRUE(cycle());
  }

  const CycleRecord& last() const { return log.records.back(); }

  // The record is one per cycle; its inputs are exactly what the fakes
  // supplied and its outputs are what the FlightComputer reports.
  void checkRecord(const CycleRecord& r, size_t deliveredBefore) {
    const FlightComputer& f = *computer;
    TEST_ASSERT_EQUAL_UINT32(log.records.size(), r.cycleCount);
    TEST_ASSERT_EQUAL_UINT32(f.cycleCount(), r.cycleCount);

    // Inputs.
    TEST_ASSERT_TRUE(sameFrame(sensors.frames().back(), r.frame));
    TEST_ASSERT_TRUE(sameFeedback(actuators.feedback, r.feedback));
    const size_t delivered = commands.delivered().size() - deliveredBefore;
    TEST_ASSERT_EQUAL_UINT32(delivered, r.commandCount);
    for (size_t i = 0; i < delivered; ++i) {
      TEST_ASSERT_TRUE(sameCommand(commands.delivered()[deliveredBefore + i], r.commands[i]));
    }

    // Outputs.
    EXPECT_STATE(f.state(), r.state);
    TEST_ASSERT_EQUAL(f.stateMachine().transitioned(), r.transitioned);
    TEST_ASSERT_EQUAL_UINT32(f.stateMachine().refusedCount(), r.refusedTransitions);
    TEST_ASSERT_EQUAL_UINT16(f.safety().active(), r.safetyActive);
    TEST_ASSERT_EQUAL_UINT16(f.safety().sticky(), r.safetySticky);
    TEST_ASSERT_TRUE(sameStatus(f.canards().status(), r.canards));
    TEST_ASSERT_TRUE(sameActuators(actuators.last(), r.actuators));
    TEST_ASSERT_TRUE(sameNav(f.navigation().state(), r.nav));
    TEST_ASSERT_EQUAL_FLOAT(f.navigation().specificForce(), r.specificForce);
    TEST_ASSERT_EQUAL_FLOAT(f.navigation().speed(), r.speed);
    TEST_ASSERT_EQUAL_FLOAT(f.navigation().tilt(), r.tilt);
    TEST_ASSERT_EQUAL(f.navigation().imuValid(), r.imuValid);
    TEST_ASSERT_EQUAL(f.navigation().attitudeValid(), r.attitudeValid);
    TEST_ASSERT_EQUAL(f.navigation().speedValid(), r.speedValid);
    TEST_ASSERT_EQUAL(f.navigation().altitudeValid(), r.altitudeValid);
    TEST_ASSERT_TRUE(sameRecord(f.lastRecord(), r));
  }

  // --- Field-by-field comparisons (exact: the same inputs must give the same bits)

  static bool same(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
  static bool same(const Quat& a, const Quat& b) { return a.w == b.w && a.x == b.x && a.y == b.y && a.z == b.z; }

  static bool sameFrame(const SensorFrame& a, const SensorFrame& b) {
    return a.timeUs == b.timeUs && a.imuOk == b.imuOk && same(a.accel, b.accel) && same(a.gyro, b.gyro) &&
           a.magFresh == b.magFresh && same(a.mag, b.mag) && a.baroFresh == b.baroFresh &&
           a.pressurePa == b.pressurePa && a.temperatureC == b.temperatureC;
  }
  static bool sameFeedback(const ActuatorFeedback& a, const ActuatorFeedback& b) {
    for (int i = 0; i < kNumCanards; ++i) {
      if (a.encoderAngleRad[i] != b.encoderAngleRad[i] || a.encoderValid[i] != b.encoderValid[i]) return false;
    }
    return true;
  }
  static bool sameCommand(const Command& a, const Command& b) {
    return a.type == b.type && a.canard == b.canard && a.angleRad == b.angleRad;
  }
  static bool sameNav(const NavState& a, const NavState& b) {
    return a.timeUs == b.timeUs && same(a.attitude, b.attitude) && same(a.bodyRate, b.bodyRate) &&
           same(a.velocityNed, b.velocityNed) && a.altitudeAgl == b.altitudeAgl &&
           a.attitudeValid == b.attitudeValid && a.velocityValid == b.velocityValid &&
           a.altitudeValid == b.altitudeValid;
  }
  static bool sameStatus(const CanardsStatus& a, const CanardsStatus& b) {
    bool ok = a.mode == b.mode && a.enabled == b.enabled && a.powered == b.powered &&
              a.lockedOut == b.lockedOut && a.cutoffLatched == b.cutoffLatched && a.request == b.request &&
              a.gates == b.gates && a.deniedCount == b.deniedCount && a.lastDenied == b.lastDenied &&
              a.rejectedCommandCount == b.rejectedCommandCount && a.angles.measured == b.angles.measured;
    for (int i = 0; i < kNumCanards; ++i) {
      ok = ok && a.commandRad[i] == b.commandRad[i] && a.angles.rad[i] == b.angles.rad[i];
    }
    return ok;
  }
  static bool sameActuators(const ActuatorCommand& a, const ActuatorCommand& b) {
    bool ok = a.servoPower == b.servoPower;
    for (int i = 0; i < kNumCanards; ++i) ok = ok && a.pulseUs[i] == b.pulseUs[i];
    return ok;
  }
  static bool sameStep(const TimeStep& a, const TimeStep& b) {
    return a.first == b.first && a.backwards == b.backwards && a.gapUs == b.gapUs && a.dt == b.dt;
  }
  static bool sameRecord(const CycleRecord& a, const CycleRecord& b) {
    bool ok = a.cycleCount == b.cycleCount && sameFrame(a.frame, b.frame) && sameFeedback(a.feedback, b.feedback) &&
              a.commandCount == b.commandCount && sameStep(a.step, b.step) && sameNav(a.nav, b.nav) &&
              a.specificForce == b.specificForce && a.speed == b.speed && a.tilt == b.tilt &&
              a.imuValid == b.imuValid && a.attitudeValid == b.attitudeValid && a.speedValid == b.speedValid &&
              a.altitudeValid == b.altitudeValid && a.state == b.state && a.transitioned == b.transitioned &&
              a.timeInStateUs == b.timeInStateUs && a.refusedTransitions == b.refusedTransitions &&
              a.safetyActive == b.safetyActive && a.safetySticky == b.safetySticky &&
              sameStatus(a.canards, b.canards) && sameActuators(a.actuators, b.actuators);
    for (size_t i = 0; i < CommandList::kCapacity; ++i) ok = ok && sameCommand(a.commands[i], b.commands[i]);
    return ok;
  }

  // Every transition in the log, as (new state, sample time).
  std::vector<std::pair<StateId, uint64_t>> transitions() const {
    std::vector<std::pair<StateId, uint64_t>> out;
    for (const CycleRecord& r : log.records) {
      if (r.transitioned) out.emplace_back(r.state, r.frame.timeUs);
    }
    return out;
  }

  const CycleRecord& recordAt(uint64_t time) const {
    for (const CycleRecord& r : log.records) {
      if (r.frame.timeUs == time) return r;
    }
    TEST_FAIL_MESSAGE("no record at that time");
    return log.records.front();
  }
};

void expectTransitions(const Harness& h, const std::vector<std::pair<StateId, uint64_t>>& expected) {
  const auto actual = h.transitions();
  TEST_ASSERT_EQUAL_UINT32(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_STATE(expected[i].first, actual[i].first);
    TEST_ASSERT_EQUAL_UINT64(expected[i].second, actual[i].second);
  }
}

void scheduleNominal(Harness& h) {
  h.schedule(h.script.t.command, makeCommand(CommandType::EnterPreFlight));
  h.schedule(h.script.t.command, makeCommand(CommandType::CanardsEnable));
}

std::vector<std::pair<StateId, uint64_t>> nominalTransitions(const Timeline& t) {
  return {{StateId::PreFlight, t.command}, {StateId::Boost, t.boost}, {StateId::Control, t.control},
          {StateId::Coast, t.coast},       {StateId::Chute, t.chute}, {StateId::Land, t.land}};
}

// ===========================================================================
// 1. Nominal flight
// ===========================================================================

void test_nominal_flight_reaches_every_state_in_order_at_the_expected_times() {
  Harness h;
  const Timeline& t = h.script.t;
  checkScript(t);
  scheduleNominal(h);
  h.runThrough(t.end);

  expectTransitions(h, nominalTransitions(t));
  EXPECT_STATE(StateId::Land, h.fc().state());

  // One record per cycle.
  TEST_ASSERT_EQUAL_UINT32((t.end - t.start) / kPeriodUs + 1, h.log.records.size());
  TEST_ASSERT_EQUAL_UINT32(h.log.records.size(), h.actuators.writeCount());

  // Every command reached the log, in order.
  std::vector<Command> logged;
  for (const CycleRecord& r : h.log.records) {
    for (size_t i = 0; i < r.commandCount; ++i) logged.push_back(r.commands[i]);
  }
  TEST_ASSERT_EQUAL_UINT32(h.commands.delivered().size(), logged.size());
  TEST_ASSERT_EQUAL_UINT32(2, logged.size());

  // No refused transitions, no safety fault ever.
  TEST_ASSERT_EQUAL_UINT32(0, h.fc().stateMachine().refusedCount());
  TEST_ASSERT_EQUAL_UINT16(0, h.fc().safety().sticky());
  for (const CycleRecord& r : h.log.records) {
    TEST_ASSERT_EQUAL_UINT32(0, r.refusedTransitions);
    TEST_ASSERT_EQUAL_UINT16(0, r.safetyActive);
  }

  // The estimator was re-initialised once, on entering PreFlight: right
  // before the update of the first cycle after the command.
  TEST_ASSERT_EQUAL_UINT32(1, h.fakeEstimator.reinitializeCount());
  const size_t cyclesThroughCommand = (t.command - t.start) / kPeriodUs + 1;
  for (size_t i = 0; i < cyclesThroughCommand; ++i) {
    TEST_ASSERT_TRUE(h.fakeEstimator.callAt(i) == FakeEstimator::Call::Update);
  }
  TEST_ASSERT_TRUE(h.fakeEstimator.callAt(cyclesThroughCommand) == FakeEstimator::Call::Reinitialize);
  TEST_ASSERT_TRUE(h.fakeEstimator.callAt(cyclesThroughCommand + 1) == FakeEstimator::Call::Update);
}

void test_nominal_flight_canards_by_state() {
  Harness h;
  const Timeline& t = h.script.t;
  checkScript(t);
  scheduleNominal(h);
  h.runThrough(t.end);

  uint32_t controlling = 0;
  for (const CycleRecord& r : h.log.records) {
    const uint64_t time = r.frame.timeUs;
    const CanardsStatus& s = r.canards;
    TEST_ASSERT_FALSE(s.lockedOut);
    TEST_ASSERT_FALSE(s.cutoffLatched);
    switch (r.state) {
      case StateId::Init:  // not yet enabled, but powered from boot, holding zero
        TEST_ASSERT_FALSE(s.enabled);
        EXPECT_MODE(CanardMode::Neutral, s.mode);
        TEST_ASSERT_TRUE(s.powered);
        TEST_ASSERT_TRUE(r.actuators.servoPower);
        TEST_ASSERT_TRUE(atCentre(r.actuators));
        break;
      case StateId::PreFlight:
      case StateId::Boost:
      case StateId::Coast:
        EXPECT_MODE(CanardMode::Neutral, s.mode);
        TEST_ASSERT_TRUE(s.powered);
        TEST_ASSERT_TRUE(r.actuators.servoPower);
        TEST_ASSERT_TRUE(atCentre(r.actuators));
        break;
      case StateId::Control:
        TEST_ASSERT_TRUE(r.actuators.servoPower);
        if (time == t.control) {
          // The entry cycle: Boost asked for neutral.
          EXPECT_MODE(CanardMode::Neutral, s.mode);
        } else if (r.speed >= fc::kMinControlSpeed) {
          // A tilted rocket above the control speed: the canards really move.
          EXPECT_MODE(CanardMode::Controlling, s.mode);
          TEST_ASSERT_EQUAL_UINT16(0, s.gates);
          TEST_ASSERT_FALSE(atCentre(r.actuators));
          ++controlling;
        } else {
          EXPECT_MODE(CanardMode::Neutral, s.mode);
          TEST_ASSERT_TRUE((s.gates & CanardGate::kSpeedLow) != 0);
        }
        break;
      case StateId::Chute:
      case StateId::Land:
        EXPECT_MODE(CanardMode::Off, s.mode);
        TEST_ASSERT_TRUE(s.enabled);
        TEST_ASSERT_FALSE(s.powered);
        TEST_ASSERT_FALSE(r.actuators.servoPower);
        TEST_ASSERT_TRUE(atCentre(r.actuators));
        break;
    }
  }
  // Control lasted from t.control to t.coast; most of it above the control speed.
  TEST_ASSERT_TRUE(controlling > (t.slow - t.peak) / kPeriodUs);

  // Nothing is ever denied. On the cycle Control hands over to Coast its last
  // attitude-hold request ends at neutral (the permission check uses the
  // state in force after the transition), but it is not a denial: Control was
  // allowed to make it.
  TEST_ASSERT_EQUAL_UINT32(0, h.fc().canards().status().deniedCount);
  const CycleRecord& handover = h.recordAt(t.coast);
  EXPECT_STATE(StateId::Coast, handover.state);
  TEST_ASSERT_TRUE(handover.canards.request == CanardRequest::AttitudeHold);
  EXPECT_MODE(CanardMode::Neutral, handover.canards.mode);
  TEST_ASSERT_TRUE(atCentre(handover.actuators));
  TEST_ASSERT_EQUAL_UINT32(0, handover.canards.deniedCount);
  TEST_ASSERT_TRUE(handover.canards.lastDenied == CanardRequest::None);
}

// ===========================================================================
// 2. Launch while in Init: canards locked out for the whole flight
// ===========================================================================

void test_launch_from_init_reaches_every_state_with_the_canards_locked_out() {
  Harness h;
  const Timeline& t = h.script.t;
  checkScript(t);
  h.schedule(t.command, makeCommand(CommandType::CanardsEnable));  // enabled, but no PreFlight
  h.runThrough(t.end);

  expectTransitions(h, {{StateId::Boost, t.boost},
                        {StateId::Control, t.control},
                        {StateId::Coast, t.coast},
                        {StateId::Chute, t.chute},
                        {StateId::Land, t.land}});
  TEST_ASSERT_EQUAL_UINT32(0, h.fakeEstimator.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(0, h.fc().stateMachine().refusedCount());

  bool sawHoldRefused = false;
  for (const CycleRecord& r : h.log.records) {
    TEST_ASSERT_TRUE(r.canards.mode != CanardMode::Controlling);
    TEST_ASSERT_TRUE(atCentre(r.actuators));
    if (r.frame.timeUs >= t.boost) {
      TEST_ASSERT_TRUE(r.canards.lockedOut);
    } else {
      TEST_ASSERT_FALSE(r.canards.lockedOut);
    }
    TEST_ASSERT_EQUAL(r.frame.timeUs >= t.command, r.canards.enabled);
    if (r.state == StateId::Control && r.canards.request == CanardRequest::AttitudeHold) {
      TEST_ASSERT_TRUE((r.canards.gates & CanardGate::kLockedOut) != 0);
      sawHoldRefused = true;
    }
  }
  TEST_ASSERT_TRUE(sawHoldRefused);
}

// ===========================================================================
// 3. NullEstimator: every navigation exit falls back to its timeout
// ===========================================================================

void test_null_estimator_every_transition_falls_back_to_its_timeout() {
  Harness h(true);
  const Timeline& t = h.script.t;
  scheduleNominal(h);

  const uint64_t control = t.boost + ceilToSample(fc::kBoostMaxUs);
  const uint64_t coast = control + ceilToSample(fc::kControlMaxUs);
  const uint64_t chute = coast + ceilToSample(fc::kCoastMaxUs);
  const uint64_t land = chute + ceilToSample(fc::kChuteMaxUs);
  h.runThrough(land + samples(10));

  // Launch comes from the IMU alone, so it is on time; the rest are timeouts.
  expectTransitions(h, {{StateId::PreFlight, t.command},
                        {StateId::Boost, t.boost},
                        {StateId::Control, control},
                        {StateId::Coast, coast},
                        {StateId::Chute, chute},
                        {StateId::Land, land}});

  // At exactly the timeout: the cycle before each transition had been in
  // its state one sample less than the (sample-rounded) limit.
  TEST_ASSERT_EQUAL_UINT64(ceilToSample(fc::kBoostMaxUs) - kPeriodUs, h.recordAt(control - kPeriodUs).timeInStateUs);
  TEST_ASSERT_EQUAL_UINT64(ceilToSample(fc::kControlMaxUs) - kPeriodUs, h.recordAt(coast - kPeriodUs).timeInStateUs);
  TEST_ASSERT_EQUAL_UINT64(ceilToSample(fc::kCoastMaxUs) - kPeriodUs, h.recordAt(chute - kPeriodUs).timeInStateUs);
  TEST_ASSERT_EQUAL_UINT64(ceilToSample(fc::kChuteMaxUs) - kPeriodUs, h.recordAt(land - kPeriodUs).timeInStateUs);

  bool sawNavGate = false;
  for (const CycleRecord& r : h.log.records) {
    TEST_ASSERT_TRUE(r.canards.mode != CanardMode::Controlling);
    TEST_ASSERT_FALSE(r.attitudeValid);
    TEST_ASSERT_FALSE(r.speedValid);
    TEST_ASSERT_FALSE(r.altitudeValid);
    if (r.state == StateId::Control) {
      // No valid attitude, so no reference: Control asks for neutral.
      TEST_ASSERT_TRUE(r.canards.request != CanardRequest::AttitudeHold);
      sawNavGate = true;
    }
  }
  TEST_ASSERT_TRUE(sawNavGate);
  TEST_ASSERT_FALSE(h.fc().controlState().haveReference());
  TEST_ASSERT_EQUAL_UINT16(0, h.fc().safety().sticky());
}

// ===========================================================================
// Ground tests and canard commands through the FlightComputer
// ===========================================================================

void test_ground_test_moves_the_canards_on_the_pad_and_ends_at_neutral_on_the_launch_cycle() {
  Harness h;
  const Timeline& t = h.script.t;
  checkScript(t);
  const float angle = 0.5f * fc::kGroundTestLimitRad;
  const uint64_t testInInit = t.start + samples(5);
  const uint64_t testInPreFlight = t.command + samples(10);
  const uint64_t stop = t.command + samples(30);
  TEST_ASSERT_TRUE(stop < t.launch);
  // Never enabled: ground tests do not need it.
  h.schedule(testInInit, makeTestCommand(-1, angle));
  h.schedule(t.command, makeCommand(CommandType::EnterPreFlight));
  h.schedule(testInPreFlight, makeTestCommand(1, -angle));
  h.schedule(stop, makeCommand(CommandType::CanardTestStop));
  h.schedule(t.boost, makeTestCommand(-1, angle));  // the launch cycle
  h.runThrough(t.boost + samples(5));

  // Init: moving, every cycle. On the command cycle itself Init's last
  // execute() still asked for the test, and PreFlight (in force after the
  // transition) permits it.
  for (uint64_t time = testInInit; time <= t.command; time += kPeriodUs) {
    const CycleRecord& r = h.recordAt(time);
    EXPECT_MODE(CanardMode::Testing, r.canards.mode);
    TEST_ASSERT_FALSE(r.canards.enabled);
    TEST_ASSERT_FALSE(atCentre(r.actuators));
    TEST_ASSERT_EQUAL_FLOAT(angle, r.canards.commandRad[0]);
  }
  // Leaving Init ends the test: PreFlight's first own cycle is neutral.
  EXPECT_STATE(StateId::PreFlight, h.recordAt(t.command).state);
  EXPECT_MODE(CanardMode::Neutral, h.recordAt(t.command + kPeriodUs).canards.mode);
  TEST_ASSERT_TRUE(atCentre(h.recordAt(t.command + kPeriodUs).actuators));
  // PreFlight: moving one canard, until the stop.
  for (uint64_t time = testInPreFlight; time < stop; time += kPeriodUs) {
    const CycleRecord& r = h.recordAt(time);
    EXPECT_MODE(CanardMode::Testing, r.canards.mode);
    TEST_ASSERT_FALSE(atCentre(r.actuators));
    TEST_ASSERT_EQUAL_FLOAT(-angle, r.canards.commandRad[1]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.canards.commandRad[0]);
  }
  EXPECT_MODE(CanardMode::Neutral, h.recordAt(stop).canards.mode);
  TEST_ASSERT_TRUE(atCentre(h.recordAt(stop).actuators));
  TEST_ASSERT_EQUAL_UINT32(0, h.recordAt(t.boost - kPeriodUs).canards.deniedCount);

  // The launch cycle: PreFlight asked for the test, Boost (now in force)
  // does not allow it, so neutral. PreFlight was allowed to ask, so it is not
  // a denial.
  const CycleRecord& launch = h.recordAt(t.boost);
  EXPECT_STATE(StateId::Boost, launch.state);
  TEST_ASSERT_TRUE(launch.canards.request == CanardRequest::TestAngles);
  EXPECT_MODE(CanardMode::Neutral, launch.canards.mode);
  TEST_ASSERT_EQUAL_UINT32(0, launch.canards.deniedCount);
  TEST_ASSERT_TRUE(launch.canards.lastDenied == CanardRequest::None);
  TEST_ASSERT_TRUE(launch.actuators.servoPower);
  TEST_ASSERT_TRUE(atCentre(launch.actuators));
  EXPECT_MODE(CanardMode::Neutral, h.last().canards.mode);
}

void test_canards_enable_in_flight_is_rejected() {
  Harness h;
  const Timeline& t = h.script.t;
  checkScript(t);
  h.schedule(t.command, makeCommand(CommandType::EnterPreFlight));  // never enabled on the ground
  h.schedule(t.boost + samples(5), makeCommand(CommandType::CanardsEnable));
  h.schedule(t.control + samples(50), makeCommand(CommandType::CanardsEnable));
  h.runThrough(t.coast);

  TEST_ASSERT_EQUAL_UINT32(2, h.fc().canards().status().rejectedCommandCount);
  bool sawHoldRefused = false;
  for (const CycleRecord& r : h.log.records) {
    // Powered from boot to Coast, holding zero: never enabled, never moved.
    TEST_ASSERT_FALSE(r.canards.enabled);
    TEST_ASSERT_TRUE(r.canards.powered);
    TEST_ASSERT_TRUE(r.actuators.servoPower);
    TEST_ASSERT_TRUE(r.canards.mode != CanardMode::Controlling);
    TEST_ASSERT_TRUE(atCentre(r.actuators));
    if (r.state == StateId::Control && r.canards.request == CanardRequest::AttitudeHold) {
      TEST_ASSERT_TRUE((r.canards.gates & CanardGate::kNotEnabled) != 0);
      sawHoldRefused = true;
    }
  }
  TEST_ASSERT_TRUE(sawHoldRefused);
}

void test_canards_disable_in_control_holds_zero_powered() {
  Harness h;
  const Timeline& t = h.script.t;
  checkScript(t);
  scheduleNominal(h);
  h.schedule(t.peak, makeCommand(CommandType::CanardsDisable));
  h.runThrough(t.coast + samples(10));

  const CycleRecord& before = h.recordAt(t.peak - kPeriodUs);
  EXPECT_STATE(StateId::Control, before.state);
  EXPECT_MODE(CanardMode::Controlling, before.canards.mode);
  for (const CycleRecord& r : h.log.records) {
    if (r.frame.timeUs < t.peak) continue;
    TEST_ASSERT_FALSE(r.canards.enabled);
    TEST_ASSERT_TRUE(r.canards.powered);
    EXPECT_MODE(CanardMode::Neutral, r.canards.mode);
    TEST_ASSERT_TRUE(r.actuators.servoPower);
    TEST_ASSERT_TRUE(atCentre(r.actuators));
    if (r.state == StateId::Control) {
      TEST_ASSERT_TRUE(r.canards.request == CanardRequest::AttitudeHold);
      TEST_ASSERT_EQUAL_HEX16(CanardGate::kNotEnabled, r.canards.gates);
    }
  }
  // The flight goes on as before.
  EXPECT_STATE(StateId::Coast, h.recordAt(t.coast).state);
}

// ===========================================================================
// Time faults
// ===========================================================================

void test_imu_gap_forces_neutral_for_that_cycle_and_control_resumes() {
  Harness h;
  const Timeline& t = h.script.t;
  checkScript(t);
  scheduleNominal(h);
  h.runThrough(t.peak);
  EXPECT_MODE(CanardMode::Controlling, h.last().canards.mode);

  // A gap of exactly the limit is not a fault.
  h.sensors.setNextTimeUs(h.last().frame.timeUs + fc::kMaxImuGapUs);
  TEST_ASSERT_TRUE(h.cycle());
  TEST_ASSERT_EQUAL_UINT16(0, h.last().safetyActive);
  EXPECT_MODE(CanardMode::Controlling, h.last().canards.mode);

  // One microsecond more is.
  h.sensors.setNextTimeUs(h.last().frame.timeUs + fc::kMaxImuGapUs + 1);
  TEST_ASSERT_TRUE(h.cycle());
  TEST_ASSERT_TRUE((h.last().safetyActive & SafetyFault::kImuGap) != 0);
  EXPECT_MODE(CanardMode::Neutral, h.last().canards.mode);
  TEST_ASSERT_TRUE((h.last().canards.gates & CanardGate::kSafetyFault) != 0);
  TEST_ASSERT_TRUE(h.last().actuators.servoPower);
  TEST_ASSERT_TRUE(atCentre(h.last().actuators));
  EXPECT_STATE(StateId::Control, h.last().state);

  // The next sample is on time again: control resumes, the sticky bit stays.
  TEST_ASSERT_TRUE(h.cycle());
  TEST_ASSERT_EQUAL_UINT16(0, h.last().safetyActive);
  TEST_ASSERT_TRUE((h.last().safetySticky & SafetyFault::kImuGap) != 0);
  EXPECT_MODE(CanardMode::Controlling, h.last().canards.mode);
  TEST_ASSERT_FALSE(atCentre(h.last().actuators));
}

void test_time_going_backwards_is_flagged_and_does_not_crash() {
  Harness h;
  const Timeline& t = h.script.t;
  checkScript(t);
  scheduleNominal(h);
  h.runThrough(t.peak);

  // A repeated timestamp.
  h.sensors.setNextTimeUs(h.last().frame.timeUs);
  TEST_ASSERT_TRUE(h.cycle());
  TEST_ASSERT_TRUE((h.last().safetyActive & SafetyFault::kTimeBackwards) != 0);
  TEST_ASSERT_TRUE(h.last().step.backwards);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, h.last().step.dt);
  EXPECT_MODE(CanardMode::Neutral, h.last().canards.mode);

  // Back by a lot, to before Control was entered.
  const uint64_t timeInStateBefore = h.last().timeInStateUs;
  const uint64_t flightTimeBefore = h.last().flightTimeUs;
  h.sensors.setNextTimeUs(t.control - samples(5));
  TEST_ASSERT_TRUE(h.cycle());
  TEST_ASSERT_TRUE((h.last().safetyActive & SafetyFault::kTimeBackwards) != 0);
  EXPECT_STATE(StateId::Control, h.last().state);
  // The flight clock does not move on a backwards sample, so time in state
  // neither jumps nor resets.
  TEST_ASSERT_EQUAL_UINT64(flightTimeBefore, h.last().flightTimeUs);
  TEST_ASSERT_EQUAL_UINT64(timeInStateBefore, h.last().timeInStateUs);
  EXPECT_MODE(CanardMode::Neutral, h.last().canards.mode);

  // Time runs forward from there; the flight still finishes.
  TEST_ASSERT_TRUE(h.cycle());
  TEST_ASSERT_EQUAL_UINT16(0, h.last().safetyActive);
  TEST_ASSERT_TRUE((h.last().safetySticky & SafetyFault::kTimeBackwards) != 0);
  h.runThrough(t.end + samples(200));
  EXPECT_STATE(StateId::Land, h.fc().state());
  TEST_ASSERT_EQUAL_UINT32(0, h.fc().stateMachine().refusedCount());
}

// ===========================================================================
// Commands and samples
// ===========================================================================

void test_more_commands_than_fit_in_one_cycle_are_spread_over_later_cycles_in_order() {
  Harness h;
  const Timeline& t = h.script.t;
  h.runThrough(t.start + samples(2));

  const size_t total = 2 * CommandList::kCapacity + 3;
  std::vector<Command> sent;
  for (size_t i = 0; i < total; ++i) {
    const Command c = makeTestCommand(static_cast<int8_t>(i % kNumCanards), 0.001f * static_cast<float>(i + 1));
    sent.push_back(c);
    h.commands.push(c);
  }
  const size_t first = h.log.records.size();
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(h.cycle());

  TEST_ASSERT_EQUAL_UINT32(CommandList::kCapacity, h.log.records[first].commandCount);
  TEST_ASSERT_EQUAL_UINT32(CommandList::kCapacity, h.log.records[first + 1].commandCount);
  TEST_ASSERT_EQUAL_UINT32(3, h.log.records[first + 2].commandCount);
  TEST_ASSERT_EQUAL_UINT32(0, h.log.records[first + 3].commandCount);
  TEST_ASSERT_EQUAL_UINT32(0, h.commands.pendingCount());

  std::vector<Command> logged;
  for (size_t k = first; k < h.log.records.size(); ++k) {
    for (size_t i = 0; i < h.log.records[k].commandCount; ++i) logged.push_back(h.log.records[k].commands[i]);
  }
  TEST_ASSERT_EQUAL_UINT32(total, logged.size());
  for (size_t i = 0; i < total; ++i) TEST_ASSERT_TRUE(Harness::sameCommand(sent[i], logged[i]));
}

void test_cycle_without_a_new_sample_returns_false_and_does_nothing() {
  Harness h;
  const Timeline& t = h.script.t;
  h.runThrough(t.start + samples(3));
  h.commands.push(makeCommand(CommandType::EnterPreFlight));

  const uint32_t cycles = h.fc().cycleCount();
  const size_t records = h.log.records.size();
  const uint32_t writes = h.actuators.writeCount();
  const uint32_t reads = h.actuators.readCount();
  const uint32_t updates = h.fakeEstimator.updateCount();
  const uint32_t polls = h.commands.pollCalls();
  const uint64_t next = h.sensors.nextTimeUs();

  h.sensors.available = false;
  for (int i = 0; i < 3; ++i) TEST_ASSERT_FALSE(h.cycle());
  TEST_ASSERT_EQUAL_UINT32(cycles, h.fc().cycleCount());
  TEST_ASSERT_EQUAL_UINT32(records, h.log.records.size());
  TEST_ASSERT_EQUAL_UINT32(writes, h.actuators.writeCount());
  TEST_ASSERT_EQUAL_UINT32(reads, h.actuators.readCount());
  TEST_ASSERT_EQUAL_UINT32(updates, h.fakeEstimator.updateCount());
  TEST_ASSERT_EQUAL_UINT32(polls, h.commands.pollCalls());
  TEST_ASSERT_EQUAL_UINT32(1, h.commands.pendingCount());
  TEST_ASSERT_EQUAL_UINT64(next, h.sensors.nextTimeUs());
  EXPECT_STATE(StateId::Init, h.fc().state());

  // The sample arrives: the waiting command is taken and acted on.
  h.sensors.available = true;
  TEST_ASSERT_TRUE(h.cycle());
  TEST_ASSERT_EQUAL_UINT32(0, h.commands.pendingCount());
  EXPECT_STATE(StateId::PreFlight, h.fc().state());
  TEST_ASSERT_EQUAL_UINT16(0, h.last().safetyActive);
}

// ===========================================================================
// Determinism
// ===========================================================================

void test_the_same_script_gives_identical_records() {
  auto runOnce = [](Harness& h) {
    scheduleNominal(h);
    h.schedule(h.script.t.command + samples(5), makeTestCommand(-1, 0.02f));
    h.runThrough(h.script.t.end);
  };
  Harness a;
  Harness b;
  runOnce(a);
  runOnce(b);
  TEST_ASSERT_EQUAL_UINT32(a.log.records.size(), b.log.records.size());
  TEST_ASSERT_TRUE(a.log.records.size() > 0);
  for (size_t i = 0; i < a.log.records.size(); ++i) {
    if (!Harness::sameRecord(a.log.records[i], b.log.records[i])) {
      TEST_FAIL_MESSAGE("records differ between two identical runs");
    }
  }
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_nominal_flight_reaches_every_state_in_order_at_the_expected_times);
  RUN_TEST(test_nominal_flight_canards_by_state);
  RUN_TEST(test_launch_from_init_reaches_every_state_with_the_canards_locked_out);
  RUN_TEST(test_null_estimator_every_transition_falls_back_to_its_timeout);
  RUN_TEST(test_ground_test_moves_the_canards_on_the_pad_and_ends_at_neutral_on_the_launch_cycle);
  RUN_TEST(test_canards_enable_in_flight_is_rejected);
  RUN_TEST(test_canards_disable_in_control_holds_zero_powered);
  RUN_TEST(test_imu_gap_forces_neutral_for_that_cycle_and_control_resumes);
  RUN_TEST(test_time_going_backwards_is_flagged_and_does_not_crash);
  RUN_TEST(test_more_commands_than_fit_in_one_cycle_are_spread_over_later_cycles_in_order);
  RUN_TEST(test_cycle_without_a_new_sample_returns_false_and_does_nothing);
  RUN_TEST(test_the_same_script_gives_identical_records);
  return UNITY_END();
}
