// GroundCommands: the ground station's named commands turned into
// flight-program Commands (src/Subsystems/Comms/GroundCommands.h).
//
//   1. handle(): every (name, state) pair, the sweep guard, unknown names.
//   2. The queue: exactly what is queued, FIFO order, all or nothing when full.
//   3. End to end: GroundCommands as the FlightComputer's CommandSource.
//
// Every time and angle comes from FlightConfig, never a copy of its value.
#include <unity.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "AttitudeControl/LqrAttitudeController.h"
#include "FakeActuators.h"
#include "FakeEstimator.h"
#include "FakeSensorSource.h"
#include "FlightComputer.h"
#include "FlightConfig.h"
#include "RecordingLogSink.h"
#include "ServoControl/ServoConfig.h"
#include "StateMachine/GroundTest.h"
#include "StateMachine/StateId.h"
#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Comms/GroundCommands.h"
#include "VehicleConfig.h"

void setUp() {}
void tearDown() {}

namespace {

namespace fc = FlightConfig;

const StateId kAllStates[] = {StateId::Init,  StateId::PreFlight, StateId::Boost, StateId::Control,
                              StateId::Coast, StateId::Chute,     StateId::Land};

#define EXPECT_REPLY(expected, actual) TEST_ASSERT_EQUAL_INT(static_cast<int>(expected), static_cast<int>(actual))
#define EXPECT_REPLY_MESSAGE(expected, actual, message) \
  TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(expected), static_cast<int>(actual), (message))
#define EXPECT_TYPE(expected, actual) TEST_ASSERT_EQUAL_INT(static_cast<int>(expected), static_cast<int>(actual))

// Everything poll() hands out, in order, until it returns false.
std::vector<Command> drain(GroundCommands& ground) {
  std::vector<Command> out;
  Command c;
  while (ground.poll(c)) {
    out.push_back(c);
    TEST_ASSERT_TRUE(out.size() <= GroundCommands::kCapacity);
  }
  TEST_ASSERT_EQUAL_UINT32(0, ground.pending());
  return out;
}

// A queued Command is exactly `type` with the default canard and angle.
void expectCommandAt(int line, const Command& c, CommandType type) {
  const Command plain;
  UNITY_TEST_ASSERT_EQUAL_INT(static_cast<int>(type), static_cast<int>(c.type), line, "type");
  UNITY_TEST_ASSERT_EQUAL_INT(plain.canard, c.canard, line, "canard");
  UNITY_TEST_ASSERT_EQUAL_FLOAT(plain.angleRad, c.angleRad, line, "angle");
}
#define EXPECT_COMMAND(c, type) expectCommandAt(__LINE__, (c), (type))

void expectArmFlightAt(int line, const std::vector<Command>& q, size_t at) {
  UNITY_TEST_ASSERT(at + 2 <= q.size(), line, "armFlight queued");
  expectCommandAt(line, q[at], CommandType::EnterPreFlight);
  expectCommandAt(line, q[at + 1], CommandType::CanardsEnable);
}

// ===========================================================================
// 1. handle()
// ===========================================================================

void test_names_match_the_ground_station() {
  TEST_ASSERT_EQUAL_STRING("armFlight", GroundCommands::kArmFlight);
  TEST_ASSERT_EQUAL_STRING("actuatorTest", GroundCommands::kActuatorTest);
}

void test_arm_flight_is_accepted_only_on_the_ground() {
  const bool sweeps[] = {false, true};
  for (StateId s : kAllStates) {
    for (bool sweepActive : sweeps) {
      GroundCommands ground;
      const GroundReply reply = ground.handle(GroundCommands::kArmFlight, s, sweepActive);
      if (isGroundState(s)) {
        EXPECT_REPLY_MESSAGE(GroundReply::Ok, reply, stateName(s));
        const std::vector<Command> q = drain(ground);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, q.size(), stateName(s));
        expectArmFlightAt(__LINE__, q, 0);
      } else {
        EXPECT_REPLY_MESSAGE(GroundReply::Rejected, reply, stateName(s));
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, ground.pending(), stateName(s));
      }
    }
  }
}

void test_actuator_test_is_accepted_only_in_init_without_a_running_sweep() {
  const bool sweeps[] = {false, true};
  for (StateId s : kAllStates) {
    for (bool sweepActive : sweeps) {
      GroundCommands ground;
      const GroundReply reply = ground.handle(GroundCommands::kActuatorTest, s, sweepActive);
      if (s == StateId::Init && !sweepActive) {
        EXPECT_REPLY_MESSAGE(GroundReply::Ok, reply, stateName(s));
        const std::vector<Command> q = drain(ground);
        TEST_ASSERT_EQUAL_UINT32(1, q.size());
        EXPECT_COMMAND(q[0], CommandType::CanardSweep);
      } else {
        EXPECT_REPLY_MESSAGE(GroundReply::Rejected, reply, stateName(s));
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, ground.pending(), stateName(s));
      }
    }
  }
}

void test_unknown_names_and_null_are_unknown_and_queue_nothing() {
  const char* names[] = {"", "armflight", "ArmFlight", "armFlight ", " armFlight", "armFligh", "armFlightX",
                         "actuatortest", "actuatorTest\n", "actuator", "CanardSweep", "EnterPreFlight", "standDown"};
  for (StateId s : kAllStates) {
    GroundCommands ground;
    for (const char* name : names) {
      EXPECT_REPLY_MESSAGE(GroundReply::Unknown, ground.handle(name, s, false), name);
      EXPECT_REPLY_MESSAGE(GroundReply::Unknown, ground.handle(name, s, true), name);
    }
    EXPECT_REPLY(GroundReply::Unknown, ground.handle(nullptr, s, false));
    EXPECT_REPLY(GroundReply::Unknown, ground.handle(nullptr, s, true));
    TEST_ASSERT_EQUAL_UINT32(0, ground.pending());
    Command c;
    TEST_ASSERT_FALSE(ground.poll(c));
  }
}

void test_names_are_compared_by_content() {
  // The telemetry parser hands over its own buffer, not the constant.
  char arm[16];
  char test[16];
  std::strcpy(arm, "armFlight");
  std::strcpy(test, "actuatorTest");
  GroundCommands ground;
  EXPECT_REPLY(GroundReply::Ok, ground.handle(arm, StateId::Init, false));
  EXPECT_REPLY(GroundReply::Ok, ground.handle(test, StateId::Init, false));
  TEST_ASSERT_EQUAL_UINT32(3, ground.pending());
}

// ===========================================================================
// 2. The queue
// ===========================================================================

void test_poll_on_an_empty_queue_returns_false() {
  GroundCommands ground;
  Command c;
  TEST_ASSERT_FALSE(ground.poll(c));
  TEST_ASSERT_EQUAL_UINT32(0, ground.pending());
  // Also after everything was handed out.
  EXPECT_REPLY(GroundReply::Ok, ground.handle(GroundCommands::kActuatorTest, StateId::Init, false));
  TEST_ASSERT_TRUE(ground.poll(c));
  TEST_ASSERT_FALSE(ground.poll(c));
}

void test_commands_come_out_in_the_order_they_were_accepted() {
  GroundCommands ground;
  EXPECT_REPLY(GroundReply::Ok, ground.handle(GroundCommands::kActuatorTest, StateId::Init, false));
  EXPECT_REPLY(GroundReply::Ok, ground.handle(GroundCommands::kArmFlight, StateId::Init, false));
  EXPECT_REPLY(GroundReply::Rejected, ground.handle(GroundCommands::kArmFlight, StateId::Boost, false));
  EXPECT_REPLY(GroundReply::Unknown, ground.handle("nope", StateId::Init, false));
  EXPECT_REPLY(GroundReply::Ok, ground.handle(GroundCommands::kArmFlight, StateId::PreFlight, false));
  TEST_ASSERT_EQUAL_UINT32(5, ground.pending());

  const std::vector<Command> q = drain(ground);
  TEST_ASSERT_EQUAL_UINT32(5, q.size());
  EXPECT_COMMAND(q[0], CommandType::CanardSweep);
  expectArmFlightAt(__LINE__, q, 1);
  expectArmFlightAt(__LINE__, q, 3);
}

void test_a_full_queue_rejects_and_queues_nothing_of_that_command() {
  static_assert(GroundCommands::kCapacity % 2 == 0, "this test fills the queue with armFlight pairs");
  GroundCommands ground;
  // Fill it with armFlight (two Commands each).
  while (ground.pending() + 2 <= GroundCommands::kCapacity) {
    EXPECT_REPLY(GroundReply::Ok, ground.handle(GroundCommands::kArmFlight, StateId::Init, false));
  }
  TEST_ASSERT_EQUAL_UINT32(GroundCommands::kCapacity, ground.pending());
  EXPECT_REPLY(GroundReply::Rejected, ground.handle(GroundCommands::kActuatorTest, StateId::Init, false));
  EXPECT_REPLY(GroundReply::Rejected, ground.handle(GroundCommands::kArmFlight, StateId::Init, false));
  TEST_ASSERT_EQUAL_UINT32(GroundCommands::kCapacity, ground.pending());

  // One slot free: armFlight needs two, so none of it goes in.
  Command c;
  TEST_ASSERT_TRUE(ground.poll(c));
  EXPECT_COMMAND(c, CommandType::EnterPreFlight);
  EXPECT_REPLY(GroundReply::Rejected, ground.handle(GroundCommands::kArmFlight, StateId::PreFlight, false));
  TEST_ASSERT_EQUAL_UINT32(GroundCommands::kCapacity - 1, ground.pending());
  // But a one-Command actuatorTest fits.
  EXPECT_REPLY(GroundReply::Ok, ground.handle(GroundCommands::kActuatorTest, StateId::Init, false));
  TEST_ASSERT_EQUAL_UINT32(GroundCommands::kCapacity, ground.pending());

  // What comes out: the rest of the first pair, the other pairs, the sweep.
  const std::vector<Command> q = drain(ground);
  TEST_ASSERT_EQUAL_UINT32(GroundCommands::kCapacity, q.size());
  EXPECT_COMMAND(q[0], CommandType::CanardsEnable);
  for (size_t i = 1; i + 1 < q.size(); i += 2) expectArmFlightAt(__LINE__, q, i);
  EXPECT_COMMAND(q.back(), CommandType::CanardSweep);
}

void test_the_queue_keeps_its_order_across_the_wrap_around() {
  GroundCommands ground;
  std::vector<CommandType> expected;
  size_t next = 0;
  // Many rounds of: queue a few, take a few, so head and tail go round the
  // ring several times.
  for (int round = 0; round < 5 * static_cast<int>(GroundCommands::kCapacity); ++round) {
    if (round % 3 != 2) {
      if (ground.handle(GroundCommands::kArmFlight, StateId::Init, false) == GroundReply::Ok) {
        expected.push_back(CommandType::EnterPreFlight);
        expected.push_back(CommandType::CanardsEnable);
      }
    }
    if (round % 2 == 0) {
      if (ground.handle(GroundCommands::kActuatorTest, StateId::Init, false) == GroundReply::Ok) {
        expected.push_back(CommandType::CanardSweep);
      }
    }
    TEST_ASSERT_EQUAL_UINT32(expected.size() - next, ground.pending());
    TEST_ASSERT_TRUE(ground.pending() <= GroundCommands::kCapacity);
    for (int k = 0; k < 2; ++k) {
      Command c;
      if (!ground.poll(c)) break;
      TEST_ASSERT_TRUE(next < expected.size());
      EXPECT_COMMAND(c, expected[next]);
      ++next;
    }
  }
  for (const Command& c : drain(ground)) {
    TEST_ASSERT_TRUE(next < expected.size());
    EXPECT_COMMAND(c, expected[next]);
    ++next;
  }
  TEST_ASSERT_EQUAL_UINT32(expected.size(), next);
  TEST_ASSERT_TRUE(expected.size() > 2 * GroundCommands::kCapacity);  // it really wrapped
}

// ===========================================================================
// 3. End to end through the FlightComputer
// ===========================================================================

constexpr uint64_t kPeriodUs = 5000;
constexpr uint64_t kStartUs = 1 * fc::kUsPerSecond;
static_assert(kPeriodUs <= fc::kMaxImuGapUs, "the nominal period must not be an IMU gap");

// The VehicleConfig rows, marked calibrated whatever VehicleConfig says, so a
// moving canard shows on its pulse.
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

// A rocket at rest on the pad, nose up, with the ground station's commands
// as its CommandSource.
struct Rig {
  GroundCommands ground;
  FakeEstimator estimator;
  FakeSensorSource sensors;
  FakeActuators actuators;
  RecordingLogSink log;
  LqrAttitudeController controller{fc::kCanardLimitRad};
  FlightComputer computer;

  Rig()
      : sensors(kStartUs, kPeriodUs,
                [](uint64_t) {
                  SensorFrame f;
                  f.imuOk = true;
                  // 1 g along the nose: no launch. The mount map is a
                  // rotation, so its transpose maps body to sensor axes.
                  const Vec3 body{fc::kGravity, 0.0f, 0.0f};
                  Mat3 t;
                  for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) t.m[r][c] = VehicleConfig::kImuToBody.m[c][r];
                  }
                  f.accel = t * body;
                  return f;
                }),
        computer(FlightIo{sensors, ground, actuators, estimator, controller, log}, kCalibratedServos) {
    estimator.setInvalid();
  }

  GroundReply send(const char* name) { return ground.handle(name, computer.state(), computer.sweepActive()); }

  void cycle() { TEST_ASSERT_TRUE(computer.cycle()); }
  const CycleRecord& last() const { return log.records.back(); }
  uint64_t timeUs() const { return computer.flightTimeUs(); }
  const CanardsStatus& canards() const { return computer.canards().status(); }
};

// The canards are at sweep step `step` this cycle.
void expectSweepStepAt(int line, const Rig& rig, int step) {
  float expected[kNumCanards];
  GroundTest::sweepAngles(step, expected);
  UNITY_TEST_ASSERT(rig.canards().mode == CanardMode::Testing, line, "testing");
  for (int i = 0; i < kNumCanards; ++i) {
    UNITY_TEST_ASSERT_EQUAL_FLOAT(expected[i], rig.canards().commandRad[i], line, "sweep angle");
  }
  UNITY_TEST_ASSERT(!atCentre(rig.last().actuators), line, "moving");
  UNITY_TEST_ASSERT(rig.last().actuators.servoPower, line, "powered");
}
#define EXPECT_SWEEP_STEP(rig, step) expectSweepStepAt(__LINE__, (rig), (step))

void test_actuator_test_then_arm_flight_through_the_flight_computer() {
  Rig rig;
  for (int i = 0; i < 5; ++i) rig.cycle();
  TEST_ASSERT_EQUAL(StateId::Init, rig.computer.state());
  TEST_ASSERT_FALSE(rig.computer.sweepActive());
  TEST_ASSERT_TRUE(rig.canards().powered);
  TEST_ASSERT_TRUE(rig.canards().mode == CanardMode::Neutral);
  TEST_ASSERT_TRUE(atCentre(rig.last().actuators));

  // --- A whole sweep, in Init, never enabled --------------------------------
  EXPECT_REPLY(GroundReply::Ok, rig.send(GroundCommands::kActuatorTest));
  TEST_ASSERT_EQUAL_UINT32(1, rig.ground.pending());
  TEST_ASSERT_FALSE(rig.computer.sweepActive());  // not before the next cycle
  rig.cycle();
  TEST_ASSERT_EQUAL_UINT32(0, rig.ground.pending());
  TEST_ASSERT_EQUAL_UINT32(1, rig.last().commandCount);
  TEST_ASSERT_TRUE(rig.last().commands[0].type == CommandType::CanardSweep);
  const uint64_t startUs = rig.timeUs();
  TEST_ASSERT_TRUE(rig.computer.sweepActive());
  TEST_ASSERT_FALSE(rig.canards().enabled);
  EXPECT_SWEEP_STEP(rig, 0);

  // While it runs, another actuatorTest is refused and queues nothing.
  EXPECT_REPLY(GroundReply::Rejected, rig.send(GroundCommands::kActuatorTest));
  TEST_ASSERT_EQUAL_UINT32(0, rig.ground.pending());

  const uint64_t endUs = startUs + GroundTest::kSweepSteps * fc::kSweepStepUs;
  int lastStep = 0;
  while (rig.timeUs() + kPeriodUs < endUs) {
    rig.cycle();
    const int step = static_cast<int>((rig.timeUs() - startUs) / fc::kSweepStepUs);
    TEST_ASSERT_TRUE(step == lastStep || step == lastStep + 1);  // no step skipped
    lastStep = step;
    TEST_ASSERT_TRUE(rig.computer.sweepActive());
    EXPECT_SWEEP_STEP(rig, step);
    TEST_ASSERT_EQUAL_UINT32(0, rig.canards().deniedCount);
  }
  TEST_ASSERT_EQUAL_INT(GroundTest::kSweepSteps - 1, lastStep);
  rig.cycle();  // the end of the last step
  TEST_ASSERT_TRUE(rig.timeUs() >= endUs);
  TEST_ASSERT_FALSE(rig.computer.sweepActive());
  TEST_ASSERT_TRUE(rig.canards().mode == CanardMode::Neutral);
  TEST_ASSERT_TRUE(atCentre(rig.last().actuators));
  TEST_ASSERT_EQUAL(StateId::Init, rig.computer.state());

  // --- A second sweep, stopped by arming ------------------------------------
  EXPECT_REPLY(GroundReply::Ok, rig.send(GroundCommands::kActuatorTest));
  rig.cycle();
  const uint64_t againUs = rig.timeUs();
  TEST_ASSERT_TRUE(rig.computer.sweepActive());
  while (rig.timeUs() < againUs + 3 * fc::kSweepStepUs) rig.cycle();
  EXPECT_SWEEP_STEP(rig, 3);

  EXPECT_REPLY(GroundReply::Ok, rig.send(GroundCommands::kArmFlight));
  TEST_ASSERT_EQUAL_UINT32(2, rig.ground.pending());
  rig.cycle();
  TEST_ASSERT_EQUAL_UINT32(2, rig.last().commandCount);
  TEST_ASSERT_TRUE(rig.last().commands[0].type == CommandType::EnterPreFlight);
  TEST_ASSERT_TRUE(rig.last().commands[1].type == CommandType::CanardsEnable);
  TEST_ASSERT_EQUAL(StateId::PreFlight, rig.computer.state());
  TEST_ASSERT_TRUE(rig.last().transitioned);
  TEST_ASSERT_TRUE(rig.canards().enabled);
  TEST_ASSERT_FALSE(rig.computer.sweepActive());
  TEST_ASSERT_EQUAL_UINT32(0, rig.canards().rejectedCommandCount);

  // PreFlight's first own cycle: the estimator re-initialised (before this
  // cycle's update), the sweep is gone, the canards hold zero.
  rig.cycle();
  TEST_ASSERT_EQUAL_UINT32(1, rig.estimator.reinitializeCount());
  TEST_ASSERT_TRUE(rig.canards().request == CanardRequest::Neutral);
  TEST_ASSERT_TRUE(rig.canards().mode == CanardMode::Neutral);
  TEST_ASSERT_TRUE(rig.canards().powered);
  TEST_ASSERT_TRUE(atCentre(rig.last().actuators));
  TEST_ASSERT_TRUE(rig.canards().enabled);

  // actuatorTest is for Init only.
  EXPECT_REPLY(GroundReply::Rejected, rig.send(GroundCommands::kActuatorTest));
  TEST_ASSERT_EQUAL_UINT32(0, rig.ground.pending());
  // Quiet PreFlight cycles: no sweep comes back.
  for (int i = 0; i < 5; ++i) {
    rig.cycle();
    TEST_ASSERT_FALSE(rig.computer.sweepActive());
    TEST_ASSERT_TRUE(rig.canards().mode == CanardMode::Neutral);
  }
  // A repeated armFlight in PreFlight is accepted (re-initialises again).
  EXPECT_REPLY(GroundReply::Ok, rig.send(GroundCommands::kArmFlight));
  rig.cycle();
  rig.cycle();
  TEST_ASSERT_EQUAL(StateId::PreFlight, rig.computer.state());
  TEST_ASSERT_EQUAL_UINT32(2, rig.estimator.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(0, rig.canards().rejectedCommandCount);
}

// --- 4. commandFromLine: the USB bench console's line format ----------------

void expectCommand(const char* expected, const char* input) {
  char buf[64];
  std::strncpy(buf, input, sizeof buf - 1);
  buf[sizeof buf - 1] = '\0';
  const char* got = GroundCommands::commandFromLine(buf);
  if (expected == nullptr) {
    TEST_ASSERT_NULL_MESSAGE(got, input);
  } else {
    TEST_ASSERT_NOT_NULL_MESSAGE(got, input);
    TEST_ASSERT_EQUAL_STRING_MESSAGE(expected, got, input);
  }
}

void test_command_from_line_accepts_bare_names() {
  expectCommand("armFlight", "armFlight");
  expectCommand("actuatorTest", "  actuatorTest \r");
  expectCommand("dump 7", "dump 7");  // passed on whole: main.cpp routes it to the log download
}

void test_command_from_line_accepts_the_radio_form_with_a_good_checksum() {
  // The same lines ground/telemetry_gui.py builds (make_line("C", [name])).
  expectCommand("armFlight", "$C,armFlight*29");
  expectCommand("actuatorTest", "$C,actuatorTest*52");
  expectCommand("armFlight", "$C,armFlight*29\r");
}

void test_command_from_line_rejects_bad_radio_lines() {
  expectCommand(nullptr, "");
  expectCommand(nullptr, "   ");
  expectCommand(nullptr, "$C,armFlight*28");   // wrong checksum
  expectCommand(nullptr, "$C,armFlight");      // no checksum
  expectCommand(nullptr, "$C,armFlight*2");    // short checksum
  expectCommand(nullptr, "$C,armFlight*ZZ");   // not hex
  expectCommand(nullptr, "$D,1,2*47");         // a good line, but not a command
}

void test_command_from_line_names_still_go_through_handle() {
  char line[] = "$C,armFlight*29";
  GroundCommands ground;
  EXPECT_REPLY(GroundReply::Ok, ground.handle(GroundCommands::commandFromLine(line), StateId::Init, false));
  char unknown[] = "launchNow";
  EXPECT_REPLY(GroundReply::Unknown, ground.handle(GroundCommands::commandFromLine(unknown), StateId::Init, false));
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_names_match_the_ground_station);
  RUN_TEST(test_arm_flight_is_accepted_only_on_the_ground);
  RUN_TEST(test_actuator_test_is_accepted_only_in_init_without_a_running_sweep);
  RUN_TEST(test_unknown_names_and_null_are_unknown_and_queue_nothing);
  RUN_TEST(test_names_are_compared_by_content);
  RUN_TEST(test_poll_on_an_empty_queue_returns_false);
  RUN_TEST(test_commands_come_out_in_the_order_they_were_accepted);
  RUN_TEST(test_a_full_queue_rejects_and_queues_nothing_of_that_command);
  RUN_TEST(test_the_queue_keeps_its_order_across_the_wrap_around);
  RUN_TEST(test_actuator_test_then_arm_flight_through_the_flight_computer);
  RUN_TEST(test_command_from_line_accepts_bare_names);
  RUN_TEST(test_command_from_line_accepts_the_radio_form_with_a_good_checksum);
  RUN_TEST(test_command_from_line_rejects_bad_radio_lines);
  RUN_TEST(test_command_from_line_names_still_go_through_handle);
  return UNITY_END();
}
