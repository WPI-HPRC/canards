// LogReplay: a flight log read back as the FlightComputer's sensor source,
// command source and encoder feedback (docs/HANDOFF-flight-skeleton-2026-11-20.md,
// section 7). The key test is end-to-end determinism: a scripted flight is
// logged through CycleLogFormat, then replayed through a FRESH FlightComputer,
// and every output of every cycle must come out identical.
//
// Set SAMPLE_LOG_OUT=<path> to also write the short flight's log to <path>
// (tools/testdata/sample_log.csv is made this way).

#include <unity.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "FakeActuators.h"
#include "FakeCommandSource.h"
#include "RecordCompare.h"
#include "ScriptedEstimator.h"
#include "ScriptedFlight.h"
#include "Subsystems/Logger/CycleLogFormat.h"
#include "Subsystems/Logger/LogReplay.h"
#include "VectorLineReader.h"

void setUp() {}
void tearDown() {}

namespace {

using namespace RecordCompare;

std::string header() {
  char buf[CycleLogFormat::kMaxLine];
  return std::string(buf, CycleLogFormat::formatHeader(buf, sizeof buf));
}

CycleRecord recordAt(uint64_t timeUs, uint8_t commands = 0) {
  CycleRecord r;
  r.frame.timeUs = timeUs;
  r.frame.imuOk = true;
  r.frame.accel = Vec3{static_cast<float>(timeUs) * 1e-3f, 0.1f, -9.80665f};
  r.frame.pressurePa = 100000.0f + static_cast<float>(timeUs % 1000);
  for (int ch = 0; ch < kNumCanards; ++ch) {
    r.feedback.encoderValid[ch] = ch % 2 == 0;
    r.feedback.encoderAngleRad[ch] = 0.5f * static_cast<float>(ch) + static_cast<float>(timeUs % 100) * 1e-3f;
  }
  r.commandCount = commands;
  for (uint8_t i = 0; i < commands; ++i) r.commands[i] = makeTestCommand(static_cast<int8_t>(i % 4), 0.01f * i);
  return r;
}

std::string row(const CycleRecord& r) {
  char buf[CycleLogFormat::kMaxLine];
  return std::string(buf, CycleLogFormat::formatRow(r, buf, sizeof buf));
}

// Releases a row only once `nowUs` (a fake wall clock, relative to the first
// row) has reached it. Remembers what it was asked.
class FakeClockRelease final : public ReleasePolicy {
 public:
  uint64_t nowUs = 0;
  uint64_t lastRowUs = 0, lastFirstUs = 0;
  int calls = 0;
  bool release(uint64_t rowTimeUs, uint64_t firstRowTimeUs) override {
    ++calls;
    lastRowUs = rowTimeUs;
    lastFirstUs = firstRowTimeUs;
    return rowTimeUs - firstRowTimeUs <= nowUs;
  }
};

ReleaseImmediately immediately;

// ---------------------------------------------------------------------------

void test_missing_header_is_end_of_log() {
  VectorLineReader empty;
  LogReplay a(empty, immediately);
  SensorFrame f;
  TEST_ASSERT_FALSE(a.read(f));
  TEST_ASSERT_FALSE(a.headerOk());
  TEST_ASSERT_TRUE(a.endOfLog());

  // A row where the header should be: not a header, nothing replays.
  VectorLineReader noHeader({row(recordAt(1000)), row(recordAt(2000))});
  LogReplay b(noHeader, immediately);
  TEST_ASSERT_FALSE(b.begin());
  TEST_ASSERT_FALSE(b.read(f));
  TEST_ASSERT_TRUE(b.endOfLog());
  TEST_ASSERT_EQUAL_UINT32(0, b.rowsRead());

  // A header missing a needed input column.
  VectorLineReader badHeader({"t_us,imu_ok,ax\n", row(recordAt(1000))});
  LogReplay c(badHeader, immediately);
  TEST_ASSERT_FALSE(c.read(f));
  TEST_ASSERT_FALSE(c.headerOk());
  TEST_ASSERT_TRUE(c.endOfLog());
}

void test_comments_and_blank_lines_before_the_header_are_skipped() {
  VectorLineReader reader({"# replay of flight 3\n", "\n", "  \r\n", header(), row(recordAt(5000))});
  LogReplay replay(reader, immediately);
  TEST_ASSERT_TRUE(replay.begin());
  SensorFrame f;
  TEST_ASSERT_TRUE(replay.read(f));
  TEST_ASSERT_EQUAL_UINT64(5000, f.timeUs);
  TEST_ASSERT_EQUAL_UINT32(0, replay.malformedRows());
}

void test_rows_come_out_in_order_then_end_of_log() {
  std::vector<CycleRecord> records = {recordAt(1000), recordAt(2000), recordAt(0x1'0000'1000ull)};
  VectorLineReader reader({header()});
  for (const CycleRecord& r : records) reader.lines.push_back(row(r));
  LogReplay replay(reader, immediately);

  for (size_t i = 0; i < records.size(); ++i) {
    SensorFrame f;
    TEST_ASSERT_TRUE(replay.read(f));
    TEST_ASSERT_TRUE(sameFrame(records[i].frame, f));
    TEST_ASSERT_FALSE(replay.endOfLog());
  }
  SensorFrame f;
  f.timeUs = 77;
  TEST_ASSERT_FALSE(replay.read(f));
  TEST_ASSERT_EQUAL_UINT64(77, f.timeUs);  // left alone
  TEST_ASSERT_TRUE(replay.endOfLog());
  TEST_ASSERT_EQUAL_UINT32(3, replay.rowsRead());
  TEST_ASSERT_FALSE(replay.read(f));
}

void test_commands_are_delivered_on_the_cycle_of_their_row() {
  VectorLineReader reader({header(), row(recordAt(1000, 2)), row(recordAt(2000, 0)), row(recordAt(3000, 8)),
                           row(recordAt(4000, 1))});
  LogReplay replay(reader, immediately);
  SensorFrame f;
  Command c;

  TEST_ASSERT_FALSE(replay.poll(c));  // nothing before the first row

  TEST_ASSERT_TRUE(replay.read(f));
  TEST_ASSERT_TRUE(replay.poll(c));
  TEST_ASSERT_TRUE(sameCommand(makeTestCommand(0, 0.0f), c));
  TEST_ASSERT_TRUE(replay.poll(c));
  TEST_ASSERT_TRUE(sameCommand(makeTestCommand(1, 0.01f), c));
  TEST_ASSERT_FALSE(replay.poll(c));

  TEST_ASSERT_TRUE(replay.read(f));
  TEST_ASSERT_FALSE(replay.poll(c));

  TEST_ASSERT_TRUE(replay.read(f));
  for (int i = 0; i < 8; ++i) TEST_ASSERT_TRUE(replay.poll(c));
  TEST_ASSERT_FALSE(replay.poll(c));

  // Commands not polled during their cycle do not leak into the next one.
  VectorLineReader reader2({header(), row(recordAt(1000, 3)), row(recordAt(2000, 1))});
  LogReplay replay2(reader2, immediately);
  TEST_ASSERT_TRUE(replay2.read(f));
  TEST_ASSERT_TRUE(replay2.poll(c));
  TEST_ASSERT_TRUE(replay2.read(f));
  TEST_ASSERT_TRUE(replay2.poll(c));
  TEST_ASSERT_TRUE(sameCommand(makeTestCommand(0, 0.0f), c));
  TEST_ASSERT_FALSE(replay2.poll(c));
}

void test_feedback_follows_the_row_and_writes_are_forwarded_or_discarded() {
  const CycleRecord r1 = recordAt(1000), r2 = recordAt(2042);
  VectorLineReader reader({header(), row(r1), row(r2)});
  FakeActuators servos;
  LogReplay replay(reader, immediately, &servos);

  ActuatorFeedback fb;
  fb.encoderValid[0] = true;
  replay.readFeedback(fb);
  TEST_ASSERT_TRUE(sameFeedback(ActuatorFeedback{}, fb));  // nothing valid before the first row

  SensorFrame f;
  TEST_ASSERT_TRUE(replay.read(f));
  replay.readFeedback(fb);
  TEST_ASSERT_TRUE(sameFeedback(r1.feedback, fb));
  TEST_ASSERT_TRUE(replay.read(f));
  replay.readFeedback(fb);
  TEST_ASSERT_TRUE(sameFeedback(r2.feedback, fb));

  ActuatorCommand out;
  out.servoPower = true;
  out.pulseUs[2] = 1612.5f;
  replay.write(out);
  TEST_ASSERT_EQUAL_UINT32(1, servos.writeCount());
  TEST_ASSERT_TRUE(sameActuators(out, servos.last()));

  replay.setOutput(nullptr);
  replay.write(out);  // discarded
  TEST_ASSERT_EQUAL_UINT32(1, servos.writeCount());
}

void test_malformed_rows_are_skipped_and_counted() {
  // Cut inside the inputs. (A row cut among the OUTPUT columns still
  // replays: replay reads only the inputs.)
  std::string truncated = row(recordAt(3000));
  truncated.resize(truncated.find(',', truncated.find(',') + 1) + 3);
  std::string badCommand = row(recordAt(4000));
  // The cmds field is the 21st column; corrupt it to a bad command.
  {
    size_t pos = 0;
    for (int i = 0; i < 20; ++i) pos = badCommand.find(',', pos) + 1;
    badCommand.insert(pos, "9:0:0");
  }
  VectorLineReader reader({header(), row(recordAt(1000)), "garbage\n", truncated, "1x,2,3\n", badCommand,
                           "# a comment is not malformed\n", row(recordAt(5000))});
  LogReplay replay(reader, immediately);
  SensorFrame f;
  TEST_ASSERT_TRUE(replay.read(f));
  TEST_ASSERT_EQUAL_UINT64(1000, f.timeUs);
  TEST_ASSERT_TRUE(replay.read(f));
  TEST_ASSERT_EQUAL_UINT64(5000, f.timeUs);
  TEST_ASSERT_EQUAL_UINT32(4, replay.malformedRows());
  TEST_ASSERT_EQUAL_UINT32(2, replay.rowsRead());
  TEST_ASSERT_FALSE(replay.read(f));
  TEST_ASSERT_TRUE(replay.endOfLog());
}

void test_a_long_run_of_bad_lines_does_not_stall_one_read_nor_end_the_log() {
  VectorLineReader reader({header()});
  for (int i = 0; i < LogReplay::kMaxLinesPerRead + 4; ++i) reader.lines.push_back("bad\n");
  reader.lines.push_back(row(recordAt(9000)));
  LogReplay replay(reader, immediately);
  SensorFrame f;
  TEST_ASSERT_FALSE(replay.read(f));  // gave up for this call
  TEST_ASSERT_FALSE(replay.endOfLog());
  TEST_ASSERT_TRUE(replay.read(f));
  TEST_ASSERT_EQUAL_UINT64(9000, f.timeUs);
  TEST_ASSERT_EQUAL_UINT32(LogReplay::kMaxLinesPerRead + 4, replay.malformedRows());
}

void test_the_release_policy_gates_each_row() {
  VectorLineReader reader({header(), row(recordAt(10000)), row(recordAt(15000)), row(recordAt(30000))});
  FakeClockRelease clock;
  LogReplay replay(reader, clock);
  SensorFrame f;

  TEST_ASSERT_TRUE(replay.read(f));  // the first row is due at once
  TEST_ASSERT_EQUAL_UINT64(10000, f.timeUs);
  TEST_ASSERT_EQUAL_UINT64(10000, clock.lastFirstUs);

  clock.nowUs = 4999;
  TEST_ASSERT_FALSE(replay.read(f));
  TEST_ASSERT_FALSE(replay.read(f));
  TEST_ASSERT_EQUAL_UINT64(15000, clock.lastRowUs);
  TEST_ASSERT_EQUAL_UINT32(1, replay.rowsRead());
  const size_t linesRead = reader.linesRead();
  TEST_ASSERT_FALSE(replay.read(f));
  TEST_ASSERT_EQUAL_UINT32(linesRead, reader.linesRead());  // a held row is not read twice

  clock.nowUs = 5000;
  TEST_ASSERT_TRUE(replay.read(f));
  TEST_ASSERT_EQUAL_UINT64(15000, f.timeUs);

  clock.nowUs = 1000000;  // far behind: rows come one per read(), never skipped
  TEST_ASSERT_TRUE(replay.read(f));
  TEST_ASSERT_EQUAL_UINT64(30000, f.timeUs);
  TEST_ASSERT_FALSE(replay.read(f));
  TEST_ASSERT_TRUE(replay.endOfLog());
}

// --- End to end -----------------------------------------------------------------

// Replays `lines` through a fresh FlightComputer with the same script's
// estimator and checks every record against `original`.
void assertReplayReproduces(const ScriptedFlight::Script& script, const std::vector<std::string>& lines,
                            const std::vector<CycleRecord>& original) {
  using namespace ScriptedFlight;
  VectorLineReader reader(lines);
  FakeActuators servos;
  LogReplay replay(reader, immediately, &servos);
  ScriptedEstimator estimator([&script](const EstimatorInput& in) { return truthFor(script, in); });
  LqrAttitudeController controller{fc::kCanardLimitRad};
  FormattingLogSink log;
  const ServoConfig config = calibratedServos();
  FlightComputer computer(FlightIo{replay, replay, replay, estimator, controller, log}, config);

  while (computer.cycle()) {
  }
  TEST_ASSERT_TRUE(replay.endOfLog());
  TEST_ASSERT_EQUAL_UINT32(0, replay.malformedRows());
  TEST_ASSERT_EQUAL_UINT32(original.size(), replay.rowsRead());
  TEST_ASSERT_EQUAL_UINT32(original.size(), log.records.size());
  TEST_ASSERT_EQUAL_UINT32(original.size(), servos.writeCount());

  for (size_t i = 0; i < original.size(); ++i) {
    const CycleRecord& a = original[i];
    const CycleRecord& b = log.records[i];
    char msg[64];
    std::snprintf(msg, sizeof msg, "cycle %zu", i + 1);
    TEST_ASSERT_TRUE_MESSAGE(sameInputs(a, b), msg);
    TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(a.state), static_cast<int>(b.state), msg);
    TEST_ASSERT_EQUAL_MESSAGE(a.transitioned, b.transitioned, msg);
    TEST_ASSERT_TRUE_MESSAGE(sameStatus(a.canards, b.canards), msg);
    TEST_ASSERT_TRUE_MESSAGE(sameActuators(a.actuators, b.actuators), msg);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(a.safetyActive, b.safetyActive, msg);
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(a.safetySticky, b.safetySticky, msg);
    TEST_ASSERT_TRUE_MESSAGE(sameOutputs(a, b), msg);
    TEST_ASSERT_EQUAL_STRING_MESSAGE(lines[i + 1].c_str(), log.lines[i + 1].c_str(), msg);
  }
  TEST_ASSERT_TRUE(sameActuators(original.back().actuators, servos.last()));
}

// The recording must actually exercise the flight program, or determinism
// proves little.
void assertFlightWasInteresting(const std::vector<CycleRecord>& records) {
  bool seen[kNumStates] = {};
  bool testing = false, controlling = false, eightCommands = false, moved = false;
  for (const CycleRecord& r : records) {
    seen[static_cast<int>(r.state)] = true;
    testing = testing || r.canards.mode == CanardMode::Testing;
    controlling = controlling || r.canards.mode == CanardMode::Controlling;
    eightCommands = eightCommands || r.commandCount == CommandList::kCapacity;
    for (int i = 0; i < kNumCanards; ++i) moved = moved || r.canards.commandRad[i] != 0.0f;
  }
  for (size_t s = 0; s < kNumStates; ++s) TEST_ASSERT_TRUE_MESSAGE(seen[s], "a state was never reached");
  TEST_ASSERT_TRUE(testing);
  TEST_ASSERT_TRUE(controlling);
  TEST_ASSERT_TRUE(eightCommands);
  TEST_ASSERT_TRUE(moved);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StateId::Land), static_cast<int>(records.back().state));
}

void test_replay_reproduces_every_output_of_a_recorded_flight() {
  ScriptedFlight::Recording rec(5000, 5);
  rec.run();
  assertFlightWasInteresting(rec.log.records);
  assertReplayReproduces(rec.script, rec.log.lines, rec.log.records);
}

void test_short_flight_replays_and_can_be_written_as_the_tools_fixture() {
  ScriptedFlight::Recording rec(20000, 1);
  rec.run();
  assertFlightWasInteresting(rec.log.records);
  TEST_ASSERT_TRUE(rec.log.records.size() < 200);
  assertReplayReproduces(rec.script, rec.log.lines, rec.log.records);

  if (const char* path = std::getenv("SAMPLE_LOG_OUT")) {
    FILE* f = std::fopen(path, "w");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, path);
    for (const std::string& line : rec.log.lines) std::fputs(line.c_str(), f);
    std::fclose(f);
  }
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_missing_header_is_end_of_log);
  RUN_TEST(test_comments_and_blank_lines_before_the_header_are_skipped);
  RUN_TEST(test_rows_come_out_in_order_then_end_of_log);
  RUN_TEST(test_commands_are_delivered_on_the_cycle_of_their_row);
  RUN_TEST(test_feedback_follows_the_row_and_writes_are_forwarded_or_discarded);
  RUN_TEST(test_malformed_rows_are_skipped_and_counted);
  RUN_TEST(test_a_long_run_of_bad_lines_does_not_stall_one_read_nor_end_the_log);
  RUN_TEST(test_the_release_policy_gates_each_row);
  RUN_TEST(test_replay_reproduces_every_output_of_a_recorded_flight);
  RUN_TEST(test_short_flight_replays_and_can_be_written_as_the_tools_fixture);
  return UNITY_END();
}
