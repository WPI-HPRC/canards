// The USB stream protocol's board side (StreamSession): a header line, then
// one input row per cycle in the log row format, one reply per row; comments
// ignored; anything else answered with "!ERR <reason>" and skipped.

#include <unity.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "FakeActuators.h"
#include "RecordCompare.h"
#include "ScriptedEstimator.h"
#include "ScriptedFlight.h"
#include "Subsystems/Logger/CycleLogFormat.h"
#include "Subsystems/Logger/StreamProtocol.h"

void setUp() {}
void tearDown() {}

namespace {

using namespace RecordCompare;
using Result = StreamSession::Result;

std::string header() {
  char buf[CycleLogFormat::kMaxLine];
  return std::string(buf, CycleLogFormat::formatHeader(buf, sizeof buf));
}

CycleRecord recordAt(uint64_t timeUs, uint8_t commands = 0) {
  CycleRecord r;
  r.frame.timeUs = timeUs;
  r.frame.imuOk = true;
  r.frame.accel = Vec3{0.25f, -0.5f, 9.80665f};
  r.frame.baroFresh = true;
  r.frame.pressurePa = 101234.5f;
  r.feedback.encoderValid[1] = true;
  r.feedback.encoderAngleRad[1] = 2.5f;
  r.commandCount = commands;
  for (uint8_t i = 0; i < commands; ++i) r.commands[i] = makeTestCommand(static_cast<int8_t>(i), 0.02f);
  return r;
}

std::string row(const CycleRecord& r) {
  char buf[CycleLogFormat::kMaxLine];
  return std::string(buf, CycleLogFormat::formatRow(r, buf, sizeof buf));
}

std::string errorReply(const StreamSession& s) {
  char buf[128];
  return std::string(buf, s.replyError(buf, sizeof buf));
}

void test_header_then_rows() {
  StreamSession s;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Header), static_cast<int>(s.feedLine(header().c_str())));
  TEST_ASSERT_TRUE(s.headerOk());

  // The reply to the header is the board's own log header.
  char buf[CycleLogFormat::kMaxLine];
  TEST_ASSERT_EQUAL_STRING(header().c_str(), (StreamSession::replyHeader(buf, sizeof buf), buf));

  const CycleRecord r = recordAt(123456789012ull, 2);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Row), static_cast<int>(s.feedLine(row(r).c_str())));
  TEST_ASSERT_TRUE(s.rowPending());

  SensorFrame f;
  TEST_ASSERT_TRUE(s.read(f));
  TEST_ASSERT_TRUE(sameFrame(r.frame, f));
  TEST_ASSERT_FALSE(s.rowPending());
  TEST_ASSERT_FALSE(s.read(f));  // one cycle per row

  Command c;
  TEST_ASSERT_TRUE(s.poll(c));
  TEST_ASSERT_TRUE(sameCommand(r.commands[0], c));
  TEST_ASSERT_TRUE(s.poll(c));
  TEST_ASSERT_FALSE(s.poll(c));
  ActuatorFeedback fb;
  s.readFeedback(fb);
  TEST_ASSERT_TRUE(sameFeedback(r.feedback, fb));
  TEST_ASSERT_EQUAL_UINT32(1, s.rowsAccepted());
  TEST_ASSERT_EQUAL_UINT32(0, s.errors());
}

void test_comments_and_blank_lines_get_no_reply() {
  StreamSession s;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Ignored), static_cast<int>(s.feedLine("# hello\n")));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Ignored), static_cast<int>(s.feedLine("   #x")));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Ignored), static_cast<int>(s.feedLine("")));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Ignored), static_cast<int>(s.feedLine("\r\n")));
  TEST_ASSERT_FALSE(s.headerOk());
  TEST_ASSERT_EQUAL_UINT32(0, s.errors());
}

void test_errors_are_answered_and_skipped() {
  StreamSession s;
  const std::string r1 = row(recordAt(1000));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Error), static_cast<int>(s.feedLine(r1.c_str())));
  TEST_ASSERT_EQUAL_STRING("!ERR no header\n", errorReply(s).c_str());

  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Error), static_cast<int>(s.feedLine("t_us,imu_ok\n")));
  TEST_ASSERT_EQUAL_STRING("!ERR bad header\n", errorReply(s).c_str());
  TEST_ASSERT_FALSE(s.headerOk());

  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Error), static_cast<int>(s.feedLine("!what\n")));
  TEST_ASSERT_EQUAL_STRING("!ERR unknown line\n", errorReply(s).c_str());

  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Header), static_cast<int>(s.feedLine(header().c_str())));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Error), static_cast<int>(s.feedLine("12,1,abc\n")));
  TEST_ASSERT_EQUAL_STRING("!ERR bad row\n", errorReply(s).c_str());

  // A row too long for a line is a bad row, not a crash.
  std::string huge(3 * CycleLogFormat::kMaxLine, '1');
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Error), static_cast<int>(s.feedLine(huge.c_str())));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Row), static_cast<int>(s.feedLine(r1.c_str())));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Error), static_cast<int>(s.feedLine(r1.c_str())));
  TEST_ASSERT_EQUAL_STRING("!ERR busy\n", errorReply(s).c_str());

  // The good row is still there for its cycle.
  SensorFrame f;
  TEST_ASSERT_TRUE(s.read(f));
  TEST_ASSERT_EQUAL_UINT64(1000, f.timeUs);
  TEST_ASSERT_EQUAL_UINT32(6, s.errors());
  TEST_ASSERT_EQUAL_UINT32(1, s.rowsAccepted());
}

void test_columns_are_found_by_name_and_a_new_header_replaces_the_old() {
  StreamSession s;
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Header),
                        static_cast<int>(s.feedLine(("sim_step," + header()).c_str())));
  const CycleRecord r = recordAt(4242, 1);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Row), static_cast<int>(s.feedLine(("7," + row(r)).c_str())));
  SensorFrame f;
  TEST_ASSERT_TRUE(s.read(f));
  TEST_ASSERT_TRUE(sameFrame(r.frame, f));

  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Header), static_cast<int>(s.feedLine(header().c_str())));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Result::Row), static_cast<int>(s.feedLine(row(r).c_str())));
  TEST_ASSERT_TRUE(s.read(f));
  TEST_ASSERT_TRUE(sameFrame(r.frame, f));
}

void test_error_reply_never_overflows() {
  char small[6];
  TEST_ASSERT_EQUAL_UINT32(0, StreamProtocol::formatError("bad row", small, sizeof small));
  char ok[16];
  TEST_ASSERT_EQUAL_UINT32(13, StreamProtocol::formatError("bad row", ok, sizeof ok));
  TEST_ASSERT_EQUAL_STRING("!ERR bad row\n", ok);
}

// A recorded flight streamed as-is (outputs included, which are ignored)
// through a fresh FlightComputer: one reply per row, and each reply is the
// full log row of that cycle, identical to the original's.
void test_streaming_a_recorded_log_replies_with_each_cycles_full_row() {
  using namespace ScriptedFlight;
  Recording rec(20000, 1);
  rec.run();

  FakeActuators servos;
  StreamSession session(&servos);
  ScriptedEstimator estimator([&rec](const EstimatorInput& in) { return truthFor(rec.script, in); });
  LqrAttitudeController controller{fc::kCanardLimitRad};
  NullLogSink log;
  const ServoConfig config = calibratedServos();
  FlightComputer computer(FlightIo{session, session, session, estimator, controller, log}, config);

  std::vector<std::string> replies;
  char reply[CycleLogFormat::kMaxLine];
  std::vector<std::string> input = rec.log.lines;
  input.insert(input.begin(), "# streamed by the test\n");
  for (const std::string& line : input) {
    switch (session.feedLine(line.c_str())) {
      case Result::Ignored:
        break;
      case Result::Header:
        replies.emplace_back(reply, StreamSession::replyHeader(reply, sizeof reply));
        break;
      case Result::Row:
        TEST_ASSERT_TRUE(computer.cycle());
        replies.emplace_back(reply, StreamSession::replyRow(computer.lastRecord(), reply, sizeof reply));
        break;
      case Result::Error:
        TEST_FAIL_MESSAGE(session.error());
        break;
    }
    TEST_ASSERT_FALSE(computer.cycle());  // nothing runs without a row
  }

  TEST_ASSERT_EQUAL_UINT32(rec.log.lines.size(), replies.size());
  for (size_t i = 0; i < replies.size(); ++i) {
    TEST_ASSERT_EQUAL_STRING(rec.log.lines[i].c_str(), replies[i].c_str());
  }
  TEST_ASSERT_EQUAL_UINT32(rec.log.records.size(), servos.writeCount());
  TEST_ASSERT_TRUE(sameActuators(rec.log.records.back().actuators, servos.last()));
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_header_then_rows);
  RUN_TEST(test_comments_and_blank_lines_get_no_reply);
  RUN_TEST(test_errors_are_answered_and_skipped);
  RUN_TEST(test_columns_are_found_by_name_and_a_new_header_replaces_the_old);
  RUN_TEST(test_error_reply_never_overflows);
  RUN_TEST(test_streaming_a_recorded_log_replies_with_each_cycles_full_row);
  return UNITY_END();
}
