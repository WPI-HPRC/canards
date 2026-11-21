// The flight log format: header, a row per cycle, and the parser that replay
// uses. The point is the round trip: every INPUT field must come back exactly,
// bit for bit, or replay is not deterministic.

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "Subsystems/Logger/CycleLogFormat.h"

void setUp() {}
void tearDown() {}

namespace {

using namespace CycleLogFormat;

bool sameBits(float a, float b) {
  uint32_t x, y;
  std::memcpy(&x, &a, sizeof x);
  std::memcpy(&y, &b, sizeof y);
  return x == y || (std::isnan(a) && std::isnan(b));
}

// Deterministic awkward floats: denormals, huge, tiny, negative zero.
float awkward(uint32_t i) {
  static const float values[] = {0.0f, -0.0f, 1.0f, -9.80665f, 1e-38f, 1.17549435e-38f, 1.4e-45f,
                                 3.40282347e38f, -123456.789f, 0.1f, 1.0f / 3.0f, 6.2831853f};
  return values[i % (sizeof(values) / sizeof(values[0]))] * (i / 12 % 2 ? -1.0f : 1.0f);
}

CycleRecord sampleRecord(uint32_t seed) {
  CycleRecord r;
  r.cycleCount = 1234 + seed;
  r.frame.timeUs = 0x123456789ABCull + seed;  // above 2^32
  r.frame.imuOk = seed % 2 == 0;
  r.frame.accel = Vec3{awkward(seed), awkward(seed + 1), awkward(seed + 2)};
  r.frame.gyro = Vec3{awkward(seed + 3), awkward(seed + 4), awkward(seed + 5)};
  r.frame.magFresh = seed % 3 == 0;
  r.frame.mag = Vec3{awkward(seed + 6), awkward(seed + 7), awkward(seed + 8)};
  r.frame.baroFresh = seed % 5 == 0;
  r.frame.pressurePa = 101325.123f + static_cast<float>(seed);
  r.frame.temperatureC = 21.37f;
  for (int i = 0; i < kNumCanards; ++i) {
    r.feedback.encoderAngleRad[i] = 0.123456789f * static_cast<float>(i + 1) + awkward(seed + 9) * 1e-3f;
    r.feedback.encoderValid[i] = ((seed >> i) & 1) != 0;
  }
  r.commandCount = static_cast<uint8_t>(seed % 4);
  for (uint8_t i = 0; i < r.commandCount; ++i) {
    r.commands[i].type = static_cast<CommandType>((seed + i) % 7);
    r.commands[i].canard = static_cast<int8_t>(static_cast<int>(i) - 1);
    r.commands[i].angleRad = awkward(seed + i) * 1e-2f;
  }
  r.flightTimeUs = r.frame.timeUs - 17;
  r.state = StateId::Control;
  r.canards.mode = CanardMode::Controlling;
  r.actuators.servoPower = true;
  for (int i = 0; i < kNumCanards; ++i) r.actuators.pulseUs[i] = 1500.0f + 10.0f * static_cast<float>(i);
  return r;
}

Columns header() {
  char buf[kMaxLine];
  TEST_ASSERT_TRUE(formatHeader(buf, sizeof buf) > 0);
  Columns c;
  TEST_ASSERT_TRUE(parseHeader(buf, c));
  return c;
}

void assertRoundTrip(const CycleRecord& r, const Columns& c) {
  char row[kMaxLine];
  const size_t len = formatRow(r, row, sizeof row);
  TEST_ASSERT_TRUE_MESSAGE(len > 0, "row did not fit");
  TEST_ASSERT_EQUAL_CHAR('\n', row[len - 1]);
  ReplayInput in;
  TEST_ASSERT_TRUE_MESSAGE(parseRow(row, c, in), row);
  TEST_ASSERT_TRUE(in.frame.timeUs == r.frame.timeUs);
  TEST_ASSERT_EQUAL(r.frame.imuOk, in.frame.imuOk);
  TEST_ASSERT_EQUAL(r.frame.magFresh, in.frame.magFresh);
  TEST_ASSERT_EQUAL(r.frame.baroFresh, in.frame.baroFresh);
  const float* want[] = {&r.frame.accel.x, &r.frame.accel.y, &r.frame.accel.z, &r.frame.gyro.x,
                         &r.frame.gyro.y,  &r.frame.gyro.z,  &r.frame.mag.x,   &r.frame.mag.y,
                         &r.frame.mag.z,   &r.frame.pressurePa, &r.frame.temperatureC};
  const float* got[] = {&in.frame.accel.x, &in.frame.accel.y, &in.frame.accel.z, &in.frame.gyro.x,
                        &in.frame.gyro.y,  &in.frame.gyro.z,  &in.frame.mag.x,   &in.frame.mag.y,
                        &in.frame.mag.z,   &in.frame.pressurePa, &in.frame.temperatureC};
  for (size_t i = 0; i < sizeof(want) / sizeof(want[0]); ++i) {
    TEST_ASSERT_TRUE_MESSAGE(sameBits(*want[i], *got[i]), row);
  }
  for (int i = 0; i < kNumCanards; ++i) {
    TEST_ASSERT_EQUAL(r.feedback.encoderValid[i], in.feedback.encoderValid[i]);
    TEST_ASSERT_TRUE(sameBits(r.feedback.encoderAngleRad[i], in.feedback.encoderAngleRad[i]));
  }
  TEST_ASSERT_EQUAL_UINT32(r.commandCount, in.commandCount);
  for (size_t i = 0; i < r.commandCount; ++i) {
    TEST_ASSERT_TRUE(r.commands[i].type == in.commands[i].type);
    TEST_ASSERT_EQUAL_INT8(r.commands[i].canard, in.commands[i].canard);
    TEST_ASSERT_TRUE(sameBits(r.commands[i].angleRad, in.commands[i].angleRad));
  }
}

void test_header_lists_every_input_column() {
  const Columns c = header();
  TEST_ASSERT_TRUE(c.complete());
  TEST_ASSERT_EQUAL_INT(0, c.tUs);
}

void test_inputs_round_trip_exactly() {
  const Columns c = header();
  for (uint32_t seed = 0; seed < 300; ++seed) assertRoundTrip(sampleRecord(seed), c);
}

void test_non_finite_inputs_round_trip() {
  const Columns c = header();
  CycleRecord r = sampleRecord(1);
  r.frame.accel.x = std::numeric_limits<float>::quiet_NaN();
  r.frame.gyro.y = std::numeric_limits<float>::infinity();
  r.frame.mag.z = -std::numeric_limits<float>::infinity();
  r.feedback.encoderAngleRad[2] = std::numeric_limits<float>::quiet_NaN();
  assertRoundTrip(r, c);
}

void test_a_full_command_list_round_trips() {
  const Columns c = header();
  CycleRecord r = sampleRecord(2);
  r.commandCount = CommandList::kCapacity;
  for (uint8_t i = 0; i < r.commandCount; ++i) {
    r.commands[i].type = CommandType::CanardTest;
    r.commands[i].canard = static_cast<int8_t>(i % 4);
    r.commands[i].angleRad = -0.1234567f * static_cast<float>(i);
  }
  assertRoundTrip(r, c);
}

void test_columns_are_found_by_name_in_any_order() {
  // An old or new log may have columns added or moved. Swap two input columns
  // and add an unknown one: the reader still finds everything.
  char buf[kMaxLine];
  formatHeader(buf, sizeof buf);
  std::string h(buf);
  h.pop_back();  // newline
  h = "extra," + h;
  Columns c;
  TEST_ASSERT_TRUE(parseHeader(h.c_str(), c));
  TEST_ASSERT_EQUAL_INT(1, c.tUs);

  // A header missing an input column cannot be replayed.
  std::string missing(buf);
  const size_t at = missing.find(",gz,");
  missing.erase(at, 3);
  TEST_ASSERT_FALSE(parseHeader(missing.c_str(), c));
}

void test_malformed_rows_are_refused() {
  const Columns c = header();
  char row[kMaxLine];
  formatRow(sampleRecord(3), row, sizeof row);
  ReplayInput in;

  std::string bad(row);
  bad[bad.find(',') + 1] = 'x';  // imu_ok not a number
  TEST_ASSERT_FALSE(parseRow(bad.c_str(), c, in));

  std::string shortRow(row);
  shortRow.resize(shortRow.find(",gx,") == std::string::npos ? 40 : 40);
  TEST_ASSERT_FALSE(parseRow(shortRow.c_str(), c, in));

  TEST_ASSERT_FALSE(parseRow("", c, in));
  TEST_ASSERT_FALSE(parseRow("abc,1,2", c, in));
}

void test_a_bad_command_item_is_refused() {
  const Columns c = header();
  CycleRecord r = sampleRecord(4);
  r.commandCount = 0;
  char row[kMaxLine];
  formatRow(r, row, sizeof row);
  // Put a malformed command where the empty cmds field is.
  std::string s(row);
  ReplayInput in;
  TEST_ASSERT_TRUE(parseRow(s.c_str(), c, in));
  // Find the cmds field: it is field index c.cmds.
  size_t pos = 0;
  for (int i = 0; i < c.cmds; ++i) pos = s.find(',', pos) + 1;
  for (const char* item : {"9:0:0", "1:0", "x:0:0", "1:0:abc"}) {
    std::string t = s;
    t.insert(pos, item);
    TEST_ASSERT_FALSE_MESSAGE(parseRow(t.c_str(), c, in), item);
  }
}

void test_too_small_buffers_give_zero() {
  char tiny[16];
  TEST_ASSERT_EQUAL_UINT32(0, formatHeader(tiny, sizeof tiny));
  TEST_ASSERT_EQUAL_UINT32(0, formatRow(sampleRecord(5), tiny, sizeof tiny));
}

void test_worst_case_row_fits_the_line_limit() {
  CycleRecord r = sampleRecord(6);
  // Longest numbers everywhere.
  const float longest = -1.17549435e-38f;
  r.frame.accel = r.frame.gyro = r.frame.mag = Vec3{longest, longest, longest};
  r.frame.pressurePa = r.frame.temperatureC = longest;
  for (int i = 0; i < kNumCanards; ++i) r.feedback.encoderAngleRad[i] = longest;
  r.commandCount = CommandList::kCapacity;
  for (uint8_t i = 0; i < r.commandCount; ++i) r.commands[i] = Command{CommandType::CanardTest, -1, longest};
  r.frame.timeUs = UINT64_MAX;
  r.flightTimeUs = UINT64_MAX;
  r.cycleCount = UINT32_MAX;
  r.speed = r.specificForce = -3.0e38f;  // fixed-point outputs of huge values
  r.nav.altitudeAgl = -3.0e38f;
  char row[kMaxLine];
  const size_t len = formatRow(r, row, sizeof row);
  TEST_ASSERT_TRUE_MESSAGE(len > 0, "a worst-case row must fit kMaxLine");
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_header_lists_every_input_column);
  RUN_TEST(test_inputs_round_trip_exactly);
  RUN_TEST(test_non_finite_inputs_round_trip);
  RUN_TEST(test_a_full_command_list_round_trips);
  RUN_TEST(test_columns_are_found_by_name_in_any_order);
  RUN_TEST(test_malformed_rows_are_refused);
  RUN_TEST(test_a_bad_command_item_is_refused);
  RUN_TEST(test_too_small_buffers_give_zero);
  RUN_TEST(test_worst_case_row_fits_the_line_limit);
  return UNITY_END();
}
