#pragma once

// The flight log's text format: one CSV row per cycle, with a header row.
// Used by the SD log sink to write it, and by SD replay and tools/ to read it
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 7).
//
// INPUTS come first and are written losslessly (floats with 9 significant
// digits, which round-trip a float exactly), so a flight can be replayed
// bit-for-bit through new code:
//   t_us, imu_ok, ax ay az [m/s^2], gx gy gz [rad/s]    (IMU sensor axes)
//   mag_new, mx my mz [uT]                              (mag sensor axes)
//   baro_new, p_pa [Pa], temp_c
//   enc_ok (bit n = channel n valid), enc0..enc3 [rad]  (by servo channel)
//   cmds: "type:canard:angle" items joined by '|', empty if none
// OUTPUTS follow, rounded for reading; replay does not need them but can
// compare against them. See formatCycleHeader() for the full list.
//
// A reader must find columns by NAME from the header, so columns can be added
// later without breaking replay of old logs.
//
// No Arduino, no heap.

#include <cstddef>
#include <cstdint>

#include "Actuators.h"
#include "SensorSource.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Logger/LogSink.h"

namespace CycleLogFormat {

// Long enough for any header or row.
constexpr size_t kMaxLine = 1024;

// Writes the header line (with '\n'). Returns its length, 0 if `n` is too small.
size_t formatHeader(char* buf, size_t n);

// Writes one row (with '\n'). Returns its length, 0 if `n` is too small.
size_t formatRow(const CycleRecord& record, char* buf, size_t n);

// What replay needs from one row: the inputs of that cycle.
struct ReplayInput {
  SensorFrame frame;
  ActuatorFeedback feedback;
  size_t commandCount = 0;
  Command commands[CommandList::kCapacity];
};

// Where each input column sits in a row, from the header.
struct Columns {
  int tUs = -1, imuOk = -1, ax = -1, ay = -1, az = -1, gx = -1, gy = -1, gz = -1;
  int magNew = -1, mx = -1, my = -1, mz = -1;
  int baroNew = -1, pPa = -1, tempC = -1;
  int encOk = -1, enc[kNumCanards] = {-1, -1, -1, -1};
  int cmds = -1;
  // True if every column replay needs was found.
  bool complete() const;
};

// Reads the header line. Returns false if a needed column is missing.
bool parseHeader(const char* line, Columns& out);

// Reads one row. Returns false if it is malformed (too few fields, a bad
// number, a bad command).
bool parseRow(const char* line, const Columns& columns, ReplayInput& out);

}  // namespace CycleLogFormat
