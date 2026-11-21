#pragma once

// Replaying a flight log through the flight program
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 7).
//
// A log written by CycleLogFormat holds every input of every cycle. LogReplay
// reads it back, one row per cycle, and stands in for the three things the
// FlightComputer reads inputs from:
//   SensorSource   read() hands out the next row's sensor frame;
//   CommandSource  poll() hands out the commands of the row just read;
//   Actuators      readFeedback() returns that row's encoder feedback, and
//                  write() forwards to an optional real Actuators (nullptr:
//                  the command is discarded, the servos never move).
// The FlightComputer calls them in exactly that order within one cycle (read,
// poll, readFeedback, write), so a replayed cycle sees exactly the inputs the
// recorded cycle saw, and with deterministic flight logic produces exactly
// the same outputs.
//
// REPLAY IS OPEN LOOP: the recorded motion was caused by the old commands, so
// the new commands do not change what happens next. Replay shows whether the
// new code changes state at the same moments and commands sane values, not
// whether it would have flown straight.
//
// Pacing: read() only hands out the next row when a ReleasePolicy says so.
// The board paces by its wall clock (or by how far the SD log is behind);
// tests release at once. The flight logic itself never sees wall time.
//
// No Arduino, no heap, no clock.

#include <cstddef>
#include <cstdint>

#include "Actuators.h"
#include "SensorSource.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Logger/CycleLogFormat.h"

// A source of text lines: an SD file, a USB port, an array in a test.
class LineReader {
 public:
  virtual ~LineReader() = default;

  // Reads the next line into `buf` (NUL-terminated; a trailing "\n" or
  // "\r\n" may or may not be kept). A line longer than n - 1 characters is
  // cut to n - 1 and the rest of it is discarded, so it cannot be mistaken
  // for the next line. Returns false if there is no line: for a file, the end
  // of it; for a live stream, nothing complete yet. Never blocks for long.
  virtual bool readLine(char* buf, size_t n) = 0;
};

// Decides when the next row may be handed out.
class ReleasePolicy {
 public:
  virtual ~ReleasePolicy() = default;

  // `rowTimeUs` is the next row's sample time, `firstRowTimeUs` the first
  // row's. Return true to hand the row out now. Called again on every read()
  // until it returns true.
  virtual bool release(uint64_t rowTimeUs, uint64_t firstRowTimeUs) = 0;
};

// Hands every row out at once (tests, and the stream).
class ReleaseImmediately final : public ReleasePolicy {
 public:
  bool release(uint64_t, uint64_t) override { return true; }
};

// The inputs of one row at a time, presented as the three interfaces the
// FlightComputer reads. A subclass supplies the rows (fetch()).
class RowPlayer : public SensorSource, public CommandSource, public Actuators {
 public:
  // `output`: where write() goes; nullptr discards it.
  explicit RowPlayer(Actuators* output = nullptr) : output_(output) {}

  // SensorSource: the next row's frame, if fetch() has one. The row's
  // commands and feedback then replace the previous row's; any of the
  // previous row's commands not yet polled are dropped (they belonged to
  // that cycle).
  bool read(SensorFrame& out) override;

  // CommandSource: the commands of the row last read, in order.
  bool poll(Command& out) override;

  // Actuators: the feedback of the row last read (none valid before the
  // first row); write() forwards to `output`.
  void readFeedback(ActuatorFeedback& out) override;
  void write(const ActuatorCommand& command) override;

  void setOutput(Actuators* output) { output_ = output; }
  // The row last handed out by read().
  const CycleLogFormat::ReplayInput& current() const { return current_; }
  bool haveRow() const { return haveRow_; }

 protected:
  // Fills `out` with the next row and returns true, or returns false if there
  // is none to hand out now.
  virtual bool fetch(CycleLogFormat::ReplayInput& out) = 0;

 private:
  Actuators* output_;
  CycleLogFormat::ReplayInput current_;
  bool haveRow_ = false;
  size_t nextCommand_ = 0;
};

// Replays a log read from a LineReader.
class LogReplay final : public RowPlayer {
 public:
  // At most this many lines are read in one read() while looking for a good
  // row, so a run of bad lines cannot stall a loop() for long.
  static constexpr int kMaxLinesPerRead = 16;

  // `reader` and `policy` must outlive the LogReplay.
  LogReplay(LineReader& reader, ReleasePolicy& policy, Actuators* output = nullptr)
      : RowPlayer(output), reader_(reader), policy_(policy) {}

  // Reads the header now if it has not been read yet (read() also does it).
  // Returns headerOk().
  bool begin();

  // The header was found and has every input column.
  bool headerOk() const { return headerOk_; }
  // Nothing more will be handed out: the reader ran out, or the header was
  // missing or bad.
  bool endOfLog() const { return endOfLog_; }
  // Rows handed out by read().
  uint32_t rowsRead() const { return rowsRead_; }
  // Lines after the header that were not a valid row (skipped).
  uint32_t malformedRows() const { return malformed_; }

 protected:
  bool fetch(CycleLogFormat::ReplayInput& out) override;

 private:
  // Reads the next line that is not blank or a '#' comment. False at the end.
  bool nextLine();
  // Parses rows until one is good (kept in pending_) or the end is reached.
  bool fillPending();

  LineReader& reader_;
  ReleasePolicy& policy_;
  CycleLogFormat::Columns columns_;
  CycleLogFormat::ReplayInput pending_;
  char line_[CycleLogFormat::kMaxLine] = {};
  bool started_ = false;
  bool headerOk_ = false;
  bool endOfLog_ = false;
  bool havePending_ = false;
  bool haveFirstTime_ = false;
  uint64_t firstTimeUs_ = 0;
  uint32_t rowsRead_ = 0;
  uint32_t malformed_ = 0;
};
