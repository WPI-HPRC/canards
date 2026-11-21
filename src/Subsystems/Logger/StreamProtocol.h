#pragma once

// Streaming inputs into the flight program over USB, one cycle per line
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 7). The host is
// tools/usb_stream.py (a recorded log, a synthetic flight, or later a sim).
//
// The protocol, one line each way, in lockstep:
//
//   host -> board                        board -> host
//   -------------                        -------------
//   the log header line (once, first)    the board's own log header line
//                                        (the columns of every reply row)
//   one input row per step, in exactly   one reply row: CycleLogFormat
//   the CycleLogFormat row format        formatRow() of the CycleRecord that
//                                        cycle produced, full outputs, so the
//                                        host can close the loop on
//                                        cmd0..3 / pulse0..3
//   "# anything"                         nothing (a comment)
//   a blank line                         nothing
//   anything else                        "!ERR <reason>", and it is skipped
//
// So a recorded log can be streamed as it is: its output columns are ignored
// (only the input columns are read, found by name from the header). A new
// header line may be sent at any time and replaces the old one.
//
// Error reasons: "no header" (a row before any good header), "bad header"
// (a needed input column is missing), "bad row" (too few fields, a bad
// number, a bad command, or too long), "unknown line", "busy" (a row arrived
// before the previous one was used by a cycle; the board never does that).
//
// StreamSession is the board side, hardware-free: main.cpp feeds it lines and
// writes the replies. It is a RowPlayer, so it is the FlightComputer's
// SensorSource, CommandSource and feedback Actuators, like LogReplay.
//
// THE STREAM IS OPEN LOOP unless the host closes it: with a recorded log the
// next row does not depend on the reply.
//
// No Arduino, no heap, no clock. The heartbeat (canards neutral if the host
// goes quiet) is wall-clock and lives in main.cpp.

#include <cstddef>
#include <cstdint>

#include "Subsystems/Logger/CycleLogFormat.h"
#include "Subsystems/Logger/LogReplay.h"
#include "Subsystems/Logger/LogSink.h"

namespace StreamProtocol {

constexpr size_t kMaxLine = CycleLogFormat::kMaxLine;

// The prefix of an error reply.
constexpr const char* kErrorPrefix = "!ERR ";

// "!ERR <reason>\n". Returns its length, 0 if `n` is too small.
size_t formatError(const char* reason, char* buf, size_t n);

}  // namespace StreamProtocol

class StreamSession final : public RowPlayer {
 public:
  enum class Result : uint8_t {
    Ignored,  // a comment or a blank line: no reply
    Header,   // a good header: reply with replyHeader()
    Row,      // a good row, ready for one cycle: run it, reply with replyRow()
    Error,    // reply with replyError()
  };

  // `output`: where the FlightComputer's servo commands go (nullptr: discarded).
  explicit StreamSession(Actuators* output = nullptr) : RowPlayer(output) {}

  // One line from the host (a trailing "\n" or "\r\n" is fine).
  Result feedLine(const char* line);

  // The reason for the last Error.
  const char* error() const { return error_; }

  bool headerOk() const { return headerOk_; }
  // A row was fed and not yet handed to a cycle.
  bool rowPending() const { return havePending_; }
  uint32_t rowsAccepted() const { return rowsAccepted_; }
  uint32_t errors() const { return errors_; }

  // The reply lines (with '\n'). Each returns its length, 0 if `n` is too small.
  static size_t replyHeader(char* buf, size_t n);
  static size_t replyRow(const CycleRecord& record, char* buf, size_t n);
  size_t replyError(char* buf, size_t n) const;

 protected:
  bool fetch(CycleLogFormat::ReplayInput& out) override;

 private:
  Result fail(const char* reason);

  CycleLogFormat::Columns columns_;
  CycleLogFormat::ReplayInput pending_;
  bool headerOk_ = false;
  bool havePending_ = false;
  const char* error_ = "";
  uint32_t rowsAccepted_ = 0;
  uint32_t errors_ = 0;
};
