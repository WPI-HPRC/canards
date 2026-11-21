#pragma once

// USB bench console: the radio telemetry downlink, copied to the board's USB
// port, so bench tests need no ground radio. Flight mode only (main.cpp);
// replay and stream own the USB port themselves.
//
// OUTPUT. The same lines the radio sends (src/telemetry.h): "$H,..." header,
// "$D,..." rows, "$A,<command>,<result>" replies, each with its *XX checksum,
// but without the radio's callsign framing. Built from the same column table
// as the radio, at the same Telemetry::ROW_PERIOD_MS, the header again every
// Telemetry::HEADER_PERIOD_MS and whenever a host opens the port. So
// `python3 ground/telemetry_gui.py --usb` shows it, and any serial terminal
// can read it.
//
// Only while a host has the port open (DTR set). With nothing plugged in, as
// in flight, it writes nothing.
//
// NEVER BLOCKS. SerialUSB.write() busy-waits while the USB transmit queue is
// full (128 bytes on this board, full-speed USB). So a line is built in RAM
// and handed over no faster than availableForWrite() says the queue has room,
// a piece each loop(). A row that comes due while the previous line is still
// going out is skipped and counted (rowsSkipped()).
//
// INPUT is not read here: main.cpp reads the lines (Platform/UsbLineReader.h)
// and decides what they do (GroundCommands::commandFromLine, then the log
// download). The console only sends the replies.
//
// Board only: everything is inside #if defined(ARDUINO).

#if defined(ARDUINO)

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

#include "telemetry.h"

class UsbConsole {
 public:
  static constexpr size_t kMaxLine = Telemetry::MAX_LINE;

  explicit UsbConsole(Stream& port) : port_(port) {}

  template <size_t N>
  void begin(const TelemetryColumn (&columns)[N]) {
    begin(columns, N);
  }
  void begin(const TelemetryColumn* columns, size_t count);

  // Call every loop() in Flight mode. `connected` is SerialUSB.dtr().
  void update(uint32_t nowMs, bool connected);

  // Queues "$A,<command>,<result>" to go out before the next row. Both
  // strings must outlive the call (string literals, GroundCommands names).
  void reply(const char* command, const char* result);

  // Drops whatever is still queued, so a log dump (which writes SerialUSB
  // itself) does not land in the middle of a console line.
  void discardOutput();

  uint32_t rowsSent() const { return rowsSent_; }
  uint32_t rowsSkipped() const { return rowsSkipped_; }

 private:
  // A Print into out_, for the column writers.
  class LineBuffer final : public Print {
   public:
    explicit LineBuffer(UsbConsole& c) : c_(c) {}
    size_t write(uint8_t b) override;
    using Print::write;

   private:
    UsbConsole& c_;
  };

  void startLine(const char* type);
  void finishLine();
  void pump();

  Stream& port_;
  LineBuffer printer_{*this};
  const TelemetryColumn* columns_ = nullptr;
  size_t count_ = 0;

  char out_[kMaxLine] = {};
  size_t len_ = 0;   // bytes in out_
  size_t sent_ = 0;  // bytes of out_ already handed to the port
  bool overflow_ = false;

  bool wasConnected_ = false;
  bool headerDue_ = true;
  uint32_t lastRowMs_ = 0;
  uint32_t lastHeaderMs_ = 0;
  const char* replyCommand_ = nullptr;
  const char* replyResult_ = nullptr;

  uint32_t rowsSent_ = 0;
  uint32_t rowsSkipped_ = 0;
};

#endif  // ARDUINO
