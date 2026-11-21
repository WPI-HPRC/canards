#pragma once

// Radio telemetry: broadcasts one CSV row per period from a table of columns,
// and listens for simple text commands from the ground.
//
// Built the same way as the SD card logger (csvlog.h on the sd-card branch):
// each column is a name plus a function that prints its value, so adding a
// field to the downlink is one line in the table:
//
//   static const TelemetryColumn tlmColumns[] = {
//     TLM_UINT("time_ms", millis()),
//     TLM_FLOAT("pressure_hpa", readings.pressure, 2),
//   };
//   telemetry.addCommand("armFlight", onArmFlight);
//   telemetry.begin(radioModule, tlmColumns, radioOk);
//   telemetry.update();                          // in loop()
//
// ---------------------------------------------------------------- protocol --
// Every message is one line of printable ASCII:
//
//   $<type>,<field>,<field>,...*<checksum>\n
//
// <checksum> is two uppercase hex digits: the XOR of every byte between '$'
// and '*' (as in NMEA). '$', '*' and '\n' never appear inside a line, so a
// receiver can find lines in a raw byte stream, and a line damaged by a lost
// packet fails its checksum and is dropped.
//
// Downlink (rocket -> ground):
//   $H,name0,name1,...        header: the column names. Sent at boot and
//                              again every HEADER_PERIOD_MS, so a receiver
//                              that missed the first one still learns them.
//   $D,value0,value1,...      data row, one value per header column.
//   $A,<command>,<result>     reply to a command: OK, REJECTED or UNKNOWN.
//
// Uplink (ground -> rocket):
//   $C,<command>              run a command added with addCommand(). Must be
//                              framed like the HAL frames it (callsign +
//                              length + payload), since LoRaE22::update()
//                              only keeps framed messages.
//
// The radio HAL (LoRaE22::sendMessage) frames each message as
// <callsign><length byte><payload>, with at most MAX_MESSAGE_SIZE payload
// bytes. Lines longer than that (the header usually is) are split over several
// messages; the receiver strips the framing and joins the payloads back into
// one stream before looking for lines.
// ----------------------------------------------------------------------------

#include <Arduino.h>
#include "LoRaE22.h"

struct TelemetryColumn {
  const char *name;
  void (*write)(Print &out);
};

// Column helpers. `expr` is evaluated each time a row is sent, so it can name
// globals directly. TLM_FLOAT's `dp` is the number of decimal places: keep it
// as small as the data needs, every character costs airtime.
#define TLM_INT(name, expr) \
  TelemetryColumn { name, [](Print &o) { o.print((long)(expr)); } }
#define TLM_UINT(name, expr) \
  TelemetryColumn { name, [](Print &o) { o.print((unsigned long)(expr)); } }
#define TLM_FLOAT(name, expr, dp) \
  TelemetryColumn { name, [](Print &o) { o.print((double)(expr), dp); } }

class Telemetry {
public:
  // How often a data row is sent. A ~150 byte row takes roughly 100 ms of
  // airtime at 15.6 kbps, so don't go much faster than this.
  static constexpr uint32_t ROW_PERIOD_MS = 250;
  // How often the header is re-sent, for receivers that missed it.
  static constexpr uint32_t HEADER_PERIOD_MS = 5000;

  static constexpr size_t MAX_COMMANDS = 4;
  // Longest line we will build. A line is written to the radio in one burst,
  // which must fit the E22's 1000 byte buffer: 900 bytes plus 5 bytes of
  // framing per message is under 925.
  static constexpr size_t MAX_LINE = 900;
  static constexpr size_t MAX_RX_LINE = 64; // longest command line accepted

  // A command handler returns true if it accepted the command; the ground is
  // told OK or REJECTED either way.
  using CommandHandler = bool (*)();

  // Registers a command, run when "$C,<name>" arrives. Call before begin().
  bool addCommand(const char *name, CommandHandler handler);

  // Starts telemetry. Pass radioOk = false if the radio failed to initialize:
  // telemetry still runs (the module may work with whatever config it already
  // has), only a warning is printed. Nothing blocks on a missing radio: with
  // AUX never high, rows are just skipped.
  template <size_t N>
  bool begin(LoRaE22 &radio, const TelemetryColumn (&columns)[N], bool radioOk) {
    return begin(radio, columns, N, radioOk);
  }
  bool begin(LoRaE22 &radio, const TelemetryColumn *columns, size_t count, bool radioOk);

  // Call every loop(). Never blocks waiting for the radio: when the module is
  // busy (AUX low) the row is skipped and counted in rowsSkipped().
  // `transmit` false: commands are still read and run, but nothing is sent
  // (main.cpp keeps the radio quiet while a USB host is connected).
  void update(uint32_t now_ms = millis(), bool transmit = true);

  bool active() const { return radio_ != nullptr; }
  uint32_t rowsSent() const { return rowsSent_; }
  uint32_t rowsSkipped() const { return rowsSkipped_; }
  uint32_t commandsReceived() const { return commandsReceived_; }

private:
  // Builds one outgoing line in RAM, so it can be checksummed and split.
  class LineBuffer : public Print {
  public:
    size_t write(uint8_t c) override;
    using Print::write;
    void clear() { len_ = 0; overflow_ = false; }
    const uint8_t *data() const { return buf_; }
    size_t length() const { return len_; }
    bool overflow() const { return overflow_; }
  private:
    uint8_t buf_[MAX_LINE];
    size_t len_ = 0;
    bool overflow_ = false;
  };

  struct Command {
    const char *name;
    CommandHandler handler;
  };

  LoRaE22 *radio_ = nullptr;
  const TelemetryColumn *columns_ = nullptr;
  size_t count_ = 0;

  Command commands_[MAX_COMMANDS] = {};
  size_t commandCount_ = 0;

  LineBuffer line_;
  bool headerDue_ = true;
  uint32_t lastRow_ = 0;
  uint32_t lastHeader_ = 0;

  // A reply to send before the next row (the radio may be busy right now).
  const char *pendingReplyCmd_ = nullptr;
  const char *pendingReplyResult_ = nullptr;

  char rx_[MAX_RX_LINE];
  size_t rxLen_ = 0;
  bool inRxLine_ = false;

  uint32_t rowsSent_ = 0;
  uint32_t rowsSkipped_ = 0;
  uint32_t commandsReceived_ = 0;

  void buildHeader();
  void buildRow();
  void buildReply(const char *cmd, const char *result);
  void finishLine();
  bool sendLine();

  void pollCommands();
  void pollByte(char c);
  void handleRxLine();
};
