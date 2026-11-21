#include "Platform/UsbConsole.h"

#if defined(ARDUINO)

#include <cstdio>

size_t UsbConsole::LineBuffer::write(uint8_t b) {
  // Keep one byte spare for the trailing '\n' of "*XX\n".
  if (c_.len_ + 1 >= sizeof(c_.out_)) {
    c_.overflow_ = true;
    return 0;
  }
  c_.out_[c_.len_++] = static_cast<char>(b);
  return 1;
}

void UsbConsole::begin(const TelemetryColumn* columns, size_t count) {
  columns_ = columns;
  count_ = count;
  headerDue_ = true;
  lastRowMs_ = lastHeaderMs_ = millis();
}

void UsbConsole::reply(const char* command, const char* result) {
  replyCommand_ = command;
  replyResult_ = result;
}

void UsbConsole::discardOutput() {
  len_ = sent_ = 0;
  overflow_ = false;
}

void UsbConsole::startLine(const char* type) {
  len_ = sent_ = 0;
  overflow_ = false;
  printer_.print('$');
  printer_.print(type);
}

// Appends "*XX\n"; XX is the XOR of every byte between '$' and '*'
// (src/telemetry.cpp). A line that did not fit is dropped whole.
void UsbConsole::finishLine() {
  if (overflow_) {
    discardOutput();
    return;
  }
  uint8_t cs = 0;
  for (size_t i = 1; i < len_; ++i) cs ^= static_cast<uint8_t>(out_[i]);
  char tail[5];
  snprintf(tail, sizeof tail, "*%02X", cs);
  printer_.print(tail);
  if (overflow_ || len_ + 1 > sizeof(out_)) {
    discardOutput();
    return;
  }
  out_[len_++] = '\n';
}

// Hands the port only as much as its transmit queue has room for, so write()
// never waits. write() can still return short (the host stopped reading and
// the core's 3 ms transmit timeout ran out): the rest waits for next time.
void UsbConsole::pump() {
  if (sent_ >= len_) return;
  const int room = port_.availableForWrite();
  if (room <= 0) return;
  size_t n = len_ - sent_;
  if (n > static_cast<size_t>(room)) n = static_cast<size_t>(room);
  sent_ += port_.write(reinterpret_cast<const uint8_t*>(out_ + sent_), n);
  if (sent_ >= len_) len_ = sent_ = 0;
}

void UsbConsole::update(uint32_t nowMs, bool connected) {
  if (!connected) {
    // Nobody listening: nothing goes out, and a new host gets the header first.
    discardOutput();
    wasConnected_ = false;
    return;
  }
  if (!wasConnected_) {
    wasConnected_ = true;
    headerDue_ = true;
  }

  pump();

  const bool rowDue = nowMs - lastRowMs_ >= Telemetry::ROW_PERIOD_MS;
  if (nowMs - lastHeaderMs_ >= Telemetry::HEADER_PERIOD_MS) headerDue_ = true;

  if (sent_ < len_) {  // still sending the last line
    if (rowDue) {
      ++rowsSkipped_;
      lastRowMs_ = nowMs;
    }
    return;
  }

  // At most one new line per call, in the radio's priority order.
  if (replyCommand_ != nullptr) {
    startLine("A,");
    printer_.print(replyCommand_);
    printer_.print(',');
    printer_.print(replyResult_);
    finishLine();
    replyCommand_ = replyResult_ = nullptr;
  } else if (headerDue_ && columns_ != nullptr) {
    startLine("H");
    for (size_t i = 0; i < count_; ++i) {
      printer_.print(',');
      printer_.print(columns_[i].name);
    }
    finishLine();
    headerDue_ = false;
    lastHeaderMs_ = nowMs;
  } else if (rowDue && columns_ != nullptr) {
    startLine("D");
    for (size_t i = 0; i < count_; ++i) {
      printer_.print(',');
      columns_[i].write(printer_);
    }
    finishLine();
    lastRowMs_ = nowMs;
    ++rowsSent_;
  } else {
    return;
  }
  pump();
}

#endif  // ARDUINO
