#pragma once

// A LineReader over a byte stream (SerialUSB), for the boot-mode line and the
// USB log stream (StreamSession). Never blocks: readLine() takes the bytes
// already received and returns true only once a whole line has arrived;
// false means "no complete line yet", not the end of anything.
//
// '\r' is dropped. A line longer than the caller's buffer (or than
// kMaxLine) is cut and the rest of it dropped, and the cut line is still
// returned, so the stream can answer it with an error and stay in lockstep.
//
// Board only: everything is inside #if defined(ARDUINO).

#if defined(ARDUINO)

#include <Arduino.h>

#include <cstddef>

#include "Subsystems/Logger/CycleLogFormat.h"
#include "Subsystems/Logger/LogReplay.h"

class UsbLineReader final : public LineReader {
 public:
  static constexpr size_t kMaxLine = CycleLogFormat::kMaxLine;

  explicit UsbLineReader(Stream& in) : in_(in) {}

  bool readLine(char* buf, size_t n) override;

 private:
  Stream& in_;
  char line_[kMaxLine] = {};
  size_t len_ = 0;
};

#endif  // ARDUINO
