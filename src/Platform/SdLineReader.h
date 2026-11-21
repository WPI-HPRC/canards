#pragma once

// A LineReader over a file on the SD card, for SD replay (LogReplay).
// The file is opened read-only on the card that sdcard.h mounted (sdFs()).
//
// Board only: everything is inside #if defined(ARDUINO).

#if defined(ARDUINO)

#include <SdFat.h>

#include <cstddef>

#include "Subsystems/Logger/LogReplay.h"

class SdLineReader final : public LineReader {
 public:
  // Opens `path` (e.g. "/logs/canardsLog7.csv") read-only. Returns false if
  // the card is not mounted or the file cannot be opened.
  bool open(const char* path);
  bool isOpen() { return file_.isOpen(); }
  void close() { file_.close(); }

  // One line per call; false at the end of the file (or on a read error).
  bool readLine(char* buf, size_t n) override;

 private:
  FsFile file_;
};

#endif  // ARDUINO
