#pragma once

// The flight log on the SD card: one CycleLogFormat row per cycle, written to
// the per-boot file that sdcard.h opens (/logs/canardsLog<N>.csv), so Nic's
// tools (pio run -t listlogs / -t dumplog) download it like any other log.
//
// write() only copies the record into a RAM ring buffer: it never touches the
// card, so the control step never waits on SD. drain() formats and writes
// buffered rows; main.cpp calls it between IMU samples. If the buffer fills
// (the card is too slow for the cycle rate), new records are dropped and
// counted, never older ones overwritten, so the log has gaps rather than
// reordering.
//
// Board only: everything is inside #if defined(ARDUINO).

#if defined(ARDUINO)

#include <Arduino.h>
#include <SdFat.h>

#include "Subsystems/Logger/LogSink.h"

class SdLogSink final : public LogSink {
 public:
  static constexpr size_t kCapacity = 64;          // records buffered in RAM (about 64 x 0.5 kB)
  static constexpr uint32_t kSyncPeriodMs = 1000;  // rows since the last sync are lost on power loss

  // Starts logging to an open file and writes the header row. Logging stays
  // off (records are dropped quietly) if the file is not open.
  bool begin(FsFile& file);

  void write(const CycleRecord& record) override;

  // Writes up to `maxRows` buffered rows. Returns how many it wrote.
  size_t drain(size_t maxRows);

  bool active() const { return file_ != nullptr; }
  size_t pending() const { return count_; }
  uint32_t rowsWritten() const { return written_; }
  uint32_t dropped() const { return dropped_; }   // buffer full
  bool writeError() const { return writeError_; }  // logging stopped

 private:
  FsFile* file_ = nullptr;
  CycleRecord ring_[kCapacity];
  size_t head_ = 0;
  size_t count_ = 0;
  uint32_t written_ = 0;
  uint32_t dropped_ = 0;
  uint32_t lastSyncMs_ = 0;
  bool writeError_ = false;
};

#endif  // ARDUINO
