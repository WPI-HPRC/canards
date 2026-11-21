#if defined(ARDUINO)

#include "Platform/SdLogSink.h"

#include "Subsystems/Logger/CycleLogFormat.h"

bool SdLogSink::begin(FsFile& file) {
  file_ = nullptr;
  if (!file.isOpen()) return false;
  char line[CycleLogFormat::kMaxLine];
  const size_t len = CycleLogFormat::formatHeader(line, sizeof line);
  if (len == 0 || file.write(line, len) != len) return false;
  file.sync();
  if (file.getWriteError()) return false;
  file_ = &file;
  lastSyncMs_ = millis();
  return true;
}

void SdLogSink::write(const CycleRecord& record) {
  if (file_ == nullptr) return;
  if (count_ >= kCapacity) {
    ++dropped_;
    return;
  }
  ring_[(head_ + count_) % kCapacity] = record;
  ++count_;
}

size_t SdLogSink::drain(size_t maxRows) {
  size_t done = 0;
  char line[CycleLogFormat::kMaxLine];
  while (file_ != nullptr && count_ > 0 && done < maxRows) {
    const size_t len = CycleLogFormat::formatRow(ring_[head_], line, sizeof line);
    head_ = (head_ + 1) % kCapacity;
    --count_;
    if (len == 0) continue;  // cannot happen with kMaxLine; never stall on it
    file_->write(line, len);
    if (file_->getWriteError()) {
      writeError_ = true;
      file_ = nullptr;
      break;
    }
    ++written_;
    ++done;
  }
  if (file_ != nullptr && millis() - lastSyncMs_ >= kSyncPeriodMs) {
    file_->sync();
    lastSyncMs_ = millis();
  }
  return done;
}

#endif  // ARDUINO
