#include "Subsystems/Logger/LogReplay.h"

namespace {

// Blank lines (only whitespace) and '#' comments carry no data.
bool ignorable(const char* line) {
  const char* p = line;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
  return *p == '\0' || *p == '#';
}

}  // namespace

// --- RowPlayer -----------------------------------------------------------------

bool RowPlayer::read(SensorFrame& out) {
  CycleLogFormat::ReplayInput next;
  if (!fetch(next)) return false;
  current_ = next;
  haveRow_ = true;
  nextCommand_ = 0;
  out = current_.frame;
  return true;
}

bool RowPlayer::poll(Command& out) {
  if (!haveRow_ || nextCommand_ >= current_.commandCount) return false;
  out = current_.commands[nextCommand_++];
  return true;
}

void RowPlayer::readFeedback(ActuatorFeedback& out) {
  out = haveRow_ ? current_.feedback : ActuatorFeedback{};
}

void RowPlayer::write(const ActuatorCommand& command) {
  if (output_ != nullptr) output_->write(command);
}

// --- LogReplay -----------------------------------------------------------------

bool LogReplay::nextLine() {
  while (reader_.readLine(line_, sizeof line_)) {
    if (!ignorable(line_)) return true;
  }
  return false;
}

bool LogReplay::begin() {
  if (started_) return headerOk_;
  started_ = true;
  headerOk_ = nextLine() && CycleLogFormat::parseHeader(line_, columns_);
  if (!headerOk_) endOfLog_ = true;
  return headerOk_;
}

bool LogReplay::fillPending() {
  if (havePending_) return true;
  for (int i = 0; i < kMaxLinesPerRead; ++i) {
    if (!nextLine()) {
      endOfLog_ = true;
      return false;
    }
    if (CycleLogFormat::parseRow(line_, columns_, pending_)) {
      havePending_ = true;
      if (!haveFirstTime_) {
        haveFirstTime_ = true;
        firstTimeUs_ = pending_.frame.timeUs;
      }
      return true;
    }
    ++malformed_;
  }
  return false;  // only bad lines so far; carry on next read()
}

bool LogReplay::fetch(CycleLogFormat::ReplayInput& out) {
  if (!begin() || endOfLog_) return false;
  if (!fillPending()) return false;
  if (!policy_.release(pending_.frame.timeUs, firstTimeUs_)) return false;
  out = pending_;
  havePending_ = false;
  ++rowsRead_;
  return true;
}
