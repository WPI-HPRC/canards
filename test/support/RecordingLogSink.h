#pragma once

// A LogSink for desktop tests: keeps every CycleRecord, in order.
//
// Test-only: uses std::vector.

#include <vector>

#include "Subsystems/Logger/LogSink.h"

class RecordingLogSink final : public LogSink {
 public:
  void write(const CycleRecord& record) override { records.push_back(record); }

  std::vector<CycleRecord> records;
};
