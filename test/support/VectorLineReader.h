#pragma once

// A LineReader over lines held in memory, for desktop tests. Follows the
// LineReader contract: a line too long for the buffer is cut, the rest of it
// dropped. Counts how many lines were read.
//
// Test-only: uses std::vector and std::string.

#include <cstddef>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "Subsystems/Logger/LogReplay.h"

class VectorLineReader final : public LineReader {
 public:
  VectorLineReader() = default;
  explicit VectorLineReader(std::vector<std::string> lines) : lines(std::move(lines)) {}

  std::vector<std::string> lines;

  bool readLine(char* buf, size_t n) override {
    if (next_ >= lines.size() || n == 0) return false;
    const std::string& line = lines[next_++];
    const size_t len = line.size() < n - 1 ? line.size() : n - 1;
    std::memcpy(buf, line.data(), len);
    buf[len] = '\0';
    return true;
  }

  size_t linesRead() const { return next_; }

 private:
  size_t next_ = 0;
};
