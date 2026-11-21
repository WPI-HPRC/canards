#pragma once

// A scripted SensorSource for desktop tests.
//
// Each read() produces ONE frame, at the source's next sample time, from a
// script that is a function of sample time. The next sample time then moves on
// by the fixed period given at construction. A test can:
//   - make the source report "no new sample" (`available = false`);
//   - move the next sample time anywhere (a gap, or time going backwards);
//   - read back every frame it handed out, in order.
//
// Test-only: uses std::function and std::vector.

#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "SensorSource.h"

class FakeSensorSource final : public SensorSource {
 public:
  // The frame for sample time `timeUs`. Its timeUs field is overwritten with
  // `timeUs`, so a script cannot disagree with the source about the time.
  using Script = std::function<SensorFrame(uint64_t timeUs)>;

  FakeSensorSource(uint64_t firstTimeUs, uint64_t periodUs, Script script)
      : script_(std::move(script)), nextTimeUs_(firstTimeUs), periodUs_(periodUs) {}

  // false: read() reports no new sample and changes nothing.
  bool available = true;

  bool read(SensorFrame& out) override {
    ++readCalls_;
    if (!available) return false;
    SensorFrame frame = script_(nextTimeUs_);
    frame.timeUs = nextTimeUs_;
    frames_.push_back(frame);
    out = frame;
    nextTimeUs_ += periodUs_;
    return true;
  }

  // The sample time of the next frame.
  uint64_t nextTimeUs() const { return nextTimeUs_; }
  // Moves the next sample time (later: an IMU gap; earlier: time going
  // backwards). Later frames follow on from it at the fixed period.
  void setNextTimeUs(uint64_t timeUs) { nextTimeUs_ = timeUs; }

  uint64_t periodUs() const { return periodUs_; }

  // Every frame handed out, in order.
  const std::vector<SensorFrame>& frames() const { return frames_; }
  // Calls of read(), including those that returned false.
  uint32_t readCalls() const { return readCalls_; }

 private:
  Script script_;
  uint64_t nextTimeUs_;
  uint64_t periodUs_;
  std::vector<SensorFrame> frames_;
  uint32_t readCalls_ = 0;
};
