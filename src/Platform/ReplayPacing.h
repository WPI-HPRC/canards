#pragma once

// When SD replay hands out the next log row (LogReplay's ReleasePolicy), on
// the board. Wall-clock time lives here, never in flight logic.
//
//   paced (default)  a row is released once micros() has advanced, since the
//                    first row was released, by as much as the recorded
//                    sample time has. If replay falls behind (SD too slow),
//                    rows follow one per loop() until it catches up; none is
//                    skipped.
//   fast             as fast as the SD log keeps up.
// In both, a row is held while the replay's own SD log is half full or more,
// so the new log never drops rows because replay ran ahead of the card.
//
// Board only: everything is inside #if defined(ARDUINO).

#if defined(ARDUINO)

#include <Arduino.h>

#include <cstdint>

#include "MicrosClock64.h"
#include "Platform/SdLogSink.h"
#include "Subsystems/Logger/LogReplay.h"

class ReplayPacing final : public ReleasePolicy {
 public:
  explicit ReplayPacing(const SdLogSink& log) : log_(log) {}

  void setFast(bool fast) { fast_ = fast; }
  bool fast() const { return fast_; }

  bool release(uint64_t rowTimeUs, uint64_t firstRowTimeUs) override {
    if (log_.pending() >= SdLogSink::kCapacity / 2) return false;
    const uint64_t now = clock_.extend(micros());
    if (!started_) {
      started_ = true;
      startUs_ = now;
    }
    if (fast_ || rowTimeUs <= firstRowTimeUs) return true;
    return now - startUs_ >= rowTimeUs - firstRowTimeUs;
  }

 private:
  const SdLogSink& log_;
  MicrosClock64 clock_;
  bool fast_ = false;
  bool started_ = false;
  uint64_t startUs_ = 0;
};

#endif  // ARDUINO
