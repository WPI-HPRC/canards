#pragma once

// A 64-bit microsecond clock built from a 32-bit counter (Arduino micros(),
// which wraps every 71.6 minutes; the rocket can sit on the pad for longer).
// Hardware-free: the caller passes the 32-bit reading in. It must be called at
// least once per wrap period, which any running loop() does.

#include <cstdint>

class MicrosClock64 {
 public:
  uint64_t extend(uint32_t now32) {
    if (started_ && now32 < last32_) high_ += (1ull << 32);
    started_ = true;
    last32_ = now32;
    return high_ | now32;
  }

  // The 64-bit time of an earlier 32-bit reading `then32`, given that the
  // current reading is `now32` / `now64` and less than one wrap has passed.
  static uint64_t backdate(uint64_t now64, uint32_t now32, uint32_t then32) {
    return now64 - static_cast<uint32_t>(now32 - then32);
  }

 private:
  bool started_ = false;
  uint32_t last32_ = 0;
  uint64_t high_ = 0;
};
