#pragma once

// Debounce: a condition must hold continuously for a length of SAMPLE time
// before it counts. Use it for every transition condition
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.2).
//
// Sample time, not wall time: feed it the cycle's IMU timestamp, so a replayed
// flight debounces exactly as the original did.

#include <cstdint>

class Persistence {
 public:
  // `holdUs` is how long the condition must stay true. 0 means the first true
  // sample is enough.
  explicit constexpr Persistence(uint64_t holdUs) : holdUs_(holdUs) {}

  // Call once per cycle. Returns true once `condition` has been true on every
  // call for at least holdUs of sample time, and keeps returning true while it
  // stays true. A false sample, or time going backwards, starts over.
  bool update(bool condition, uint64_t timeUs);

  // Forget everything. Call from a state's init().
  void reset();

  // The result of the last update().
  bool held() const { return held_; }

 private:
  uint64_t holdUs_;
  uint64_t sinceUs_ = 0;   // sample time of the first true in the current run
  uint64_t lastUs_ = 0;    // sample time of the previous update
  bool running_ = false;   // inside a run of true samples
  bool held_ = false;
};
