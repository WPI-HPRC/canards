#pragma once

// System-wide safety checks: the ones that are not about any single state or
// subsystem (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.4).
//
// Each fault has two bits:
//   active  true on the cycles where the fault is present. While any fault is
//           active the canards are forced to neutral.
//   sticky  set the first time the fault is seen and never cleared, for the
//           logs.
//
// PROPOSAL (not confirmed by the user): a transient fault is NOT latched for
// the rest of the flight, because one SD-card stall should not end control.
// Only the active bits affect the canards.

#include <cstdint>

#include "SensorSource.h"

namespace SafetyFault {
constexpr uint16_t kImuGap = 1u << 0;             // gap between IMU samples > kMaxImuGapUs
constexpr uint16_t kTimeBackwards = 1u << 1;      // sample time did not increase
constexpr uint16_t kImuReadFailed = 1u << 2;      // the source reported a failed IMU read
constexpr uint16_t kImuNonFinite = 1u << 3;       // NaN or infinity in accel or gyro
constexpr uint16_t kTransitionRefused = 1u << 4;  // the state machine refused a transition
}  // namespace SafetyFault

// How this cycle's sample time relates to the previous one. Computed once per
// cycle by the FlightComputer and shared by everything that needs it.
struct TimeStep {
  bool first = true;       // no previous sample
  bool backwards = false;  // timeUs <= previous timeUs
  uint64_t gapUs = 0;      // timeUs - previous timeUs (0 if first or backwards)
  float dt = 0.0f;         // gapUs in seconds
};

// Works out the TimeStep for a new sample. `havePrevious` is false on the
// first cycle.
TimeStep makeTimeStep(bool havePrevious, uint64_t previousTimeUs, uint64_t timeUs);

class SafetyMonitor {
 public:
  // Once per cycle, before the state machine runs.
  // `refusedTransitionCount` is StateMachine::refusedCount(); the monitor
  // raises kTransitionRefused on the cycle after it increases.
  void check(const SensorFrame& frame, const TimeStep& step, uint32_t refusedTransitionCount);

  // Faults present this cycle.
  uint16_t active() const { return active_; }
  bool anyActive() const { return active_ != 0; }

  // Every fault seen since boot (or since clearSticky()).
  uint16_t sticky() const { return sticky_; }

  // Forgets the history ("clearFaults" from the ground station). The active
  // faults, the only ones that affect the canards, are untouched.
  void clearSticky() { sticky_ = active_; }

 private:
  uint16_t active_ = 0;
  uint16_t sticky_ = 0;
  uint32_t lastRefusedCount_ = 0;
};
