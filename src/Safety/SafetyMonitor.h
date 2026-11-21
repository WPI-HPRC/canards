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
// A transient fault is NOT latched for the rest of the flight. Only the
// active bits affect the canards.
//
// LAX (the user's choice, 2026-11-21): a fault is only active once it has been
// present on every check for `holdUs` in a row, or, for an IMU gap, when one
// gap is longer than `catastrophicGapUs`. raw() is what was seen this cycle
// before that. sticky records active faults only. With holdUs = 0 and
// catastrophicGapUs = kMaxImuGapUs it is the old strict monitor.

#include <cstdint>

#include "FlightConfig.h"
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
  explicit SafetyMonitor(uint64_t holdUs = FlightConfig::kSafetyFaultHoldUs,
                         uint64_t catastrophicGapUs = FlightConfig::kImuGapCatastrophicUs)
      : holdUs_(holdUs), catastrophicGapUs_(catastrophicGapUs) {}

  // Once per cycle, before the state machine runs.
  // `refusedTransitionCount` is StateMachine::refusedCount(); the monitor
  // raises kTransitionRefused on the cycle after it increases.
  void check(const SensorFrame& frame, const TimeStep& step, uint32_t refusedTransitionCount);

  // Faults present this cycle.
  uint16_t active() const { return active_; }
  bool anyActive() const { return active_ != 0; }

  // Every fault that was active since boot.
  uint16_t sticky() const { return sticky_; }

  // Faults seen this cycle before the hold: glitches included.
  uint16_t raw() const { return raw_; }

 private:
  static constexpr int kNumFaults = 5;
  uint64_t holdUs_;
  uint64_t catastrophicGapUs_;
  uint16_t raw_ = 0;
  uint64_t rawSinceUs_[kNumFaults] = {};  // sample time each raw fault started
  uint16_t active_ = 0;
  uint16_t sticky_ = 0;
  uint32_t lastRefusedCount_ = 0;
};
