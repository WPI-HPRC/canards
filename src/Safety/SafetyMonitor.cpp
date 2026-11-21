#include "Safety/SafetyMonitor.h"

#include "FlightConfig.h"
#include "Frames.h"

TimeStep makeTimeStep(bool havePrevious, uint64_t previousTimeUs, uint64_t timeUs) {
  TimeStep step;
  step.first = !havePrevious;
  step.backwards = false;
  step.gapUs = 0;
  step.dt = 0.0f;

  if (!havePrevious) return step;

  // A repeated timestamp counts as backwards too: no time has passed, so
  // there is no usable dt.
  if (timeUs <= previousTimeUs) {
    step.backwards = true;
    return step;
  }

  step.gapUs = timeUs - previousTimeUs;
  step.dt = static_cast<float>(step.gapUs) * 1e-6f;
  return step;
}

void SafetyMonitor::check(const SensorFrame& frame, const TimeStep& step,
                          uint32_t refusedTransitionCount) {
  uint16_t raw = 0;

  if (!step.first && step.gapUs > FlightConfig::kMaxImuGapUs) raw |= SafetyFault::kImuGap;
  if (step.backwards) raw |= SafetyFault::kTimeBackwards;
  if (!frame.imuOk) raw |= SafetyFault::kImuReadFailed;
  if (!isFinite(frame.accel) || !isFinite(frame.gyro)) raw |= SafetyFault::kImuNonFinite;

  // The state machine runs after this check, so a refusal shows up here one
  // cycle later, as a changed count. Comparing for "changed" and not "greater"
  // keeps working if the counter ever wraps.
  if (refusedTransitionCount != lastRefusedCount_) raw |= SafetyFault::kTransitionRefused;
  lastRefusedCount_ = refusedTransitionCount;

  // Lax: a raw fault becomes active only after holdUs_ on every check in a row.
  uint16_t active = 0;
  for (int i = 0; i < kNumFaults; ++i) {
    const uint16_t bit = static_cast<uint16_t>(1u << i);
    if ((raw & bit) == 0) continue;
    if ((raw_ & bit) == 0) rawSinceUs_[i] = frame.timeUs;  // just started
    const uint64_t lasted = frame.timeUs >= rawSinceUs_[i] ? frame.timeUs - rawSinceUs_[i] : 0;
    if (lasted >= holdUs_) active |= bit;
  }
  // Catastrophic: one IMU gap this long counts at once.
  if (!step.first && step.gapUs > catastrophicGapUs_) active |= SafetyFault::kImuGap;

  raw_ = raw;
  active_ = active;
  sticky_ |= active;
}
