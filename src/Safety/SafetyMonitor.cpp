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
  uint16_t active = 0;

  if (!step.first && step.gapUs > FlightConfig::kMaxImuGapUs) active |= SafetyFault::kImuGap;
  if (step.backwards) active |= SafetyFault::kTimeBackwards;
  if (!frame.imuOk) active |= SafetyFault::kImuReadFailed;
  if (!isFinite(frame.accel) || !isFinite(frame.gyro)) active |= SafetyFault::kImuNonFinite;

  // The state machine runs after this check, so a refusal shows up here one
  // cycle later, as a changed count. Comparing for "changed" and not "greater"
  // keeps working if the counter ever wraps.
  if (refusedTransitionCount != lastRefusedCount_) active |= SafetyFault::kTransitionRefused;
  lastRefusedCount_ = refusedTransitionCount;

  active_ = active;
  sticky_ |= active;
}
