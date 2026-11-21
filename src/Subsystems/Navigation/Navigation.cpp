#include "Subsystems/Navigation/Navigation.h"

#include <cmath>

#include "FlightConfig.h"
#include "VehicleConfig.h"

Navigation::Navigation(Estimator& estimator) : estimator_(estimator) {}

void Navigation::requestReinitialize() { reinitializePending_ = true; }

void Navigation::update(const SensorFrame& frame, float dt, bool onPad) {
  // Sensor axes -> body axes. The frame stays in sensor axes all the way to
  // the log, so a replay picks up a corrected mount map here.
  accelBody_ = VehicleConfig::kImuToBody * frame.accel;
  gyroBody_ = VehicleConfig::kImuToBody * frame.gyro;
  const Vec3 magBody = VehicleConfig::kMagToBody * frame.mag;

  // |specific force| comes from the IMU alone: launch detection must work
  // whatever the estimator is doing. The magnitude is taken in sensor axes so
  // that even a wrong mount map cannot change it.
  imuValid_ = frame.imuOk && isFinite(frame.accel) && isFinite(frame.gyro) &&
              isFinite(accelBody_) && isFinite(gyroBody_);
  specificForce_ = imuValid_ ? norm(frame.accel) : 0.0f;
  if (!std::isfinite(specificForce_)) {
    // Finite components can still overflow when squared. A magnitude of
    // infinity must never reach launch detection as "above 5 g".
    imuValid_ = false;
    specificForce_ = 0.0f;
  }

  // Re-initialise before this cycle's update, so the estimator starts over
  // from this sample and not from the one after it.
  if (reinitializePending_) {
    reinitializePending_ = false;
    estimator_.reinitialize();
    ++reinitializeCount_;
  }

  // A bad IMU sample never reaches the estimator. The previous estimate is
  // kept and simply goes stale.
  if (imuValid_) {
    EstimatorInput in;
    in.timeUs = frame.timeUs;
    in.dt = dt;
    in.onPad = onPad;
    in.accelBody = accelBody_;
    in.gyroBody = gyroBody_;
    // A slow-sensor sample that is not a finite number is treated as "no new
    // sample", so it cannot poison the estimator.
    in.magFresh = frame.magFresh && isFinite(magBody);
    in.magBody = magBody;
    in.baroFresh = frame.baroFresh && std::isfinite(frame.pressurePa);
    in.pressurePa = frame.pressurePa;
    estimator_.update(in, state_);
  }

  // Fresh: the estimate is not from the future and not older than the limit.
  const bool fresh = state_.timeUs <= frame.timeUs &&
                     frame.timeUs - state_.timeUs <= FlightConfig::kNavStaleUs;

  attitudeValid_ = state_.attitudeValid && fresh && isFinite(state_.attitude) &&
                   isUnit(state_.attitude) && isFinite(state_.bodyRate);
  tilt_ = attitudeValid_ ? ::tilt(state_.attitude) : 0.0f;

  speedValid_ = state_.velocityValid && fresh && isFinite(state_.velocityNed);
  speed_ = speedValid_ ? norm(state_.velocityNed) : 0.0f;
  if (!std::isfinite(speed_)) {
    speedValid_ = false;
    speed_ = 0.0f;
  }

  altitudeValid_ = state_.altitudeValid && fresh && std::isfinite(state_.altitudeAgl);
}
