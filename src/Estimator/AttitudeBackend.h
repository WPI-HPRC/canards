#pragma once

// The attitude filter that PadAlignedEstimator drives: aligned once at rest on
// the pad, then propagated with the gyro. On the board this is the estimator
// owner's SimpleEstimator (SimpleEstimatorBackend); tests use a fake.
//
// Every vector is in the body frame (x out of the nose), SI units.

#include <cstddef>

#include "Frames.h"

class AttitudeBackend {
 public:
  virtual ~AttitudeBackend() = default;

  // How many at-rest gyro samples align() needs first.
  virtual size_t gyroSamplesNeeded() const = 0;

  // Adds one gyro sample [rad/s] taken at rest, for the bias. False if refused.
  virtual bool collectGyro(const Vec3& gyro) = 0;

  // Pad alignment from one accelerometer [any unit], magnetometer [any unit]
  // and pressure [Pa] sample. Also sets the gyro bias from the collected
  // samples and the altitude reference. False if it cannot align.
  virtual bool align(const Vec3& accel, const Vec3& mag, float pressurePa) = 0;

  // Propagates the attitude by one gyro sample [rad/s] over dt [s] > 0.
  virtual bool propagate(const Vec3& gyro, float dt) = 0;

  // Body -> NED, unit length.
  virtual Quat attitude() const = 0;

  // gyro minus the bias found by align() [rad/s].
  virtual Vec3 unbiasedGyro(const Vec3& gyro) const = 0;

  // Height above the pressure given to align() [m].
  virtual float heightAgl(float pressurePa) const = 0;
};
