#pragma once

// AttitudeBackend over the estimator owner's SimpleEstimator
// (src/simple_estimator.*). Board-only: SimpleEstimator uses
// BasicLinearAlgebra, which includes Arduino.h, so the desktop tests use a
// fake backend instead. Only main.cpp should include this header.

#if defined(ARDUINO)

#include "Estimator/AttitudeBackend.h"
#include "simple_estimator.h"

class SimpleEstimatorBackend final : public AttitudeBackend {
 public:
  size_t gyroSamplesNeeded() const override { return SimpleEstimator::gyro_buffer_size; }
  bool collectGyro(const Vec3& gyro) override;
  bool align(const Vec3& accel, const Vec3& mag, float pressurePa) override;
  bool propagate(const Vec3& gyro, float dt) override;
  Quat attitude() const override;
  Vec3 unbiasedGyro(const Vec3& gyro) const override;
  float heightAgl(float pressurePa) const override;

 private:
  SimpleEstimator estimator_;
};

#endif  // ARDUINO
