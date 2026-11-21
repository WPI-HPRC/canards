#if defined(ARDUINO)

#include "Estimator/SimpleEstimatorBackend.h"

namespace {

BLA::Matrix<3, 1> toBla(const Vec3& v) { return {v.x, v.y, v.z}; }
Vec3 fromBla(const BLA::Matrix<3, 1>& m) { return {m(0), m(1), m(2)}; }

}  // namespace

bool SimpleEstimatorBackend::collectGyro(const Vec3& gyro) { return estimator_.initialize_loop(toBla(gyro)); }

bool SimpleEstimatorBackend::align(const Vec3& accel, const Vec3& mag, float pressurePa) {
  return estimator_.initialize(toBla(accel), toBla(mag), pressurePa);
}

bool SimpleEstimatorBackend::propagate(const Vec3& gyro, float dt) {
  return estimator_.flight_loop(toBla(gyro), dt);
}

Quat SimpleEstimatorBackend::attitude() const {
  const BLA::Matrix<4, 1> q = estimator_.get_orientation_quaternion();
  return {q(0), q(1), q(2), q(3)};
}

Vec3 SimpleEstimatorBackend::unbiasedGyro(const Vec3& gyro) const {
  return fromBla(estimator_.unbiased_gyro(toBla(gyro)));
}

float SimpleEstimatorBackend::heightAgl(float pressurePa) const { return estimator_.get_height(pressurePa); }

#endif  // ARDUINO
