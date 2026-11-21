#pragma once

// Conversions from the ST drivers' units to SI, in one place so every sensor
// source converts the same way. No Arduino.

#include <cstdint>

#include "Frames.h"

namespace SensorUnits {

constexpr float kGravity = 9.80665f;
constexpr float kPi = 3.14159265358979323846f;

// LSM6DSO32 Get_X_Axes(): milli-g -> m/s^2.
constexpr float accelFromMg(int32_t mg) { return static_cast<float>(mg) * (kGravity / 1000.0f); }
// LSM6DSO32 Get_G_Axes(): milli-degrees per second -> rad/s.
constexpr float gyroFromMdps(int32_t mdps) { return static_cast<float>(mdps) * (kPi / 180000.0f); }
// LIS2MDL GetAxes(): milligauss -> microtesla (1 G = 100 uT).
constexpr float magFromMgauss(int32_t mgauss) { return static_cast<float>(mgauss) * 0.1f; }
// LPS22HB GetPressure(): hPa -> Pa.
constexpr float pressureFromHpa(float hpa) { return hpa * 100.0f; }

inline Vec3 accelFromMg(const int32_t mg[3]) { return {accelFromMg(mg[0]), accelFromMg(mg[1]), accelFromMg(mg[2])}; }
inline Vec3 gyroFromMdps(const int32_t mdps[3]) {
  return {gyroFromMdps(mdps[0]), gyroFromMdps(mdps[1]), gyroFromMdps(mdps[2])};
}
inline Vec3 magFromMgauss(const int32_t mg[3]) {
  return {magFromMgauss(mg[0]), magFromMgauss(mg[1]), magFromMgauss(mg[2])};
}

}  // namespace SensorUnits
