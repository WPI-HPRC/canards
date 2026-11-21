#pragma once

// An Estimator built from a pad-aligned, gyro-propagated attitude filter
// (AttitudeBackend), plus:
//   - velocity in NED by integrating the accelerometer once off the pad;
//   - altitude above the pad from the barometer.
//
// Written by the flight-software side on 2026-11-21 so the canards can be
// tested end to end before the estimator owner's full estimator lands. It is
// deliberately simple. Know its limits:
//   - Attitude is aligned from ONE accelerometer and ONE magnetometer sample
//     and is gyro-only afterwards: it drifts from the moment of alignment,
//     including while the rocket waits on the pad.
//   - Velocity is dead reckoning from the accelerometer, with no correction:
//     an accelerometer bias of b m/s^2 gives an error of b * t m/s. It is
//     inertial speed, not airspeed.
//   - Heading comes from the magnetometer, whose mount map is a placeholder.
//     Tilt does not depend on it.
//
// Sequence:
//   boot          nothing valid
//   reinitialize  start collecting at-rest gyro samples (only while onPad)
//   collected     once gyroSamplesNeeded() samples are in and a magnetometer
//                 and a barometer sample have been seen, align and run
//   running       attitude and body rate valid; velocity held at zero while
//                 onPad, integrated otherwise; altitude valid while the
//                 barometer is fresh (FlightConfig::kBaroStaleUs)
// If the rocket leaves the pad before alignment, nothing becomes valid. A
// failed propagation step invalidates attitude (and velocity) until the next
// reinitialize(): that sample's rotation is lost and cannot be recovered.
//
// No Arduino, no heap, no clock.

#include <cstddef>
#include <cstdint>

#include "Estimator/AttitudeBackend.h"
#include "Estimator/Estimator.h"

class PadAlignedEstimator final : public Estimator {
 public:
  enum class Phase : uint8_t { Idle, Collecting, Running };

  explicit PadAlignedEstimator(AttitudeBackend& backend) : backend_(backend) {}

  void reinitialize() override;
  void update(const EstimatorInput& in, NavState& out) override;

  Phase phase() const { return phase_; }
  size_t collected() const { return collected_; }

 private:
  AttitudeBackend& backend_;
  Phase phase_ = Phase::Idle;
  size_t collected_ = 0;
  bool propagateOk_ = true;
  Vec3 velocityNed_;

  bool haveMag_ = false;
  Vec3 lastMag_;
  bool haveBaro_ = false;
  float lastPressurePa_ = 0.0f;
  uint64_t baroTimeUs_ = 0;
};
