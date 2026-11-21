#pragma once

// Navigation: everything the flight logic knows about where the rocket is and
// how it is moving (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 5
// and 6.2).
//
// It wraps the Estimator and is the ONLY consumer of it:
//   - applies the sensor mount maps (sensor axes -> body axes);
//   - runs the estimator once per cycle;
//   - derives |specific force| straight from the IMU (launch detection must
//     never depend on the estimator);
//   - derives speed and tilt, and checks that the estimate is finite and
//     fresh.
//
// Every estimator-derived value has a validity flag. A caller must check the
// flag before using the value.

#include <cstdint>

#include "Estimator/Estimator.h"
#include "Frames.h"
#include "SensorSource.h"

class Navigation {
 public:
  explicit Navigation(Estimator& estimator);

  // Ask for estimator.reinitialize() at the start of the next update().
  // PreFlight calls this on entry (and again on a repeated EnterPreFlight).
  void requestReinitialize();

  // Once per cycle. `dt` is seconds since the previous sample (0 on the first
  // cycle). `onPad` is true in Init and PreFlight.
  void update(const SensorFrame& frame, float dt, bool onPad);

  // --- Straight from the IMU. Never from the estimator. --------------------

  // The IMU read succeeded and accel and gyro are finite.
  bool imuValid() const { return imuValid_; }
  // |specific force|, m/s^2. About 9.81 at rest. 0 if !imuValid().
  float specificForce() const { return specificForce_; }
  // Body-axis IMU data (mount map applied).
  const Vec3& accelBody() const { return accelBody_; }
  const Vec3& gyroBody() const { return gyroBody_; }

  // --- From the estimator. Check the flag first. ---------------------------

  // Attitude and body rate are usable: the estimator says so, the values are
  // finite, the quaternion is unit length, and the estimate is no older than
  // FlightConfig::kNavStaleUs.
  bool attitudeValid() const { return attitudeValid_; }
  const Quat& attitude() const { return state_.attitude; }  // body -> NED
  const Vec3& bodyRate() const { return state_.bodyRate; }  // rad/s
  // Angle between the nose and straight up, rad. Only if attitudeValid().
  float tilt() const { return tilt_; }

  // Inertial speed |velocityNed|, m/s. NOT airspeed: wind makes them differ.
  bool speedValid() const { return speedValid_; }
  float speed() const { return speed_; }

  // Altitude above the pad, m.
  bool altitudeValid() const { return altitudeValid_; }
  float altitudeAgl() const { return state_.altitudeAgl; }

  // The estimator's output as it arrived, for logging.
  const NavState& state() const { return state_; }
  // How many times the estimator has been re-initialised.
  uint32_t reinitializeCount() const { return reinitializeCount_; }

 private:
  Estimator& estimator_;
  NavState state_;
  Vec3 accelBody_;
  Vec3 gyroBody_;
  float specificForce_ = 0.0f;
  float tilt_ = 0.0f;
  float speed_ = 0.0f;
  bool imuValid_ = false;
  bool attitudeValid_ = false;
  bool speedValid_ = false;
  bool altitudeValid_ = false;
  bool reinitializePending_ = false;
  uint32_t reinitializeCount_ = 0;
};
