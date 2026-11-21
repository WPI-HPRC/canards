#pragma once

// The boundary between the flight program and the state estimator
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.5).
//
// The estimator has its own owner. The flight program only calls this
// interface and only reads NavState. To plug an estimator in, implement
// `Estimator` and hand the object to the FlightComputer.
//
// Conventions (see lib/Frames/Frames.h):
//   - Body frame: x out of the nose. Inertial frame: NED.
//   - `attitude` is a unit Hamilton quaternion [w, x, y, z] that rotates body
//     vectors into NED.
//   - SI units throughout.
//   - The only clock is `timeUs`, the IMU sample time. An estimator must not
//     read millis()/micros(), or a replayed flight will not reproduce.

#include <cstdint>

#include "Frames.h"

struct EstimatorInput {
  uint64_t timeUs = 0;  // IMU sample time, microseconds
  float dt = 0.0f;      // seconds since the previous sample (0 on the first)
  bool onPad = false;   // true in Init and PreFlight while the raw IMU reads
                        // below launch-level specific force: the rocket is at rest

  Vec3 accelBody;  // specific force, m/s^2, body axes
  Vec3 gyroBody;   // angular rate, rad/s, body axes

  bool magFresh = false;  // magBody is a new sample this cycle
  Vec3 magBody;           // magnetic field, uT, body axes

  bool baroFresh = false;    // pressurePa is a new sample this cycle
  float pressurePa = 0.0f;   // static pressure, Pa
};

struct NavState {
  uint64_t timeUs = 0;  // sample time this state is valid for

  Quat attitude;        // body -> NED
  Vec3 bodyRate;        // rad/s, body axes, bias removed
  Vec3 velocityNed;     // m/s
  float altitudeAgl = 0.0f;  // m above the pad

  // A consumer must ignore any field whose flag is false.
  bool attitudeValid = false;  // covers `attitude` and `bodyRate`
  bool velocityValid = false;
  bool altitudeValid = false;
};

class Estimator {
 public:
  virtual ~Estimator() = default;

  // Start over: gyro bias, pad alignment and altitude reference. Called on
  // entering PreFlight, with the rocket at rest on the pad. May be called
  // more than once.
  virtual void reinitialize() = 0;

  // Called once per IMU sample. Fill `out` completely; clear a valid flag
  // whenever the corresponding field cannot be trusted.
  virtual void update(const EstimatorInput& in, NavState& out) = 0;
};
