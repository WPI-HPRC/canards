#pragma once

// GNC GO / NO-GO for the pad: whether the flight program is ready to control
// a flight that launches now. Sent in the telemetry ("gnc_go", "nogo") so the
// ground station shows it; it decides nothing on the rocket. GO only when
// nogo() == 0. No Arduino, no heap.

#include <cstdint>

#include "FlightConfig.h"

namespace GoNoGo {

// The reasons, as bits of nogo().
enum : uint16_t {
  kNotPreFlight = 1,     // not armed (armFlight puts it in PreFlight)
  kGreeting = 2,         // the PreFlight Morse greeting is still running
  kNavNotReady = 4,      // IMU, attitude, speed or altitude not valid (alignment not done)
  kSensorDown = 8,       // IMU, magnetometer, barometer or SD log not OK
  kSafetyFault = 16,     // any sticky safety fault since boot
  kCanardsNotReady = 32, // not enabled, locked out, cut off, or unpowered
  kTilted = 64,          // more than kGoMaxTiltRad from vertical
  kEncoderBad = 128,     // an encoder reading is not valid
};

struct Inputs {
  bool inPreFlight = false;
  bool greetingRunning = false;
  bool imuValid = false, attitudeValid = false, speedValid = false, altitudeValid = false;
  bool imuOk = false, magOk = false, baroOk = false, sdOk = false;
  uint16_t safetySticky = 0;
  bool canardsEnabled = false, lockedOut = false, cutoffLatched = false, powered = false;
  float tiltRad = 0.0f;
  bool encodersValid = false;  // all four
};

inline uint16_t nogo(const Inputs& in) {
  uint16_t bits = 0;
  if (!in.inPreFlight) bits |= kNotPreFlight;
  if (in.greetingRunning) bits |= kGreeting;
  if (!(in.imuValid && in.attitudeValid && in.speedValid && in.altitudeValid)) bits |= kNavNotReady;
  if (!(in.imuOk && in.magOk && in.baroOk && in.sdOk)) bits |= kSensorDown;
  if (in.safetySticky != 0) bits |= kSafetyFault;
  if (!in.canardsEnabled || in.lockedOut || in.cutoffLatched || !in.powered) bits |= kCanardsNotReady;
  if (!(in.tiltRad <= FlightConfig::kGoMaxTiltRad)) bits |= kTilted;  // NaN counts as tilted
  if (!in.encodersValid) bits |= kEncoderBad;
  return bits;
}

}  // namespace GoNoGo
