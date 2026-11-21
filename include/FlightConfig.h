#pragma once

// Flight thresholds, canard limits and safety limits
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 6 and 9).
//
// PLACEHOLDER RULE: every value without an agreed number is tagged with the
// word PLACEHOLDER on its line. List what is left before flight with
//     grep -n PLACEHOLDER include/
// Never replace a placeholder with a guess. Ask whoever is named as the owner.
//
// No Arduino in this file. SI units; times in microseconds of SAMPLE time.

#include <cstdint>

namespace FlightConfig {

constexpr float kGravity = 9.80665f;         // m/s^2
constexpr float kDegToRad = 0.017453292519943295f;
constexpr uint64_t kUsPerSecond = 1000000ull;
constexpr uint64_t kUsPerMs = 1000ull;

// --- Launch detection (Init or PreFlight -> Boost) --------------------------
// |specific force| from the raw IMU, never from the estimator.
constexpr float kLaunchAccel = 5.0f * kGravity;       // decided: 5 g
constexpr uint64_t kLaunchHoldUs = 50 * kUsPerMs;     // PLACEHOLDER (user): 50 ms

// --- Boost -> Control -------------------------------------------------------
constexpr float kControlStartAltitudeM = 15.24f;            // decided: 50 ft AGL
constexpr uint64_t kControlStartHoldUs = 50 * kUsPerMs;     // PLACEHOLDER (user): hold time
constexpr uint64_t kBoostMaxUs = 2 * kUsPerSecond;          // PLACEHOLDER (user): "x seconds" fallback; 2 s is Canards-Rocket's old cut-off, no analysis

// --- Control -> Coast -------------------------------------------------------
// Must stay above kChuteSpeed.
constexpr float kControlEndSpeed = 40.0f;                   // PLACEHOLDER (user / GNC): m/s
constexpr uint64_t kControlEndHoldUs = 100 * kUsPerMs;      // PLACEHOLDER (user): hold time
constexpr uint64_t kControlMaxUs = 20 * kUsPerSecond;       // PLACEHOLDER (user): state timeout

// --- Coast -> Chute ---------------------------------------------------------
constexpr float kChuteSpeed = 30.0f;                        // PLACEHOLDER (user to confirm): m/s, "like 30"
constexpr uint64_t kChuteHoldUs = 100 * kUsPerMs;           // PLACEHOLDER (user): hold time
constexpr uint64_t kCoastMaxUs = 30 * kUsPerSecond;         // PLACEHOLDER (user): state timeout

// --- Chute -> Land ----------------------------------------------------------
constexpr float kLandAltitudeM = 30.0f;                     // PLACEHOLDER (user): m AGL, "nearing the ground"
constexpr uint64_t kLandHoldUs = 1 * kUsPerSecond;          // PLACEHOLDER (user): hold time
constexpr uint64_t kChuteMaxUs = 300 * kUsPerSecond;        // PLACEHOLDER (user): state timeout

// --- Ground tests (Init, PreFlight) ----------------------------------------
// A test command holds its angles this long, then the canards go back to
// neutral unless the command is repeated.
constexpr uint64_t kGroundTestHoldUs = 5 * kUsPerSecond;    // PLACEHOLDER (user)
constexpr float kGroundTestLimitRad = 7.5f * kDegToRad;     // decided: the linkage failsafe

// --- Canards in flight ------------------------------------------------------
constexpr float kCanardLimitRad = 4.0f * kDegToRad;         // PLACEHOLDER (GNC): 4 deg; re-check in sim (roll curve reverses near 4.2 deg, team data says 7)
// Attitude hold gives neutral below this speed (no control authority).
// 30 m/s is the lowest row of the LQR gain table.
constexpr float kMinControlSpeed = 30.0f;                   // PLACEHOLDER (GNC): m/s
// Attitude hold gives neutral if the navigation estimate is older than this.
constexpr uint64_t kNavStaleUs = 50 * kUsPerMs;             // PLACEHOLDER (user)

// Envelope cut-off: beyond either limit the canards latch neutral for the
// rest of the flight.
constexpr float kMaxTiltRad = 30.0f * kDegToRad;            // PLACEHOLDER (GNC): old placeholder
constexpr float kMaxPitchYawRate = 5.0f;                    // PLACEHOLDER (GNC): rad/s, old placeholder

// --- System safety ----------------------------------------------------------
// A gap between IMU samples longer than this raises a safety fault.
constexpr uint64_t kMaxImuGapUs = 25 * kUsPerMs;            // PLACEHOLDER (user): depends on the undecided IMU rate

// --- Compile-time sanity checks --------------------------------------------
static_assert(kControlEndSpeed > kChuteSpeed, "Control must end above the Chute speed");
static_assert(kCanardLimitRad <= kGroundTestLimitRad, "flight limit must not exceed the linkage failsafe");

}  // namespace FlightConfig
