#include "ServoControl/ServoControl.h"

#include <cmath>

#include "ServoControl/ControlMath.h"

// Ported from Canards-Rocket src/canards/CanardControl.cpp, steps 1, 3 and 4
// of CanardControl::step(). Step 2 (the attitude law) is not here: the Canards
// subsystem runs it between measure() and command().
//
// What differs from the reference, on purpose:
//   - everything comes from the ServoConfig, nothing from global tables;
//   - the encoder is on the canard shaft, so there is no linkage maths;
//   - the interface is in radians (the calibration stays in degrees and us);
//   - the servo model and the position-loop cadence follow the real `dt`
//     instead of assuming a 5 ms cycle;
//   - a value that is not a finite number never reaches a pulse.

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kDegToRad = kPi / 180.0f;

// True if `map` sends the four canards to four different channels that exist.
bool isChannelMap(const int map[kNumCanards]) {
  bool taken[kNumCanards] = {};
  for (int canard = 0; canard < kNumCanards; ++canard) {
    const int ch = map[canard];
    if (ch < 0 || ch >= kNumCanards || taken[ch]) return false;
    taken[ch] = true;
  }
  return true;
}

// One channel's encoder reading as a shaft angle about its zero [deg],
// positive in the canard-positive direction. Returns false, and leaves
// `shaftDeg` alone, if the reading cannot be used: the channel's zero and
// direction are not known (uncalibrated), the encoder sample is not valid, or
// the result is not a finite number.
bool readShaftDeg(const ServoChannel& channel, const ActuatorFeedback& feedback, int ch,
                  float& shaftDeg) {
  if (!channel.calibrated || !feedback.encoderValid[ch]) return false;
  const float absoluteDeg = feedback.encoderAngleRad[ch] * kRadToDeg;
  // The tuner's expression: the shortest way round from the zero angle, so a
  // zero that sits near the encoder's 0/360 wrap does not jump. -180..+180 deg.
  const float deg = channel.direction * tuning::wrappedError(absoluteDeg, channel.zeroAngle);
  if (!std::isfinite(deg)) return false;
  shaftDeg = deg;
  return true;
}

// The servo model is a first-order lag. This is the share of the gap to the
// command that is still left after `dt`: exp(-dt / tau). The reference
// hard-codes 0.846482, which is this value for dt = 5 ms and tau = 30 ms.
float servoModelKeep(float dt, float tauS) {
  // Without a usable time constant there is no lag to model: follow the command.
  if (!(tauS > 0.0f)) return 0.0f;
  return std::exp(-dt / tauS);
}

}  // namespace

ServoControl::ServoControl(const ServoConfig& config)
    : config_(config), layoutValid_(isChannelMap(config.channelOfCanard)) {
  reset();
}

void ServoControl::reset() {
  for (int i = 0; i < kNumCanards; ++i) {
    modelCanardRad_[i] = 0.0f;
    correctionDeg_[i] = 0.0f;
    const ServoChannel& channel = config_.channels[i];
    // setGains() also clears the loop's history. It refuses a gain that is
    // negative, not finite or above the tuner's cap, and then changes nothing.
    // That channel keeps the zero gains it was built with (open loop on its
    // calibration), but its history still has to be cleared here.
    if (!servoLoop_[i].setGains(channel.kp, channel.ki, channel.kd)) servoLoop_[i].reset();
  }
  servoLoopElapsedS_ = 0.0f;
}

void ServoControl::measure(const ActuatorFeedback& feedback, CanardAngles& out) {
  out.measured = layoutValid_;
  if (layoutValid_) {
    for (int canard = 0; canard < kNumCanards; ++canard) {
      const int ch = config_.channelOfCanard[canard];
      float shaftDeg = 0.0f;
      if (readShaftDeg(config_.channels[ch], feedback, ch, shaftDeg)) {
        out.rad[canard] = config_.canardSign[canard] * shaftDeg * kDegToRad;
      } else {
        out.measured = false;
      }
    }
  }

  if (out.measured) {
    // Keep the model on the truth, so a later switch to the model starts from
    // where the canards really were.
    for (int i = 0; i < kNumCanards; ++i) modelCanardRad_[i] = out.rad[i];
  } else {
    // All-or-nothing: the attitude controller gets four values from one
    // source, never a mix of measured and modelled.
    for (int i = 0; i < kNumCanards; ++i) out.rad[i] = modelCanardRad_[i];
  }
}

void ServoControl::command(const float canardRad[kNumCanards], float dt,
                           const ActuatorFeedback& feedback, float pulseUs[kNumCanards]) {
  // Every channel starts at its centre pulse. Only a channel that passes every
  // check below is moved off it.
  centerPulses(pulseUs);
  if (!layoutValid_) return;

  // A cycle without a usable `dt` still sends pulses, but moves neither the
  // model nor the position loop.
  const bool dtUsable = std::isfinite(dt) && dt > 0.0f;

  bool runServoLoop = false;
  if (dtUsable) {
    // Servo model: one lag step towards the command, as the reference does
    // right after the attitude law. A command that is not a finite number
    // sends that servo to centre, so the model heads for neutral too.
    const float keep = servoModelKeep(dt, config_.servoModelTauS);
    for (int canard = 0; canard < kNumCanards; ++canard) {
      const float toward = std::isfinite(canardRad[canard]) ? canardRad[canard] : 0.0f;
      modelCanardRad_[canard] = keep * modelCanardRad_[canard] + (1.0f - keep) * toward;
    }

    // Position-loop cadence. The loop was tuned at a fixed period, and its
    // derivative filter is per sample, so it must run at that period whatever
    // the cycle rate is. Running once the accumulated time is within half a
    // cycle of the period picks the whole number of cycles nearest to it:
    // every 2nd cycle at 200 or 208 Hz, every cycle at 104 Hz.
    servoLoopElapsedS_ += dt;
    runServoLoop = servoLoopElapsedS_ >= config_.servoLoopPeriodS - 0.5f * dt;
  }

  for (int canard = 0; canard < kNumCanards; ++canard) {
    const int ch = config_.channelOfCanard[canard];
    const ServoChannel& channel = config_.channels[ch];

    // Where the shaft should go [deg], inside its travel limits.
    const float wanted = canardRad[canard];
    const float target = tuning::clamp(config_.canardSign[canard] * wanted * kRadToDeg,
                                       channel.minAngle, channel.maxAngle);
    // clamp() lets NaN through and turns an infinite command into a travel
    // limit, so the command is checked by itself. Comparing the target with
    // the limits again also catches limits that are NaN or the wrong way round.
    const bool drivable = channel.calibrated && std::isfinite(wanted) &&
                          target >= channel.minAngle && target <= channel.maxAngle;

    float measuredDeg = 0.0f;
    if (!drivable || !readShaftDeg(channel, feedback, ch, measuredDeg)) {
      // Nothing to correct, or no feedback to correct it with: open loop on
      // the calibration alone. The loop starts fresh when feedback returns.
      servoLoop_[ch].reset();
      correctionDeg_[ch] = 0.0f;
    } else if (runServoLoop) {
      // The output limits keep target + correction inside the shaft travel.
      correctionDeg_[ch] = servoLoop_[ch].update(target, measuredDeg, servoLoopElapsedS_,
                                                 channel.minAngle - target,
                                                 channel.maxAngle - target);
    }
    // Between loop runs the last correction is held.

    if (!drivable) continue;  // stays on its centre pulse

    const float pulse =
        channel.centerPulseUs + channel.pulseUsPerDegree * (target + correctionDeg_[ch]);
    const float limited = tuning::clamp(pulse, channel.minPulseUs, channel.maxPulseUs);
    // Same checks as for the target: a pulse that is not finite, or that the
    // limits cannot bound, is not sent. The channel stays on its centre pulse.
    if (std::isfinite(pulse) && limited >= channel.minPulseUs && limited <= channel.maxPulseUs) {
      pulseUs[ch] = limited;
    }
  }

  if (runServoLoop) servoLoopElapsedS_ = 0.0f;
}

void ServoControl::centerPulses(float pulseUs[kNumCanards]) const {
  for (int ch = 0; ch < kNumCanards; ++ch) pulseUs[ch] = config_.channels[ch].centerPulseUs;
}

bool ServoControl::allCalibrated() const {
  // A layout that does not name four different channels cannot drive anything.
  if (!layoutValid_) return false;
  for (int ch = 0; ch < kNumCanards; ++ch) {
    if (!config_.channels[ch].calibrated) return false;
  }
  return true;
}
