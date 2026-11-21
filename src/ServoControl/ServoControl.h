#pragma once

// Canard angles <-> servo pulses
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 8.2).
//
//   pulse = centre + us_per_deg * (target + PID correction)
//
// The PID is an outer position loop on the encoder on the canard shaft. It is
// the bench tuner's PidController, unchanged, so bench-tuned gains stay valid.
//
// Ported from Canards-Rocket src/canards/CanardControl.cpp (steps 1, 3, 4).
// The attitude law is NOT here: the Canards subsystem calls measure(), then
// the attitude controller, then command(), in that order.
//
// Interface angles are radians, indexed by CANARD. Pulses are microseconds,
// indexed by SERVO CHANNEL. See lib/Actuators/Actuators.h for the indices.
//
// No Arduino, no heap, no clock: time enters only through `dt`.

#include "Actuators.h"
#include "ServoControl/PidController.h"
#include "ServoControl/ServoConfig.h"

// Where the canards are now, for the attitude controller.
struct CanardAngles {
  float rad[kNumCanards] = {};  // by canard
  // true:  all four encoders are usable and `rad` is measured.
  // false: at least one encoder is unusable, so ALL four values come from the
  //        servo model instead (the model is all-or-nothing on purpose).
  bool measured = false;
};

class ServoControl {
 public:
  // `config` must outlive this object.
  explicit ServoControl(const ServoConfig& config);

  // Forget servo-loop and servo-model history. Call when the servos are
  // powered up or control starts.
  void reset();

  // Encoders -> canard angles. Call once per cycle, before command().
  void measure(const ActuatorFeedback& feedback, CanardAngles& out);

  // Canard commands -> pulses. Call once per cycle while the servos are
  // powered. `canardRad` is by canard; `pulseUs` is by servo channel.
  //   - advances the servo model by `dt`;
  //   - runs the encoder position loop every ServoConfig::servoLoopPeriodS of
  //     accumulated `dt`, per channel, only on channels with a valid encoder
  //     (a channel without one runs open loop on its calibration);
  //   - clamps to the shaft travel limits and the pulse limits;
  //   - an uncalibrated channel gets its centre pulse, whatever is commanded;
  //   - a non-finite command gives that channel its centre pulse.
  void command(const float canardRad[kNumCanards], float dt, const ActuatorFeedback& feedback,
               float pulseUs[kNumCanards]);

  // The centre pulse of every channel (by servo channel). What to send when
  // the canards are not being driven.
  void centerPulses(float pulseUs[kNumCanards]) const;

  // True if every channel is calibrated.
  bool allCalibrated() const;

 private:
  const ServoConfig& config_;
  float modelCanardRad_[kNumCanards];          // by canard
  tuning::PidController servoLoop_[kNumCanards];  // by servo channel
  float correctionDeg_[kNumCanards];           // by servo channel
  float servoLoopElapsedS_;
  // channelOfCanard names four different channels that exist. If it does not,
  // nothing is driven: every channel stays on its centre pulse.
  bool layoutValid_;
};
