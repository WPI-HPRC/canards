#pragma once

// The canard servos and their encoders, as the flight logic sees them
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 5, 6.2 and 8.3).
//
// THREE DIFFERENT INDICES exist. Do not mix them:
//   canard         0..3  controller numbering: canard i sits a quarter turn * i
//                        from body +y towards body +z
//   servo channel  0..3  CH1..CH4, the servo outputs on the board
//   encoder input  0..3  the capture inputs on the board
// Everything in THIS header is indexed by SERVO CHANNEL. The canard -> channel
// map lives in VehicleConfig.h and is applied in ServoControl. The
// channel -> encoder-input map is a wiring fact and is applied inside the
// hardware implementation (MarsActuators), so it never reaches flight logic.
//
// Implementations: MarsActuators (the board), and the replay / stream rigs,
// which feed recorded or simulated feedback.

#include <cstdint>

constexpr int kNumCanards = 4;

// What the four shaft encoders read, by servo channel.
struct ActuatorFeedback {
  // Absolute encoder angle in [0, 2*pi), before zero offset and direction.
  float encoderAngleRad[kNumCanards] = {};
  // The reading is fresh and decoded. If false, ignore the angle.
  bool encoderValid[kNumCanards] = {};
};

// What the flight logic sends to the servos, by servo channel.
struct ActuatorCommand {
  bool servoPower = false;            // false: servo power rail off
  float pulseUs[kNumCanards] = {};    // pulse width, microseconds
};

class Actuators {
 public:
  virtual ~Actuators() = default;

  // Latest encoder readings. Never blocks.
  virtual void readFeedback(ActuatorFeedback& out) = 0;

  // Applies power and pulses. Never blocks. An implementation must treat any
  // non-finite pulse as that channel's centre pulse.
  virtual void write(const ActuatorCommand& command) = 0;
};
