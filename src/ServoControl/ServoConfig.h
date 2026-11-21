#pragma once

// Servo calibration and layout types. The values live in
// include/VehicleConfig.h. Tests build their own tables.
//
// UNITS EXCEPTION: servo calibration is in DEGREES and MICROSECONDS, not SI.
// The numbers come straight from the bench tools (pid-tuner `dump`,
// canard_bench `cal N`) and stay pasteable that way. ServoControl takes and
// returns radians at its interface and converts inside.

// One row per SERVO CHANNEL. Field order matches the tuner's `dump`.
struct ServoChannel {
  float kp, ki, kd;        // outer position loop on shaft degrees (-, 1/s, s)
  float zeroAngle;         // absolute encoder angle at canard neutral [deg]
  float direction;         // +1 or -1: encoder-positive to canard-positive
  float centerPulseUs;     // pulse at canard neutral [us]
  float pulseUsPerDegree;  // signed slope [us per shaft degree]
  float minAngle;          // shaft travel limits [deg]
  float maxAngle;
  float minPulseUs;        // pulse limits [us]
  float maxPulseUs;
  bool calibrated;         // true only once every number above is measured.
                           // An uncalibrated channel only ever gets centerPulseUs.
};

struct ServoConfig {
  ServoChannel channels[4];  // by servo channel
  int channelOfCanard[4];    // which servo channel drives canard 0..3
  float canardSign[4];       // +1 or -1 per canard: controller-positive to shaft-positive
  float servoLoopPeriodS;    // period of the encoder position loop [s]
  float servoModelTauS;      // time constant of the servo model [s]
};
