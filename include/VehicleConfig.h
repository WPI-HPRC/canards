#pragma once

// Facts about this airframe and this board's wiring that the flight logic
// needs: sensor mount maps, servo calibration and the canard -> channel map
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 5, 8.1, 8.2, 8.3).
//
// PLACEHOLDER RULE: every value that is not confirmed is tagged with the word
// PLACEHOLDER on its line. List what is left before flight with
//     grep -n PLACEHOLDER include/
//
// No Arduino in this file. Pin numbers are not here: they live with the
// hardware code in lib/Actuators and lib/SensorSource.

#include "Frames.h"
#include "ServoControl/ServoConfig.h"

namespace VehicleConfig {

// --- Sensor mount maps: body = map * sensor --------------------------------
// Body frame: x out of the nose (lib/Frames/Frames.h).

// LSM6DSO32 (the IMU that flies; the ASM330 on this board is dead).
// Measured from gravity readings, determinant +1.
constexpr Mat3 kImuToBody = {{{0.0f, 0.0f, 1.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}}};

// LIS2MDL magnetometer.
constexpr Mat3 kMagToBody = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};  // PLACEHOLDER (EE / user): identity, not measured

// --- Canard layout -----------------------------------------------------------
// Canard i (controller numbering) sits a quarter turn * i from body +y towards
// body +z. Measured on the airframe: CH1 on +y, CH3 on +z, CH4 on -y, CH2 on
// -z. So canard 0..3 = CH1, CH3, CH4, CH2.
// This assumes CH1 -> CH3 is a quarter turn CLOCKWISE seen from the tail. If it
// is anticlockwise the layout is mirrored and pitch/yaw mixing is wrong.
// The map itself is `channelOfCanard` in kServoConfig below.

// --- Servo calibration, by SERVO CHANNEL (CH1..CH4) ---------------------------
// Bench values from 2026-11-20 (canard_bench `cal N`, Canards-Rocket
// src/canards/CanardConfig.h, uncommitted there). On every channel a longer
// pulse turns the canard anticlockwise seen from the tail, so us/deg is
// negative. The us/deg slope is a two-point measurement over about +/-2 deg.
//
// Every row is marked NOT calibrated, so each channel only ever gets its
// centre pulse and no canard moves off neutral. The handoff says to trust
// these rows only after the canard numbering above is confirmed. Once it is,
// set `calibrated` to true on each row.
//
// Servo PID gains are zero: nothing has been tuned, so a calibrated channel
// runs open loop on its calibration.
//
constexpr ServoConfig kServoConfig = {
    // channels: kp ki kd  zeroAngle direction centerPulseUs usPerDeg  minAngle maxAngle  minPulseUs maxPulseUs  calibrated
    {
        {0.0f, 0.0f, 0.0f, 105.835f, 1.0f, 1477.3f, -17.18f, -10.0f, 10.0f, 1305.0f, 1649.0f, false},  // CH1, +y  PLACEHOLDER (bench / user)
        {0.0f, 0.0f, 0.0f, 24.207f, 1.0f, 1386.3f, -18.17f, -10.0f, 10.0f, 1205.0f, 1568.0f, false},   // CH2, -z  PLACEHOLDER (bench / user)
        {0.0f, 0.0f, 0.0f, 289.523f, 1.0f, 1451.3f, -22.10f, -10.0f, 10.0f, 1230.0f, 1672.0f, false},  // CH3, +z  PLACEHOLDER (bench / user)
        {0.0f, 0.0f, 0.0f, 342.260f, 1.0f, 1487.3f, -18.97f, -10.0f, 10.0f, 1298.0f, 1677.0f, false},  // CH4, -y  PLACEHOLDER (bench / user)
    },
    {0, 2, 3, 1},              // channelOfCanard  PLACEHOLDER (user, on the airframe): handedness unconfirmed
    {1.0f, 1.0f, 1.0f, 1.0f},  // canardSign  PLACEHOLDER (GNC): check each canard with the ground test
    0.010f,                    // servoLoopPeriodS: the encoder position loop runs at 100 Hz
    0.030f,                    // servoModelTauS  PLACEHOLDER (bench): 30 ms servo lag, needs bench data
};

}  // namespace VehicleConfig
