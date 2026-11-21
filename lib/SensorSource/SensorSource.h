#pragma once

// Where sensor data comes from. Three implementations feed the same flight
// code (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 7):
//   LiveSensorSource   the real sensors on the board
//   UsbStreamSource    simulated data streamed from a host over USB
//   SdReplaySource     a flight log replayed from the SD card
//
// A SensorFrame holds one IMU sample plus whatever slower sensors produced a
// new value since the previous frame. Values are in SI units and in SENSOR
// axes: the mount rotation is applied later, in Navigation, so that replaying
// an old log picks up a corrected mount map.
//
// `timeUs` is the only clock the flight logic ever sees. Nothing downstream of
// a SensorSource may call millis() or micros().

#include <cstdint>

#include "Frames.h"

struct SensorFrame {
  // IMU sample time in microseconds. 64-bit: a 32-bit counter wraps after
  // 71 minutes and the rocket can sit on the pad for longer.
  uint64_t timeUs = 0;

  bool imuOk = false;  // the IMU read succeeded; accel and gyro are real data
  Vec3 accel;          // specific force, m/s^2, IMU sensor axes
  Vec3 gyro;           // angular rate, rad/s, IMU sensor axes

  bool magFresh = false;  // `mag` is a new sample in this frame
  Vec3 mag;               // magnetic field, uT, magnetometer sensor axes

  bool baroFresh = false;      // `pressurePa` is a new sample in this frame
  float pressurePa = 0.0f;     // Pa
  float temperatureC = 0.0f;   // deg C, from the barometer
};

class SensorSource {
 public:
  virtual ~SensorSource() = default;

  // If a new IMU sample is available, fills `out` and returns true. Otherwise
  // returns false at once and leaves `out` alone. Never blocks.
  virtual bool read(SensorFrame& out) = 0;
};
