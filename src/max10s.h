#pragma once

// u-blox MAX-M10S GPS over I2C.
// Ported from rocket-base-2026 (src/boilerplate/Sensors/Impl/MAX10S.h),
// without the boilerplate Sensor<> framework.

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

struct MAX10SData {
  double lat;          // degrees
  double lon;          // degrees
  float ecefX;         // m
  float ecefY;         // m
  float ecefZ;         // m
  float altMSL;        // m
  float altEllipsoid;  // m
  int32_t velN;        // mm/s
  int32_t velE;        // mm/s
  int32_t velD;        // mm/s
  uint32_t epochTime;  // not filled in yet (always 0), same as rocket-base
  uint8_t satellites;  // satellites in view
  uint8_t gpsLockType; // u-blox fix type: 0 none, 2 2D, 3 3D, ...
};

class MAX10S {
public:
  // Navigation solution rate the module is configured for.
  static constexpr uint8_t NAV_RATE_HZ = 25;
  static constexpr uint32_t POLL_PERIOD_MS = 1000 / NAV_RATE_HZ;

  explicit MAX10S(TwoWire &wire) : wire_(wire) {}

  // Call after wire.begin(). Returns false if the module does not respond.
  bool begin();

  // Reads new data at most every POLL_PERIOD_MS. Returns true if `data()`
  // was updated by this call.
  bool update(uint32_t now_ms = millis());

  const MAX10SData &data() const { return data_; }
  // millis() of the last update, 0 if there has been none.
  uint32_t lastUpdated() const { return lastUpdated_; }

private:
  SFE_UBLOX_GNSS gnss_;
  TwoWire &wire_;
  MAX10SData data_{};
  uint32_t lastUpdated_ = 0;
  bool initialized_ = false;
};
