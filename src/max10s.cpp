#include "max10s.h"

static constexpr uint8_t MAX10S_I2C_ADDR = 0x42;

bool MAX10S::begin() {
  SerialUSB.print("Initializing MAX10S... ");

  // Hold the module out of reset.
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH);

  wire_.beginTransmission(MAX10S_I2C_ADDR);
  uint8_t i2cErr = wire_.endTransmission();
  if (i2cErr != 0) {
    // 2 = NACK on address (no device), others = bus error
    SerialUSB.printf("no I2C ACK at 0x%02X (err %u)... ", MAX10S_I2C_ADDR, i2cErr);
  }

  if (!gnss_.begin(wire_, MAX10S_I2C_ADDR)) {
    SerialUSB.println("FAILED");
    initialized_ = false;
    return false;
  }

  gnss_.setNavigationFrequency(NAV_RATE_HZ);
  gnss_.setAutoPVT(true);
  SerialUSB.println("OK");
  initialized_ = true;
  return true;
}

bool MAX10S::update(uint32_t now_ms) {
  if (!initialized_ || now_ms - lastUpdated_ < POLL_PERIOD_MS) {
    return false;
  }

  gnss_.checkUblox();

  data_.gpsLockType = gnss_.getFixType();
  data_.lat = gnss_.getLatitude() / 1e7;
  data_.lon = gnss_.getLongitude() / 1e7;

  if (gnss_.getNAVPOSECEF(20)) {
    data_.ecefX = gnss_.packetUBXNAVPOSECEF->data.ecefX * 0.01f;
    data_.ecefY = gnss_.packetUBXNAVPOSECEF->data.ecefY * 0.01f;
    data_.ecefZ = gnss_.packetUBXNAVPOSECEF->data.ecefZ * 0.01f;
  }

  data_.altMSL = gnss_.getAltitudeMSL() / 1000.0f;
  data_.altEllipsoid = gnss_.getAltitude() / 1000.0f;
  data_.velN = gnss_.getNedNorthVel();
  data_.velE = gnss_.getNedEastVel();
  data_.velD = gnss_.getNedDownVel();
  data_.epochTime = 0; // gnss_.getUnixEpoch();
  data_.satellites = gnss_.getSIV();

  lastUpdated_ = now_ms;
  return true;
}
