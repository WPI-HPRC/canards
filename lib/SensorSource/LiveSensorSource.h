#pragma once

// The real sensors on the MARS v2.1 board, as a SensorSource
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 8.3).
//
//   IMU   LSM6DSO32 (CS PB4), gyro and accelerometer both at kImuOdrHz.
//         The ASM330 on this board reads WHO_AM_I 0xFF and is not used.
//         A new sample is the gyro data-ready bit; its time is taken in the
//         INT1 (PB3) interrupt when that fired for it, else when it is read.
//   Mag   LIS2MDL (CS PA15), read every kMagPeriodUs.
//   Baro  LPS22HB (CS PD0), read every kBaroPeriodUs.
// Values are in SENSOR axes and SI units (SensorUnits.h); the mount maps are
// applied later, in Navigation.
//
// Board only: everything is inside #if defined(ARDUINO).

#if defined(ARDUINO)

#include <Arduino.h>
#include <SPI.h>

#include "LIS2MDLSensor.h"
#include "LPS22HBSensor.h"
#include "LSM6DSO32Sensor.h"
#include "MicrosClock64.h"
#include "SensorSource.h"

class LiveSensorSource final : public SensorSource {
 public:
  // The LSM6DSO32 has no 200 Hz setting; 208 Hz is the closest to the 5 ms
  // step the LQR gains were designed for. PLACEHOLDER (user / GNC): the
  // control rate is this rate.
  static constexpr float kImuOdrHz = 208.0f;
  static constexpr uint64_t kMagPeriodUs = 10000;   // LIS2MDL at 100 Hz
  static constexpr uint64_t kBaroPeriodUs = 13333;  // LPS22HB at 75 Hz

  struct Status {
    bool lsm = false;   // began and WHO_AM_I matched
    bool lis = false;
    bool lps = false;
    uint8_t lsmId = 0, lisId = 0, lpsId = 0;
  };

  LiveSensorSource();

  // Configures the bus and the sensors and attaches the data-ready interrupt.
  // Call once from setup(). Returns true if the IMU is usable.
  bool begin();

  bool read(SensorFrame& out) override;

  const Status& status() const { return status_; }
  // Gyro samples that arrived (interrupt) but were overwritten before read()
  // got to them. A rising count means the loop is too slow for kImuOdrHz.
  uint32_t missedSamples() const { return missed_; }
  // Microseconds now, on the same 64-bit clock as SensorFrame::timeUs.
  uint64_t nowUs() { return clock_.extend(micros()); }

 private:
  SPIClass spi_;
  LSM6DSO32Sensor lsm_;
  LIS2MDLSensor lis_;
  LPS22HBSensor lps_;
  MicrosClock64 clock_;
  Status status_;
  uint32_t lastIsrCount_ = 0;
  uint32_t missed_ = 0;
  uint64_t lastMagUs_ = 0;
  uint64_t lastBaroUs_ = 0;
  bool haveMag_ = false;
  bool haveBaro_ = false;
};

#endif  // ARDUINO
