#if defined(ARDUINO)

#include "LiveSensorSource.h"

#include "SensorUnits.h"

namespace {

// LSM6DSO32 registers (datasheet DS13202).
constexpr uint8_t kRegInt1Ctrl = 0x0D;   // bit 1: INT1_DRDY_G
constexpr uint8_t kRegStatus = 0x1E;     // bit 0: XLDA, bit 1: GDA
constexpr uint8_t kInt1DrdyG = 0x02;
constexpr uint8_t kStatusGda = 0x02;

// Written only by the interrupt.
volatile uint32_t gIsrCount = 0;
volatile uint32_t gIsrMicros = 0;

void onGyroReady() {
  gIsrMicros = micros();
  gIsrCount = gIsrCount + 1;
}

}  // namespace

LiveSensorSource::LiveSensorSource()
    : spi_(SENSORS_SPI_MOSI, SENSORS_SPI_MISO, SENSORS_SPI_SCK),
      lsm_(&spi_, SENSORS_LSM_CS),
      lis_(&spi_, SENSORS_LIS_CS),
      lps_(&spi_, SENSORS_LPS_CS) {}

bool LiveSensorSource::begin() {
  // Every chip on the bus deselected before the bus starts, the dead ASM330
  // included, so it cannot answer for another one.
  for (uint32_t cs : {SENSORS_ASM_CS, SENSORS_LSM_CS, SENSORS_LIS_CS, SENSORS_LPS_CS}) {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
  }
  spi_.begin();
  delay(20);

  status_.lsm = lsm_.begin() == LSM6DSO32_OK && lsm_.ReadID(&status_.lsmId) == LSM6DSO32_OK &&
                status_.lsmId == LSM6DSO32_ID;
  lsm_.Set_G_FS(2000);  // dps
  lsm_.Set_X_FS(32);    // g
  lsm_.Set_G_ODR(kImuOdrHz);
  lsm_.Set_X_ODR(kImuOdrHz);
  lsm_.Enable_X();
  lsm_.Enable_G();

  status_.lis = lis_.begin() == LIS2MDL_OK && lis_.ReadID(&status_.lisId) == LIS2MDL_OK &&
                status_.lisId == LIS2MDL_ID;
  lis_.SetOutputDataRate(100.0f);
  lis_.Enable();

  status_.lps = lps_.begin() == LPS22HB_STATUS_OK && lps_.ReadID(&status_.lpsId) == LPS22HB_STATUS_OK &&
                status_.lpsId == LPS22HB_WHO_AM_I_VAL;
  lps_.SetODR(75.0f);
  lps_.Enable();

  // Gyro data-ready on INT1. Attach before routing, so the first edge is not
  // lost. read() polls the status register as well, so a missed edge can
  // never stall sampling.
  pinMode(LSM_INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(LSM_INT1), onGyroReady, RISING);
  lsm_.Write_Reg(kRegInt1Ctrl, kInt1DrdyG);

  nowUs();
  return status_.lsm;
}

bool LiveSensorSource::read(SensorFrame& out) {
  const uint32_t now32 = micros();
  const uint64_t now64 = clock_.extend(now32);

  uint8_t st = 0;
  if (lsm_.Read_Reg(kRegStatus, &st) != LSM6DSO32_OK || (st & kStatusGda) == 0) return false;

  // The sample's time: from the interrupt if it fired for this sample.
  noInterrupts();
  const uint32_t isrCount = gIsrCount;
  const uint32_t isrMicros = gIsrMicros;
  interrupts();
  const uint32_t edges = isrCount - lastIsrCount_;
  lastIsrCount_ = isrCount;
  if (edges > 1) missed_ += edges - 1;

  SensorFrame f;
  f.timeUs = edges > 0 ? MicrosClock64::backdate(now64, now32, isrMicros) : now64;

  int32_t accel[3] = {}, gyro[3] = {};
  f.imuOk = lsm_.Get_X_Axes(accel) == LSM6DSO32_OK;
  f.imuOk = (lsm_.Get_G_Axes(gyro) == LSM6DSO32_OK) && f.imuOk;
  f.accel = SensorUnits::accelFromMg(accel);
  f.gyro = SensorUnits::gyroFromMdps(gyro);

  if (!haveMag_ || now64 - lastMagUs_ >= kMagPeriodUs) {
    lastMagUs_ = now64;
    haveMag_ = true;
    int32_t mag[3] = {};
    if (lis_.GetAxes(mag) == LIS2MDL_OK) {
      f.magFresh = true;
      f.mag = SensorUnits::magFromMgauss(mag);
    }
  }

  if (!haveBaro_ || now64 - lastBaroUs_ >= kBaroPeriodUs) {
    lastBaroUs_ = now64;
    haveBaro_ = true;
    float hpa = 0.0f, celsius = 0.0f;
    if (lps_.GetPressure(&hpa) == LPS22HB_STATUS_OK && lps_.GetTemperature(&celsius) == LPS22HB_STATUS_OK) {
      f.baroFresh = true;
      f.pressurePa = SensorUnits::pressureFromHpa(hpa);
      f.temperatureC = celsius;
    }
  }

  out = f;
  return true;
}

#endif  // ARDUINO
