#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <Servo.h>


#include "LSM6DSO32Sensor.h"
#include "ASM330LHHSensor.h"
#include "LIS2MDLSensor.h"
#include "LPS22HBSensor.h"

#include "LoRaE22.h"
#include "RadioConfigs.h"

#include "PwmInput.h"

#include "sdcard.h"
#include "max10s.h"
#include "telemetry.h"

#include "simple_estimator.h"

// Debug output over SerialUSB, only in DEBUG builds (the marsv21_debug /
// marsv20_debug envs). Comment out what you don't need.
#ifdef DEBUG
#define SENSOR_CHECK   // at boot: per-device init result, WHO_AM_I check and a summary
#define SD_TEST        // at boot: write/read-back test of a CSV on the SD card
#define SENSOR_PRINT   // in loop: IMU, mag and baro readings    } both on one line,
#define GPS_PRINT      // in loop: GPS fix, position and velocity } see debugPrint()
#define RADIO_DEBUG
#endif

// DEBUG builds wait this long at boot for a serial monitor to connect, then
// carry on without one so the board can't get stuck waiting for USB.
static constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 60UL * 1000UL;

static constexpr uint32_t DEBUG_PRINT_PERIOD_MS = 200;

// Initialize the estimator object
SimpleEstimator estimator = SimpleEstimator();
float prev_time_s = 0.0;


// initialize our sensor buses 
SPIClass SENSORS_SPI(SENSORS_SPI_MOSI, SENSORS_SPI_MISO, SENSORS_SPI_SCK);
TwoWire GPS_I2C(GPS_I2C_SDA, GPS_I2C_SCL);
HardwareSerial GPS_SERIAL(GPS_SERIAL_RX, GPS_SERIAL_TX);
TwoWire CONNECTOR_I2C(CONNECTOR_I2C_SDA, CONNECTOR_I2C_SCL);
SPIClass CAMERA_SPI(CAMERA_MOSI, CAMERA_MISO, CAMERA_SCK);
HardwareSerial RADIO_SERIAL(RADIO_SERIAL_RX, RADIO_SERIAL_TX);

LSM6DSO32Sensor lsm(&SENSORS_SPI, SENSORS_LSM_CS);
ASM330LHHSensor asmSens(&SENSORS_SPI, SENSORS_ASM_CS);
LIS2MDLSensor lis(&SENSORS_SPI, SENSORS_LIS_CS);
LPS22HBSensor lps(&SENSORS_SPI, SENSORS_LPS_CS);
MAX10S gps(GPS_I2C);


const char* callsign = "KV0R";
LoRaE22 radioModule(&RADIO_SERIAL, RADIO_M0, RADIO_M1, RADIO_AUX, callsign);

// Implement this per your platform, then pass a callback to it in setup().
// This is a MARS SPECIFIC implementation.
bool changeSerialPortConfig(RadioConfigTypes::SerialSpeeds baudRate, RadioConfigTypes::ParityConfig parity){
  // this is safe to call even when the port is not open.
  RADIO_SERIAL.end();

  uint32_t baud = 0;
  uint16_t parityConfig = 0;
  
  // the radio's baud rates don't follow any pattern over the entire range, so ugly switch statement it is
  switch(baudRate){
    case RadioConfigTypes::SerialSpeeds::BAUD_1200:
      baud = 1200; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_2400:
      baud = 2400; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_4800:
      baud = 4800; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_9600:
      baud = 9600; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_19200:
      baud = 19200; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_38400:
      baud = 38400; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_57600:
      baud = 57600; break;
    case RadioConfigTypes::SerialSpeeds::BAUD_115200:
      baud = 115200; break;
  };
  // this is just easier
  switch(parity){
    case RadioConfigTypes::ParityConfig::Parity_8N1:
      parityConfig = SERIAL_8N1; break;
    case RadioConfigTypes::ParityConfig::Parity_8E1:
      parityConfig = SERIAL_8E1; break;
    case RadioConfigTypes::ParityConfig::Parity_8O1:
      parityConfig = SERIAL_8O1; break;
  };
  
  RADIO_SERIAL.begin(baud, parityConfig);

  return true;
};


void sensorInit();
void sensorUpdate();
bool radioInit();
void gpsInit();
void gpsUpdate();
void printSensorSummary();
void debugPrint();

// Init results, for the boot summary.
struct DeviceStatus {
  bool sd = false;
  bool sdTest = false;
  bool gps = false;
  bool lsm = false;
  bool asm330 = false;
  bool lis = false;
  bool lps = false;
  int8_t radioCode = -1; // LoRaE22::init(): 0 ok, 1 ok after programming, -1 failed
};
DeviceStatus deviceStatus;

// Latest sensor readings, filled by sensorUpdate().
struct SensorReadings {
  int32_t lsmAccel[3] = {};  // mg
  int32_t lsmGyro[3] = {};   // mdps
  int32_t asmAccel[3] = {};  // mg
  int32_t asmGyro[3] = {};   // mdps
  int32_t mag[3] = {};       // mgauss
  float pressure = 0;        // hPa
  float temperature = 0;     // C
  bool lsmOk = false, asmOk = false, lisOk = false, lpsOk = false;
};
static SensorReadings readings;

// ------------------------------------------------------------- commands --
// Set from the ground with the telemetry commands below.
static bool flightArmed = false;
static bool actuatorTestActive = false;
static uint32_t actuatorTestStart = 0;
static constexpr uint32_t ACTUATOR_TEST_DURATION_MS = 3000;

// "$C,armFlight": arm for flight. Stays armed until reboot.
static bool onArmFlight() {
  flightArmed = true;
  return true;
}

// "$C,actuatorTest": run the actuator test (see loop()). Refused while armed
// or while a test is already running.
static bool onActuatorTest() {
  if (flightArmed || actuatorTestActive) {
    return false;
  }
  actuatorTestActive = true;
  actuatorTestStart = millis();
  return true;
}

// ------------------------------------------------------------ telemetry --
// Columns of the radio downlink, sent every Telemetry::ROW_PERIOD_MS. To send
// something new, add a line here; the header the ground station sees follows
// the table. Keep it short: this is airtime, not the SD log. A full row should
// stay well under MAX_MESSAGE_SIZE (232) characters.
static uint32_t tlmSeq = 0;  // incremented per row, so the ground can see gaps
static const TelemetryColumn tlmColumns[] = {
  TLM_UINT("time_ms", millis()),
  TLM_UINT("seq", tlmSeq++),
  TLM_UINT("armed", flightArmed),
  TLM_UINT("act_test", actuatorTestActive),

  TLM_INT("ax_mg", readings.lsmAccel[0]),
  TLM_INT("ay_mg", readings.lsmAccel[1]),
  TLM_INT("az_mg", readings.lsmAccel[2]),
  TLM_INT("gx_mdps", readings.lsmGyro[0]),
  TLM_INT("gy_mdps", readings.lsmGyro[1]),
  TLM_INT("gz_mdps", readings.lsmGyro[2]),

  TLM_FLOAT("pressure_hpa", readings.pressure, 2),
  TLM_FLOAT("temp_c", readings.temperature, 1),

  TLM_UINT("gps_fix", gps.data().gpsLockType),
  TLM_UINT("gps_sats", gps.data().satellites),
  TLM_FLOAT("gps_lat_deg", gps.data().lat, 6),
  TLM_FLOAT("gps_lon_deg", gps.data().lon, 6),
  TLM_FLOAT("gps_alt_m", gps.data().altMSL, 1),
  TLM_INT("gps_vel_d_mmps", gps.data().velD),

  // A sensor bit is set while its last read succeeded: 1 LSM, 2 ASM, 4 LIS, 8 LPS.
  TLM_UINT("sensor_ok", readings.lsmOk | readings.asmOk << 1 | readings.lisOk << 2 | readings.lpsOk << 3),
};
static Telemetry telemetry;


void setup()
{
  SerialUSB.begin();
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);

  #ifdef DEBUG
    // Wait for a serial monitor to open the port, but never forever.
    const uint32_t serialWaitStart = millis();
    while (!SerialUSB && (millis() - serialWaitStart) < SERIAL_WAIT_TIMEOUT_MS) {
      delay(10);
    }
    delay(200); // let the host settle so the first prints aren't lost
    SerialUSB.println("canards DEBUG build " __DATE__ " " __TIME__);
  #endif



  
  
  deviceStatus.sd = initializeSD();
  #ifdef SD_TEST
  deviceStatus.sdTest = sdCsvSelfTest();
  #endif
  gpsInit();
  sensorInit();

  // Telemetry runs even if the radio fails to configure: the module may still
  // work with its old config, and boot carries on either way.
  telemetry.addCommand("armFlight", onArmFlight);
  telemetry.addCommand("actuatorTest", onActuatorTest);
  telemetry.begin(radioModule, tlmColumns, radioInit());

  printSensorSummary();
}




// Configures the radio. Returns false if it failed; never blocks for good.
bool radioInit(){
  // build our config
  RadioConfig config;
  config.address = ADDRESS;
  config.networkId = NETWORKID;
  config.encryptionKey = ENCRYPTIONKEY;
  config.parityConfig = PARITYCONFIG;
  config.serialSpeed = SERIALSPEED;
  config.airDataRate = AIRDATARATE;
  config.packetSize = PACKETSIZE;
  config.worMode = WORMODE;
  config.worPeriod = WORPERIOD;
  config.relayMode = RELAYMODE;
  config.destination = DESTINATIONMODE;
  config.txPower = dBm33;
  config.ambientRSSIEnabled = AMBIENTRSSI;
  config.rssiReadingsEnabled = RSSIREADINGS;
  config.listenBeforeTxEnable = LISTENBEFORETX;
  radioModule.setConfig(config);
  radioModule.setFrequency(FREQUENCY);

  radioModule.changeSerialPortCallback(changeSerialPortConfig);
  radioModule.setTimeout(2000);


  #ifdef RADIO_DEBUG
    uint8_t configBuffer[9];
    radioModule.buildConfigBuffer((unsigned char*)&configBuffer);
    SerialUSB.print("desired config registers: ");
    for (size_t i = 0; i<sizeof(configBuffer);i++){
      SerialUSB.printf("%0X ",configBuffer[i]);
    }
    SerialUSB.println("");
  #endif

  int8_t code = radioModule.init(3);
  radioModule.setMode(RadioMode::Normal);
  deviceStatus.radioCode = code;

  #ifdef RADIO_DEBUG
    SerialUSB.printf("Radio init: %s (code %d)\n", code >= 0 ? "OK" : "FAILED", code);
  #endif

  return code >= 0;
}

// Prints one device's init result and returns whether it looks healthy.
static bool reportDevice(const char *name, bool beginOk, bool idReadOk, uint8_t id, uint8_t expectedId){
  bool ok = beginOk && idReadOk && id == expectedId;
  #ifdef SENSOR_CHECK
  SerialUSB.printf("%-10s begin: %-6s WHO_AM_I: 0x%02X (expected 0x%02X)  -> %s\n",
                   name, beginOk ? "OK" : "FAILED", id, expectedId, ok ? "OK" : "FAILED");
  #endif
  return ok;
}

void sensorInit(){
    pinMode(SENSORS_ASM_CS, OUTPUT); digitalWrite(SENSORS_ASM_CS, HIGH);
    pinMode(SENSORS_LSM_CS, OUTPUT); digitalWrite(SENSORS_LSM_CS, HIGH);
    pinMode(SENSORS_LIS_CS, OUTPUT); digitalWrite(SENSORS_LIS_CS, HIGH);
    pinMode(SENSORS_LPS_CS, OUTPUT); digitalWrite(SENSORS_LPS_CS, HIGH);

    SENSORS_SPI.begin();
    delay(20);

    uint8_t id;

    bool lsmBegin = lsm.begin() == LSM6DSO32_OK;
    id = 0;
    bool lsmId = lsm.ReadID(&id) == LSM6DSO32_OK;
    deviceStatus.lsm = reportDevice("LSM6DSO32", lsmBegin, lsmId, id, LSM6DSO32_ID);
    lsm.Set_G_FS(2000);
    lsm.Set_G_ODR(100);
    lsm.Set_X_FS(32);
    lsm.Set_X_ODR(500);
    lsm.Enable_X();
    lsm.Enable_G();

    delay(20);

    bool asmBegin = asmSens.begin() == ASM330LHH_OK;
    id = 0;
    bool asmId = asmSens.ReadID(&id) == ASM330LHH_OK;
    deviceStatus.asm330 = reportDevice("ASM330LHH", asmBegin, asmId, id, ASM330LHH_ID);
    asmSens.Set_G_FS(4000);
    asmSens.Set_G_ODR(100);
    asmSens.Set_X_FS(16);
    asmSens.Set_X_ODR(500);
    asmSens.Enable_X();
    asmSens.Enable_G();

    delay(20);

    bool lisBegin = lis.begin() == LIS2MDL_OK;
    id = 0;
    bool lisId = lis.ReadID(&id) == LIS2MDL_OK;
    deviceStatus.lis = reportDevice("LIS2MDL", lisBegin, lisId, id, LIS2MDL_ID);
    lis.SetOutputDataRate(100);
    lis.Enable();

    delay(20);

    bool lpsBegin = lps.begin() == LPS22HB_STATUS_OK;
    id = 0;
    bool lpsId = lps.ReadID(&id) == LPS22HB_STATUS_OK;
    deviceStatus.lps = reportDevice("LPS22HB", lpsBegin, lpsId, id, LPS22HB_WHO_AM_I_VAL);
    lps.SetODR(100);
    lps.Enable();
}

void printSensorSummary(){
  #ifdef SENSOR_CHECK
  auto res = [](bool ok) { return ok ? "OK" : "FAILED"; };
  SerialUSB.println("========== Device check ==========");
  SerialUSB.printf("SD card    %s\n", res(deviceStatus.sd));
  #ifdef SD_TEST
  SerialUSB.printf("SD CSV     %s\n", res(deviceStatus.sdTest));
  #endif
  SerialUSB.printf("LSM6DSO32  %s\n", res(deviceStatus.lsm));
  SerialUSB.printf("ASM330LHH  %s\n", res(deviceStatus.asm330));
  SerialUSB.printf("LIS2MDL    %s\n", res(deviceStatus.lis));
  SerialUSB.printf("LPS22HB    %s\n", res(deviceStatus.lps));
  SerialUSB.printf("MAX10S GPS %s\n", res(deviceStatus.gps));
  SerialUSB.printf("Radio      %s (code %d)\n", res(deviceStatus.radioCode >= 0), deviceStatus.radioCode);
  SerialUSB.println("==================================");
  #endif
}

static float magnitude(const int32_t v[3]){
  return sqrtf((float)v[0] * v[0] + (float)v[1] * v[1] + (float)v[2] * v[2]);
}

void sensorUpdate()
{
  SensorReadings &r = readings;
  r.lsmOk = lsm.Get_X_Axes(r.lsmAccel) == LSM6DSO32_OK;
  r.lsmOk &= lsm.Get_G_Axes(r.lsmGyro) == LSM6DSO32_OK;
  r.asmOk = asmSens.Get_X_Axes(r.asmAccel) == ASM330LHH_OK;
  r.asmOk &= asmSens.Get_G_Axes(r.asmGyro) == ASM330LHH_OK;
  r.lisOk = lis.GetAxes(r.mag) == LIS2MDL_OK;
  r.lpsOk = lps.GetPressure(&r.pressure) == LPS22HB_STATUS_OK;
  r.lpsOk &= lps.GetTemperature(&r.temperature) == LPS22HB_STATUS_OK;
}

void gpsInit(){
  GPS_I2C.begin();
  deviceStatus.gps = gps.begin();
}

void gpsUpdate(){
  gps.update();
}

// Prints every sensor and the GPS on one line, each field fixed-width,
// signed and zero-padded so the columns stay put from line to line.
//   LSM/ASM  acc[g] x y z |a|   gyr[dps] x y z
//   MAG      [mgauss] x y z |B|
//   BARO     [hPa] [C]
//   GPS      fix type, sats, lat/lon [deg], alt MSL [m], vel N E D [mm/s],
//            age [ms] since the last GPS solution (----- if none yet)
// A '!' right after a device name (e.g. "LSM!") means its last read failed.
// At rest: each |a| ~1 g, gyros ~0, |B| ~250-650 mgauss. Outdoors expect
// fix 3 (3D) and 4+ sats.
void debugPrint(){
  #if defined(SENSOR_PRINT) || defined(GPS_PRINT)
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < DEBUG_PRINT_PERIOD_MS) {
    return;
  }
  lastPrint = millis();

  char line[400];
  size_t n = 0;
  auto add = [&](const char *fmt, auto... args) {
    if (n < sizeof(line)) {
      int w = snprintf(line + n, sizeof(line) - n, fmt, args...);
      if (w > 0) n += w;
    }
  };
  auto flag = [](bool ok) { return ok ? ' ' : '!'; };

  #ifdef SENSOR_PRINT
  const SensorReadings &r = readings;
  auto imu = [&](const char *name, bool ok, const int32_t acc[3], const int32_t gyr[3]) {
    add("%s%c acc %+07.3f %+07.3f %+07.3f |%06.3f| gyr %+08.2f %+08.2f %+08.2f | ",
        name, flag(ok), acc[0] / 1000.0, acc[1] / 1000.0, acc[2] / 1000.0, magnitude(acc) / 1000.0,
        gyr[0] / 1000.0, gyr[1] / 1000.0, gyr[2] / 1000.0);
  };
  imu("LSM", r.lsmOk, r.lsmAccel, r.lsmGyro);
  imu("ASM", r.asmOk, r.asmAccel, r.asmGyro);
  add("MAG%c %+06ld %+06ld %+06ld |%07.1f| | ",
      flag(r.lisOk), (long)r.mag[0], (long)r.mag[1], (long)r.mag[2], magnitude(r.mag));
  add("BARO%c %07.2f %+06.2f | ", flag(r.lpsOk), r.pressure, r.temperature);
  #endif

  #ifdef GPS_PRINT
  const MAX10SData &d = gps.data();
  add("GPS fix %u sats %02u lat %+012.7f lon %+012.7f alt %+08.1f vel %+07ld %+07ld %+07ld age ",
      d.gpsLockType, d.satellites, d.lat, d.lon, d.altMSL, (long)d.velN, (long)d.velE, (long)d.velD);
  uint32_t updated = gps.lastUpdated();
  if (updated == 0) {
    add("-----");
  } else {
    add("%05lu", (unsigned long)min<uint32_t>(millis() - updated, 99999));
  }
  #endif

  SerialUSB.println(line);
  #endif
}

void loop()
{
  digitalToggle(LED_GREEN);

  sensorUpdate();
  gpsUpdate();

  if (actuatorTestActive) {
    // TODO: drive the canard servos through their test sweep here.
    if (millis() - actuatorTestStart >= ACTUATOR_TEST_DURATION_MS) {
      actuatorTestActive = false;
    }
  }

  telemetry.update();

  // SerialUSB.println(String(analogRead(MOSFET_CURRENT)));

  const SensorReadings &r = readings;
  BLA::Matrix<3, 1> accel = {r.lsmAccel[0], r.lsmAccel[1], r.lsmAccel[2]};
  BLA::Matrix<3, 1> gyro = {r.lsmGyro[0], r.lsmGyro[1], r.lsmGyro[2]};
  BLA::Matrix<3, 1> mag = {r.mag[0], r.mag[1], r.mag[2]};
  float baro = r.pressure;

  float curr_s = millis() / 1000.0;
  float dt = curr_s - prev_time_s;


  //TODO: This block runs about 1 min before flight (during pre-flight)
  if (curr_s < 15.0) {
    estimator.initialize_loop(gyro);
  } else if (curr_s < 16.0) {
    estimator.initialize(accel, mag, baro);
  } else {
    estimator.flight_loop(gyro, dt);
    SerialUSB.println(estimator.get_orientation_rpy());
    SerialUSB.println(estimator.get_height(baro));
  }

  // Note: After 10 seconds

  // SerialUSB.println(accel);
  // SerialUSB.println(gyro);



  // estimator.initialize(accel, mag, baro);

  // estimator.flight_loop(gyro, dt);

  // BLA::Matrix<3, 1> unbiased_gyro = estimator.unbiased_gyro(gyro);

  // BLA::Matrix<3, 1> orientation_rpy = get_orientation_rpy();

  // float height = get_height(baro);



  // SerialUSB.print(" freq: "); SerialUSB.print(encoder1.getFrequency());
  // SerialUSB.print(" duty cycle: "); SerialUSB.print(encoder1.getDutyCycle());
  // SerialUSB.println("");`
  // servo1.write(90);

  prev_time_s = curr_s;

  delay(50);

  // debugPrint();
}