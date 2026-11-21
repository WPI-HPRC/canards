// The flight program (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 6.2
// and 7). Builds the board's sources and sinks and one FlightComputer, and
// calls FlightComputer::cycle() from loop(). All flight logic is in the
// FlightComputer; this file only wires hardware to it and runs the
// housekeeping between samples: the SD log, telemetry, the watchdog.
//
// The bring-up sketch that used to live here is bringup/main.cpp
// (envs marsv21_bringup / marsv21_bringup_debug).
//
// ------------------------------------------------------------- boot modes --
// Flight by default. Two ways out of it, both chosen before the first cycle:
//
//   SD replay (started by the ground station, ground/telemetry_gui.py --usb):
//     the USB console command "replay <N> [servos] [fast]", accepted only in
//     Init before arming. The board stores the request in RAM_D3 (kept across
//     a reset), restarts itself, and the next boot replays
//     /logs/canardsLog<N>.csv from SD. The request is used once: the boot
//     after that is Flight again. See requestReplayReboot().
//
//   USB stream (tools/usb_stream.py): only if a USB host has the port open
//     (DTR set) at the end of setup(), the board waits up to kModeWaitMs for
//     ONE line, "mode stream [servos]" (StreamProtocol.h).
//
// Anything else, or nothing, is Flight. With no host connected and no stored
// request, Flight never waits.
//
// USB bench console (Flight mode, Platform/UsbConsole.h): while a host has the
// port open, the telemetry rows also go out over USB, and the ground-station
// commands (armFlight, actuatorTest) and Nic's log download (list, dump) are
// taken over USB, one per line. Same rules as over the radio. View it with
// `python3 ground/telemetry_gui.py --usb`. Nothing is written with USB
// unplugged, and nothing on it ever blocks the loop.
//
// Rules (section 7):
//   - Flight never accepts injected data.
//   - Replay and stream are never flight-ready: no watchdog, no log download
//     (logDumpPoll), no ground-station commands. Telemetry rows still go out;
//     the `sim_mode` column says which mode (0 flight, 1 replay, 2 stream).
//   - The servos move in replay / stream only with `servos`; otherwise every
//     servo command is discarded and the servos hold the centre pulses
//     MarsActuators::begin() set.
//   - Replay writes a NEW log (this boot's file) through the same SdLogSink,
//     so the original and the replay can be compared (tools/flight_log.py).
//     Paced by the recorded sample times, or as fast as the SD log keeps up
//     with `fast` (Platform/ReplayPacing.h). Replay is OPEN LOOP.
//   - Stream heartbeat: with `servos`, if no line arrives for
//     kStreamHeartbeatMs, the canards are put at 0 deg (centre pulses written
//     straight to the actuators) until lines resume. The flight logic never
//     sees wall time.
//   - Banners go out only before the first cycle (plus one summary line when a
//     replay ends). In stream mode the last banner is "# stream ready"; after
//     it the board writes nothing on SerialUSB but reply lines.
//   - The DFU 1200-baud touch works in every mode (it runs from SysTick).

#include <Arduino.h>
#include <IWatchdog.h>

#include "usbd_cdc.h"  // hUSBD_Device_CDC: is a computer on USB? (radioMayTransmit)
extern "C" USBD_HandleTypeDef hUSBD_Device_CDC;

#include <cstdlib>
#include <cstring>

#include "AttitudeControl/LqrAttitudeController.h"
#include "Estimator/PadAlignedEstimator.h"
#include "Estimator/SimpleEstimatorBackend.h"
#include "FlightComputer.h"
#include "FlightConfig.h"
#include "LiveSensorSource.h"
#include "MarsActuators.h"
#include "Platform/RadioLink.h"
#include "Safety/GoNoGo.h"
#include "Platform/ReplayPacing.h"
#include "Platform/SdLineReader.h"
#include "Platform/SdLogSink.h"
#include "Platform/UsbConsole.h"
#include "Platform/UsbLineReader.h"
#include "Subsystems/Comms/GroundCommands.h"
#include "Subsystems/Logger/CycleLogFormat.h"
#include "Subsystems/Logger/LogReplay.h"
#include "Subsystems/Logger/StreamProtocol.h"
#include "VehicleConfig.h"
#include "logdump.h"
#include "sdcard.h"
#include "telemetry.h"

namespace {

constexpr float centre(int channel) { return VehicleConfig::kServoConfig.channels[channel].centerPulseUs; }
const float kCentrePulsesUs[kNumCanards] = {centre(0), centre(1), centre(2), centre(3)};

// --- Boot mode ------------------------------------------------------------------
enum class BootMode : uint8_t { Flight = 0, Replay = 1, Stream = 2 };

struct BootOptions {
  BootMode mode = BootMode::Flight;
  int replayIndex = -1;
  bool servos = false;  // replay / stream: servo commands reach the servos
  bool fast = false;    // replay: as fast as the SD log keeps up
};

constexpr uint32_t kModeWaitMs = 1500;         // how long a connected host has to send the mode line
constexpr uint32_t kStreamHeartbeatMs = 500;   // PLACEHOLDER (user): stream silence before the canards go to 0 deg
constexpr uint32_t kHeartbeatWritePeriodMs = 20;  // one 50 Hz servo frame

BootOptions boot;

// --- The flight program and what it talks to ----------------------------------
LiveSensorSource sensors;
MarsActuators actuators(kCentrePulsesUs);
SimpleEstimatorBackend estimatorBackend;
PadAlignedEstimator estimator(estimatorBackend);
LqrAttitudeController controller(FlightConfig::kCanardLimitRad);
GroundCommands ground;
SdLogSink sdLog;
Telemetry telemetry;

// Replay and stream.
UsbLineReader usbLines(SerialUSB);
SdLineReader replayFile;
ReplayPacing replayPacing(sdLog);
LogReplay replay(replayFile, replayPacing);
StreamSession stream;

// The FlightComputer is built once, against these; setup() points them at the
// live hardware or at replay / stream, once, before the first cycle.
// --- Loop timing, for telemetry --------------------------------------------------
// The peak over each kPeakWindowMs window. Reported values change only when a
// window closes, so the radio and the USB console (two readers, at their own
// moments) show the same numbers and neither resets the other's.
constexpr uint32_t kPeakWindowMs = 1000;

struct Peak {
  uint32_t current = 0;   // the window still open
  uint32_t reported = 0;  // the last window that closed
  void add(uint32_t v) {
    if (v > current) current = v;
  }
  void closeWindow() {
    reported = current;
    current = 0;
  }
};
Peak peakCycleUs;     // longest FlightComputer::cycle()
Peak peakGapUs;       // longest time between IMU samples
Peak peakReadUs;      // longest sensor read (inside the cycle)
Peak peakFeedbackUs;  // longest encoder feedback read (inside the cycle)
Peak peakWriteUs;     // longest servo write (inside the cycle)

// The FlightComputer's sources and sinks, pointed at the live hardware or at
// replay / stream once in setup(). They also time the hardware calls made
// inside a cycle, so a slow cycle can be traced to its part.
class SelectedSensors final : public SensorSource {
 public:
  SensorSource* target = &sensors;
  bool read(SensorFrame& out) override {
    const uint32_t start = micros();
    const bool ok = target->read(out);
    if (ok) peakReadUs.add(micros() - start);
    return ok;
  }
};
class SelectedCommands final : public CommandSource {
 public:
  CommandSource* target = &ground;
  bool poll(Command& out) override { return target->poll(out); }
};
class SelectedActuators final : public Actuators {
 public:
  Actuators* target = &actuators;
  void readFeedback(ActuatorFeedback& out) override {
    const uint32_t start = micros();
    target->readFeedback(out);
    peakFeedbackUs.add(micros() - start);
  }
  void write(const ActuatorCommand& command) override {
    const uint32_t start = micros();
    target->write(command);
    peakWriteUs.add(micros() - start);
  }
};
SelectedSensors selectedSensors;
SelectedCommands selectedCommands;
SelectedActuators selectedActuators;

FlightComputer flight(FlightIo{selectedSensors, selectedCommands, selectedActuators, estimator, controller, sdLog},
                      VehicleConfig::kServoConfig);

uint32_t peakWindowStartMs = 0;
LiveSensorSource::ReadPeaks sensorPeaks;  // the last window's, by chip
bool watchdogRunning = false;

void closePeakWindow(uint32_t nowMs) {
  if (nowMs - peakWindowStartMs < kPeakWindowMs) return;
  peakWindowStartMs = nowMs;
  peakCycleUs.closeWindow();
  peakGapUs.closeWindow();
  peakReadUs.closeWindow();
  peakFeedbackUs.closeWindow();
  peakWriteUs.closeWindow();
  sensorPeaks = sensors.takeReadPeaks();
}

// --- Ground-station commands (ground/telemetry_gui.py), Flight mode only -------
bool onArmFlight() {
  return ground.handle(GroundCommands::kArmFlight, flight.state(), flight.sweepActive()) == GroundReply::Ok;
}
bool onActuatorTest() {
  return ground.handle(GroundCommands::kActuatorTest, flight.state(), flight.sweepActive()) == GroundReply::Ok;
}
bool onRezeroGyro() {
  return ground.handle(GroundCommands::kRezeroGyro, flight.state(), flight.sweepActive()) == GroundReply::Ok;
}
bool onClearFaults() { return flight.clearSafetyHistory(); }

// --- Telemetry columns ---------------------------------------------------------
// Keep a row short: this is airtime, not the SD log.
uint32_t tlmSeq = 0;
// GNC GO / NO-GO for the ground station (src/Safety/GoNoGo.h). 0 = GO.
uint16_t gncNoGo() {
  const Navigation& nav = flight.navigation();
  const CanardsStatus& c = flight.canards().status();
  const ActuatorFeedback& fb = flight.lastRecord().feedback;
  GoNoGo::Inputs in;
  in.inPreFlight = flight.state() == StateId::PreFlight && boot.mode == BootMode::Flight;
  in.greetingRunning = flight.preFlightState().greetingRunning();
  in.imuValid = nav.imuValid();
  in.attitudeValid = nav.attitudeValid();
  in.speedValid = nav.speedValid();
  in.altitudeValid = nav.altitudeValid();
  in.imuOk = sensors.status().lsm;
  in.magOk = sensors.status().lis;
  in.baroOk = sensors.status().lps;
  in.sdOk = sdLog.active();
  in.safetySticky = flight.safety().sticky();
  in.canardsEnabled = c.enabled;
  in.lockedOut = c.lockedOut;
  in.cutoffLatched = c.cutoffLatched;
  in.powered = c.powered;
  in.tiltRad = nav.tilt();
  in.encodersValid = fb.encoderValid[0] && fb.encoderValid[1] && fb.encoderValid[2] && fb.encoderValid[3];
  return GoNoGo::nogo(in);
}

const TelemetryColumn tlmColumns[] = {
    TLM_UINT("time_ms", millis()),
    TLM_UINT("seq", tlmSeq++),
    TLM_UINT("sim_mode", static_cast<unsigned>(boot.mode)),
    TLM_UINT("state", static_cast<unsigned>(flight.state())),
    TLM_UINT("mode", static_cast<unsigned>(flight.canards().status().mode)),
    TLM_UINT("enabled", flight.canards().status().enabled),
    TLM_UINT("locked", flight.canards().status().lockedOut),
    TLM_UINT("cutoff", flight.canards().status().cutoffLatched),
    TLM_UINT("act_test", flight.sweepActive()),
    TLM_UINT("gnc_go", gncNoGo() == 0),  // 1 = GNC GO (armed, aligned, everything OK)
    TLM_UINT("nogo", gncNoGo()),         // why not: bits, src/Safety/GoNoGo.h
    TLM_UINT("nav_ok", flight.navigation().imuValid() | flight.navigation().attitudeValid() << 1 |
                           flight.navigation().speedValid() << 2 | flight.navigation().altitudeValid() << 3),
    TLM_FLOAT("speed", flight.navigation().speed(), 1),
    TLM_FLOAT("alt_m", flight.navigation().altitudeAgl(), 1),
    TLM_FLOAT("tilt_deg", flight.navigation().tilt() * 57.29578f, 1),
    TLM_FLOAT("sf_g", flight.navigation().specificForce() / FlightConfig::kGravity, 2),
    // Body axes (mount map applied), straight from the IMU: for the bench
    // orientation checks (nose up: fx_g = +1; turning clockwise seen from the
    // tail: p_dps > 0). Drop them if the downlink needs the airtime.
    TLM_FLOAT("fx_g", flight.navigation().accelBody().x / FlightConfig::kGravity, 2),
    TLM_FLOAT("fy_g", flight.navigation().accelBody().y / FlightConfig::kGravity, 2),
    TLM_FLOAT("fz_g", flight.navigation().accelBody().z / FlightConfig::kGravity, 2),
    TLM_FLOAT("p_dps", flight.navigation().gyroBody().x * 57.29578f, 1),
    TLM_FLOAT("q_dps", flight.navigation().gyroBody().y * 57.29578f, 1),
    TLM_FLOAT("r_dps", flight.navigation().gyroBody().z * 57.29578f, 1),
    TLM_UINT("safety", flight.safety().sticky()),
    TLM_UINT("gates", flight.canards().status().gates),
    TLM_UINT("cyc_max_us", peakCycleUs.reported),
    TLM_UINT("gap_max_us", peakGapUs.reported),
    TLM_UINT("read_max_us", peakReadUs.reported),    // parts of cyc_max_us: sensor read,
    TLM_UINT("fb_max_us", peakFeedbackUs.reported),  // encoder feedback,
    TLM_UINT("wr_max_us", peakWriteUs.reported),     // servo write
    TLM_UINT("imu_rd_us", sensorPeaks.imuUs),        // parts of read_max_us, by chip
    TLM_UINT("mag_rd_us", sensorPeaks.magUs),
    TLM_UINT("baro_rd_us", sensorPeaks.baroUs),
    TLM_UINT("enc_edges", actuators.encoderEdges()),  // since boot; a few thousand per second is normal (2 edges per MA4 period, 4 inputs), far more is a storm
    TLM_UINT("imu_missed", sensors.missedSamples()),
    TLM_UINT("log_rows", sdLog.rowsWritten()),
    TLM_UINT("log_drop", sdLog.dropped()),
    TLM_UINT("sensor_ok", sensors.status().lsm | sensors.status().lis << 1 | sensors.status().lps << 2 |
                              sdLog.active() << 3),
};

// --- Boot-mode selection -----------------------------------------------------------

// "mode replay <N> [servos] [fast]" or "mode stream [servos]". False for
// anything else.
bool parseModeLine(char* line, BootOptions& out) {
  BootOptions o;
  const char* word = strtok(line, " \t");
  if (word == nullptr || strcmp(word, "mode") != 0) return false;
  word = strtok(nullptr, " \t");
  if (word == nullptr) return false;
  if (strcmp(word, "replay") == 0) {
    o.mode = BootMode::Replay;
    const char* n = strtok(nullptr, " \t");
    if (n == nullptr || *n < '0' || *n > '9') return false;
    char* end = nullptr;
    const long idx = strtol(n, &end, 10);
    if (*end != '\0' || idx < 0 || idx > 100000) return false;
    o.replayIndex = static_cast<int>(idx);
  } else if (strcmp(word, "stream") == 0) {
    o.mode = BootMode::Stream;
  } else {
    return false;
  }
  while ((word = strtok(nullptr, " \t")) != nullptr) {
    if (strcmp(word, "servos") == 0 && !o.servos) {
      o.servos = true;
    } else if (strcmp(word, "fast") == 0 && o.mode == BootMode::Replay && !o.fast) {
      o.fast = true;
    } else {
      return false;
    }
  }
  out = o;
  return true;
}

// --- Restart into SD replay ----------------------------------------------------
// The replay request survives one reset in the top of RAM_D3, just below the
// DFU word in src/dfu_bootloader.cpp (0x3800FFFC). Nothing else uses RAM_D3.
// After a power-up the words hold noise, which `check` rejects.
struct ReplayRequest {
  uint32_t magic;
  uint32_t index;
  uint32_t flags;  // bit 0 servos, bit 1 fast
  uint32_t check;  // ~(magic ^ index ^ flags)
};
constexpr uint32_t kReplayRequestMagic = 0x5E9A7001UL;
volatile ReplayRequest* const kReplayRequest = reinterpret_cast<volatile ReplayRequest*>(0x3800FFE0UL);

// Stores the request, then resets. Does not return.
[[noreturn]] void requestReplayReboot(const BootOptions& o) {
  const uint32_t flags = (o.servos ? 1u : 0u) | (o.fast ? 2u : 0u);
  kReplayRequest->magic = kReplayRequestMagic;
  kReplayRequest->index = static_cast<uint32_t>(o.replayIndex);
  kReplayRequest->flags = flags;
  kReplayRequest->check = ~(kReplayRequestMagic ^ static_cast<uint32_t>(o.replayIndex) ^ flags);
  // The core enables the D-cache: push the words to SRAM before the reset.
  SCB_CleanDCache_by_Addr(const_cast<ReplayRequest*>(kReplayRequest), 32);
  __DSB();
  SerialUSB.flush();
  delay(100);  // let the reply line leave over USB
  NVIC_SystemReset();
  while (true) {
  }
}

// A stored request, used once. False if there is none.
bool takeReplayRequest(BootOptions& out) {
  const uint32_t magic = kReplayRequest->magic;
  const uint32_t index = kReplayRequest->index;
  const uint32_t flags = kReplayRequest->flags;
  const uint32_t check = kReplayRequest->check;
  kReplayRequest->magic = 0;
  SCB_CleanDCache_by_Addr(const_cast<ReplayRequest*>(kReplayRequest), 32);
  __DSB();
  if (magic != kReplayRequestMagic || check != ~(magic ^ index ^ flags) || index > 100000) return false;
  BootOptions o;
  o.mode = BootMode::Replay;
  o.replayIndex = static_cast<int>(index);
  o.servos = (flags & 1u) != 0;
  o.fast = (flags & 2u) != 0;
  out = o;
  return true;
}

BootOptions selectBootMode() {
  BootOptions o;
  if (takeReplayRequest(o)) return o;
  // dtr(), not `if (SerialUSB)`: the bool operator delays 10 ms, and with no
  // host the flight path must not wait at all.
  if (!SerialUSB.dtr()) return BootOptions{};
  char line[64];
  const uint32_t start = millis();
  while (millis() - start < kModeWaitMs) {
    if (usbLines.readLine(line, sizeof line)) {
      // Stream only: an SD replay is started with the "replay" command.
      return parseModeLine(line, o) && o.mode == BootMode::Stream ? o : BootOptions{};
    }
  }
  return BootOptions{};
}

// Points the FlightComputer at replay. Returns false (with the reason) if the
// log cannot be replayed; the caller then runs Flight.
bool startReplay(const char*& why) {
  if (!sdMounted() || !sdLog.active()) {
    why = "no SD card or no new log file";
    return false;
  }
  if (boot.replayIndex == sdLogFileIdx()) {
    why = "that is this boot's (new, empty) log file";
    return false;
  }
  char path[64];
  sdLogFileName(boot.replayIndex, path, sizeof path);
  if (!replayFile.open(path)) {
    why = "cannot open the log";
    return false;
  }
  if (!replay.begin()) {
    why = "no valid header";
    return false;
  }
  replayPacing.setFast(boot.fast);
  replay.setOutput(boot.servos ? &actuators : nullptr);
  selectedSensors.target = &replay;
  selectedCommands.target = &replay;
  selectedActuators.target = &replay;
  return true;
}

void startStream() {
  stream.setOutput(boot.servos ? &actuators : nullptr);
  selectedSensors.target = &stream;
  selectedCommands.target = &stream;
  selectedActuators.target = &stream;
}

// --- Per-mode loop bodies ---------------------------------------------------------

// One FlightComputer cycle, timed for telemetry. Returns cycle()'s result.
bool runCycle() {
  const uint32_t start = micros();
  if (!flight.cycle()) return false;
  peakCycleUs.add(micros() - start);
  const uint64_t gap = flight.lastRecord().step.gapUs;
  peakGapUs.add(gap > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(gap));
  return true;
}

// --- Radio vs USB --------------------------------------------------------------------
// The E22's 33 dBm bursts break the USB link on the bench. The radio sends only
// when no computer has enumerated the board (USB state "configured"), no host
// has the port open (DTR), and the first kRadioQuietAfterBootMs are over, so a
// computer plugged in at boot can enumerate before the first burst. In flight
// there is no USB cable: the radio starts kRadioQuietAfterBootMs after power-up.
constexpr uint32_t kRadioQuietAfterBootMs = 5000;

bool radioMayTransmit() {
  if (millis() < kRadioQuietAfterBootMs) return false;
  if (SerialUSB.dtr()) return false;
  return hUSBD_Device_CDC.dev_state != USBD_STATE_CONFIGURED;
}

// --- Canard identification (USB console, on the ground only) -----------------------
//   canard <which> <deg>   hold one canard at <deg> for 5 s (kGroundTestHoldUs)
//   canard off             all back to neutral now
//   jitter <which>         wiggle one canard +/- kJitterDeg for kJitterCycles
// <which> is 1-4 or the body axis the canard sits on: +y, +z, -y, -z.
// Canard 1..4 = CH1 (+y), CH3 (+z), CH4 (-y), CH2 (-z): include/VehicleConfig.h.
constexpr const char* kCanardAxis[kNumCanards] = {"+y", "+z", "-y", "-z"};
constexpr const char* kCanardChannel[kNumCanards] = {"CH1", "CH3", "CH4", "CH2"};
constexpr float kJitterDeg = 3.0f;
constexpr int kJitterCycles = 6;                 // 6 x (+, -) = 12 moves
constexpr uint32_t kJitterStepMs = 250;

struct Jitter {
  int canard = -1;   // -1: none running
  int step = 0;
  uint32_t lastMs = 0;
};
Jitter jitter;

// 0..3 from "1".."4" or "+y" / "+z" / "-y" / "-z"; -1 if neither.
int parseCanard(const char* word) {
  if (word == nullptr) return -1;
  if (word[0] >= '1' && word[0] <= '4' && word[1] == '\0') return word[0] - '1';
  for (int i = 0; i < kNumCanards; ++i) {
    if (strcmp(word, kCanardAxis[i]) == 0) return i;
  }
  return -1;
}

// Handles "canard ..." and "jitter ...". False if `name` is neither.
bool handleCanardCommand(const char* name) {
  const bool isCanard = strncmp(name, "canard", 6) == 0 && (name[6] == ' ' || name[6] == '\0');
  const bool isJitter = strncmp(name, "jitter", 6) == 0 && (name[6] == ' ' || name[6] == '\0');
  if (!isCanard && !isJitter) return false;

  char buf[64];
  strncpy(buf, name, sizeof buf - 1);
  buf[sizeof buf - 1] = '\0';
  strtok(buf, " \t");
  const char* which = strtok(nullptr, " \t");
  const char* deg = strtok(nullptr, " \t");

  if (isCanard && which != nullptr && strcmp(which, "off") == 0) {
    jitter.canard = -1;
    const bool ok = ground.handleCanardTestStop(flight.state()) == GroundReply::Ok;
    SerialUSB.println(ok ? "# canards to neutral" : "---ERROR canard off refused (not on the ground)---");
    return true;
  }
  const int c = parseCanard(which);
  if (c < 0 || (isCanard && deg == nullptr)) {
    SerialUSB.println("---ERROR usage: canard <1-4|+y|+z|-y|-z> <deg> | canard off | jitter <1-4|+y|+z|-y|-z>---");
    return true;
  }
  if (isJitter) {
    jitter.canard = c;
    jitter.step = 0;
    jitter.lastMs = millis() - kJitterStepMs;  // first move now
    SerialUSB.printf("# jitter canard %d (%s, %s): +/-%.0f deg\n", c + 1, kCanardAxis[c], kCanardChannel[c],
                     kJitterDeg);
    return true;
  }
  char* end = nullptr;
  const float angleDeg = strtof(deg, &end);
  if (end == deg || *end != '\0' || !(fabsf(angleDeg) <= FlightConfig::kGroundTestLimitRad / FlightConfig::kDegToRad)) {
    SerialUSB.printf("---ERROR angle must be a number within +/-%.1f deg---\n",
                     FlightConfig::kGroundTestLimitRad / FlightConfig::kDegToRad);
    return true;
  }
  jitter.canard = -1;
  if (ground.handleCanardTest(flight.state(), c, angleDeg * FlightConfig::kDegToRad) != GroundReply::Ok) {
    SerialUSB.println("---ERROR canard test refused (only in Init or PreFlight)---");
    return true;
  }
  SerialUSB.printf("# canard %d (%s, %s) to %.1f deg for 5 s; positive = leading edge clockwise from the tail\n",
                   c + 1, kCanardAxis[c], kCanardChannel[c], angleDeg);
  return true;
}

// Runs the jitter, one move per kJitterStepMs, then back to neutral.
void updateJitter() {
  if (jitter.canard < 0 || millis() - jitter.lastMs < kJitterStepMs) return;
  jitter.lastMs = millis();
  if (jitter.step >= 2 * kJitterCycles || !isGroundState(flight.state())) {
    ground.handleCanardTestStop(flight.state());
    jitter.canard = -1;
    return;
  }
  const float deg = (jitter.step % 2 == 0) ? kJitterDeg : -kJitterDeg;
  ground.handleCanardTest(flight.state(), jitter.canard, deg * FlightConfig::kDegToRad);
  ++jitter.step;
}

// --- USB bench console (Flight mode) -----------------------------------------------
// One command per line over USB: the ground-station commands, bare
// ("armFlight") or in the radio form ("$C,armFlight*29"), with the same rules
// as over the radio; or Nic's log download ("list", "dump", "dump <N>").
UsbConsole console(SerialUSB);

void handleConsoleLine(char* line) {
  const char* name = GroundCommands::commandFromLine(line);
  if (name == nullptr) return;  // empty, or a "$" line that failed its checksum

  // Replies go out later, after `line` is reused: name them by the constants.
  if (strcmp(name, GroundCommands::kClearFaults) == 0) {
    console.reply(GroundCommands::kClearFaults, onClearFaults() ? "OK" : "REJECTED");
    return;
  }
  const char* stable = strcmp(name, GroundCommands::kArmFlight) == 0      ? GroundCommands::kArmFlight
                       : strcmp(name, GroundCommands::kActuatorTest) == 0 ? GroundCommands::kActuatorTest
                       : strcmp(name, GroundCommands::kRezeroGyro) == 0   ? GroundCommands::kRezeroGyro
                                                                          : "?";
  switch (ground.handle(name, flight.state(), flight.sweepActive())) {
    case GroundReply::Ok: console.reply(stable, "OK"); return;
    case GroundReply::Rejected: console.reply(stable, "REJECTED"); return;
    case GroundReply::Unknown: break;
  }

  if (handleCanardCommand(name)) return;

  // "replay <N> [servos] [fast]": restart into an SD replay (the ground
  // station's SD replay panel). Init before arming only, like log download.
  if (strncmp(name, "replay", 6) == 0 && (name[6] == ' ' || name[6] == '\0')) {
    if (flight.state() != StateId::Init || watchdogRunning) {
      SerialUSB.println("---ERROR replay only in Init before arming; power-cycle first---");
      return;
    }
    char modeLine[72];
    snprintf(modeLine, sizeof modeLine, "mode %s", name);
    BootOptions o;
    if (!parseModeLine(modeLine, o) || o.mode != BootMode::Replay) {
      SerialUSB.println("---ERROR usage: replay <N> [servos] [fast]---");
      return;
    }
    // Check now what the next boot would refuse, so the board does not
    // restart for nothing.
    char path[64];
    sdLogFileName(o.replayIndex, path, sizeof path);
    const int nextBootLog = sdLogFileIdx() + 1 > 100 ? 0 : sdLogFileIdx() + 1;  // nextLogFileIdx() in sdcard.cpp
    if (!sdMounted() || !sdFs().exists(path)) {
      SerialUSB.printf("---ERROR no %s on the SD card---\n", path);
      return;
    }
    if (o.replayIndex == nextBootLog) {
      SerialUSB.printf("---ERROR log %d is the number the next boot writes to; copy the flight to another---\n",
                       o.replayIndex);
      return;
    }
    SerialUSB.printf("# restarting into replay %d%s%s\n", o.replayIndex, o.servos ? " servos" : "",
                     o.fast ? " fast" : "");
    requestReplayReboot(o);
  }

  const bool logCommand = strcmp(name, "list") == 0 || strncmp(name, "dump", 4) == 0;
  if (!logCommand) {
    console.reply("?", "UNKNOWN");  // `name` lives in the line buffer, which is reused
    return;
  }
  // Log download: on the pad only, it blocks loop() for seconds. Not after
  // arming or a scrub (StandDown back to Init): the STM32 watchdog cannot be
  // stopped once started, so a download would reset the board. Power-cycle to
  // download then. scripts/dump_log.py shows the ERROR line.
  if (flight.state() != StateId::Init || watchdogRunning) {
    SerialUSB.println("\n---ERROR log download only in Init before arming; power-cycle first---");
    return;
  }
  console.discardOutput();
  logDumpCommand(const_cast<char*>(name));
}

void loopFlight() {
  // Between samples: one log row at a time, so a slow SD write delays at
  // most the next sample, which waits in the IMU's output register.
  if (!runCycle()) sdLog.drain(1);

  static char consoleLine[64];
  if (usbLines.readLine(consoleLine, sizeof consoleLine)) handleConsoleLine(consoleLine);
  console.update(millis(), SerialUSB.dtr());
  updateJitter();

  if (flight.state() != StateId::Init && !watchdogRunning) {
    IWatchdog.begin(FlightConfig::kWatchdogTimeoutUs);
    watchdogRunning = true;
  }
  if (watchdogRunning) IWatchdog.reload();
}

void loopReplay() {
  if (!runCycle()) sdLog.drain(1);
  console.update(millis(), SerialUSB.dtr());  // telemetry rows only; no commands in replay

  static bool reported = false;
  if (!reported && replay.endOfLog() && sdLog.pending() == 0) {
    reported = true;
    SerialUSB.printf("# replay done: %lu rows replayed, %lu malformed skipped, %lu log rows written, %lu dropped\n",
                     static_cast<unsigned long>(replay.rowsRead()), static_cast<unsigned long>(replay.malformedRows()),
                     static_cast<unsigned long>(sdLog.rowsWritten()), static_cast<unsigned long>(sdLog.dropped()));
  }
}

void writeReply(const char* buf, size_t len) {
  if (len > 0) SerialUSB.write(reinterpret_cast<const uint8_t*>(buf), len);
}

void loopStream() {
  static uint32_t lastLineMs = millis();
  static uint32_t lastNeutralMs = 0;
  static char line[StreamProtocol::kMaxLine];
  static char reply[StreamProtocol::kMaxLine];

  if (usbLines.readLine(line, sizeof line)) {
    lastLineMs = millis();
    switch (stream.feedLine(line)) {
      case StreamSession::Result::Ignored:
        break;
      case StreamSession::Result::Header:
        writeReply(reply, StreamSession::replyHeader(reply, sizeof reply));
        break;
      case StreamSession::Result::Row:
        if (runCycle()) {
          writeReply(reply, StreamSession::replyRow(flight.lastRecord(), reply, sizeof reply));
        } else {
          writeReply(reply, StreamProtocol::formatError("cycle did not run", reply, sizeof reply));
        }
        break;
      case StreamSession::Result::Error:
        writeReply(reply, stream.replyError(reply, sizeof reply));
        break;
    }
  } else {
    sdLog.drain(1);
  }

  // Heartbeat: the host has gone quiet, so hold the canards at 0 deg.
  if (boot.servos && millis() - lastLineMs >= kStreamHeartbeatMs &&
      millis() - lastNeutralMs >= kHeartbeatWritePeriodMs) {
    lastNeutralMs = millis();
    ActuatorCommand neutral;
    // Keep the power rule of the last cycle (off in Chute and Land), on before any.
    neutral.servoPower = flight.cycleCount() == 0 || flight.lastRecord().actuators.servoPower;
    for (int ch = 0; ch < kNumCanards; ++ch) neutral.pulseUs[ch] = kCentrePulsesUs[ch];
    actuators.write(neutral);
  }
}

}  // namespace

void setup() {
  // USB CDC must be running for the 1200-baud DFU touch (src/dfu_bootloader.cpp).
  SerialUSB.begin();
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);

  if (initializeSD()) sdLog.begin(sdLogFile());
  sensors.begin();
  actuators.begin();
  // The E22 transmits at 33 dBm. On the bench its bursts knocked out the USB
  // link whenever DC power was on, so the radio stays quiet while a computer
  // is on USB: see radioMayTransmit(). With no USB (the pad, the flight) it sends.
  const bool radioOk = radioLinkBegin();

  boot = selectBootMode();
  const char* why = "";
  if (boot.mode == BootMode::Replay && !startReplay(why)) {
    SerialUSB.printf("mode: replay %d failed (%s); running flight\n", boot.replayIndex, why);
    boot = BootOptions{};
  }
  if (boot.mode == BootMode::Stream) startStream();

  if (boot.mode == BootMode::Flight) {
    // Ground-station commands only in Flight: replay and stream are never
    // flight-ready.
    telemetry.addCommand(GroundCommands::kArmFlight, onArmFlight);
    telemetry.addCommand(GroundCommands::kActuatorTest, onActuatorTest);
    telemetry.addCommand(GroundCommands::kRezeroGyro, onRezeroGyro);
    telemetry.addCommand(GroundCommands::kClearFaults, onClearFaults);
  }
  // The USB console's telemetry rows go out in Flight and in replay (so the
  // ground station can watch a replay over USB). Commands are read in Flight
  // only. Stream mode owns the USB port, so no console there.
  if (boot.mode != BootMode::Stream) console.begin(tlmColumns);
  telemetry.begin(radioLink(), tlmColumns, radioOk);

  switch (boot.mode) {
    case BootMode::Flight:
      SerialUSB.println("mode: flight");
      break;
    case BootMode::Replay: {
      char path[64];
      sdLogFileName(boot.replayIndex, path, sizeof path);
      SerialUSB.printf("mode: replay %s%s%s -> %s log %d. OPEN LOOP: shows state changes and commands, "
                       "not whether the rocket would have flown straight.\n",
                       path, boot.servos ? " servos" : "", boot.fast ? " fast" : " paced",
                       sdLog.active() ? "new" : "NO", sdLogFileIdx());
      break;
    }
    case BootMode::Stream:
      SerialUSB.printf("mode: stream%s\n", boot.servos ? " servos" : "");
      SerialUSB.println("# stream ready");  // the host waits for this; after it only replies
      break;
  }
}

void loop() {
  closePeakWindow(millis());
  switch (boot.mode) {
    case BootMode::Flight: loopFlight(); break;
    case BootMode::Replay: loopReplay(); break;
    case BootMode::Stream: loopStream(); break;
  }

  // Radio quiet while a computer is on USB: see radioMayTransmit().
  telemetry.update(millis(), radioMayTransmit());

  // Heartbeat: about 2 Hz.
  static uint32_t lastBlinkMs = 0;
  if (millis() - lastBlinkMs >= 250) {
    lastBlinkMs = millis();
    digitalToggle(LED_GREEN);
  }
}
