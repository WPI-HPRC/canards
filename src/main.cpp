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
// Flight by default. Only if a USB host has the port open (DTR set) at the end
// of setup() does the board wait, up to kModeWaitMs, for ONE line:
//
//   mode replay <N> [servos] [fast]   replay /logs/canardsLog<N>.csv from SD
//   mode stream [servos]              take input rows over USB (StreamProtocol.h)
//
// Anything else, or nothing, is Flight. With no host connected, Flight never
// waits. The options may come in any order.
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

// --- Telemetry columns ---------------------------------------------------------
// Keep a row short: this is airtime, not the SD log.
uint32_t tlmSeq = 0;
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

BootOptions selectBootMode() {
  // dtr(), not `if (SerialUSB)`: the bool operator delays 10 ms, and with no
  // host the flight path must not wait at all.
  if (!SerialUSB.dtr()) return BootOptions{};
  char line[64];
  const uint32_t start = millis();
  while (millis() - start < kModeWaitMs) {
    if (usbLines.readLine(line, sizeof line)) {
      BootOptions o;
      return parseModeLine(line, o) ? o : BootOptions{};
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

// --- USB bench console (Flight mode) -----------------------------------------------
// One command per line over USB: the ground-station commands, bare
// ("armFlight") or in the radio form ("$C,armFlight*29"), with the same rules
// as over the radio; or Nic's log download ("list", "dump", "dump <N>").
UsbConsole console(SerialUSB);

void handleConsoleLine(char* line) {
  const char* name = GroundCommands::commandFromLine(line);
  if (name == nullptr) return;  // empty, or a "$" line that failed its checksum

  // Replies go out later, after `line` is reused: name them by the constants.
  const char* stable = strcmp(name, GroundCommands::kArmFlight) == 0      ? GroundCommands::kArmFlight
                       : strcmp(name, GroundCommands::kActuatorTest) == 0 ? GroundCommands::kActuatorTest
                                                                          : "?";
  switch (ground.handle(name, flight.state(), flight.sweepActive())) {
    case GroundReply::Ok: console.reply(stable, "OK"); return;
    case GroundReply::Rejected: console.reply(stable, "REJECTED"); return;
    case GroundReply::Unknown: break;
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

  telemetry.update();

  // Heartbeat: about 2 Hz.
  static uint32_t lastBlinkMs = 0;
  if (millis() - lastBlinkMs >= 250) {
    lastBlinkMs = millis();
    digitalToggle(LED_GREEN);
  }
}
