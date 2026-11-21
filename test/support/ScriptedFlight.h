#pragma once

// A scripted flight for the replay and stream tests, recorded through a real
// FlightComputer: pad, ground tests (a burst of more commands than fit in one
// cycle), arm, a 5 g+ launch, burn, coast, apogee, descent, landing. Every
// input (frame, encoder feedback, estimate) is a pure function of sample
// time, so the same flight can be re-run through a fresh FlightComputer from
// its log.
//
// `periodUs` is the IMU sample period; `scale` stretches every phase (in
// samples). periodUs = 20000, scale = 1 is short (under 200 cycles), for the
// tools/ fixture.
//
// Test-only: uses std::vector, std::string and std::function.

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "Actuators.h"
#include "AttitudeControl/LqrAttitudeController.h"
#include "FakeActuators.h"
#include "FakeCommandSource.h"
#include "FakeSensorSource.h"
#include "FlightComputer.h"
#include "FlightConfig.h"
#include "Frames.h"
#include "ScriptedEstimator.h"
#include "Subsystems/Logger/CycleLogFormat.h"
#include "VehicleConfig.h"

namespace ScriptedFlight {

namespace fc = FlightConfig;

inline ServoConfig calibratedServos() {
  ServoConfig config = VehicleConfig::kServoConfig;
  for (ServoChannel& channel : config.channels) channel.calibrated = true;
  return config;
}

// A LogSink that keeps every record and its formatted log line (header first).
class FormattingLogSink final : public LogSink {
 public:
  FormattingLogSink() {
    char buf[CycleLogFormat::kMaxLine];
    const size_t len = CycleLogFormat::formatHeader(buf, sizeof buf);
    lines.emplace_back(buf, len);
  }
  void write(const CycleRecord& record) override {
    records.push_back(record);
    char buf[CycleLogFormat::kMaxLine];
    const size_t len = CycleLogFormat::formatRow(record, buf, sizeof buf);
    lines.emplace_back(buf, len);  // len 0 (did not fit) shows up as an empty, malformed line
  }
  std::vector<CycleRecord> records;
  std::vector<std::string> lines;
};

struct Script {
  uint64_t periodUs;
  uint64_t start, burst, testStop, command, launch, boost, altitude, control, peak, slow, coast, chuteSlow, chute,
      low, land, end;
  float vPeak = 2.0f * fc::kControlEndSpeed;
  float apogeeM = 10.0f * (fc::kControlStartAltitudeM + fc::kLandAltitudeM) + 100.0f;

  Script(uint64_t period, uint64_t scale) : periodUs(period) {
    auto n = [&](uint64_t samples) { return samples * scale * periodUs; };
    auto hold = [&](uint64_t us) { return (us + periodUs - 1) / periodUs * periodUs; };
    start = 1 * fc::kUsPerSecond;
    burst = start + n(1);
    testStop = start + n(3);
    command = start + n(5);
    launch = command + n(10);
    boost = launch + hold(fc::kLaunchHoldUs);
    altitude = boost + n(2);
    control = altitude + hold(fc::kControlStartHoldUs);
    peak = control + n(20);
    slow = peak + n(20);
    coast = slow + hold(fc::kControlEndHoldUs);
    chuteSlow = coast + n(20);
    chute = chuteSlow + hold(fc::kChuteHoldUs);
    low = chute + n(20);
    land = low + hold(fc::kLandHoldUs);
    end = land + n(5);
  }

  static float lerp(float a, float b, uint64_t t, uint64_t t0, uint64_t t1) {
    return a + (b - a) * (static_cast<float>(t - t0) / static_cast<float>(t1 - t0));
  }

  float speed(uint64_t t) const {
    const float between = fc::kControlEndSpeed - fc::kChuteSpeed;
    if (t < launch) return 0.0f;
    if (t < peak) return lerp(0.0f, vPeak, t, launch, peak);
    if (t < slow) return lerp(vPeak, fc::kControlEndSpeed, t, peak, slow);
    if (t < chuteSlow) return lerp(fc::kChuteSpeed + 0.9f * between, fc::kChuteSpeed + 0.1f * between, t, slow, chuteSlow);
    return 0.5f * fc::kChuteSpeed;
  }
  float altitude_(uint64_t t) const {
    if (t < launch) return 0.0f;
    if (t < altitude) return lerp(0.0f, 0.5f * fc::kControlStartAltitudeM, t, launch, altitude);
    if (t < chuteSlow) return lerp(fc::kControlStartAltitudeM, apogeeM, t, altitude, chuteSlow);
    if (t < low) return lerp(apogeeM, fc::kLandAltitudeM + 1.0f, t, chuteSlow, low);
    if (t < end) return lerp(0.9f * fc::kLandAltitudeM, 0.0f, t, low, end);
    return 0.0f;
  }
  // Specific force along the nose, m/s^2.
  float thrust(uint64_t t) const {
    if (t < launch) return fc::kGravity;
    if (t < peak) return 2.0f * fc::kLaunchAccel;
    if (t < chuteSlow) return -0.5f * fc::kGravity;
    return fc::kGravity;
  }
  // Body -> NED: nose up on the pad, then tilted and rolled, with a slow wobble
  // so the controller has something to do.
  Quat attitude(uint64_t t) const {
    const float kHalfPi = 1.57079632679489661923f;
    const float s = static_cast<float>(t % 2000000) * 1e-6f;
    const float tilt = t < launch ? 0.0f : 0.1f + 0.02f * std::sin(6.2831853f * s);
    const float roll = t < launch ? 0.0f : 0.3f;
    return fromAxisAngle(Vec3{0.0f, 1.0f, 0.0f}, kHalfPi - tilt) * fromAxisAngle(Vec3{1.0f, 0.0f, 0.0f}, roll);
  }

  static Vec3 sensorAxes(const Vec3& body) {
    Mat3 tr;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) tr.m[r][c] = VehicleConfig::kImuToBody.m[c][r];
    }
    return tr * body;
  }

  SensorFrame frame(uint64_t t) const {
    SensorFrame f;
    f.timeUs = t;
    f.imuOk = true;
    const uint64_t k = (t - start) / periodUs;
    // Awkward low bits, to prove the log keeps every bit.
    f.accel = sensorAxes(Vec3{thrust(t), 0.0123456789f * static_cast<float>(k % 7), -0.001f * static_cast<float>(k % 3)});
    f.gyro = Vec3{0.001f * static_cast<float>(k % 5), -0.0007f * static_cast<float>(k % 11), 1.0f / 3.0f * 1e-3f};
    f.magFresh = k % 4 == 0;
    f.mag = Vec3{20.0f + 0.01f * static_cast<float>(k % 13), 1.0f, 45.0f};
    f.baroFresh = k % 2 == 0;
    f.pressurePa = 101325.0f - 12.0f * altitude_(t);
    f.temperatureC = 20.0f + 0.001f * static_cast<float>(k % 17);
    return f;
  }

  ActuatorFeedback feedback(uint64_t t) const {
    ActuatorFeedback fb;
    const uint64_t k = (t - start) / periodUs;
    for (int ch = 0; ch < kNumCanards; ++ch) {
      fb.encoderValid[ch] = ((k / 9 + static_cast<uint64_t>(ch)) % 3) != 0;
      fb.encoderAngleRad[ch] = 1.0f + 0.37f * static_cast<float>(ch) + 0.001f * static_cast<float>(k % 29);
    }
    return fb;
  }

  NavState truth(uint64_t t) const {
    NavState s;
    s.timeUs = t;
    s.attitude = attitude(t);
    s.bodyRate = Vec3{0.01f, -0.02f, 0.005f};
    const float v = speed(t);
    s.velocityNed = Vec3{0.0f, 0.0f, t < chuteSlow ? -v : v};
    s.altitudeAgl = altitude_(t);
    s.attitudeValid = true;
    s.velocityValid = true;
    s.altitudeValid = true;
    return s;
  }

  // The commands sent just before the cycle at `t`.
  std::vector<Command> commandsAt(uint64_t t) const {
    std::vector<Command> out;
    if (t == burst) {
      // Ten at once: eight fit in this cycle, two go into the next.
      out = {makeTestCommand(0, 0.02f),  makeTestCommand(1, -0.03f), makeTestCommand(-1, 0.01f),
             makeCommand(CommandType::CanardTestStop), makeTestCommand(2, 0.04f), makeTestCommand(3, 0.05f),
             makeCommand(CommandType::CanardTestStop), makeTestCommand(1, 0.06f), makeTestCommand(2, -0.07f),
             makeTestCommand(0, 0.08f)};
    } else if (t == testStop) {
      out = {makeCommand(CommandType::CanardTestStop)};
    } else if (t == command) {
      out = {makeCommand(CommandType::EnterPreFlight), makeCommand(CommandType::CanardsEnable)};
    }
    return out;
  }
};

inline NavState truthFor(const Script& s, const EstimatorInput& in) { return s.truth(in.timeUs); }

// Records the scripted flight through a fresh FlightComputer.
struct Recording {
  Script script;
  ServoConfig servos = calibratedServos();
  ScriptedEstimator estimator;
  FakeSensorSource sensors;
  FakeCommandSource commands;
  FakeActuators actuators;
  LqrAttitudeController controller{fc::kCanardLimitRad};
  FormattingLogSink log;
  FlightComputer computer;

  Recording(uint64_t periodUs, uint64_t scale)
      : script(periodUs, scale),
        estimator([this](const EstimatorInput& in) { return truthFor(script, in); }),
        sensors(script.start, periodUs, [this](uint64_t t) { return script.frame(t); }),
        computer(FlightIo{sensors, commands, actuators, estimator, controller, log}, servos) {}

  void run() {
    while (sensors.nextTimeUs() <= script.end) {
      const uint64_t t = sensors.nextTimeUs();
      for (const Command& c : script.commandsAt(t)) commands.push(c);
      actuators.feedback = script.feedback(t);
      computer.cycle();
    }
  }
};

}  // namespace ScriptedFlight
