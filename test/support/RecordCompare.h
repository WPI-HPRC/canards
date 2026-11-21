#pragma once

// Exact, field-by-field comparisons of CycleRecords and their parts, for the
// replay and stream tests: the same inputs must give the same bits.

#include <cstddef>

#include "Subsystems/Logger/LogSink.h"

namespace RecordCompare {

inline bool same(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
inline bool same(const Quat& a, const Quat& b) { return a.w == b.w && a.x == b.x && a.y == b.y && a.z == b.z; }

inline bool sameFrame(const SensorFrame& a, const SensorFrame& b) {
  return a.timeUs == b.timeUs && a.imuOk == b.imuOk && same(a.accel, b.accel) && same(a.gyro, b.gyro) &&
         a.magFresh == b.magFresh && same(a.mag, b.mag) && a.baroFresh == b.baroFresh &&
         a.pressurePa == b.pressurePa && a.temperatureC == b.temperatureC;
}
inline bool sameFeedback(const ActuatorFeedback& a, const ActuatorFeedback& b) {
  for (int i = 0; i < kNumCanards; ++i) {
    if (a.encoderAngleRad[i] != b.encoderAngleRad[i] || a.encoderValid[i] != b.encoderValid[i]) return false;
  }
  return true;
}
inline bool sameCommand(const Command& a, const Command& b) {
  return a.type == b.type && a.canard == b.canard && a.angleRad == b.angleRad;
}
inline bool sameNav(const NavState& a, const NavState& b) {
  return a.timeUs == b.timeUs && same(a.attitude, b.attitude) && same(a.bodyRate, b.bodyRate) &&
         same(a.velocityNed, b.velocityNed) && a.altitudeAgl == b.altitudeAgl &&
         a.attitudeValid == b.attitudeValid && a.velocityValid == b.velocityValid &&
         a.altitudeValid == b.altitudeValid;
}
inline bool sameStatus(const CanardsStatus& a, const CanardsStatus& b) {
  bool ok = a.mode == b.mode && a.enabled == b.enabled && a.powered == b.powered && a.lockedOut == b.lockedOut &&
            a.cutoffLatched == b.cutoffLatched && a.request == b.request && a.gates == b.gates &&
            a.deniedCount == b.deniedCount && a.lastDenied == b.lastDenied &&
            a.rejectedCommandCount == b.rejectedCommandCount && a.angles.measured == b.angles.measured;
  for (int i = 0; i < kNumCanards; ++i) {
    ok = ok && a.commandRad[i] == b.commandRad[i] && a.angles.rad[i] == b.angles.rad[i];
  }
  return ok;
}
inline bool sameActuators(const ActuatorCommand& a, const ActuatorCommand& b) {
  bool ok = a.servoPower == b.servoPower;
  for (int i = 0; i < kNumCanards; ++i) ok = ok && a.pulseUs[i] == b.pulseUs[i];
  return ok;
}
inline bool sameStep(const TimeStep& a, const TimeStep& b) {
  return a.first == b.first && a.backwards == b.backwards && a.gapUs == b.gapUs && a.dt == b.dt;
}

// The outputs (and derived values) of a cycle.
inline bool sameOutputs(const CycleRecord& a, const CycleRecord& b) {
  return a.cycleCount == b.cycleCount && sameStep(a.step, b.step) && a.flightTimeUs == b.flightTimeUs &&
         sameNav(a.nav, b.nav) && a.specificForce == b.specificForce && a.speed == b.speed && a.tilt == b.tilt &&
         a.imuValid == b.imuValid && a.attitudeValid == b.attitudeValid && a.speedValid == b.speedValid &&
         a.altitudeValid == b.altitudeValid && a.state == b.state && a.transitioned == b.transitioned &&
         a.timeInStateUs == b.timeInStateUs && a.refusedTransitions == b.refusedTransitions &&
         a.safetyActive == b.safetyActive && a.safetySticky == b.safetySticky && sameStatus(a.canards, b.canards) &&
         sameActuators(a.actuators, b.actuators);
}

// The inputs of a cycle.
inline bool sameInputs(const CycleRecord& a, const CycleRecord& b) {
  bool ok = sameFrame(a.frame, b.frame) && sameFeedback(a.feedback, b.feedback) && a.commandCount == b.commandCount;
  for (size_t i = 0; i < CommandList::kCapacity; ++i) ok = ok && sameCommand(a.commands[i], b.commands[i]);
  return ok;
}

inline bool sameRecord(const CycleRecord& a, const CycleRecord& b) { return sameInputs(a, b) && sameOutputs(a, b); }

}  // namespace RecordCompare
