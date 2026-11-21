#pragma once

// What gets logged each cycle, and where it goes
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 6.2 and 7).
//
// A CycleRecord holds every INPUT of the cycle, losslessly, so that a flight
// flown on old code can be replayed through new code:
//   - the sensor frame, in sensor axes and SI units;
//   - the actuator feedback (encoders);
//   - every ground command received.
// It also holds the OUTPUTS, so a replay can be compared with the original:
//   state, canard status, pulses and safety bits.
//
// The FlightComputer fills one record per cycle and hands it to a LogSink.
// A sink must never block: the SD sink buffers and writes outside the
// control step. The on-disk format lives in lib/FlightLog.

#include <cstdint>

#include "Actuators.h"
#include "Estimator/Estimator.h"
#include "Safety/SafetyMonitor.h"
#include "SensorSource.h"
#include "StateMachine/StateId.h"
#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"

struct CycleRecord {
  uint32_t cycleCount = 0;  // cycles since boot, starting at 1

  // --- Inputs (everything needed for replay) --------------------------------
  SensorFrame frame;
  ActuatorFeedback feedback;
  uint8_t commandCount = 0;
  Command commands[CommandList::kCapacity];

  // --- Derived ---------------------------------------------------------------
  TimeStep step;
  uint64_t flightTimeUs = 0;    // the monotonic flight clock the states saw (FlightComputer::cycle)
  NavState nav;                 // the estimator's output as it arrived
  float specificForce = 0.0f;   // m/s^2, from the raw IMU
  float speed = 0.0f;           // m/s
  float tilt = 0.0f;            // rad
  bool imuValid = false;
  bool attitudeValid = false;   // after Navigation's own checks (finite, unit, fresh)
  bool speedValid = false;
  bool altitudeValid = false;

  // --- Outputs ---------------------------------------------------------------
  StateId state = StateId::Init;     // the state in force after this cycle's transition
  bool transitioned = false;         // this cycle changed state
  uint64_t timeInStateUs = 0;
  uint32_t refusedTransitions = 0;   // StateMachine::refusedCount()
  uint16_t safetyActive = 0;         // SafetyFault bits
  uint16_t safetySticky = 0;
  CanardsStatus canards;
  ActuatorCommand actuators;
};

class LogSink {
 public:
  virtual ~LogSink() = default;

  // Called once per cycle, last. Must return quickly and must not block.
  virtual void write(const CycleRecord& record) = 0;
};

// Discards everything. For tests, and for running without an SD card.
class NullLogSink final : public LogSink {
 public:
  void write(const CycleRecord&) override {}
};
