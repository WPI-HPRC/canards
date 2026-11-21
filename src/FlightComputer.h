#pragma once

// Owns the flight program and runs one fixed cycle per IMU sample
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.2).
//
// main.cpp builds the concrete sources and sinks (live, USB stream or SD
// replay) and one FlightComputer, then calls cycle() from loop(). Everything
// after the SensorSource is the same code in every mode, which is what makes
// replay meaningful.
//
// No heap, no clock: time is the IMU sample time in the sensor frame.

#include <cstdint>

#include "Actuators.h"
#include "AttitudeControl/AttitudeController.h"
#include "Estimator/Estimator.h"
#include "Safety/SafetyMonitor.h"
#include "SensorSource.h"
#include "ServoControl/ServoControl.h"
#include "StateMachine/StateMachine.h"
#include "StateMachine/states/BoostState.h"
#include "StateMachine/states/ChuteState.h"
#include "StateMachine/states/CoastState.h"
#include "StateMachine/states/ControlState.h"
#include "StateMachine/states/InitState.h"
#include "StateMachine/states/LandState.h"
#include "StateMachine/states/PreFlightState.h"
#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Logger/LogSink.h"
#include "Subsystems/Navigation/Navigation.h"

// The outside world. Every reference must outlive the FlightComputer.
struct FlightIo {
  SensorSource& sensors;
  CommandSource& commands;
  Actuators& actuators;
  Estimator& estimator;
  AttitudeController& attitudeController;
  LogSink& log;
};

class FlightComputer {
 public:
  // `servoConfig` must outlive the FlightComputer.
  FlightComputer(const FlightIo& io, const ServoConfig& servoConfig);

  FlightComputer(const FlightComputer&) = delete;
  FlightComputer& operator=(const FlightComputer&) = delete;

  // The only thing loop() calls. Returns false at once, doing nothing, if the
  // sensor source has no new IMU sample. Otherwise runs one cycle and returns
  // true.
  bool cycle();

  StateId state() const { return stateMachine_.current(); }
  const StateMachine& stateMachine() const { return stateMachine_; }
  const Navigation& navigation() const { return navigation_; }
  const Canards& canards() const { return canards_; }
  const SafetyMonitor& safety() const { return safety_; }
  const ControlState& controlState() const { return control_; }
  const PreFlightState& preFlightState() const { return preFlight_; }
  // A ground-station actuator test (sweep) is running in Init.
  bool sweepActive() const { return state() == StateId::Init && init_.sweeping(); }
  const CycleRecord& lastRecord() const { return record_; }
  // The monotonic flight clock (see cycle()), microseconds.
  uint64_t flightTimeUs() const { return flightTimeUs_; }
  uint32_t cycleCount() const { return cycleCount_; }

 private:
  FlightIo io_;

  // Construction order matters: each member only uses ones above it.
  Navigation navigation_;
  SafetyMonitor safety_;
  ServoControl servoControl_;
  Canards canards_;

  InitState init_;
  PreFlightState preFlight_;
  BoostState boost_;
  ControlState control_;
  CoastState coast_;
  ChuteState chute_;
  LandState land_;
  State* const states_[kNumStates];
  StateMachine stateMachine_;

  CommandList commands_;
  CycleRecord record_;
  bool havePrevious_ = false;
  uint64_t previousTimeUs_ = 0;
  uint64_t flightTimeUs_ = 0;
  uint32_t cycleCount_ = 0;
};
