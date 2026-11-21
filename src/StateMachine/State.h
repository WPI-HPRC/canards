#pragma once

// The State interface and the per-cycle data a state may read
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.2).
//
// States are long-lived objects: constructed once, never allocated per entry.
// A state acts on the rocket only through subsystems it was constructed with
// (e.g. Canards), never through hardware.

#include <cstdint>

#include "StateMachine/StateId.h"

class Navigation;
class CommandList;

// What a state may read this cycle.
struct Cycle {
  uint64_t timeUs;        // flight clock, us: follows the IMU sample time but
                          // never goes backwards and never jumps by more than
                          // FlightConfig::kMaxImuGapUs (FlightComputer::cycle)
  float dt;               // seconds since the previous sample
  uint64_t timeInStateUs; // sample time since the state was entered; filled in
                          // by the StateMachine (0 in init())
  const Navigation& nav;
  const CommandList& commands;  // ground commands received this cycle
};

class State {
 public:
  virtual ~State() = default;

  virtual void init(const Cycle&) {}     // once, on entry
  virtual void execute(const Cycle&) {}  // every cycle while active
  virtual void end(const Cycle&) {}      // once, right after the last execute()

  // Where to go next. Return this state's own id to stay. Called every cycle,
  // after execute(). A transition that is not in the allowed list is refused
  // by the StateMachine.
  virtual StateId next(const Cycle&) = 0;
};
