#pragma once

// Ground tests of the canards, shared by Init and PreFlight
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.2).
//
// Turns CanardTest / CanardTestStop commands into a held set of test angles.
// Each accepted CanardTest (re)starts a hold of FlightConfig::kGroundTestHoldUs
// of sample time; while the hold lasts the angles are requested every cycle.
// When it runs out the canards go back to neutral unless the command is
// repeated, so a lost "stop" packet cannot leave a canard deflected.
//
// It never clamps: the Canards subsystem owns every limit, and it refuses test
// angles once the rocket has left the ground.

#include <cstdint>

#include "Actuators.h"
#include "StateMachine/State.h"

class Canards;

class GroundTest {
 public:
  explicit GroundTest(Canards& canards) : canards_(canards) {}

  // Back to neutral, no hold. Call on state entry and exit.
  void reset();

  // Reads this cycle's commands. Returns true if a test is active, in which
  // case it has requested the test angles; the caller must not make another
  // canard request this cycle.
  bool update(const Cycle& cycle);

  bool active() const { return active_; }
  const float* angles() const { return anglesRad_; }  // by canard

 private:
  Canards& canards_;
  bool active_ = false;
  uint64_t untilUs_ = 0;
  float anglesRad_[kNumCanards] = {};
};
