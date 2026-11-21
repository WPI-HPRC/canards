#pragma once

// Ground tests of the canards, shared by Init and PreFlight
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.2).
//
// Two kinds of test:
//   - CanardTest / CanardTestStop: a held set of test angles. Each accepted
//     CanardTest (re)starts a hold of FlightConfig::kGroundTestHoldUs of
//     sample time; when it runs out the canards go back to neutral unless the
//     command is repeated, so a lost "stop" packet cannot leave a canard
//     deflected.
//   - CanardSweep (the ground station's "actuatorTest"): a fixed sequence,
//     each canard to +kSweepAmplitudeRad then -kSweepAmplitudeRad in turn,
//     then all four together, kSweepStepUs per step. Only where the owner
//     allows it (Init); elsewhere the command is ignored. A CanardTest or
//     CanardTestStop cancels a running sweep.
// While a test runs its angles are requested every cycle.
//
// It never clamps: the Canards subsystem owns every limit, and it refuses test
// angles once the rocket has left the ground.

#include <cstdint>

#include "Actuators.h"
#include "StateMachine/State.h"

class Canards;

class GroundTest {
 public:
  static constexpr int kSweepSteps = 2 * kNumCanards + 2;

  // `allowSweep`: whether CanardSweep commands are accepted here.
  GroundTest(Canards& canards, bool allowSweep) : canards_(canards), allowSweep_(allowSweep) {}

  // Back to neutral, no hold, no sweep. Call on state entry and exit.
  void reset();

  // Reads this cycle's commands. Returns true if a test is active, in which
  // case it has requested the test angles; the caller must not make another
  // canard request this cycle.
  bool update(const Cycle& cycle);

  bool active() const { return active_ || sweeping_; }
  bool sweeping() const { return sweeping_; }
  const float* angles() const { return anglesRad_; }  // by canard, current request

  // The angles of sweep step `step` (0 .. kSweepSteps-1), by canard.
  static void sweepAngles(int step, float out[kNumCanards]);

 private:
  Canards& canards_;
  bool allowSweep_;
  bool active_ = false;  // a CanardTest hold
  uint64_t untilUs_ = 0;
  bool sweeping_ = false;
  uint64_t sweepStartUs_ = 0;
  float heldRad_[kNumCanards] = {};   // CanardTest angles
  float anglesRad_[kNumCanards] = {}; // what is requested this cycle
};
