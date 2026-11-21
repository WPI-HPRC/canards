#pragma once

// One of the flight states: see StateMachine/states/README.md.

#include "StateMachine/GroundTest.h"
#include "StateMachine/State.h"
#include "StateMachine/states/LaunchDetector.h"

class Canards;

// Powered on the pad. Left by a ground command or by a launch.
// No timeout: the board sits here until commanded, which can be a long time.
class InitState final : public State {
 public:
  explicit InitState(Canards& canards) : canards_(canards), groundTest_(canards, /*allowSweep=*/true) {}
  void init(const Cycle&) override;
  void execute(const Cycle& cycle) override;
  void end(const Cycle&) override;
  StateId next(const Cycle& cycle) override;

  // A ground-station actuator test (sweep) is running.
  bool sweeping() const { return groundTest_.sweeping(); }

 private:
  Canards& canards_;
  GroundTest groundTest_;
  LaunchDetector launch_;
};
