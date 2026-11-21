#pragma once

// One of the flight states: see StateMachine/states/README.md.

#include "StateMachine/GroundTest.h"
#include "StateMachine/State.h"
#include "StateMachine/states/LaunchDetector.h"

class Canards;
class Navigation;

// About a minute before launch: the estimator is re-initialised on entry.
// No timeout: the rocket may wait on the pad.
class PreFlightState final : public State {
 public:
  PreFlightState(Canards& canards, Navigation& nav) : canards_(canards), nav_(nav), groundTest_(canards) {}
  void init(const Cycle&) override;
  void execute(const Cycle& cycle) override;
  void end(const Cycle&) override;
  StateId next(const Cycle& cycle) override;

 private:
  Canards& canards_;
  Navigation& nav_;
  GroundTest groundTest_;
  LaunchDetector launch_;
};
