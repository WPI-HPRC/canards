#pragma once

// One of the flight states: see StateMachine/states/README.md.

#include "FlightConfig.h"
#include "Frames.h"
#include "StateMachine/Persistence.h"
#include "StateMachine/State.h"

class Canards;

// The canards hold "nose to true vertical, keep the roll the rocket had on
// entering Control", captured on the first cycle with a valid attitude.
class ControlState final : public State {
 public:
  explicit ControlState(Canards& canards) : canards_(canards), slow_(FlightConfig::kControlEndHoldUs) {}
  void init(const Cycle&) override;
  void execute(const Cycle& cycle) override;
  StateId next(const Cycle& cycle) override;

  bool haveReference() const { return haveReference_; }
  const Quat& reference() const { return reference_; }
  bool speedExitArmed() const { return armed_; }

 private:
  Canards& canards_;
  Persistence slow_;
  Quat reference_;
  bool haveReference_ = false;
  bool armed_ = false;
};
