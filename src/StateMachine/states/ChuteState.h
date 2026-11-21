#pragma once

// One of the flight states: see StateMachine/states/README.md.

#include "FlightConfig.h"
#include "StateMachine/Persistence.h"
#include "StateMachine/State.h"

class Canards;

// Descent. The Canards subsystem turns servo power off here by itself.
class ChuteState final : public State {
 public:
  explicit ChuteState(Canards& canards) : canards_(canards), low_(FlightConfig::kLandHoldUs) {}
  void init(const Cycle&) override;
  void execute(const Cycle&) override;
  StateId next(const Cycle& cycle) override;

 private:
  Canards& canards_;
  Persistence low_;
};
