#pragma once

// One of the flight states: see StateMachine/states/README.md.

#include "FlightConfig.h"
#include "StateMachine/Persistence.h"
#include "StateMachine/State.h"

class Canards;

// Motor burning. Canards neutral, servos powered.
class BoostState final : public State {
 public:
  explicit BoostState(Canards& canards) : canards_(canards), altitude_(FlightConfig::kControlStartHoldUs) {}
  void init(const Cycle&) override;
  void execute(const Cycle&) override;
  StateId next(const Cycle& cycle) override;

 private:
  Canards& canards_;
  Persistence altitude_;
};
