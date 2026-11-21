#pragma once

// One of the flight states: see StateMachine/states/README.md.

#include "FlightConfig.h"
#include "StateMachine/Persistence.h"
#include "StateMachine/State.h"

class Canards;

// Canards neutral and powered, up to near apogee.
class CoastState final : public State {
 public:
  explicit CoastState(Canards& canards) : canards_(canards), slow_(FlightConfig::kChuteHoldUs) {}
  void init(const Cycle&) override;
  void execute(const Cycle&) override;
  StateId next(const Cycle& cycle) override;

 private:
  Canards& canards_;
  Persistence slow_;
};
