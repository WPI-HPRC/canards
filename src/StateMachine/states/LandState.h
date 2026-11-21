#pragma once

// One of the flight states: see StateMachine/states/README.md.

#include "StateMachine/State.h"

class Canards;

// Terminal. Logging carries on.
class LandState final : public State {
 public:
  explicit LandState(Canards& canards) : canards_(canards) {}
  void execute(const Cycle&) override;
  StateId next(const Cycle&) override { return StateId::Land; }

 private:
  Canards& canards_;
};
