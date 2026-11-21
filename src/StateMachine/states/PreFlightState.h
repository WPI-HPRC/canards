#pragma once

// One of the flight states: see StateMachine/states/README.md.

#include "FlightConfig.h"
#include "StateMachine/GroundTest.h"
#include "StateMachine/State.h"
#include "StateMachine/states/LaunchDetector.h"

class Canards;
class Navigation;

// About a minute before launch: the estimator is re-initialised on entry,
// and again when the Morse greeting (FlightConfig.h) ends.
// No timeout: the rocket may wait on the pad.
class PreFlightState final : public State {
 public:
  // `morseText`: the greeting, "" for none.
  PreFlightState(Canards& canards, Navigation& nav, const char* morseText = FlightConfig::kPreFlightMorseText)
      : canards_(canards), nav_(nav), groundTest_(canards, /*allowSweep=*/false), morseText_(morseText) {}
  void init(const Cycle&) override;
  void execute(const Cycle& cycle) override;
  void end(const Cycle&) override;
  StateId next(const Cycle& cycle) override;

  bool greetingRunning() const { return morseRunning_; }

 private:
  Canards& canards_;
  Navigation& nav_;
  GroundTest groundTest_;
  LaunchDetector launch_;
  // The Morse greeting (FlightConfig::kPreFlightMorseText), from entry.
  const char* morseText_;
  bool morseRunning_ = false;
  uint64_t morseStartUs_ = 0;
};
