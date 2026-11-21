#pragma once

// One of the flight states: see StateMachine/states/README.md.

#include "FlightConfig.h"
#include "StateMachine/Persistence.h"
#include "StateMachine/State.h"

// Launch: |specific force| from the raw IMU >= 5 g, held kLaunchHoldUs. Never
// the estimator. Shared by Init and PreFlight.
class LaunchDetector {
 public:
  LaunchDetector() : hold_(FlightConfig::kLaunchHoldUs) {}
  void reset() { hold_.reset(); }
  bool update(const Cycle& cycle);

 private:
  Persistence hold_;
};
