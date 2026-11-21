#include "StateMachine/states/LaunchDetector.h"

#include "Subsystems/Navigation/Navigation.h"

bool LaunchDetector::update(const Cycle& cycle) {
  const bool high = cycle.nav.imuValid() && cycle.nav.specificForce() >= FlightConfig::kLaunchAccel;
  return hold_.update(high, cycle.timeUs);
}
