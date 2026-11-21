#include "StateMachine/states/PreFlightState.h"

#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Navigation/Navigation.h"

void PreFlightState::init(const Cycle&) {
  groundTest_.reset();
  launch_.reset();
  nav_.requestReinitialize();  // gyro bias, pad alignment, altitude reference
}

void PreFlightState::execute(const Cycle& cycle) {
  // PROPOSAL (not yet confirmed by the user): a repeated "go to preflight"
  // while already here re-initialises the estimator again.
  if (cycle.commands.has(CommandType::EnterPreFlight)) nav_.requestReinitialize();
  if (!groundTest_.update(cycle)) canards_.requestNeutral();
}

void PreFlightState::end(const Cycle&) { groundTest_.reset(); }

StateId PreFlightState::next(const Cycle& cycle) {
  if (launch_.update(cycle)) return StateId::Boost;
  // PROPOSAL (not yet confirmed by the user): stand down for a scrub.
  if (cycle.commands.has(CommandType::StandDown)) return StateId::Init;
  return StateId::PreFlight;
}
