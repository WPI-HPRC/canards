#include "StateMachine/states/InitState.h"

#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"

void InitState::init(const Cycle&) {
  groundTest_.reset();
  launch_.reset();
}

void InitState::execute(const Cycle& cycle) {
  if (!groundTest_.update(cycle)) canards_.requestNeutral();
}

void InitState::end(const Cycle&) { groundTest_.reset(); }

StateId InitState::next(const Cycle& cycle) {
  // Launch wins over a command in the same cycle. A launch from here skips
  // PreFlight, and the Canards subsystem then locks the canards out.
  if (launch_.update(cycle)) return StateId::Boost;
  if (cycle.commands.has(CommandType::EnterPreFlight)) return StateId::PreFlight;
  return StateId::Init;
}
