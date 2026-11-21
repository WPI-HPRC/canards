#include "StateMachine/states/LandState.h"

#include "Subsystems/Canards/Canards.h"

void LandState::execute(const Cycle&) { canards_.requestNeutral(); }
