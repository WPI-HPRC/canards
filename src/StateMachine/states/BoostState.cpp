#include "StateMachine/states/BoostState.h"

#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Navigation/Navigation.h"

void BoostState::init(const Cycle&) { altitude_.reset(); }

void BoostState::execute(const Cycle&) { canards_.requestNeutral(); }

StateId BoostState::next(const Cycle& cycle) {
  const Navigation& nav = cycle.nav;
  const bool high = nav.altitudeValid() && nav.altitudeAgl() >= FlightConfig::kControlStartAltitudeM;
  if (altitude_.update(high, cycle.timeUs)) return StateId::Control;
  if (cycle.timeInStateUs >= FlightConfig::kBoostMaxUs) return StateId::Control;
  return StateId::Boost;
}
