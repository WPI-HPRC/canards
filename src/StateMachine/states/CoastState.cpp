#include "StateMachine/states/CoastState.h"

#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Navigation/Navigation.h"

void CoastState::init(const Cycle&) { slow_.reset(); }

void CoastState::execute(const Cycle&) { canards_.requestNeutral(); }

StateId CoastState::next(const Cycle& cycle) {
  const Navigation& nav = cycle.nav;
  const bool slow = nav.speedValid() && nav.speed() < FlightConfig::kChuteSpeed;
  if (slow_.update(slow, cycle.timeUs)) return StateId::Chute;
  if (cycle.timeInStateUs >= FlightConfig::kCoastMaxUs) return StateId::Chute;
  return StateId::Coast;
}
