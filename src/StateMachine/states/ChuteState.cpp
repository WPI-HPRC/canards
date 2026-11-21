#include "StateMachine/states/ChuteState.h"

#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Navigation/Navigation.h"

void ChuteState::init(const Cycle&) { low_.reset(); }

void ChuteState::execute(const Cycle&) { canards_.requestNeutral(); }

StateId ChuteState::next(const Cycle& cycle) {
  const Navigation& nav = cycle.nav;
  const bool low = nav.altitudeValid() && nav.altitudeAgl() < FlightConfig::kLandAltitudeM;
  if (low_.update(low, cycle.timeUs)) return StateId::Land;
  if (cycle.timeInStateUs >= FlightConfig::kChuteMaxUs) return StateId::Land;
  return StateId::Chute;
}
