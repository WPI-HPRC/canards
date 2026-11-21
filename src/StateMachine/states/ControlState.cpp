#include "StateMachine/states/ControlState.h"

#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Navigation/Navigation.h"

void ControlState::init(const Cycle&) {
  slow_.reset();
  haveReference_ = false;
  armed_ = false;
}

void ControlState::execute(const Cycle& cycle) {
  if (!haveReference_ && cycle.nav.attitudeValid()) {
    reference_ = verticalReference(cycle.nav.attitude());
    haveReference_ = true;
  }
  if (haveReference_) {
    canards_.requestAttitudeHold(reference_);
  } else {
    canards_.requestNeutral();
  }
}

StateId ControlState::next(const Cycle& cycle) {
  if (canards_.status().cutoffLatched) return StateId::Coast;

  // PROPOSAL (not yet confirmed by the user): the speed exit arms only once
  // the speed has been at or above kControlEndSpeed in this state. Control is
  // entered at 50 ft AGL while the motor is still burning, possibly below
  // kControlEndSpeed; without arming, the state would leave at once and never
  // control.
  const Navigation& nav = cycle.nav;
  if (nav.speedValid() && nav.speed() >= FlightConfig::kControlEndSpeed) armed_ = true;
  const bool slow = armed_ && nav.speedValid() && nav.speed() < FlightConfig::kControlEndSpeed;
  if (slow_.update(slow, cycle.timeUs)) return StateId::Coast;

  if (cycle.timeInStateUs >= FlightConfig::kControlMaxUs) return StateId::Coast;
  return StateId::Control;
}
