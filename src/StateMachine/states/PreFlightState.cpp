#include "StateMachine/states/PreFlightState.h"

#include "FlightConfig.h"
#include "StateMachine/Morse.h"
#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Navigation/Navigation.h"

void PreFlightState::init(const Cycle& cycle) {
  groundTest_.reset();
  launch_.reset();
  nav_.requestReinitialize();  // gyro bias, pad alignment, altitude reference
  morseRunning_ = morse::totalUnits(morseText_) > 0;
  morseStartUs_ = cycle.timeUs;
}

void PreFlightState::execute(const Cycle& cycle) {
  // PROPOSAL (not yet confirmed by the user): a repeated "go to preflight"
  // while already here re-initialises the estimator again.
  if (cycle.commands.has(CommandType::EnterPreFlight)) nav_.requestReinitialize();

  // Any ground-test command ends the greeting: the operator wants the canards.
  if (morseRunning_ && (cycle.commands.has(CommandType::CanardTest) ||
                        cycle.commands.has(CommandType::CanardTestStop))) {
    morseRunning_ = false;
    nav_.requestReinitialize();
  }
  if (groundTest_.update(cycle)) return;

  if (morseRunning_) {
    const uint64_t unit = (cycle.timeUs - morseStartUs_) / FlightConfig::kPreFlightMorseUnitUs;
    if (unit >= morse::totalUnits(morseText_)) {
      morseRunning_ = false;
      // The canards were moving during the first alignment: align again, still.
      nav_.requestReinitialize();
    } else {
      const float a = morse::isOn(morseText_, static_cast<unsigned>(unit))
                          ? FlightConfig::kPreFlightMorseAngleRad
                          : 0.0f;
      const float rad[kNumCanards] = {a, a, a, a};
      canards_.requestTestAngles(rad);
      return;
    }
  }
  canards_.requestNeutral();
}

void PreFlightState::end(const Cycle&) {
  groundTest_.reset();
  morseRunning_ = false;
}

StateId PreFlightState::next(const Cycle& cycle) {
  if (launch_.update(cycle)) return StateId::Boost;
  // PROPOSAL (not yet confirmed by the user): stand down for a scrub.
  if (cycle.commands.has(CommandType::StandDown)) return StateId::Init;
  return StateId::PreFlight;
}
