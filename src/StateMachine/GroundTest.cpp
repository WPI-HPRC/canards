#include "StateMachine/GroundTest.h"

#include <cmath>

#include "FlightConfig.h"
#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"

void GroundTest::reset() {
  active_ = false;
  untilUs_ = 0;
  for (int i = 0; i < kNumCanards; ++i) anglesRad_[i] = 0.0f;
}

bool GroundTest::update(const Cycle& cycle) {
  // Commands are applied in the order they arrived.
  for (const Command& command : cycle.commands) {
    if (command.type == CommandType::CanardTestStop) {
      reset();
    } else if (command.type == CommandType::CanardTest) {
      if (!std::isfinite(command.angleRad)) continue;
      if (command.canard == -1) {
        for (int i = 0; i < kNumCanards; ++i) anglesRad_[i] = command.angleRad;
      } else if (command.canard >= 0 && command.canard < kNumCanards) {
        anglesRad_[command.canard] = command.angleRad;
      } else {
        continue;  // no such canard
      }
      active_ = true;
      untilUs_ = cycle.timeUs + FlightConfig::kGroundTestHoldUs;
    }
  }

  if (active_ && cycle.timeUs >= untilUs_) reset();  // the hold ran out
  if (active_) canards_.requestTestAngles(anglesRad_);
  return active_;
}
