#include "StateMachine/GroundTest.h"

#include <cmath>

#include "FlightConfig.h"
#include "Subsystems/Canards/Canards.h"
#include "Subsystems/Comms/Command.h"

void GroundTest::reset() {
  active_ = false;
  untilUs_ = 0;
  sweeping_ = false;
  sweepStartUs_ = 0;
  for (int i = 0; i < kNumCanards; ++i) {
    heldRad_[i] = 0.0f;
    anglesRad_[i] = 0.0f;
  }
}

void GroundTest::sweepAngles(int step, float out[kNumCanards]) {
  for (int i = 0; i < kNumCanards; ++i) out[i] = 0.0f;
  if (step < 0 || step >= kSweepSteps) return;
  const float a = FlightConfig::kSweepAmplitudeRad;
  if (step < 2 * kNumCanards) {
    out[step / 2] = (step % 2 == 0) ? a : -a;  // canard step/2: + then -
  } else {
    for (int i = 0; i < kNumCanards; ++i) out[i] = (step == 2 * kNumCanards) ? a : -a;  // all: + then -
  }
}

bool GroundTest::update(const Cycle& cycle) {
  // Commands are applied in the order they arrived.
  for (const Command& command : cycle.commands) {
    if (command.type == CommandType::CanardTestStop) {
      reset();
    } else if (command.type == CommandType::CanardSweep) {
      if (!allowSweep_) continue;
      // The sweep replaces any held test: forget its angles, so a later
      // single-canard CanardTest starts from neutral on the others.
      active_ = false;
      for (int i = 0; i < kNumCanards; ++i) heldRad_[i] = 0.0f;
      sweeping_ = true;
      sweepStartUs_ = cycle.timeUs;
    } else if (command.type == CommandType::CanardTest) {
      if (!std::isfinite(command.angleRad)) continue;
      if (command.canard == -1) {
        for (int i = 0; i < kNumCanards; ++i) heldRad_[i] = command.angleRad;
      } else if (command.canard >= 0 && command.canard < kNumCanards) {
        heldRad_[command.canard] = command.angleRad;
      } else {
        continue;  // no such canard
      }
      sweeping_ = false;
      active_ = true;
      untilUs_ = cycle.timeUs + FlightConfig::kGroundTestHoldUs;
    }
  }

  if (sweeping_) {
    const uint64_t step = (cycle.timeUs - sweepStartUs_) / FlightConfig::kSweepStepUs;
    if (step >= static_cast<uint64_t>(kSweepSteps)) {
      reset();  // finished
    } else {
      sweepAngles(static_cast<int>(step), anglesRad_);
      canards_.requestTestAngles(anglesRad_);
      return true;
    }
  }

  if (active_ && cycle.timeUs >= untilUs_) reset();  // the hold ran out
  if (active_) {
    for (int i = 0; i < kNumCanards; ++i) anglesRad_[i] = heldRad_[i];
    canards_.requestTestAngles(anglesRad_);
  }
  return active_;
}
