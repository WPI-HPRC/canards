#pragma once

// A scriptable AttitudeController for desktop tests.
//
// A test programs what the next compute() does:
//   - setCommand(): return true with these four canard commands [rad];
//   - setFailure(): return false with an all-zero command, as the interface
//     promises for a controller that cannot produce a trustworthy command;
//   - setNonFiniteOutput(): return true, but with one canard's command set to
//     a value that is not a finite number (a broken controller).
// The script stays in force until it is changed.
//
// Afterwards it reads back how many times compute() was called and the input
// of the most recent call.

#include <cstdint>
#include <limits>

#include "AttitudeControl/AttitudeController.h"

class FakeAttitudeController final : public AttitudeController {
 public:
  // --- Script ----------------------------------------------------------------

  void setCommand(float c0, float c1, float c2, float c3) {
    const float c[4] = {c0, c1, c2, c3};
    setCommand(c);
  }
  void setCommand(const float rad[4]) {
    result_ = true;
    for (int i = 0; i < 4; ++i) command_[i] = rad[i];
  }

  void setFailure() { result_ = false; }

  // The other canards keep their programmed commands.
  void setNonFiniteOutput(int canard,
                          float value = std::numeric_limits<float>::quiet_NaN()) {
    result_ = true;
    command_[canard] = value;
  }

  // --- AttitudeController ----------------------------------------------------

  bool compute(const AttitudeControlInput& in, float canardCommand[4]) override {
    lastInput_ = in;
    ++callCount_;
    for (int i = 0; i < 4; ++i) canardCommand[i] = result_ ? command_[i] : 0.0f;
    return result_;
  }

  // --- What happened ---------------------------------------------------------

  uint32_t callCount() const { return callCount_; }
  // Default-constructed before the first call.
  const AttitudeControlInput& lastInput() const { return lastInput_; }

  // Forgets the recorded calls. The script is kept.
  void clearLog() {
    callCount_ = 0;
    lastInput_ = AttitudeControlInput{};
  }

 private:
  bool result_ = true;
  float command_[4] = {};
  AttitudeControlInput lastInput_;
  uint32_t callCount_ = 0;
};
