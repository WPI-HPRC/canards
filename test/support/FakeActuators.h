#pragma once

// Actuators for desktop tests. readFeedback() returns whatever the test put in
// `feedback` (by default: no encoder valid, so the servo model is used).
// write() remembers the last command and how many were written.
//
// Test-only: uses std::vector.

#include <cstdint>
#include <vector>

#include "Actuators.h"

class FakeActuators final : public Actuators {
 public:
  // What readFeedback() hands out.
  ActuatorFeedback feedback;

  void readFeedback(ActuatorFeedback& out) override {
    ++readCount_;
    out = feedback;
  }

  void write(const ActuatorCommand& command) override {
    ++writeCount_;
    last_ = command;
  }

  const ActuatorCommand& last() const { return last_; }
  uint32_t readCount() const { return readCount_; }
  uint32_t writeCount() const { return writeCount_; }

 private:
  ActuatorCommand last_;
  uint32_t readCount_ = 0;
  uint32_t writeCount_ = 0;
};
