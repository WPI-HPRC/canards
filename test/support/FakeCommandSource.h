#pragma once

// A CommandSource for desktop tests: a FIFO the test pushes ground commands
// into. poll() hands them out in order, one per call, and remembers every
// command it handed out, so a test can check that none was lost or reordered.
//
// Test-only: uses std::vector.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "Subsystems/Comms/Command.h"

class FakeCommandSource final : public CommandSource {
 public:
  void push(const Command& command) { queued_.push_back(command); }

  bool poll(Command& out) override {
    ++pollCalls_;
    if (next_ >= queued_.size()) return false;
    out = queued_[next_++];
    delivered_.push_back(out);
    return true;
  }

  // Commands pushed but not yet handed out.
  size_t pendingCount() const { return queued_.size() - next_; }
  // Every command handed out, in order.
  const std::vector<Command>& delivered() const { return delivered_; }
  // Calls of poll(), including those that returned false.
  uint32_t pollCalls() const { return pollCalls_; }

 private:
  std::vector<Command> queued_;
  size_t next_ = 0;
  std::vector<Command> delivered_;
  uint32_t pollCalls_ = 0;
};

// Shorthands for building commands.
inline Command makeCommand(CommandType type) {
  Command c;
  c.type = type;
  return c;
}

inline Command makeTestCommand(int8_t canard, float angleRad) {
  Command c;
  c.type = CommandType::CanardTest;
  c.canard = canard;
  c.angleRad = angleRad;
  return c;
}
