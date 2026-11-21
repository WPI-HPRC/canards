#include "Subsystems/Comms/GroundCommands.h"

#include <cstring>

namespace {

Command make(CommandType type) {
  Command c;
  c.type = type;
  return c;
}

}  // namespace

GroundReply GroundCommands::handle(const char* name, StateId state, bool sweepActive) {
  if (name == nullptr) return GroundReply::Unknown;

  if (std::strcmp(name, kArmFlight) == 0) {
    if (!isGroundState(state)) return GroundReply::Rejected;
    const Command commands[] = {make(CommandType::EnterPreFlight), make(CommandType::CanardsEnable)};
    return push(commands, 2) ? GroundReply::Ok : GroundReply::Rejected;
  }

  if (std::strcmp(name, kActuatorTest) == 0) {
    if (state != StateId::Init || sweepActive) return GroundReply::Rejected;
    const Command commands[] = {make(CommandType::CanardSweep)};
    return push(commands, 1) ? GroundReply::Ok : GroundReply::Rejected;
  }

  return GroundReply::Unknown;
}

bool GroundCommands::push(const Command* commands, size_t n) {
  if (count_ + n > kCapacity) return false;
  for (size_t i = 0; i < n; ++i) queue_[(head_ + count_ + i) % kCapacity] = commands[i];
  count_ += n;
  return true;
}

bool GroundCommands::poll(Command& out) {
  if (count_ == 0) return false;
  out = queue_[head_];
  head_ = (head_ + 1) % kCapacity;
  --count_;
  return true;
}
