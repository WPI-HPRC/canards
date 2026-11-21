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

const char* GroundCommands::commandFromLine(char* line) {
  if (line == nullptr) return nullptr;
  while (*line == ' ' || *line == '\t') ++line;
  size_t len = std::strlen(line);
  while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t' || line[len - 1] == '\r')) line[--len] = '\0';
  if (len == 0) return nullptr;
  if (line[0] != '$') return line;

  // "$C,<name>*XX": XX is the XOR of every byte between '$' and '*'.
  char* star = std::strrchr(line, '*');
  if (star == nullptr || std::strlen(star) != 3) return nullptr;
  auto hex = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  };
  const int hi = hex(star[1]);
  const int lo = hex(star[2]);
  if (hi < 0 || lo < 0) return nullptr;
  uint8_t cs = 0;
  for (const char* p = line + 1; p < star; ++p) cs ^= static_cast<uint8_t>(*p);
  if (cs != static_cast<uint8_t>(hi << 4 | lo)) return nullptr;
  *star = '\0';
  if (std::strncmp(line + 1, "C,", 2) != 0) return nullptr;
  return line + 3;
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
