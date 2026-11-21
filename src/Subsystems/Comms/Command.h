#pragma once

// Ground commands: the boundary between the flight program and the ground
// station (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.5).
//
// The ground-station owner maps radio packets to `Command` and implements a
// `CommandSource`. The flight program never sees radio packets.

#include <cstddef>
#include <cstdint>

enum class CommandType : uint8_t {
  EnterPreFlight,  // Init -> PreFlight; in PreFlight, re-initialise the estimator
  StandDown,       // PreFlight -> Init (a scrub)
  CanardsEnable,   // accepted only on the ground (Init, PreFlight)
  CanardsDisable,  // always accepted
  CanardTest,      // ground test: move `canard` to `angleRad`
  CanardTestStop,  // ground test: back to neutral
};

struct Command {
  CommandType type = CommandType::CanardTestStop;
  int8_t canard = -1;      // canard index 0..3, or -1 for all (CanardTest only)
  float angleRad = 0.0f;   // CanardTest only
};

// Where commands come from: the radio in flight, the recorded log in replay,
// the host in a USB stream.
class CommandSource {
 public:
  virtual ~CommandSource() = default;

  // Fetches the next pending command. Returns false if there is none.
  virtual bool poll(Command& out) = 0;
};

// The commands received in one cycle. Fixed size: no heap.
class CommandList {
 public:
  static constexpr size_t kCapacity = 8;

  // Returns false (and drops nothing already stored) if the list is full.
  bool push(const Command& c) {
    if (count_ >= kCapacity) return false;
    items_[count_++] = c;
    return true;
  }
  void clear() { count_ = 0; }

  size_t size() const { return count_; }
  bool full() const { return count_ >= kCapacity; }
  const Command& operator[](size_t i) const { return items_[i]; }
  const Command* begin() const { return items_; }
  const Command* end() const { return items_ + count_; }

  // True if any command of this type arrived this cycle.
  bool has(CommandType type) const {
    for (size_t i = 0; i < count_; ++i) {
      if (items_[i].type == type) return true;
    }
    return false;
  }

 private:
  Command items_[kCapacity];
  size_t count_ = 0;
};
