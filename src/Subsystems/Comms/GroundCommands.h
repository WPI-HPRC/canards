#pragma once

// The ground station's commands, turned into flight-program Commands.
//
// The ground station (ground/telemetry_gui.py, src/telemetry.*) sends named
// commands ("$C,<name>") and wants an immediate OK or REJECTED. The handlers
// that telemetry.addCommand() registers call handle(), which judges the
// command against the flight program's state right now, queues the Commands
// it stands for, and gives the reply. The FlightComputer then polls this
// object as its CommandSource, and the flight logic judges every Command
// again when it is applied.
//
//   armFlight     Init or PreFlight: go to PreFlight (the estimator
//                 re-initialises) and enable the canards for control.
//                 REJECTED once in flight.
//   actuatorTest  Init only, and not while a sweep is already running: the
//                 GroundTest sweep. REJECTED otherwise.
//
// Wiring in main.cpp, once the FlightComputer runs there:
//
//   static GroundCommands ground;   // also the FlightComputer's CommandSource
//   static bool onArmFlight() {
//     return ground.handle(GroundCommands::kArmFlight, flight.state(), flight.sweepActive()) == GroundReply::Ok;
//   }
//   static bool onActuatorTest() {
//     return ground.handle(GroundCommands::kActuatorTest, flight.state(), flight.sweepActive()) == GroundReply::Ok;
//   }
//   telemetry.addCommand(GroundCommands::kArmFlight, onArmFlight);
//   telemetry.addCommand(GroundCommands::kActuatorTest, onActuatorTest);
//
// No Arduino, no heap: a fixed-size queue.

#include <cstddef>

#include "StateMachine/StateId.h"
#include "Subsystems/Comms/Command.h"

enum class GroundReply : uint8_t { Ok, Rejected, Unknown };

class GroundCommands final : public CommandSource {
 public:
  // The names the ground station sends. Must match COMMANDS in
  // ground/telemetry_gui.py.
  static constexpr const char* kArmFlight = "armFlight";
  static constexpr const char* kActuatorTest = "actuatorTest";

  static constexpr size_t kCapacity = 8;

  // Judges `name` against the flight state now and queues its Commands.
  // REJECTED also if the queue has no room for all of them (nothing is
  // queued then).
  GroundReply handle(const char* name, StateId state, bool sweepActive);

  bool poll(Command& out) override;

  size_t pending() const { return count_; }

 private:
  bool push(const Command* commands, size_t n);

  Command queue_[kCapacity];
  size_t head_ = 0;
  size_t count_ = 0;
};
