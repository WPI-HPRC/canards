#pragma once

// The flight states, in flight order
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.1).

#include <cstddef>
#include <cstdint>

enum class StateId : uint8_t {
  Init = 0,   // powered on the pad; ground tests allowed
  PreFlight,  // about a minute before launch; estimator re-initialised
  Boost,      // motor burning; canards neutral, servos powered
  Control,    // canards hold attitude (LQR)
  Coast,      // canards neutral, powered, up to apogee
  Chute,      // descent; servos powered off
  Land,       // terminal
};

constexpr size_t kNumStates = 7;

constexpr size_t indexOf(StateId id) { return static_cast<size_t>(id); }

// True for Init and PreFlight: the rocket is sitting on the pad.
constexpr bool isGroundState(StateId id) {
  return id == StateId::Init || id == StateId::PreFlight;
}

// True from Boost onwards.
constexpr bool isFlightState(StateId id) { return !isGroundState(id); }

constexpr const char* stateName(StateId id) {
  switch (id) {
    case StateId::Init: return "Init";
    case StateId::PreFlight: return "PreFlight";
    case StateId::Boost: return "Boost";
    case StateId::Control: return "Control";
    case StateId::Coast: return "Coast";
    case StateId::Chute: return "Chute";
    case StateId::Land: return "Land";
  }
  return "?";
}
