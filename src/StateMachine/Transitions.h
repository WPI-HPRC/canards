#pragma once

// The allowed state transitions, declared once
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.1).
// The StateMachine refuses anything that is not in this list.

#include <cstddef>

#include "StateMachine/StateId.h"

struct Transition {
  StateId from;
  StateId to;
};

constexpr Transition kAllowedTransitions[] = {
    {StateId::Init, StateId::PreFlight},
    {StateId::Init, StateId::Boost},      // launch without PreFlight: canards locked out
    {StateId::PreFlight, StateId::Init},  // stand-down (PROPOSAL, for a scrub)
    {StateId::PreFlight, StateId::Boost},
    {StateId::Boost, StateId::Control},
    {StateId::Control, StateId::Coast},
    {StateId::Coast, StateId::Chute},
    {StateId::Chute, StateId::Land},
};

constexpr size_t kNumAllowedTransitions =
    sizeof(kAllowedTransitions) / sizeof(kAllowedTransitions[0]);

constexpr bool isAllowedTransition(StateId from, StateId to) {
  for (size_t i = 0; i < kNumAllowedTransitions; ++i) {
    if (kAllowedTransitions[i].from == from && kAllowedTransitions[i].to == to) return true;
  }
  return false;
}
