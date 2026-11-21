#pragma once

// Runs the states (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.2).
//
// Order within one step():
//   1. current.execute()
//   2. current.next()
//   3. if next() named another state and the transition is allowed:
//        current.end(), then new.init(), in this same step
//   4. the new state's first execute() runs on the NEXT step
// On the very first step, Init.init() is called first.
//
// A transition that is not in kAllowedTransitions is refused: the machine
// stays where it is, counts the refusal and remembers it for the log.

#include <cstdint>

#include "StateMachine/State.h"
#include "StateMachine/StateId.h"

class StateMachine {
 public:
  // states[i] handles StateId(i). Every entry must be non-null and must
  // outlive the machine. The machine starts in Init.
  explicit StateMachine(State* const states[kNumStates]);

  // Runs one cycle. Fills in cycle.timeInStateUs before calling the state.
  void step(Cycle& cycle);

  StateId current() const { return current_; }

  // Sample time at which the current state was entered.
  uint64_t enteredAtUs() const { return enteredAtUs_; }

  // True if the last step() changed state.
  bool transitioned() const { return transitioned_; }
  // The state left by the last transition (Init before any transition).
  StateId previous() const { return previous_; }
  uint32_t transitionCount() const { return transitionCount_; }

  // Refused transitions.
  bool refusedThisStep() const { return refusedThisStep_; }
  uint32_t refusedCount() const { return refusedCount_; }
  StateId lastRefusedFrom() const { return lastRefusedFrom_; }
  StateId lastRefusedTo() const { return lastRefusedTo_; }

 private:
  State* states_[kNumStates];
  StateId current_ = StateId::Init;
  StateId previous_ = StateId::Init;
  uint64_t enteredAtUs_ = 0;
  bool started_ = false;
  bool transitioned_ = false;
  uint32_t transitionCount_ = 0;
  bool refusedThisStep_ = false;
  uint32_t refusedCount_ = 0;
  StateId lastRefusedFrom_ = StateId::Init;
  StateId lastRefusedTo_ = StateId::Init;
};
