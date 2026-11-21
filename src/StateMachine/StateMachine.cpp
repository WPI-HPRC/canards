#include "StateMachine/StateMachine.h"

#include <cstddef>

#include "StateMachine/Transitions.h"

StateMachine::StateMachine(State* const states[kNumStates]) {
  for (size_t i = 0; i < kNumStates; ++i) states_[i] = states[i];
}

void StateMachine::step(Cycle& cycle) {
  transitioned_ = false;
  refusedThisStep_ = false;

  if (!started_) {
    // The machine starts in Init, but Init is only "entered" when the first
    // sample arrives: that sample's time is the entry time.
    started_ = true;
    enteredAtUs_ = cycle.timeUs;
    cycle.timeInStateUs = 0;
    states_[indexOf(current_)]->init(cycle);
  }

  // If sample time jumps back to before the entry time, report 0 instead of a
  // huge unsigned difference that would fire every state timeout at once.
  cycle.timeInStateUs = cycle.timeUs >= enteredAtUs_ ? cycle.timeUs - enteredAtUs_ : 0;

  State& state = *states_[indexOf(current_)];
  state.execute(cycle);
  const StateId wanted = state.next(cycle);

  if (wanted == current_) return;  // stay

  // An id outside the enum can never be in the allowed list, but check the
  // range first so it can never index states_ either.
  if (indexOf(wanted) >= kNumStates || !isAllowedTransition(current_, wanted)) {
    refusedThisStep_ = true;
    ++refusedCount_;
    lastRefusedFrom_ = current_;
    lastRefusedTo_ = wanted;
    return;
  }

  // end() still sees the old state's time in state.
  state.end(cycle);

  previous_ = current_;
  current_ = wanted;
  enteredAtUs_ = cycle.timeUs;
  transitioned_ = true;
  ++transitionCount_;

  // The new state is entered in this same step. Its first execute() is on the
  // next step, so one step can never chain two transitions.
  cycle.timeInStateUs = 0;
  states_[indexOf(current_)]->init(cycle);
}
