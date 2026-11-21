// The state machine core: lifecycle order, the allowed-transition list and
// Persistence (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 6.1, 6.2
// and step 3 of section 10).
//
// The states here are recording fakes that write every call into one shared
// log, so the ORDER of calls across states can be checked.
#include <unity.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "FakeEstimator.h"
#include "FlightConfig.h"
#include "StateMachine/Persistence.h"
#include "StateMachine/State.h"
#include "StateMachine/StateId.h"
#include "StateMachine/StateMachine.h"
#include "StateMachine/Transitions.h"
#include "Subsystems/Comms/Command.h"
#include "Subsystems/Navigation/Navigation.h"

void setUp() {}
void tearDown() {}

namespace {

// ---------------------------------------------------------------------------
// Recording fakes
// ---------------------------------------------------------------------------

enum class Method : uint8_t { Init, Execute, Next, End };

struct Entry {
  StateId state;
  Method method;
  uint64_t timeUs;
  uint64_t timeInStateUs;
  const Navigation* nav;
  const CommandList* commands;
};

// What a test expects to find in the log: which state, which method.
struct Expected {
  StateId state;
  Method method;
};

struct CallLog {
  static constexpr size_t kCapacity = 256;
  Entry entries[kCapacity] = {};
  size_t count = 0;
  bool overflowed = false;

  void add(StateId state, Method method, const Cycle& cycle) {
    if (count >= kCapacity) {
      overflowed = true;
      return;
    }
    entries[count++] = {state, method, cycle.timeUs, cycle.timeInStateUs, &cycle.nav,
                        &cycle.commands};
  }
  void clear() {
    count = 0;
    overflowed = false;
  }
};

// Records every call. next() returns whatever the test put in `wanted`.
class RecordingState final : public State {
 public:
  RecordingState(StateId id, CallLog& log) : wanted(id), id_(id), log_(log) {}

  StateId wanted;  // own id = stay

  void init(const Cycle& cycle) override { log_.add(id_, Method::Init, cycle); }
  void execute(const Cycle& cycle) override { log_.add(id_, Method::Execute, cycle); }
  void end(const Cycle& cycle) override { log_.add(id_, Method::End, cycle); }
  StateId next(const Cycle& cycle) override {
    log_.add(id_, Method::Next, cycle);
    return wanted;
  }

 private:
  StateId id_;
  CallLog& log_;
};

constexpr StateId kAllStates[kNumStates] = {
    StateId::Init,  StateId::PreFlight, StateId::Boost, StateId::Control,
    StateId::Coast, StateId::Chute,     StateId::Land,
};

constexpr uint64_t kStepUs = 5000;       // an arbitrary sample period for these tests
constexpr uint64_t kStartUs = 5000000;   // not zero, so "entry time" is a real value

// One state machine with seven recording states, plus the real Navigation and
// CommandList that Cycle refers to.
struct Rig {
  FakeEstimator estimator;
  Navigation nav{estimator};
  CommandList commands;
  CallLog log;
  RecordingState states[kNumStates] = {
      {StateId::Init, log},  {StateId::PreFlight, log}, {StateId::Boost, log},
      {StateId::Control, log}, {StateId::Coast, log},   {StateId::Chute, log},
      {StateId::Land, log},
  };
  State* table[kNumStates] = {&states[0], &states[1], &states[2], &states[3],
                              &states[4], &states[5], &states[6]};
  StateMachine machine{table};
  Cycle cycle{kStartUs - kStepUs, 0.005f, 0, nav, commands};

  RecordingState& state(StateId id) { return states[indexOf(id)]; }

  void stepAt(uint64_t timeUs) {
    cycle.timeUs = timeUs;
    machine.step(cycle);
  }
  // Steps one sample period later than the previous step.
  void step() { stepAt(cycle.timeUs + kStepUs); }

  // Walks the machine along Init -> PreFlight -> Boost -> ... until it is in
  // `target`, leaves every state asking to stay, and clears the log.
  void driveTo(StateId target) {
    if (target == StateId::Init) step();  // so that Init has been entered
    for (size_t i = 0; i < indexOf(target); ++i) {
      states[i].wanted = kAllStates[i + 1];
      step();
      states[i].wanted = kAllStates[i];
    }
    TEST_ASSERT_EQUAL_UINT8(indexOf(target), indexOf(machine.current()));
    TEST_ASSERT_EQUAL_UINT32(0, machine.refusedCount());
    log.clear();
  }
};

void expectLogAt(int line, const CallLog& log, std::initializer_list<Expected> expected) {
  UNITY_TEST_ASSERT(!log.overflowed, line, "call log overflowed");
  UNITY_TEST_ASSERT_EQUAL_UINT32(expected.size(), log.count, line, "number of state calls");
  size_t i = 0;
  for (const Expected& e : expected) {
    UNITY_TEST_ASSERT_EQUAL_STRING(stateName(e.state), stateName(log.entries[i].state), line,
                                   "state called, in order");
    UNITY_TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(e.method),
                                  static_cast<uint8_t>(log.entries[i].method), line,
                                  "method called, in order (0 init, 1 execute, 2 next, 3 end)");
    ++i;
  }
}

#define EXPECT_LOG(log, ...) expectLogAt(__LINE__, (log), {__VA_ARGS__})

void expectStateAt(int line, StateId expected, StateId actual) {
  UNITY_TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected), static_cast<uint8_t>(actual), line,
                                "state id");
}

#define EXPECT_STATE(expected, actual) expectStateAt(__LINE__, (expected), (actual))

// The allowed transitions, typed in from the table in doc section 6.1 and NOT
// from Transitions.h. Row = from, column = to.
constexpr bool kDocAllowed[kNumStates][kNumStates] = {
    //            Init   PreFl  Boost  Contr  Coast  Chute  Land
    /* Init  */ {false, true,  true,  false, false, false, false},
    /* PreFl */ {true,  false, true,  false, false, false, false},
    /* Boost */ {false, false, false, true,  false, false, false},
    /* Contr */ {false, false, false, false, true,  false, false},
    /* Coast */ {false, false, false, false, false, true,  false},
    /* Chute */ {false, false, false, false, false, false, true},
    /* Land  */ {false, false, false, false, false, false, false},
};

}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle order
// ---------------------------------------------------------------------------

static void test_constructing_calls_no_state_and_starts_in_init() {
  Rig rig;
  TEST_ASSERT_EQUAL_UINT32(0, rig.log.count);
  EXPECT_STATE(StateId::Init, rig.machine.current());
  EXPECT_STATE(StateId::Init, rig.machine.previous());
  TEST_ASSERT_FALSE(rig.machine.transitioned());
  TEST_ASSERT_FALSE(rig.machine.refusedThisStep());
  TEST_ASSERT_EQUAL_UINT32(0, rig.machine.transitionCount());
  TEST_ASSERT_EQUAL_UINT32(0, rig.machine.refusedCount());
}

static void test_first_step_runs_init_then_execute_then_next() {
  Rig rig;
  rig.stepAt(kStartUs);

  EXPECT_LOG(rig.log, {StateId::Init, Method::Init}, {StateId::Init, Method::Execute},
             {StateId::Init, Method::Next});
  EXPECT_STATE(StateId::Init, rig.machine.current());
  TEST_ASSERT_FALSE(rig.machine.transitioned());
  TEST_ASSERT_EQUAL_UINT64(kStartUs, rig.machine.enteredAtUs());
  // Entering Init on the first sample is not a transition.
  TEST_ASSERT_EQUAL_UINT32(0, rig.machine.transitionCount());
}

static void test_init_is_called_only_once_while_staying() {
  Rig rig;
  rig.stepAt(kStartUs);
  rig.log.clear();

  rig.step();
  rig.step();

  EXPECT_LOG(rig.log, {StateId::Init, Method::Execute}, {StateId::Init, Method::Next},
             {StateId::Init, Method::Execute}, {StateId::Init, Method::Next});
  TEST_ASSERT_EQUAL_UINT64(kStartUs, rig.machine.enteredAtUs());
}

static void test_transition_ends_old_then_inits_new_in_the_same_step() {
  Rig rig;
  rig.stepAt(kStartUs);
  rig.log.clear();

  rig.state(StateId::Init).wanted = StateId::PreFlight;
  rig.step();

  EXPECT_LOG(rig.log, {StateId::Init, Method::Execute}, {StateId::Init, Method::Next},
             {StateId::Init, Method::End}, {StateId::PreFlight, Method::Init});
  EXPECT_STATE(StateId::PreFlight, rig.machine.current());
  EXPECT_STATE(StateId::Init, rig.machine.previous());
  TEST_ASSERT_TRUE(rig.machine.transitioned());
  TEST_ASSERT_FALSE(rig.machine.refusedThisStep());
  TEST_ASSERT_EQUAL_UINT32(1, rig.machine.transitionCount());
  TEST_ASSERT_EQUAL_UINT32(0, rig.machine.refusedCount());
}

static void test_new_state_first_executes_on_the_next_step() {
  Rig rig;
  rig.stepAt(kStartUs);
  rig.state(StateId::Init).wanted = StateId::PreFlight;
  rig.step();
  rig.log.clear();

  rig.step();

  EXPECT_LOG(rig.log, {StateId::PreFlight, Method::Execute}, {StateId::PreFlight, Method::Next});
  EXPECT_STATE(StateId::PreFlight, rig.machine.current());
  // transitioned() is about the LAST step only.
  TEST_ASSERT_FALSE(rig.machine.transitioned());
  TEST_ASSERT_EQUAL_UINT32(1, rig.machine.transitionCount());
  EXPECT_STATE(StateId::Init, rig.machine.previous());
}

static void test_a_step_makes_at_most_one_transition() {
  Rig rig;
  // Every state wants to leave at once. The machine must still visit each
  // one, and each must get an execute() before it is asked where to go.
  for (size_t i = 0; i + 1 < kNumStates; ++i) rig.states[i].wanted = kAllStates[i + 1];

  rig.stepAt(kStartUs);
  EXPECT_LOG(rig.log, {StateId::Init, Method::Init}, {StateId::Init, Method::Execute},
             {StateId::Init, Method::Next}, {StateId::Init, Method::End},
             {StateId::PreFlight, Method::Init});
  EXPECT_STATE(StateId::PreFlight, rig.machine.current());

  for (size_t i = 1; i + 1 < kNumStates; ++i) {
    rig.log.clear();
    rig.step();
    EXPECT_LOG(rig.log, {kAllStates[i], Method::Execute}, {kAllStates[i], Method::Next},
               {kAllStates[i], Method::End}, {kAllStates[i + 1], Method::Init});
    EXPECT_STATE(kAllStates[i + 1], rig.machine.current());
    EXPECT_STATE(kAllStates[i], rig.machine.previous());
    TEST_ASSERT_TRUE(rig.machine.transitioned());
  }

  EXPECT_STATE(StateId::Land, rig.machine.current());
  TEST_ASSERT_EQUAL_UINT32(kNumStates - 1, rig.machine.transitionCount());
  TEST_ASSERT_EQUAL_UINT32(0, rig.machine.refusedCount());
}

static void test_states_are_given_the_callers_navigation_and_commands() {
  Rig rig;
  rig.state(StateId::Init).wanted = StateId::PreFlight;
  rig.stepAt(kStartUs);

  TEST_ASSERT_EQUAL_UINT32(5, rig.log.count);
  for (size_t i = 0; i < rig.log.count; ++i) {
    TEST_ASSERT_EQUAL_PTR(&rig.nav, rig.log.entries[i].nav);
    TEST_ASSERT_EQUAL_PTR(&rig.commands, rig.log.entries[i].commands);
    TEST_ASSERT_EQUAL_UINT64(kStartUs, rig.log.entries[i].timeUs);
  }
}

static void test_stand_down_re_enters_init_and_preflight_again() {
  Rig rig;
  rig.driveTo(StateId::PreFlight);

  rig.state(StateId::PreFlight).wanted = StateId::Init;
  rig.step();
  EXPECT_LOG(rig.log, {StateId::PreFlight, Method::Execute}, {StateId::PreFlight, Method::Next},
             {StateId::PreFlight, Method::End}, {StateId::Init, Method::Init});
  EXPECT_STATE(StateId::Init, rig.machine.current());
  EXPECT_STATE(StateId::PreFlight, rig.machine.previous());
  TEST_ASSERT_EQUAL_UINT64(rig.cycle.timeUs, rig.machine.enteredAtUs());

  // Going back to PreFlight calls its init() again: that is where the
  // estimator gets re-initialised after a scrub.
  rig.state(StateId::PreFlight).wanted = StateId::PreFlight;
  rig.state(StateId::Init).wanted = StateId::PreFlight;
  rig.log.clear();
  rig.step();
  EXPECT_LOG(rig.log, {StateId::Init, Method::Execute}, {StateId::Init, Method::Next},
             {StateId::Init, Method::End}, {StateId::PreFlight, Method::Init});
  TEST_ASSERT_EQUAL_UINT32(3, rig.machine.transitionCount());
}

// ---------------------------------------------------------------------------
// timeInStateUs
// ---------------------------------------------------------------------------

static void test_time_in_state_is_zero_on_the_first_step() {
  Rig rig;
  rig.cycle.timeInStateUs = 987654321;  // whatever the caller left there is overwritten
  rig.stepAt(kStartUs);

  TEST_ASSERT_EQUAL_UINT32(3, rig.log.count);
  TEST_ASSERT_EQUAL_UINT64(0, rig.log.entries[0].timeInStateUs);  // init
  TEST_ASSERT_EQUAL_UINT64(0, rig.log.entries[1].timeInStateUs);  // execute
  TEST_ASSERT_EQUAL_UINT64(0, rig.log.entries[2].timeInStateUs);  // next
  TEST_ASSERT_EQUAL_UINT64(0, rig.cycle.timeInStateUs);
}

static void test_time_in_state_is_sample_time_since_entry() {
  Rig rig;
  rig.stepAt(kStartUs);
  rig.log.clear();

  rig.cycle.timeInStateUs = 987654321;
  rig.stepAt(kStartUs + 7000);
  TEST_ASSERT_EQUAL_UINT64(7000, rig.log.entries[0].timeInStateUs);  // execute
  TEST_ASSERT_EQUAL_UINT64(7000, rig.log.entries[1].timeInStateUs);  // next
  TEST_ASSERT_EQUAL_UINT64(7000, rig.cycle.timeInStateUs);

  // An irregular sample period: it is sample time, not a cycle count.
  rig.stepAt(kStartUs + 7000 + 123456);
  TEST_ASSERT_EQUAL_UINT64(130456, rig.cycle.timeInStateUs);
}

static void test_time_in_state_restarts_on_a_transition() {
  Rig rig;
  rig.stepAt(kStartUs);
  rig.state(StateId::Init).wanted = StateId::PreFlight;
  rig.log.clear();

  const uint64_t transitionUs = kStartUs + 40000;
  rig.stepAt(transitionUs);

  // execute, next and end of the OLD state still see its own time in state.
  TEST_ASSERT_EQUAL_UINT32(4, rig.log.count);
  TEST_ASSERT_EQUAL_UINT64(40000, rig.log.entries[0].timeInStateUs);  // Init.execute
  TEST_ASSERT_EQUAL_UINT64(40000, rig.log.entries[1].timeInStateUs);  // Init.next
  TEST_ASSERT_EQUAL_UINT64(40000, rig.log.entries[2].timeInStateUs);  // Init.end
  // init of the NEW state sees 0, and so does the caller after the step.
  TEST_ASSERT_EQUAL_UINT64(0, rig.log.entries[3].timeInStateUs);      // PreFlight.init
  TEST_ASSERT_EQUAL_UINT64(0, rig.cycle.timeInStateUs);
  TEST_ASSERT_EQUAL_UINT64(transitionUs, rig.machine.enteredAtUs());

  rig.log.clear();
  rig.stepAt(transitionUs + 5000);
  TEST_ASSERT_EQUAL_UINT64(5000, rig.log.entries[0].timeInStateUs);  // PreFlight.execute
}

static void test_time_in_state_is_zero_if_time_goes_back_past_the_entry() {
  Rig rig;
  rig.stepAt(kStartUs);

  rig.stepAt(kStartUs - 1);
  TEST_ASSERT_EQUAL_UINT64(0, rig.cycle.timeInStateUs);
  rig.stepAt(0);
  TEST_ASSERT_EQUAL_UINT64(0, rig.cycle.timeInStateUs);
  // The entry time is not rewritten by a glitch.
  TEST_ASSERT_EQUAL_UINT64(kStartUs, rig.machine.enteredAtUs());
  EXPECT_STATE(StateId::Init, rig.machine.current());

  rig.stepAt(kStartUs + 10000);
  TEST_ASSERT_EQUAL_UINT64(10000, rig.cycle.timeInStateUs);
  // Backwards, but still after the entry: the plain difference.
  rig.stepAt(kStartUs + 4000);
  TEST_ASSERT_EQUAL_UINT64(4000, rig.cycle.timeInStateUs);
}

static void test_time_in_state_works_above_32_bits_of_microseconds() {
  Rig rig;
  const uint64_t entryUs = (1ull << 32) - 2500;  // 71.6 minutes on the pad
  rig.stepAt(entryUs);
  rig.stepAt(entryUs + 5000);  // crosses 2^32
  TEST_ASSERT_EQUAL_UINT64(5000, rig.cycle.timeInStateUs);

  const uint64_t longWaitUs = 3ull << 32;  // 3.6 hours in the state
  rig.stepAt(entryUs + longWaitUs);
  TEST_ASSERT_EQUAL_UINT64(longWaitUs, rig.cycle.timeInStateUs);
  TEST_ASSERT_EQUAL_UINT64(entryUs, rig.machine.enteredAtUs());
}

namespace {

// A state written the way the real ones will be: it leaves on a timeout it
// reads from cycle.timeInStateUs.
class TimeoutState final : public State {
 public:
  TimeoutState(StateId self, StateId to, uint64_t timeoutUs)
      : self_(self), to_(to), timeoutUs_(timeoutUs) {}
  StateId next(const Cycle& cycle) override {
    return cycle.timeInStateUs >= timeoutUs_ ? to_ : self_;
  }

 private:
  StateId self_;
  StateId to_;
  uint64_t timeoutUs_;
};

}  // namespace

static void test_a_state_timeout_fires_on_the_sample_that_reaches_it() {
  FakeEstimator estimator;
  Navigation nav(estimator);
  CommandList commands;
  CallLog log;
  const uint64_t timeoutUs = 4 * kStepUs;
  TimeoutState boost(StateId::Boost, StateId::Control, timeoutUs);
  RecordingState init(StateId::Init, log);
  RecordingState others[4] = {{StateId::Control, log}, {StateId::Coast, log},
                              {StateId::Chute, log}, {StateId::Land, log}};
  RecordingState preFlight(StateId::PreFlight, log);
  State* table[kNumStates] = {&init, &preFlight, &boost, &others[0],
                              &others[1], &others[2], &others[3]};
  StateMachine machine(table);
  Cycle cycle{kStartUs, 0.005f, 0, nav, commands};

  init.wanted = StateId::Boost;
  machine.step(cycle);  // Init -> Boost at kStartUs
  EXPECT_STATE(StateId::Boost, machine.current());

  // Boost was entered at kStartUs, so it must leave on the sample at
  // kStartUs + timeoutUs and not one sample either side.
  for (uint64_t t = kStartUs + kStepUs; t < kStartUs + timeoutUs; t += kStepUs) {
    cycle.timeUs = t;
    machine.step(cycle);
    EXPECT_STATE(StateId::Boost, machine.current());
  }
  cycle.timeUs = kStartUs + timeoutUs;
  machine.step(cycle);
  EXPECT_STATE(StateId::Control, machine.current());
  TEST_ASSERT_EQUAL_UINT64(kStartUs + timeoutUs, machine.enteredAtUs());
}

// ---------------------------------------------------------------------------
// Refused transitions
// ---------------------------------------------------------------------------

static void test_refused_transition_stays_put_and_is_counted() {
  Rig rig;
  rig.stepAt(kStartUs);
  rig.log.clear();

  rig.state(StateId::Init).wanted = StateId::Control;  // not in the list
  rig.stepAt(kStartUs + 20000);

  // No end(), no init() of anything.
  EXPECT_LOG(rig.log, {StateId::Init, Method::Execute}, {StateId::Init, Method::Next});
  EXPECT_STATE(StateId::Init, rig.machine.current());
  TEST_ASSERT_TRUE(rig.machine.refusedThisStep());
  TEST_ASSERT_EQUAL_UINT32(1, rig.machine.refusedCount());
  EXPECT_STATE(StateId::Init, rig.machine.lastRefusedFrom());
  EXPECT_STATE(StateId::Control, rig.machine.lastRefusedTo());
  TEST_ASSERT_FALSE(rig.machine.transitioned());
  TEST_ASSERT_EQUAL_UINT32(0, rig.machine.transitionCount());
  EXPECT_STATE(StateId::Init, rig.machine.previous());
  // The state's clock keeps running: a refusal is not a re-entry.
  TEST_ASSERT_EQUAL_UINT64(kStartUs, rig.machine.enteredAtUs());
  TEST_ASSERT_EQUAL_UINT64(20000, rig.cycle.timeInStateUs);
}

static void test_refusal_is_counted_every_step_it_is_asked() {
  Rig rig;
  rig.stepAt(kStartUs);
  rig.state(StateId::Init).wanted = StateId::Land;

  for (uint32_t i = 1; i <= 5; ++i) {
    rig.step();
    TEST_ASSERT_TRUE(rig.machine.refusedThisStep());
    TEST_ASSERT_EQUAL_UINT32(i, rig.machine.refusedCount());
    EXPECT_STATE(StateId::Init, rig.machine.current());
  }
}

static void test_refused_flag_clears_but_the_count_and_pair_are_kept() {
  Rig rig;
  rig.driveTo(StateId::Boost);
  rig.state(StateId::Boost).wanted = StateId::Chute;
  rig.step();
  TEST_ASSERT_TRUE(rig.machine.refusedThisStep());

  rig.state(StateId::Boost).wanted = StateId::Boost;
  rig.step();
  TEST_ASSERT_FALSE(rig.machine.refusedThisStep());
  TEST_ASSERT_EQUAL_UINT32(1, rig.machine.refusedCount());
  EXPECT_STATE(StateId::Boost, rig.machine.lastRefusedFrom());
  EXPECT_STATE(StateId::Chute, rig.machine.lastRefusedTo());

  // A refusal does not block a later, allowed transition.
  rig.state(StateId::Boost).wanted = StateId::Control;
  rig.step();
  EXPECT_STATE(StateId::Control, rig.machine.current());
  TEST_ASSERT_TRUE(rig.machine.transitioned());
  TEST_ASSERT_FALSE(rig.machine.refusedThisStep());
  TEST_ASSERT_EQUAL_UINT32(1, rig.machine.refusedCount());
}

static void test_transitioned_flag_clears_on_a_refused_step() {
  Rig rig;
  rig.stepAt(kStartUs);
  rig.state(StateId::Init).wanted = StateId::PreFlight;
  rig.step();
  TEST_ASSERT_TRUE(rig.machine.transitioned());

  rig.state(StateId::PreFlight).wanted = StateId::Coast;
  rig.step();
  TEST_ASSERT_FALSE(rig.machine.transitioned());
  TEST_ASSERT_TRUE(rig.machine.refusedThisStep());
  EXPECT_STATE(StateId::PreFlight, rig.machine.current());
}

static void test_an_id_outside_the_enum_is_refused() {
  const uint8_t badIds[] = {static_cast<uint8_t>(kNumStates), 8, 100, 255};
  uint32_t refusals = 0;
  Rig rig;
  rig.driveTo(StateId::PreFlight);

  for (uint8_t raw : badIds) {
    const StateId bad = static_cast<StateId>(raw);
    rig.log.clear();
    rig.state(StateId::PreFlight).wanted = bad;
    rig.step();

    EXPECT_LOG(rig.log, {StateId::PreFlight, Method::Execute}, {StateId::PreFlight, Method::Next});
    EXPECT_STATE(StateId::PreFlight, rig.machine.current());
    TEST_ASSERT_TRUE(rig.machine.refusedThisStep());
    TEST_ASSERT_EQUAL_UINT32(++refusals, rig.machine.refusedCount());
    EXPECT_STATE(StateId::PreFlight, rig.machine.lastRefusedFrom());
    TEST_ASSERT_EQUAL_UINT8(raw, static_cast<uint8_t>(rig.machine.lastRefusedTo()));
  }
}

static void test_self_transition_is_stay_not_a_refusal() {
  for (StateId id : kAllStates) {
    Rig rig;
    rig.driveTo(id);
    const uint64_t enteredUs = rig.machine.enteredAtUs();
    const uint32_t transitions = rig.machine.transitionCount();

    rig.state(id).wanted = id;
    rig.step();
    rig.step();

    EXPECT_LOG(rig.log, {id, Method::Execute}, {id, Method::Next}, {id, Method::Execute},
               {id, Method::Next});
    EXPECT_STATE(id, rig.machine.current());
    TEST_ASSERT_FALSE(rig.machine.refusedThisStep());
    TEST_ASSERT_EQUAL_UINT32(0, rig.machine.refusedCount());
    TEST_ASSERT_FALSE(rig.machine.transitioned());
    TEST_ASSERT_EQUAL_UINT32(transitions, rig.machine.transitionCount());
    TEST_ASSERT_EQUAL_UINT64(enteredUs, rig.machine.enteredAtUs());
  }
}

static void test_land_never_leaves() {
  Rig rig;
  rig.driveTo(StateId::Land);
  const uint64_t enteredUs = rig.machine.enteredAtUs();

  uint32_t refusals = 0;
  for (StateId to : kAllStates) {
    if (to == StateId::Land) continue;
    rig.log.clear();
    rig.state(StateId::Land).wanted = to;
    rig.step();

    EXPECT_LOG(rig.log, {StateId::Land, Method::Execute}, {StateId::Land, Method::Next});
    EXPECT_STATE(StateId::Land, rig.machine.current());
    TEST_ASSERT_EQUAL_UINT32(++refusals, rig.machine.refusedCount());
    EXPECT_STATE(StateId::Land, rig.machine.lastRefusedFrom());
    EXPECT_STATE(to, rig.machine.lastRefusedTo());
  }
  TEST_ASSERT_EQUAL_UINT64(enteredUs, rig.machine.enteredAtUs());
  TEST_ASSERT_EQUAL_UINT32(kNumStates - 1, rig.machine.transitionCount());
}

// ---------------------------------------------------------------------------
// The allowed list
// ---------------------------------------------------------------------------

static void test_allowed_list_is_exactly_the_one_in_the_doc() {
  size_t docCount = 0;
  for (StateId from : kAllStates) {
    for (StateId to : kAllStates) {
      const bool expected = kDocAllowed[indexOf(from)][indexOf(to)];
      if (expected) ++docCount;
      char message[64];
      snprintf(message, sizeof(message), "%s -> %s", stateName(from), stateName(to));
      TEST_ASSERT_EQUAL_MESSAGE(expected, isAllowedTransition(from, to), message);
    }
  }
  // Same number of entries, so the list has no duplicates and nothing extra.
  TEST_ASSERT_EQUAL_UINT32(docCount, kNumAllowedTransitions);
  for (size_t i = 0; i < kNumAllowedTransitions; ++i) {
    for (size_t j = i + 1; j < kNumAllowedTransitions; ++j) {
      TEST_ASSERT_FALSE_MESSAGE(kAllowedTransitions[i].from == kAllowedTransitions[j].from &&
                                    kAllowedTransitions[i].to == kAllowedTransitions[j].to,
                                "duplicate entry in kAllowedTransitions");
    }
  }
}

static void test_an_id_outside_the_enum_is_never_allowed() {
  const StateId bad = static_cast<StateId>(kNumStates);
  for (StateId id : kAllStates) {
    TEST_ASSERT_FALSE(isAllowedTransition(id, bad));
    TEST_ASSERT_FALSE(isAllowedTransition(bad, id));
  }
}

// The machine itself, not just the list: every one of the 49 pairs, driven
// through a real StateMachine.
static void test_machine_follows_the_doc_for_all_49_pairs() {
  for (StateId from : kAllStates) {
    for (StateId to : kAllStates) {
      char message[64];
      snprintf(message, sizeof(message), "%s -> %s", stateName(from), stateName(to));

      Rig rig;
      rig.driveTo(from);
      const uint32_t transitionsBefore = rig.machine.transitionCount();
      rig.state(from).wanted = to;
      rig.step();

      if (from == to) {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, rig.log.count, message);
        TEST_ASSERT_TRUE_MESSAGE(rig.machine.current() == from, message);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, rig.machine.refusedCount(), message);
        TEST_ASSERT_FALSE_MESSAGE(rig.machine.transitioned(), message);
      } else if (kDocAllowed[indexOf(from)][indexOf(to)]) {
        TEST_ASSERT_TRUE_MESSAGE(rig.machine.current() == to, message);
        TEST_ASSERT_TRUE_MESSAGE(rig.machine.previous() == from, message);
        TEST_ASSERT_TRUE_MESSAGE(rig.machine.transitioned(), message);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(transitionsBefore + 1, rig.machine.transitionCount(),
                                         message);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, rig.machine.refusedCount(), message);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(4, rig.log.count, message);
        TEST_ASSERT_TRUE_MESSAGE(rig.log.entries[2].state == from, message);
        TEST_ASSERT_TRUE_MESSAGE(rig.log.entries[2].method == Method::End, message);
        TEST_ASSERT_TRUE_MESSAGE(rig.log.entries[3].state == to, message);
        TEST_ASSERT_TRUE_MESSAGE(rig.log.entries[3].method == Method::Init, message);
      } else {
        TEST_ASSERT_TRUE_MESSAGE(rig.machine.current() == from, message);
        TEST_ASSERT_FALSE_MESSAGE(rig.machine.transitioned(), message);
        TEST_ASSERT_TRUE_MESSAGE(rig.machine.refusedThisStep(), message);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(1, rig.machine.refusedCount(), message);
        TEST_ASSERT_TRUE_MESSAGE(rig.machine.lastRefusedFrom() == from, message);
        TEST_ASSERT_TRUE_MESSAGE(rig.machine.lastRefusedTo() == to, message);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(transitionsBefore, rig.machine.transitionCount(),
                                         message);
        // execute and next only: neither end() nor init() ran.
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, rig.log.count, message);
      }
    }
  }
}

static void test_launch_from_init_goes_straight_to_boost() {
  Rig rig;
  rig.state(StateId::Init).wanted = StateId::Boost;
  rig.stepAt(kStartUs);

  EXPECT_LOG(rig.log, {StateId::Init, Method::Init}, {StateId::Init, Method::Execute},
             {StateId::Init, Method::Next}, {StateId::Init, Method::End},
             {StateId::Boost, Method::Init});
  EXPECT_STATE(StateId::Boost, rig.machine.current());
  // PreFlight was never touched.
  EXPECT_STATE(StateId::Init, rig.machine.previous());
}

static void test_state_helpers_match_the_doc() {
  TEST_ASSERT_TRUE(isGroundState(StateId::Init));
  TEST_ASSERT_TRUE(isGroundState(StateId::PreFlight));
  for (StateId id : {StateId::Boost, StateId::Control, StateId::Coast, StateId::Chute,
                     StateId::Land}) {
    TEST_ASSERT_FALSE(isGroundState(id));
    TEST_ASSERT_TRUE(isFlightState(id));
  }
  TEST_ASSERT_FALSE(isFlightState(StateId::Init));
  TEST_ASSERT_FALSE(isFlightState(StateId::PreFlight));
  for (size_t i = 0; i < kNumStates; ++i) TEST_ASSERT_EQUAL_UINT32(i, indexOf(kAllStates[i]));
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------

static void test_persistence_holds_exactly_when_the_hold_time_is_reached() {
  const uint64_t holdUs = FlightConfig::kLaunchHoldUs;
  TEST_ASSERT_TRUE_MESSAGE(holdUs >= 2, "this test needs a hold of at least 2 us");
  const uint64_t t0 = 1000000;
  Persistence p(holdUs);

  TEST_ASSERT_FALSE(p.held());
  TEST_ASSERT_FALSE(p.update(true, t0));  // the first true sample starts the clock
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs / 2));
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs - 1));
  TEST_ASSERT_FALSE(p.held());
  TEST_ASSERT_TRUE(p.update(true, t0 + holdUs));
  TEST_ASSERT_TRUE(p.held());
}

static void test_persistence_stays_true_while_the_condition_does() {
  const uint64_t holdUs = 50000;
  Persistence p(holdUs);
  p.update(true, 0);  // a run may start at time 0
  TEST_ASSERT_TRUE(p.update(true, holdUs));
  TEST_ASSERT_TRUE(p.update(true, holdUs + 1));
  TEST_ASSERT_TRUE(p.update(true, 10 * holdUs));
  TEST_ASSERT_TRUE(p.held());
}

static void test_persistence_false_sample_starts_over() {
  const uint64_t holdUs = 50000;
  const uint64_t t0 = 1000000;
  Persistence p(holdUs);

  p.update(true, t0);
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs - 1));
  TEST_ASSERT_FALSE(p.update(false, t0 + holdUs));  // one false sample, right at the hold time
  TEST_ASSERT_FALSE(p.held());

  // The clock starts again from the NEXT true sample, not from t0.
  const uint64_t t1 = t0 + holdUs + 5000;
  TEST_ASSERT_FALSE(p.update(true, t1));
  TEST_ASSERT_FALSE(p.update(true, t1 + holdUs - 1));
  TEST_ASSERT_TRUE(p.update(true, t1 + holdUs));
}

static void test_persistence_false_after_held_drops_it_at_once() {
  const uint64_t holdUs = 50000;
  Persistence p(holdUs);
  p.update(true, 100);
  TEST_ASSERT_TRUE(p.update(true, 100 + holdUs));

  TEST_ASSERT_FALSE(p.update(false, 200 + holdUs));
  TEST_ASSERT_FALSE(p.held());
  TEST_ASSERT_FALSE(p.update(true, 300 + holdUs));
  TEST_ASSERT_TRUE(p.update(true, 300 + 2 * holdUs));
}

static void test_persistence_zero_hold_is_true_on_the_first_true_sample() {
  Persistence p(0);
  TEST_ASSERT_FALSE(p.held());
  TEST_ASSERT_FALSE(p.update(false, 10));
  TEST_ASSERT_TRUE(p.update(true, 20));
  TEST_ASSERT_TRUE(p.held());
  TEST_ASSERT_FALSE(p.update(false, 30));
  TEST_ASSERT_TRUE(p.update(true, 40));
  // Even with time going backwards: the first true sample of a run is enough.
  TEST_ASSERT_TRUE(p.update(true, 5));
}

static void test_persistence_time_going_backwards_starts_over() {
  const uint64_t holdUs = 50000;
  const uint64_t t0 = 1000000;
  Persistence p(holdUs);

  p.update(true, t0);
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs - 1));
  // Time steps back. The run restarts from this sample.
  const uint64_t t1 = t0 + 10000;
  TEST_ASSERT_FALSE(p.update(true, t1));
  // The old run would have been satisfied here. The new one is not.
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs));
  TEST_ASSERT_FALSE(p.update(true, t1 + holdUs - 1));
  TEST_ASSERT_TRUE(p.update(true, t1 + holdUs));
}

static void test_persistence_time_going_backwards_drops_a_held_condition() {
  const uint64_t holdUs = 50000;
  const uint64_t t0 = 1000000;
  Persistence p(holdUs);
  p.update(true, t0);
  TEST_ASSERT_TRUE(p.update(true, t0 + holdUs));

  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs - 1));
  TEST_ASSERT_FALSE(p.held());
  TEST_ASSERT_TRUE(p.update(true, t0 + 2 * holdUs - 1));
}

static void test_persistence_time_going_backwards_before_the_run_started() {
  // Back to before the first true sample: the difference must not wrap into
  // a huge unsigned number that reads as "held".
  const uint64_t holdUs = 50000;
  Persistence p(holdUs);
  p.update(true, 1000000);
  TEST_ASSERT_FALSE(p.update(true, 10));
  TEST_ASSERT_FALSE(p.update(true, 10 + holdUs - 1));
  TEST_ASSERT_TRUE(p.update(true, 10 + holdUs));
}

static void test_persistence_repeated_timestamp_neither_restarts_nor_advances() {
  const uint64_t holdUs = 50000;
  const uint64_t t0 = 1000000;
  Persistence p(holdUs);
  p.update(true, t0);
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs - 1));
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs - 1));  // same sample time again
  TEST_ASSERT_TRUE(p.update(true, t0 + holdUs));       // the run was kept
}

static void test_persistence_reset_forgets_everything() {
  const uint64_t holdUs = 50000;
  const uint64_t t0 = 1000000;
  Persistence p(holdUs);
  p.update(true, t0);
  TEST_ASSERT_TRUE(p.update(true, t0 + holdUs));

  p.reset();
  TEST_ASSERT_FALSE(p.held());
  // A full hold is needed again, counted from the first sample after reset.
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs + 1));
  TEST_ASSERT_FALSE(p.update(true, t0 + 2 * holdUs));
  TEST_ASSERT_TRUE(p.update(true, t0 + 2 * holdUs + 1));
}

static void test_persistence_reset_mid_run_restarts_the_clock() {
  const uint64_t holdUs = 50000;
  const uint64_t t0 = 1000000;
  Persistence p(holdUs);
  p.update(true, t0);
  p.update(true, t0 + holdUs - 1);

  p.reset();
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs));
  TEST_ASSERT_TRUE(p.update(true, t0 + 2 * holdUs));
}

static void test_persistence_works_above_32_bits_of_microseconds() {
  const uint64_t holdUs = 50000;
  // The run starts just below 2^32 us and ends just above: a 32-bit time
  // would wrap in the middle of it.
  const uint64_t t0 = (1ull << 32) - holdUs / 2;
  Persistence p(holdUs);
  TEST_ASSERT_FALSE(p.update(true, t0));
  TEST_ASSERT_FALSE(p.update(true, (1ull << 32) + 1));
  TEST_ASSERT_FALSE(p.update(true, t0 + holdUs - 1));
  TEST_ASSERT_TRUE(p.update(true, t0 + holdUs));

  // A hold longer than 2^32 us, at times far above it.
  const uint64_t longHoldUs = 5ull << 32;
  const uint64_t t1 = 9ull << 40;
  Persistence q(longHoldUs);
  TEST_ASSERT_FALSE(q.update(true, t1));
  TEST_ASSERT_FALSE(q.update(true, t1 + (1ull << 32)));  // would read as 0 elapsed in 32 bits
  TEST_ASSERT_FALSE(q.update(true, t1 + longHoldUs - 1));
  TEST_ASSERT_TRUE(q.update(true, t1 + longHoldUs));
}

static void test_persistence_held_reports_the_last_update() {
  Persistence p(1000);
  TEST_ASSERT_EQUAL(p.update(true, 0), p.held());
  TEST_ASSERT_EQUAL(p.update(true, 1000), p.held());
  TEST_ASSERT_TRUE(p.held());
  TEST_ASSERT_EQUAL(p.update(false, 2000), p.held());
  TEST_ASSERT_FALSE(p.held());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_constructing_calls_no_state_and_starts_in_init);
  RUN_TEST(test_first_step_runs_init_then_execute_then_next);
  RUN_TEST(test_init_is_called_only_once_while_staying);
  RUN_TEST(test_transition_ends_old_then_inits_new_in_the_same_step);
  RUN_TEST(test_new_state_first_executes_on_the_next_step);
  RUN_TEST(test_a_step_makes_at_most_one_transition);
  RUN_TEST(test_states_are_given_the_callers_navigation_and_commands);
  RUN_TEST(test_stand_down_re_enters_init_and_preflight_again);
  RUN_TEST(test_time_in_state_is_zero_on_the_first_step);
  RUN_TEST(test_time_in_state_is_sample_time_since_entry);
  RUN_TEST(test_time_in_state_restarts_on_a_transition);
  RUN_TEST(test_time_in_state_is_zero_if_time_goes_back_past_the_entry);
  RUN_TEST(test_time_in_state_works_above_32_bits_of_microseconds);
  RUN_TEST(test_a_state_timeout_fires_on_the_sample_that_reaches_it);
  RUN_TEST(test_refused_transition_stays_put_and_is_counted);
  RUN_TEST(test_refusal_is_counted_every_step_it_is_asked);
  RUN_TEST(test_refused_flag_clears_but_the_count_and_pair_are_kept);
  RUN_TEST(test_transitioned_flag_clears_on_a_refused_step);
  RUN_TEST(test_an_id_outside_the_enum_is_refused);
  RUN_TEST(test_self_transition_is_stay_not_a_refusal);
  RUN_TEST(test_land_never_leaves);
  RUN_TEST(test_allowed_list_is_exactly_the_one_in_the_doc);
  RUN_TEST(test_an_id_outside_the_enum_is_never_allowed);
  RUN_TEST(test_machine_follows_the_doc_for_all_49_pairs);
  RUN_TEST(test_launch_from_init_goes_straight_to_boost);
  RUN_TEST(test_state_helpers_match_the_doc);
  RUN_TEST(test_persistence_holds_exactly_when_the_hold_time_is_reached);
  RUN_TEST(test_persistence_stays_true_while_the_condition_does);
  RUN_TEST(test_persistence_false_sample_starts_over);
  RUN_TEST(test_persistence_false_after_held_drops_it_at_once);
  RUN_TEST(test_persistence_zero_hold_is_true_on_the_first_true_sample);
  RUN_TEST(test_persistence_time_going_backwards_starts_over);
  RUN_TEST(test_persistence_time_going_backwards_drops_a_held_condition);
  RUN_TEST(test_persistence_time_going_backwards_before_the_run_started);
  RUN_TEST(test_persistence_repeated_timestamp_neither_restarts_nor_advances);
  RUN_TEST(test_persistence_reset_forgets_everything);
  RUN_TEST(test_persistence_reset_mid_run_restarts_the_clock);
  RUN_TEST(test_persistence_works_above_32_bits_of_microseconds);
  RUN_TEST(test_persistence_held_reports_the_last_update);
  return UNITY_END();
}
