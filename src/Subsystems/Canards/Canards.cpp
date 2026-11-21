#include "Subsystems/Canards/Canards.h"

#include <cmath>

#include "FlightConfig.h"
#include "Subsystems/Navigation/Navigation.h"

namespace {

float clampAbs(float value, float limit) {
  return value > limit ? limit : (value < -limit ? -limit : value);
}

// The permission table from Canards.h.
bool permitted(CanardRequest request, StateId state) {
  switch (request) {
    case CanardRequest::Neutral: return true;
    case CanardRequest::TestAngles: return isGroundState(state);
    case CanardRequest::AttitudeHold: return state == StateId::Control;
    case CanardRequest::None:
    case CanardRequest::Conflict: return false;
  }
  return false;
}

bool servoPowerAllowed(StateId state) {
  // No power in descent: the canards have nothing to do, and an unpowered
  // servo cannot be driven by a fault.
  return state != StateId::Chute && state != StateId::Land;
}

void zero(float values[kNumCanards]) {
  for (int i = 0; i < kNumCanards; ++i) values[i] = 0.0f;
}

}  // namespace

Canards::Canards(AttitudeController& attitudeController, ServoControl& servoControl)
    : attitudeController_(attitudeController), servoControl_(servoControl) {}

void Canards::handleCommand(const Command& command, StateId currentState) {
  switch (command.type) {
    case CommandType::CanardsEnable:
      // Enabling is a ground decision. Once in flight it is refused, so a
      // stray packet cannot power the servos mid-flight.
      if (isGroundState(currentState)) {
        // Judged again in periodic() against the state after this cycle's
        // transition, so an enable on the launch cycle is refused too.
        if (!enabled_) enabledThisCycle_ = true;
        enabled_ = true;
      } else {
        ++status_.rejectedCommandCount;
      }
      break;
    case CommandType::CanardsDisable:
      enabled_ = false;  // always accepted
      enabledThisCycle_ = false;
      break;
    default:
      break;  // the states handle the other commands
  }
}

void Canards::beginCycle() { request_ = CanardRequest::None; }

// A second request of the same kind replaces the first; a second request of
// another kind makes the cycle a conflict, which ends at neutral.
static void record(CanardRequest& pending, CanardRequest incoming) {
  if (pending == CanardRequest::None || pending == incoming) {
    pending = incoming;
  } else {
    pending = CanardRequest::Conflict;
  }
}

void Canards::requestNeutral() { record(request_, CanardRequest::Neutral); }

void Canards::requestAttitudeHold(const Quat& reference) {
  record(request_, CanardRequest::AttitudeHold);
  reference_ = reference;
}

void Canards::requestTestAngles(const float rad[kNumCanards]) {
  record(request_, CanardRequest::TestAngles);
  for (int i = 0; i < kNumCanards; ++i) testRad_[i] = rad[i];
}

void Canards::periodic(const Cycle& cycle, StateId currentState, bool safetyActive,
                       const ActuatorFeedback& feedback, ActuatorCommand& out) {
  status_.request = request_;
  status_.gates = 0;

  // 1. Lock-out. Judged on the first flight-state cycle after the ground:
  // only a flight that starts from PreFlight may control.
  if (isGroundState(currentState)) {
    lastGroundState_ = currentState;
    inFlight_ = false;
  } else if (!inFlight_) {
    inFlight_ = true;
    if (lastGroundState_ != StateId::PreFlight) status_.lockedOut = true;
  }

  // An enable that arrived this cycle counts only if the rocket is still on
  // the ground after this cycle's transition.
  if (enabledThisCycle_ && !isGroundState(currentState)) {
    enabled_ = false;
    ++status_.rejectedCommandCount;
  }
  enabledThisCycle_ = false;

  // Measured every cycle, also unpowered, so the log always has the angles.
  servoControl_.measure(feedback, status_.angles);

  // 2. Power: on from boot, holding 0 deg, in every state but Chute and
  // Land. Enabling only decides whether attitude hold may move the canards.
  const bool powered = servoPowerAllowed(currentState);
  status_.enabled = enabled_;
  status_.powered = powered;

  // 3. Resolve the request. Denials are counted whether or not the servos
  // are powered, so a forbidden request always shows up in the log. On the
  // cycle the state changes, a request the old state was allowed to make
  // still ends at neutral but is not counted: it is the old state's last
  // request, not a fault (e.g. Control's last attitude hold on Control->Coast).
  const bool stateChanged = currentState != lastState_;
  const StateId previousState = lastState_;
  lastState_ = currentState;
  CanardRequest effective = request_;
  if (effective == CanardRequest::None) {
    effective = CanardRequest::Neutral;
  } else if (effective == CanardRequest::Conflict) {
    ++status_.deniedCount;
    status_.lastDenied = CanardRequest::Conflict;
    effective = CanardRequest::Neutral;
  } else if (!permitted(effective, currentState)) {
    if (!(stateChanged && permitted(effective, previousState))) {
      ++status_.deniedCount;
      status_.lastDenied = effective;
    }
    effective = CanardRequest::Neutral;
  }

  if (!powered) {
    if (wasPowered_) servoControl_.reset();
    wasPowered_ = false;
    cutoff_.reset();
    status_.mode = CanardMode::Off;
    zero(status_.commandRad);
    out.servoPower = false;
    servoControl_.centerPulses(out.pulseUs);
    return;
  }
  if (!wasPowered_) servoControl_.reset();  // fresh servo loop and model on power-up
  wasPowered_ = true;

  float command[kNumCanards] = {};
  CanardMode mode = CanardMode::Neutral;
  bool cutoffEvaluated = false;

  if (effective == CanardRequest::TestAngles) {
    // 4-5. Safety first, then the linkage failsafe limit.
    bool finite = true;
    for (int i = 0; i < kNumCanards; ++i) finite = finite && std::isfinite(testRad_[i]);
    if (!safetyActive && finite) {
      for (int i = 0; i < kNumCanards; ++i) {
        command[i] = clampAbs(testRad_[i], FlightConfig::kGroundTestLimitRad);
      }
      mode = CanardMode::Testing;
    }
  } else if (effective == CanardRequest::AttitudeHold) {
    // 6-7. Every gate is evaluated, so the status shows all that are closed.
    const Navigation& nav = cycle.nav;
    uint16_t gates = 0;
    if (!enabled_) gates |= CanardGate::kNotEnabled;
    if (status_.lockedOut) gates |= CanardGate::kLockedOut;
    if (safetyActive) gates |= CanardGate::kSafetyFault;
    if (!nav.attitudeValid() || !nav.speedValid()) gates |= CanardGate::kNavInvalid;
    if (nav.speedValid() && nav.speed() < FlightConfig::kMinControlSpeed) gates |= CanardGate::kSpeedLow;
    if (!isUnit(reference_)) gates |= CanardGate::kNonFinite;

    // The cut-off only looks at a valid attitude, and only here, where
    // attitude hold is allowed: a rocket on its side on the pad never trips it.
    // Debounced, so one bad estimator sample cannot end control for the flight.
    bool outside = false;
    if (nav.attitudeValid()) {
      const Vec3& rate = nav.bodyRate();
      outside = nav.tilt() > FlightConfig::kMaxTiltRad || std::fabs(rate.y) > FlightConfig::kMaxPitchYawRate ||
                std::fabs(rate.z) > FlightConfig::kMaxPitchYawRate;
    }
    if (cutoff_.update(outside, cycle.timeUs)) status_.cutoffLatched = true;
    cutoffEvaluated = true;
    if (status_.cutoffLatched) gates |= CanardGate::kCutoff;

    if (gates == 0) {
      AttitudeControlInput in;
      in.attitudeError = attitudeError(reference_, nav.attitude());
      in.bodyRate = nav.bodyRate();
      for (int i = 0; i < kNumCanards; ++i) in.canardAngle[i] = status_.angles.rad[i];
      in.speed = nav.speed();
      float raw[kNumCanards] = {};
      if (!attitudeController_.compute(in, raw)) {
        gates |= CanardGate::kControllerFailed;
      } else {
        for (int i = 0; i < kNumCanards; ++i) {
          if (!std::isfinite(raw[i])) gates |= CanardGate::kNonFinite;
        }
      }
      if (gates == 0) {
        for (int i = 0; i < kNumCanards; ++i) command[i] = clampAbs(raw[i], FlightConfig::kCanardLimitRad);
        mode = CanardMode::Controlling;
      }
    }
    status_.gates = gates;
  }

  // A cut-off run only counts over consecutive attitude-hold cycles.
  if (!cutoffEvaluated) cutoff_.reset();

  status_.mode = mode;
  if (mode == CanardMode::Neutral) zero(command);
  for (int i = 0; i < kNumCanards; ++i) status_.commandRad[i] = command[i];
  servoControl_.command(command, cycle.dt, feedback, out.pulseUs);
  out.servoPower = true;
}
