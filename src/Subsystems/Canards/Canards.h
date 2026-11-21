#pragma once

// The Canards subsystem: the ONLY way a state touches the canards
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.4).
//
// States make requests. A request lasts ONE cycle:
//     canards.requestNeutral();
//     canards.requestAttitudeHold(referenceQuat);
//     canards.requestTestAngles(rad);
// If nothing asks in a cycle the result is neutral, so a forgotten branch or
// a state change can never leave the canards on a stale command.
//
// periodic() then decides what actually happens, every cycle, whatever the
// state. In order:
//
//  1. LOCK-OUT. If the rocket enters a flight state from anything other than
//     PreFlight (a launch from Init), attitude hold is refused for the rest of
//     the flight.
//
//  2. POWER. Servo power is on from boot, holding 0 deg, in every state but
//     Chute and Land (user decision 2026-11-21). Not powered means mode Off,
//     centre pulses, and the servo loop reset. ENABLE does not switch power:
//     it only allows attitude hold to move the canards. Canards start
//     DISABLED; CanardsDisable returns them to a powered 0 deg hold.
//
//  3. REQUEST. No request -> neutral. Two requests of different kinds in one
//     cycle -> neutral, counted as denied (PROPOSAL). A request the permission
//     table forbids in the current state -> neutral, counted as denied:
//
//         request        Init PreFlight Boost Control Coast Chute Land
//         neutral         ok     ok      ok     ok     ok    ok    ok
//         test angles     ok     ok      -      -      -     -     -
//         attitude hold   -      -       -      ok     -     -     -
//
//     The state used is the one in force AFTER this cycle's transition, so a
//     test requested on the cycle launch is detected is refused. On the cycle
//     the state changes, a request the old state was allowed to make is not
//     counted as denied (it is that state's last request, not a fault).
//
//  4. SAFETY. While any system safety fault is active, the result is neutral.
//
//  5. TEST ANGLES are clamped to FlightConfig::kGroundTestLimitRad. A
//     non-finite angle makes the whole request neutral.
//
//  6. ATTITUDE HOLD gives neutral if any gate is closed (CanardGate below).
//     Otherwise the attitude controller runs and its output is clamped to
//     FlightConfig::kCanardLimitRad.
//
//  7. ENVELOPE CUT-OFF. While attitude hold is permitted and navigation is
//     valid, tilt > kMaxTiltRad or |pitch or yaw rate| > kMaxPitchYawRate,
//     held for kCutoffHoldUs over consecutive attitude-hold cycles, latches
//     neutral for the rest of the flight. It is only evaluated then, so a
//     rocket lying on its side on the pad cannot trip it.
//
// The subsystem never reads a clock: time enters through Cycle only.

#include <cstdint>

#include "Actuators.h"
#include "AttitudeControl/AttitudeController.h"
#include "FlightConfig.h"
#include "Frames.h"
#include "ServoControl/ServoControl.h"
#include "StateMachine/Persistence.h"
#include "StateMachine/State.h"
#include "StateMachine/StateId.h"
#include "Subsystems/Comms/Command.h"

enum class CanardMode : uint8_t {
  Off,          // servo power off
  Neutral,      // powered, holding zero deflection
  Controlling,  // attitude hold is driving the canards
  Testing,      // a ground test is driving the canards
};

enum class CanardRequest : uint8_t {
  None,          // nothing asked this cycle
  Neutral,
  AttitudeHold,
  TestAngles,
  Conflict,      // two different kinds asked in one cycle
};

// Why an attitude-hold request produced neutral this cycle. Bit mask.
namespace CanardGate {
constexpr uint16_t kNotEnabled = 1u << 0;        // canards not enabled for control
constexpr uint16_t kLockedOut = 1u << 1;         // launch skipped PreFlight
constexpr uint16_t kSafetyFault = 1u << 2;       // a system safety fault is active
constexpr uint16_t kNavInvalid = 1u << 3;        // attitude or speed invalid or stale
constexpr uint16_t kSpeedLow = 1u << 4;          // speed < kMinControlSpeed
constexpr uint16_t kCutoff = 1u << 5;            // envelope cut-off latched
constexpr uint16_t kControllerFailed = 1u << 6;  // the attitude controller reported failure
constexpr uint16_t kNonFinite = 1u << 7;         // reference not finite or not unit length, or output not finite
}  // namespace CanardGate

struct CanardsStatus {
  CanardMode mode = CanardMode::Off;
  bool enabled = false;        // CanardsEnable accepted and not since disabled: attitude hold may act
  bool powered = false;        // servo power commanded on this cycle (on except in Chute and Land)
  bool lockedOut = false;      // attitude hold refused for the rest of the flight
  bool cutoffLatched = false;  // envelope cut-off tripped

  CanardRequest request = CanardRequest::None;  // what was asked this cycle
  uint16_t gates = 0;          // CanardGate bits that blocked attitude hold this cycle

  uint32_t deniedCount = 0;    // requests refused by the permission table, or conflicts
  CanardRequest lastDenied = CanardRequest::None;
  uint32_t rejectedCommandCount = 0;  // CanardsEnable commands refused (not on the ground)

  float commandRad[kNumCanards] = {};  // commanded canard angles, by canard
  CanardAngles angles;                 // where the canards are (measured or modelled), by canard
};

class Canards {
 public:
  // Both references must outlive this object.
  Canards(AttitudeController& attitudeController, ServoControl& servoControl);

  // --- Ground commands ------------------------------------------------------
  // Handles CanardsEnable (accepted only in Init or PreFlight) and
  // CanardsDisable (always accepted). Other command types are ignored.
  // `currentState` is the state at the time the command arrives; an enable is
  // judged again in periodic() against the state after this cycle's
  // transition, so one that arrives on the launch cycle is refused.
  void handleCommand(const Command& command, StateId currentState);

  // --- Requests (from states), valid for one cycle --------------------------
  // Clears last cycle's request. Called by the FlightComputer before the
  // state machine runs.
  void beginCycle();

  void requestNeutral();
  // Hold `reference` (body -> NED). Runs the attitude controller.
  void requestAttitudeHold(const Quat& reference);
  // Ground test: drive the canards to these angles [rad], by canard.
  void requestTestAngles(const float rad[kNumCanards]);

  // --- Once per cycle, after the state machine ------------------------------
  // `currentState` is the state in force after this cycle's transition.
  // `safetyActive` is SafetyMonitor::anyActive().
  void periodic(const Cycle& cycle, StateId currentState, bool safetyActive,
                const ActuatorFeedback& feedback, ActuatorCommand& out);

  const CanardsStatus& status() const { return status_; }

 private:
  AttitudeController& attitudeController_;
  ServoControl& servoControl_;
  CanardsStatus status_;

  CanardRequest request_ = CanardRequest::None;
  Quat reference_;
  float testRad_[kNumCanards] = {};

  bool enabled_ = false;
  Persistence cutoff_{FlightConfig::kCutoffHoldUs};
  StateId lastState_ = StateId::Init;            // the state seen by the previous periodic()
  bool enabledThisCycle_ = false;  // enabled by a command since the last periodic()
  bool wasPowered_ = false;
  bool inFlight_ = false;                        // a flight state has been seen
  StateId lastGroundState_ = StateId::Init;      // the most recent ground state seen
};
