#include "FlightComputer.h"

#include "FlightConfig.h"

FlightComputer::FlightComputer(const FlightIo& io, const ServoConfig& servoConfig)
    : io_(io),
      navigation_(io.estimator),
      servoControl_(servoConfig),
      canards_(io.attitudeController, servoControl_),
      init_(canards_),
      preFlight_(canards_, navigation_),
      boost_(canards_),
      control_(canards_),
      coast_(canards_),
      chute_(canards_),
      land_(canards_),
      states_{&init_, &preFlight_, &boost_, &control_, &coast_, &chute_, &land_},
      stateMachine_(states_) {}

bool FlightComputer::cycle() {
  // 1. Read the sensor frame. Nothing happens until there is a new sample.
  SensorFrame frame;
  if (!io_.sensors.read(frame)) return false;
  const TimeStep step = makeTimeStep(havePrevious_, previousTimeUs_, frame.timeUs);
  havePrevious_ = true;
  previousTimeUs_ = frame.timeUs;  // kept even if it went backwards
  ++cycleCount_;

  // The flight clock: what states, debounces and the ground-test hold see.
  // It follows the sample time but never goes backwards and never jumps by
  // more than kMaxImuGapUs, so one bad timestamp cannot complete a debounce,
  // fire a timeout, or stretch a ground-test hold. The raw time still goes to
  // the estimator and the log, and SafetyMonitor flags the anomaly.
  if (step.first) {
    flightTimeUs_ = frame.timeUs;
  } else if (!step.backwards) {
    flightTimeUs_ += step.gapUs < FlightConfig::kMaxImuGapUs ? step.gapUs : FlightConfig::kMaxImuGapUs;
  }

  // 2. Navigation: mount maps, estimator, |accel|, speed, tilt, freshness.
  // "On the pad" ends as soon as the raw IMU reads launch-level specific
  // force, not when the launch debounce completes, so the estimator never
  // treats the first moments of boost as rest. (|accel| is the same in
  // sensor and body axes, so the raw frame is enough.)
  const bool launchLevel = frame.imuOk && isFinite(frame.accel) && norm(frame.accel) >= FlightConfig::kLaunchAccel;
  navigation_.update(frame, step.dt, isGroundState(stateMachine_.current()) && !launchLevel);

  // 3. Ground commands: at most CommandList::kCapacity per cycle; the rest
  // stay in the source for the next cycle.
  commands_.clear();
  Command command;
  while (!commands_.full() && io_.commands.poll(command)) {
    commands_.push(command);
    canards_.handleCommand(command, stateMachine_.current());
  }

  // 4. System-wide safety checks.
  safety_.check(frame, step, stateMachine_.refusedCount());

  // 5. Clear last cycle's canard request.
  canards_.beginCycle();

  // 6. The state machine.
  Cycle cycle{flightTimeUs_, step.dt, 0, navigation_, commands_};
  stateMachine_.step(cycle);

  // 7. Encoder feedback.
  ActuatorFeedback feedback;
  io_.actuators.readFeedback(feedback);

  // 8. The canards, against the state in force after this cycle's transition.
  ActuatorCommand out;
  canards_.periodic(cycle, stateMachine_.current(), safety_.anyActive(), feedback, out);

  // 9. Servos.
  io_.actuators.write(out);

  // 10. Log every input and output of the cycle.
  record_.cycleCount = cycleCount_;
  record_.frame = frame;
  record_.feedback = feedback;
  record_.commandCount = static_cast<uint8_t>(commands_.size());
  for (size_t i = 0; i < CommandList::kCapacity; ++i) {
    record_.commands[i] = i < commands_.size() ? commands_[i] : Command{};
  }
  record_.step = step;
  record_.flightTimeUs = flightTimeUs_;
  record_.nav = navigation_.state();
  record_.specificForce = navigation_.specificForce();
  record_.speed = navigation_.speed();
  record_.tilt = navigation_.tilt();
  record_.imuValid = navigation_.imuValid();
  record_.attitudeValid = navigation_.attitudeValid();
  record_.speedValid = navigation_.speedValid();
  record_.altitudeValid = navigation_.altitudeValid();
  record_.state = stateMachine_.current();
  record_.transitioned = stateMachine_.transitioned();
  record_.timeInStateUs = cycle.timeInStateUs;
  record_.refusedTransitions = stateMachine_.refusedCount();
  record_.safetyActive = safety_.active();
  record_.safetySticky = safety_.sticky();
  record_.canards = canards_.status();
  record_.actuators = out;
  io_.log.write(record_);
  return true;
}
