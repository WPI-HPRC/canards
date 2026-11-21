#include "MarsActuators.h"

#if defined(ARDUINO)

#include <PeripheralPins.h>
#include <pinmap.h>

#include <cmath>

namespace {

constexpr float kRadPerDeg = 3.14159265358979323846f / 180.0f;

// Servo channel 0..3 = CH1..CH4.
constexpr uint32_t kServoPins[kNumCanards] = {SERVO1_PWM, SERVO2_PWM, SERVO3_PWM, SERVO4_PWM};
constexpr uint32_t kServoPowerPin = CANARD_POWER_ENABLE;
constexpr uint32_t kLevelShiftPin = LEVEL_SHIFT_ENABLE;

// Encoder input 0..3. Input 1 was rewired to PA0 (the PWM_OUT1 pad) on
// 2026-11-20 because of problems with the variant's ENCODER2_PWM (PB6).
constexpr uint32_t kEncoderPins[4] = {ENCODER1_PWM, PA0, ENCODER3_PWM, ENCODER4_PWM};
// Capture channel of each encoder input on its timer (TIM3, TIM2, TIM3, TIM15).
constexpr uint32_t kCaptureChannel[4] = {2, 1, 1, 1};

static_assert(kEncoderPins[0] == PB5 && kEncoderPins[1] == PA0 && kEncoderPins[2] == PC6 &&
                  kEncoderPins[3] == PE5,
              "Update the capture timers and channels when the encoder pins change");

constexpr bool isPermutation(const int (&map)[kNumCanards]) {
  for (int i = 0; i < kNumCanards; ++i) {
    if (map[i] < 0 || map[i] >= kNumCanards) return false;
    for (int j = 0; j < i; ++j) {
      if (map[i] == map[j]) return false;
    }
  }
  return true;
}
static_assert(isPermutation(MarsActuators::kEncoderOfServo),
              "kEncoderOfServo must use each encoder input once");

float clampPulse(float pulseUs) {
  if (pulseUs < MarsActuators::kAbsMinPulseUs) return MarsActuators::kAbsMinPulseUs;
  if (pulseUs > MarsActuators::kAbsMaxPulseUs) return MarsActuators::kAbsMaxPulseUs;
  return pulseUs;
}

// Makes a pin a push-pull output that starts at `high`. The level goes into
// the output register first, so the pin never shows the other level.
// (digitalWrite() before pinMode() would be ignored: the port clock is off.)
void configureOutput(uint32_t pin, bool high) {
  const PinName name = digitalPinToPinName(pin);
  set_GPIO_Port_Clock(STM_PORT(name));
  digitalWriteFast(name, high ? HIGH : LOW);
  pinMode(pin, OUTPUT);
}

// True if the board's pin map puts `pin` on `timer`, channel `channel`.
bool pinIsOn(uint32_t pin, TIM_TypeDef* timer, uint32_t channel) {
  const PinName name = digitalPinToPinName(pin);
  return static_cast<TIM_TypeDef*>(pinmap_peripheral(name, PinMap_TIM)) == timer &&
         STM_PIN_CHANNEL(pinmap_function(name, PinMap_TIM)) == channel;
}

}  // namespace

MarsActuators::MarsActuators(const float centerPulseUs[kNumCanards]) {
  for (int ch = 0; ch < kNumCanards; ++ch) {
    const float center = centerPulseUs[ch];
    centerPulseUs_[ch] = clampPulse(std::isfinite(center) ? center : 1500.0f);
  }
}

bool MarsActuators::begin() {
  if (begun_) {
    return diagnostics_.beginOk;
  }
  begun_ = true;

  // Power off first, written before the pin becomes an output.
  configureOutput(kServoPowerPin, !kServoPowerOnLevelHigh);
  powered_ = false;
  for (uint32_t pin : kServoPins) {
    configureOutput(pin, false);
  }
  configureOutput(kLevelShiftPin, kLevelShiftOnLevelHigh);  // pass pulses

  diagnostics_.servosOk = beginServos();
  diagnostics_.encodersOk = beginEncoders();
  diagnostics_.beginOk = diagnostics_.servosOk && diagnostics_.encodersOk;

  // Pulses first, then power, so the servos wake up to a valid command.
  if (diagnostics_.servosOk) {
    writePower(true);
  }
  return diagnostics_.beginOk;
}

bool MarsActuators::beginServos() {
  // All four servo pins are channels of one timer (TIM1 on MARS v2.1), so one
  // hardware timer makes every pulse with no interrupt jitter.
  TIM_TypeDef* instance = nullptr;
  uint32_t channel[kNumCanards] = {};
  for (int ch = 0; ch < kNumCanards; ++ch) {
    const PinName name = digitalPinToPinName(kServoPins[ch]);
    auto* timer = static_cast<TIM_TypeDef*>(pinmap_peripheral(name, PinMap_TIM));
    if (timer == nullptr || (instance != nullptr && timer != instance)) {
      return false;
    }
    instance = timer;
    channel[ch] = STM_PIN_CHANNEL(pinmap_function(name, PinMap_TIM));
  }

  servoTimer_ = new HardwareTimer(instance);
  servoTimer_->setOverflow(kServoPwmHz, HERTZ_FORMAT);
  diagnostics_.servoTimerClockHz = servoTimer_->getTimerClkFreq();
  diagnostics_.servoPrescaler = servoTimer_->getPrescaleFactor();
  diagnostics_.servoPeriodTicks = servoTimer_->getOverflow(TICK_FORMAT);
  diagnostics_.servoTicksPerUs =
      float(diagnostics_.servoTimerClockHz) / float(diagnostics_.servoPrescaler) * 1.0e-6f;

  for (int ch = 0; ch < kNumCanards; ++ch) {
    servoTimerChannel_[ch] = channel[ch];
    servoTimer_->setMode(channel[ch], TIMER_OUTPUT_COMPARE_PWM1, kServoPins[ch]);
    servoTimer_->setCaptureCompare(
        channel[ch], uint32_t(centerPulseUs_[ch] * diagnostics_.servoTicksPerUs + 0.5f),
        TICK_COMPARE_FORMAT);
  }
  servoTimer_->resume();
  return true;
}

bool MarsActuators::beginEncoders() {
  // Inputs 0 and 2 share TIM3's free-running counter; PB5/PE5 and PB6/PC6
  // share EXTI lines, so GPIO interrupts could not read them independently.
  // Both-edge capture uses one channel per input.
  if (!pinIsOn(kEncoderPins[0], TIM3, kCaptureChannel[0]) ||
      !pinIsOn(kEncoderPins[1], TIM2, kCaptureChannel[1]) ||
      !pinIsOn(kEncoderPins[2], TIM3, kCaptureChannel[2]) ||
      !pinIsOn(kEncoderPins[3], TIM15, kCaptureChannel[3])) {
    return false;
  }

  captureTimer2_ = new HardwareTimer(TIM2);
  captureTimer3_ = new HardwareTimer(TIM3);
  captureTimer15_ = new HardwareTimer(TIM15);
  HardwareTimer* const timers[3] = {captureTimer2_, captureTimer3_, captureTimer15_};
  for (int i = 0; i < 3; ++i) {
    const uint32_t clockHz = timers[i]->getTimerClkFreq();
    diagnostics_.captureTimerClockHz[i] = clockHz;
    // The decoder's period bands are in 0.1 us ticks: the count rate must be exact.
    if (clockHz == 0 || clockHz % ma4::kCaptureClockHz != 0) {
      return false;
    }
    timers[i]->setPrescaleFactor(clockHz / ma4::kCaptureClockHz);
    timers[i]->setOverflow(kCapturePeriodTicks, TICK_FORMAT);
    diagnostics_.capturePrescaler[i] = timers[i]->getPrescaleFactor();
  }

  captureTimerOf_[0] = captureTimer3_;
  captureTimerOf_[1] = captureTimer2_;
  captureTimerOf_[2] = captureTimer3_;
  captureTimerOf_[3] = captureTimer15_;
  for (unsigned input = 0; input < kNumEncoderInputs; ++input) {
    captureTimerOf_[input]->setMode(kCaptureChannel[input], TIMER_INPUT_CAPTURE_BOTHEDGE,
                                    kEncoderPins[input]);
    captureTimerOf_[input]->attachInterrupt(kCaptureChannel[input],
                                            [this, input]() { recordEdge(input); });
  }
  for (HardwareTimer* timer : timers) {
    timer->resume();
  }
  return true;
}

// Capture interrupt for one encoder input.
void MarsActuators::recordEdge(unsigned input) {
  const uint16_t captureTicks =
      static_cast<uint16_t>(captureTimerOf_[input]->getCaptureCompare(kCaptureChannel[input]));
  const bool high = digitalRead(kEncoderPins[input]) == HIGH;
  decoders_[input].recordEdge(high, captureTicks, micros());
}

ma4::PwmSample MarsActuators::sampleOf(unsigned input, uint32_t& nowUs) const {
  // Copy a coherent sample, preserving the caller's interrupt state.
  const uint32_t interruptMask = __get_PRIMASK();
  __disable_irq();
  const ma4::PwmSample sample = decoders_[input].getSample();
  __set_PRIMASK(interruptMask);
  // Read the clock after the copy, with interrupts back on, so "now" is never
  // earlier than the sample's own time stamp.
  nowUs = micros();
  return sample;
}

void MarsActuators::readFeedback(ActuatorFeedback& out) {
  for (int ch = 0; ch < kNumCanards; ++ch) {
    uint32_t nowUs = 0;
    const ma4::PwmSample sample = sampleOf(unsigned(kEncoderOfServo[ch]), nowUs);
    float angleDeg = 0.0f;
    const bool valid = sample.isFresh(nowUs) &&
                       ma4::PwmDecoder::decodeAngle(sample.highTicks, sample.periodTicks, angleDeg);
    out.encoderValid[ch] = valid;
    out.encoderAngleRad[ch] = valid ? angleDeg * kRadPerDeg : 0.0f;
  }
}

MarsActuators::EncoderReading MarsActuators::readEncoderInput(unsigned input) const {
  EncoderReading reading;
  if (input >= kNumEncoderInputs) {
    return reading;
  }
  uint32_t nowUs = 0;
  const ma4::PwmSample sample = sampleOf(input, nowUs);
  if (sample.periodTicks != 0) {
    reading.frequencyHz = float(ma4::kCaptureClockHz) / float(sample.periodTicks);
    reading.dutyPercent = 100.0f * float(sample.highTicks) / float(sample.periodTicks);
  }
  reading.valid = sample.isFresh(nowUs) &&
                  ma4::PwmDecoder::decodeAngle(sample.highTicks, sample.periodTicks,
                                               reading.angleDeg);
  return reading;
}

void MarsActuators::write(const ActuatorCommand& command) {
  if (servoTimer_ == nullptr) {
    return;  // no pulses: power stays off
  }
  for (int ch = 0; ch < kNumCanards; ++ch) {
    float pulseUs = command.pulseUs[ch];
    if (!std::isfinite(pulseUs)) {
      pulseUs = centerPulseUs_[ch];
    }
    pulseUs = clampPulse(pulseUs);
    servoTimer_->setCaptureCompare(servoTimerChannel_[ch],
                                   uint32_t(pulseUs * diagnostics_.servoTicksPerUs + 0.5f),
                                   TICK_COMPARE_FORMAT);
  }
  // After the pulses, so power returning wakes the servos to this command.
  writePower(command.servoPower);
}

void MarsActuators::writePower(bool on) {
  digitalWriteFast(digitalPinToPinName(kServoPowerPin),
                   on == kServoPowerOnLevelHigh ? HIGH : LOW);
  powered_ = on;
}

#endif  // defined(ARDUINO)
