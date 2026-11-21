#pragma once

// The canard hardware on the MARS v2.1 board: four RS100 servo outputs, four
// MA4 encoder inputs, the servo power enable and the level-shifter enable.
// Only moves signals; what to command is decided in ServoControl.
//
// Ported from Canards-Rocket src/canards/CanardActuators.{h,cpp} and
// src/canards/pid_tuner/EncoderCapture.h. The decoder is PwmDecoder.h.
//
// Everything here is by SERVO CHANNEL (see Actuators.h). The encoder cables
// run in reverse, so servo channel n is read on encoder input
// kEncoderOfServo[n]; that map is applied here and nowhere else.
//
// Pins and timers (board/variants/MARSv21/variant_MARSV21.h, PeripheralPins.c):
//   servo channel 0..3   PE11, PE13, PE14, PA8   TIM1 CH2, CH3, CH4, CH1, 50 Hz
//   encoder input 0      PB5                     TIM3_CH2
//   encoder input 1      PA0 (rewired; NOT the variant's ENCODER2_PWM = PB6)
//                                                TIM2_CH1
//   encoder input 2      PC6                     TIM3_CH1
//   encoder input 3      PE5                     TIM15_CH1
//   servo power enable   PA4                     GPIO
//   level-shift enable   PB12                    GPIO, active-low
// This class owns TIM1, TIM2, TIM3 and TIM15. Never use analogWrite, Servo,
// tone or PwmInput on a pin of those timers: the core keeps one handle per
// timer, and a second user would take it over.
//
// Servo power follows the flight rule: on from begin(), off only while
// ActuatorCommand::servoPower is false. Centre (or commanded) pulses keep
// running while power is off.
//
// Arduino only: the desktop (native) build compiles nothing from this file.

#if defined(ARDUINO)

#include <Arduino.h>
#include <HardwareTimer.h>

#include <cstdint>

#include "Actuators.h"
#include "PwmDecoder.h"

class MarsActuators final : public Actuators {
 public:
  // Hard sanity range for any pulse, whatever the calibration says.
  // ServoControl already clamps to each channel's calibrated limits; this is
  // the last guard.
  static constexpr float kAbsMinPulseUs = 900.0f;
  static constexpr float kAbsMaxPulseUs = 2100.0f;

  static constexpr uint32_t kServoPwmHz = 50;

  // Encoder input that reads each servo channel's shaft (index = servo
  // channel, value = encoder input). The cables run in reverse.
  static constexpr int kEncoderOfServo[kNumCanards] = {3, 2, 1, 0};

  // Servo power enable PA4: true if driving it HIGH turns servo power on.
  // Assumed active-high; not yet checked on the schematic.
  static constexpr bool kServoPowerOnLevelHigh = true;  // PLACEHOLDER (EE)

  // Level-shift enable PB12 is active-low: driven low, pulses reach the servo
  // connectors (bench finding 2026-11-19, not yet checked on the schematic).
  static constexpr bool kLevelShiftOnLevelHigh = false;

  // What begin() found, for a bench program to print. Nothing is printed here.
  struct Diagnostics {
    bool beginOk = false;     // servos and encoders both started
    bool servosOk = false;    // TIM1 running with the centre pulses
    bool encodersOk = false;  // capture running on all four inputs
    uint32_t servoTimerClockHz = 0;
    uint32_t servoPrescaler = 0;   // prescale factor (PSC + 1)
    uint32_t servoPeriodTicks = 0; // ticks in one 50 Hz frame (ARR + 1)
    float servoTicksPerUs = 0.0f;
    // Capture timers, in the order TIM2, TIM3, TIM15. Each must count at
    // exactly ma4::kCaptureClockHz.
    uint32_t captureTimerClockHz[3] = {};
    uint32_t capturePrescaler[3] = {};
  };

  // One encoder input's latest period, for bench checks.
  struct EncoderReading {
    bool valid = false;  // fresh and decoded
    float angleDeg = 0.0f;
    float frequencyHz = 0.0f;
    float dutyPercent = 0.0f;
  };

  // centerPulseUs: each servo channel's centre pulse (the ServoConfig
  // centres). A non-finite centre becomes 1500 us, and every centre is held
  // to [kAbsMinPulseUs, kAbsMaxPulseUs].
  explicit MarsActuators(const float centerPulseUs[kNumCanards]);

  MarsActuators(const MarsActuators&) = delete;
  MarsActuators& operator=(const MarsActuators&) = delete;

  // Sets up every pin and timer, starts the centre pulses, then turns servo
  // power on. Call once, from setup(): it allocates the HardwareTimer objects
  // on the heap. Returns diagnostics().beginOk. If the servo timer cannot be
  // set up, the servo pins stay low outputs and power stays off; if capture
  // cannot be set up, every encoder reads invalid.
  bool begin();

  // Actuators. Neither blocks.
  void readFeedback(ActuatorFeedback& out) override;
  void write(const ActuatorCommand& command) override;

  bool isPowered() const { return powered_; }
  const Diagnostics& diagnostics() const { return diagnostics_; }

  // One encoder INPUT (0..3, not a servo channel), raw. Never blocks.
  EncoderReading readEncoderInput(unsigned input) const;

 private:
  static constexpr unsigned kNumEncoderInputs = 4;
  static constexpr uint32_t kCapturePeriodTicks = 65536;  // 16-bit wrap, 6.55 ms

  bool beginServos();
  bool beginEncoders();
  void writePower(bool on);
  void recordEdge(unsigned input);
  ma4::PwmSample sampleOf(unsigned input, uint32_t& nowUs) const;

  float centerPulseUs_[kNumCanards];

  HardwareTimer* servoTimer_ = nullptr;
  uint32_t servoTimerChannel_[kNumCanards] = {};

  HardwareTimer* captureTimer2_ = nullptr;
  HardwareTimer* captureTimer3_ = nullptr;
  HardwareTimer* captureTimer15_ = nullptr;
  HardwareTimer* captureTimerOf_[kNumEncoderInputs] = {};
  ma4::PwmDecoder decoders_[kNumEncoderInputs];

  bool begun_ = false;
  bool powered_ = false;
  Diagnostics diagnostics_;
};

#endif  // defined(ARDUINO)
