#pragma once

// Decodes the PWM output of one US Digital MA4 absolute shaft encoder from
// capture-timer edges. No Arduino: the board's capture interrupt feeds it
// (MarsActuators), and the desktop tests drive it directly.
//
// Ported from WPI-HPRC/pid-tuner include/PwmDecoder.h (commit d7b3440, as
// copied unchanged into Canards-Rocket src/canards/pid_tuner/PwmDecoder.h).
// The decoding and validity logic is unchanged so bench-tuned calibration
// carries over. The only change is the clock: the tuner passes millis(); this
// takes a time in microseconds from the caller (micros() on the board), so its
// two time limits are written in microseconds (6 ms -> 6000 us, 30 ms ->
// 30000 us). Times are uint32_t and compared by unsigned difference, so they
// survive the 71.6-minute wrap of micros().

#include <cstdint>

namespace ma4 {

// The capture timers count at 10 MHz: one tick is 0.1 microseconds.
constexpr uint32_t kCaptureClockHz = 10000000;
// A period only counts if the rising edge before it was at most this long ago.
constexpr uint32_t kMaxRisingEdgeGapUs = 6000;
// A completed period stays fresh for this long after its closing rising edge.
constexpr uint32_t kFreshForUs = 30000;

// A completed MA4 PWM period, in capture ticks (0.1 us).
struct PwmSample {
  uint32_t periodTicks = 0;
  uint32_t highTicks = 0;
  uint32_t timestampUs = 0;  // time of the rising edge that closed the period
  bool valid = false;

  bool isFresh(uint32_t nowUs) const {
    return valid && uint32_t(nowUs - timestampUs) <= kFreshForUs;
  }
};

class PwmDecoder {
 public:
  // Shaft angle in degrees, [0, 360), from one period's high time. False (and
  // angleDeg untouched) if the duty is outside what an MA4 can output.
  static bool decodeAngle(uint32_t highTicks, uint32_t periodTicks, float& angleDeg) {
    if (periodTicks == 0 || highTicks >= periodTicks) {
      return false;
    }
    // 12-bit MA4 PWM as the tuner decodes it: a 4351-clock frame whose high
    // time is 128 clocks plus one clock per count.
    float counts = 4351.f * float(highTicks) / float(periodTicks) - 128.f;
    // Allow two counts of capture quantization at either MA4 endpoint.
    if (counts < -2 || counts > 4097) {
      return false;
    }
    angleDeg = clamp(counts, 0, 4095) * 360.f / 4096.f;
    return true;
  }

  const PwmSample& getSample() const { return sample; }

  // One edge: high is the pin level after it, captureTicks the 16-bit capture
  // counter at it, nowUs the time in microseconds.
  void recordEdge(bool high, uint16_t captureTicks, uint32_t nowUs) {
    if (!high) {
      if (haveRisingEdge) {
        pendingHighTicks = uint16_t(captureTicks - risingEdgeTicks);
        haveFallingEdge = true;
      }
      return;
    }

    // Unsigned 16-bit subtraction accounts for one capture-timer wrap.
    uint32_t periodTicks = uint16_t(captureTicks - risingEdgeTicks);
    float ignoredAngle;
    bool lowFrequency = periodTicks >= 41300 && periodTicks <= 46000;
    bool highFrequency = periodTicks >= 10300 && periodTicks <= 11500;
    sample.valid = haveRisingEdge && haveFallingEdge &&
                   uint32_t(nowUs - sample.timestampUs) <= kMaxRisingEdgeGapUs &&
                   (lowFrequency || highFrequency) &&
                   decodeAngle(pendingHighTicks, periodTicks, ignoredAngle);

    // Publish high time and period together; the next falling edge must
    // not modify the completed sample being consumed by the main loop.
    sample.highTicks = pendingHighTicks;
    sample.periodTicks = periodTicks;
    sample.timestampUs = nowUs;
    risingEdgeTicks = captureTicks;
    haveRisingEdge = true;
    haveFallingEdge = false;
  }

 private:
  // Same as the tuner's tuning::clamp (ControlMath.h).
  static float clamp(float value, float minimum, float maximum) {
    if (value < minimum) {
      return minimum;
    }
    if (value > maximum) {
      return maximum;
    }
    return value;
  }

  uint16_t risingEdgeTicks = 0;
  uint32_t pendingHighTicks = 0;
  bool haveRisingEdge = false;
  bool haveFallingEdge = false;
  PwmSample sample;
};

}  // namespace ma4
