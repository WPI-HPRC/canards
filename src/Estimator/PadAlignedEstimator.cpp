#include "Estimator/PadAlignedEstimator.h"

#include <cmath>

#include "FlightConfig.h"

void PadAlignedEstimator::reinitialize() {
  phase_ = Phase::Collecting;
  collected_ = 0;
  propagateOk_ = true;
  velocityNed_ = Vec3{};
  // The last magnetometer and barometer samples are kept: they are still the
  // latest readings, and alignment needs one of each.
}

void PadAlignedEstimator::update(const EstimatorInput& in, NavState& out) {
  out = NavState{};
  out.timeUs = in.timeUs;

  if (in.magFresh && isFinite(in.magBody)) {
    lastMag_ = in.magBody;
    haveMag_ = true;
  }
  if (in.baroFresh && std::isfinite(in.pressurePa) && in.pressurePa > 0.0f) {
    lastPressurePa_ = in.pressurePa;
    baroTimeUs_ = in.timeUs;
    haveBaro_ = true;
  }

  if (phase_ == Phase::Collecting) {
    // Only samples taken at rest count. Off the pad nothing is collected, so
    // a rocket that launches before alignment never reports a valid state.
    if (in.onPad) {
      if (collected_ < backend_.gyroSamplesNeeded() && backend_.collectGyro(in.gyroBody)) ++collected_;
      if (collected_ >= backend_.gyroSamplesNeeded() && haveMag_ && haveBaro_ &&
          backend_.align(in.accelBody, lastMag_, lastPressurePa_)) {
        phase_ = Phase::Running;
        velocityNed_ = Vec3{};
        return;  // aligned on this sample; the first valid output is the next one
      }
    }
    return;
  }
  if (phase_ != Phase::Running) return;

  // dt is 0 on the very first sample; there is nothing to propagate then.
  if (in.dt > 0.0f) propagateOk_ = backend_.propagate(in.gyroBody, in.dt) && propagateOk_;

  const Quat q = backend_.attitude();
  const Vec3 rate = backend_.unbiasedGyro(in.gyroBody);
  out.attitude = q;
  out.bodyRate = rate;
  out.attitudeValid = propagateOk_ && isUnit(q) && isFinite(rate);

  if (out.attitudeValid) {
    if (in.onPad) {
      velocityNed_ = Vec3{};  // at rest
    } else if (in.dt > 0.0f) {
      // Specific force in NED plus gravity (NED z is down) is the acceleration.
      const Vec3 accelNed = rotate(q, in.accelBody) + Vec3{0.0f, 0.0f, FlightConfig::kGravity};
      velocityNed_ = velocityNed_ + accelNed * in.dt;
    }
  }
  out.velocityNed = velocityNed_;
  out.velocityValid = out.attitudeValid && isFinite(velocityNed_);

  if (haveBaro_ && in.timeUs >= baroTimeUs_ && in.timeUs - baroTimeUs_ <= FlightConfig::kBaroStaleUs) {
    const float h = backend_.heightAgl(lastPressurePa_);
    if (std::isfinite(h)) {
      out.altitudeAgl = h;
      out.altitudeValid = true;
    }
  }
}
