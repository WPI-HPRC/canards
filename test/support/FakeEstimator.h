#pragma once

// A scriptable Estimator for desktop tests.
//
// A test sets `output` to the NavState the next update() should return, and
// afterwards reads back what the estimator was given:
//   - every EstimatorInput (the first kLogCapacity of them, plus the last);
//   - how many times update() and reinitialize() were called;
//   - the ORDER of those calls (the first kLogCapacity of them), so a test can
//     prove reinitialize() ran before a given update().
//
// By default update() stamps the output with the input's sample time, so the
// estimate always looks fresh. Set `stampTime = false` to script the time too
// (a stale estimate, or one from the future).
//
// Fixed-size logs, no heap: a long scripted flight just stops recording
// detail after kLogCapacity calls; the counters and lastInput() keep going.

#include <cstddef>
#include <cstdint>

#include "Estimator/Estimator.h"
#include "Frames.h"

class FakeEstimator final : public Estimator {
 public:
  enum class Call : uint8_t { Reinitialize, Update };

  static constexpr size_t kLogCapacity = 64;

  // --- Script ----------------------------------------------------------------

  // What update() writes to its `out` argument.
  NavState output;
  // true: update() overwrites output.timeUs with the input's sample time.
  bool stampTime = true;

  // Makes `output` a fully valid estimate.
  void setValid(const Quat& attitude, const Vec3& bodyRate, const Vec3& velocityNed,
                float altitudeAgl) {
    output.attitude = attitude;
    output.bodyRate = bodyRate;
    output.velocityNed = velocityNed;
    output.altitudeAgl = altitudeAgl;
    output.attitudeValid = true;
    output.velocityValid = true;
    output.altitudeValid = true;
  }

  // Makes `output` report nothing valid (what NullEstimator does).
  void setInvalid() {
    output.attitudeValid = false;
    output.velocityValid = false;
    output.altitudeValid = false;
  }

  // --- Estimator ---------------------------------------------------------------

  void reinitialize() override {
    ++reinitializeCount_;
    record(Call::Reinitialize);
  }

  void update(const EstimatorInput& in, NavState& out) override {
    if (updateCount_ < kLogCapacity) inputs_[updateCount_] = in;
    lastInput_ = in;
    ++updateCount_;
    record(Call::Update);

    if (stampTime) output.timeUs = in.timeUs;
    out = output;
  }

  // --- What happened -----------------------------------------------------------

  uint32_t updateCount() const { return updateCount_; }
  uint32_t reinitializeCount() const { return reinitializeCount_; }

  // The input of the most recent update(). Default-constructed before any.
  const EstimatorInput& lastInput() const { return lastInput_; }

  // The input of the i-th update() (0 = first). Only the first kLogCapacity
  // are kept; `i` must be below loggedInputCount().
  const EstimatorInput& inputAt(size_t i) const { return inputs_[i]; }
  size_t loggedInputCount() const {
    return updateCount_ < kLogCapacity ? updateCount_ : kLogCapacity;
  }

  // Every call in the order it happened (the first kLogCapacity of them).
  // `i` must be below loggedCallCount().
  Call callAt(size_t i) const { return calls_[i]; }
  size_t loggedCallCount() const { return callCount_ < kLogCapacity ? callCount_ : kLogCapacity; }
  // Calls of both kinds, including those past the end of the log.
  uint32_t callCount() const { return callCount_; }

  // Forgets the recorded calls. The script (`output`, `stampTime`) is kept.
  void clearLog() {
    updateCount_ = 0;
    reinitializeCount_ = 0;
    callCount_ = 0;
    lastInput_ = EstimatorInput{};
  }

 private:
  void record(Call call) {
    if (callCount_ < kLogCapacity) calls_[callCount_] = call;
    ++callCount_;
  }

  EstimatorInput inputs_[kLogCapacity];
  EstimatorInput lastInput_;
  Call calls_[kLogCapacity] = {};
  uint32_t updateCount_ = 0;
  uint32_t reinitializeCount_ = 0;
  uint32_t callCount_ = 0;
};
