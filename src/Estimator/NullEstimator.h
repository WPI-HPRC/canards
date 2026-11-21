#pragma once

// The estimator that knows nothing
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.5).
//
// It reports nothing valid. With it the canards never control, and every
// navigation-based transition falls back to its timeout. That is the safe
// default until the real estimator is plugged in.

#include "Estimator/Estimator.h"

class NullEstimator final : public Estimator {
 public:
  void reinitialize() override {}

  // Writes a default NavState: identity attitude, zeros, every valid flag
  // false. Only the time is real, so the logs show the estimator was called.
  void update(const EstimatorInput& in, NavState& out) override {
    out = NavState{};
    out.timeUs = in.timeUs;
  }
};
