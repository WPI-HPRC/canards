#pragma once

// An Estimator whose output is a pure function of its input (in practice, of
// the sample time). Unlike FakeEstimator it needs nobody to set its output
// before each update(), so it gives exactly the same answers whether the
// frames come from a scripted source or from a replayed log.
//
// Test-only: uses std::function.

#include <cstdint>
#include <functional>
#include <utility>

#include "Estimator/Estimator.h"

class ScriptedEstimator final : public Estimator {
 public:
  using Script = std::function<NavState(const EstimatorInput&)>;

  explicit ScriptedEstimator(Script script) : script_(std::move(script)) {}

  void reinitialize() override { ++reinitializeCount_; }
  void update(const EstimatorInput& in, NavState& out) override {
    ++updateCount_;
    out = script_(in);
  }

  uint32_t updateCount() const { return updateCount_; }
  uint32_t reinitializeCount() const { return reinitializeCount_; }

 private:
  Script script_;
  uint32_t updateCount_ = 0;
  uint32_t reinitializeCount_ = 0;
};
