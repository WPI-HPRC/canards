#include "AttitudeControl/LqrAttitudeController.h"

#include <cmath>

#include "AttitudeControl/lqr_gain_table.h"

namespace {

constexpr int kNumCanardsLqr = 4;
constexpr int kNumGains = 6;

// Limits a value to [-limit, +limit]. `limit` must be positive.
float clampSymmetric(float value, float limit) {
  return value > limit ? limit : (value < -limit ? -limit : value);
}

// The checks behind valid(). lookupGains() trusts the table completely (it
// divides by the gap between two speeds and reads rows 0 and 1 without
// looking), so everything it relies on is checked here, once.
bool tableIsUsable(const float (*table)[7], int rows) {
  if (table == nullptr || rows < 2) return false;
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < 1 + kNumGains; ++column) {
      if (!std::isfinite(table[row][column])) return false;
    }
    // Strictly increasing speeds: equal speeds would divide by zero.
    if (row > 0 && !(table[row][0] > table[row - 1][0])) return false;
  }
  return true;
}

}  // namespace

LqrAttitudeController::LqrAttitudeController(const float (*table)[7], int rows,
                                             float canardLimitRad)
    : table_(table),
      rows_(rows),
      canardLimitRad_(canardLimitRad),
      valid_(tableIsUsable(table, rows) && std::isfinite(canardLimitRad) && canardLimitRad > 0.0f) {}

LqrAttitudeController::LqrAttitudeController(float canardLimitRad)
    : LqrAttitudeController(gnc::kGainTable, gnc::kGainRows, canardLimitRad) {}

void LqrAttitudeController::lookupGains(float speed, float k[6]) const {
  // Find the pair of rows around `speed`. Exactly on a row, the pair below it
  // is used. Outside the table the first or last pair is used, and the clamp
  // on t below then holds the end row.
  int hi = 1;
  while (hi < rows_ - 1 && speed > table_[hi][0]) {
    ++hi;
  }
  const float* a = table_[hi - 1];
  const float* b = table_[hi];
  float t = (speed - a[0]) / (b[0] - a[0]);
  t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
  for (int i = 0; i < kNumGains; ++i) {
    k[i] = a[i + 1] + t * (b[i + 1] - a[i + 1]);
  }
}

bool LqrAttitudeController::law(const AttitudeControlInput& in, float command[4]) const {
  if (!valid_) return false;

  // Reject every non-finite input before any arithmetic. This is deliberately
  // stricter than the old code, which only checked the three axis commands:
  // there an infinite roll error was clamped to 10 deg and flown, and an
  // infinite speed picked an end row of the table and was flown. A value like
  // that means something upstream is broken, so the answer is neutral.
  if (!isFinite(in.attitudeError) || !isFinite(in.bodyRate) || !std::isfinite(in.speed)) {
    return false;
  }
  for (int i = 0; i < kNumCanardsLqr; ++i) {
    if (!std::isfinite(in.canardAngle[i])) return false;
  }

  float k[kNumGains];
  lookupGains(in.speed, k);

  const Vec3& e = in.attitudeError;
  const Vec3& w = in.bodyRate;
  const float* sigma = in.canardAngle;

  // The three single-axis laws. The expressions are kept in the old code's
  // shape, term for term, so the port rounds the same way.
  const float rollError = clampSymmetric(e.x, kRollErrorLimitRad);
  const float vRoll = -(k[0] * rollError + k[1] * w.x +
                        k[2] * 0.25f * (sigma[0] + sigma[1] + sigma[2] + sigma[3]));
  const float vPitch = -(k[3] * e.y + k[4] * w.y + k[5] * 0.5f * (sigma[2] - sigma[0]));
  const float vYaw = -(k[3] * e.z + k[4] * w.z + k[5] * 0.5f * (sigma[3] - sigma[1]));

  // Finite inputs can still overflow (an error of 1e38 rad times a gain).
  if (!std::isfinite(vRoll) || !std::isfinite(vPitch) || !std::isfinite(vYaw)) {
    return false;
  }

  // Roll priority: roll takes what it asks for up to the limit, and pitch and
  // yaw are scaled together into whatever is left. Roll is the axis the
  // canards can actually move; pitch and yaw fight the fins and would
  // otherwise use up the whole limit.
  // No division by zero: largest > remaining >= 0 means largest > 0.
  const float roll = clampSymmetric(vRoll, canardLimitRad_);
  const float remaining = canardLimitRad_ - std::fabs(roll);
  const float largest = std::fmax(std::fabs(vPitch), std::fabs(vYaw));
  const float scale = largest > remaining ? remaining / largest : 1.0f;
  const float pitch = scale * vPitch;
  const float yaw = scale * vYaw;

  // Back to the four canards. The clamp only removes rounding.
  command[0] = clampSymmetric(roll - pitch, canardLimitRad_);
  command[1] = clampSymmetric(roll - yaw, canardLimitRad_);
  command[2] = clampSymmetric(roll + pitch, canardLimitRad_);
  command[3] = clampSymmetric(roll + yaw, canardLimitRad_);

  for (int i = 0; i < kNumCanardsLqr; ++i) {
    if (!std::isfinite(command[i])) return false;
  }
  return true;
}

bool LqrAttitudeController::compute(const AttitudeControlInput& in, float canardCommand[4]) {
  // Work in a local array and copy out at the end, so a caller may pass
  // in.canardAngle as the output, and so a failure part-way through can
  // never leave a half-written command behind.
  float command[kNumCanardsLqr] = {0.0f, 0.0f, 0.0f, 0.0f};
  const bool ok = law(in, command);
  for (int i = 0; i < kNumCanardsLqr; ++i) {
    canardCommand[i] = ok ? command[i] : 0.0f;
  }
  return ok;
}
