// Tests of the LQR attitude controller port, LqrAttitudeController
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 8.1 and step 7).
//
// The centre of this suite is the EQUIVALENCE test: the same physical
// situation is given to the frozen old controller (in its frame) and to the
// port (in the team frame), and the canard commands must be the same. The old
// code is in reference/ and is reached through OldController.h.
//
// Frames, for reading the tests:
//   team frame (the port):  x = roll (nose), y = pitch, z = yaw
//   old controller frame:   x = pitch,       y = yaw,   z = roll (nose)
// Canard arrays are indexed the same way in both.
//
// Numbers in this file:
//   - Limits and thresholds that are tunable come from FlightConfig, or from
//     the frozen old code (OldBuild::canardLimitRad) where a test compares
//     against it. No test holds a copy of a FlightConfig value.
//   - Expected gains are those of the frozen reference table
//     (reference/reference_table/lqr_gain_table.h), which never changes.

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <limits>

#include "AttitudeControl/LqrAttitudeController.h"
#include "FlightConfig.h"
#include "Frames.h"
#include "OldController.h"

void setUp() {}
void tearDown() {}

namespace {

// Unit conversion only, not a tunable.
constexpr float kRadPerDeg = 0.017453292519943295f;

float rad(float degrees) { return degrees * kRadPerDeg; }
float deg(float radians) { return radians / kRadPerDeg; }

const float kNaN = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();

// Old and new must agree to this. They come out bit-identical on the machine
// this was written on, but the compiler is free to fuse a multiply and an add
// differently in two source files, which can move a result by one unit in the
// last place. 1e-7 rad is 6e-6 deg, far below anything physical.
constexpr float kEquivalenceTolRad = 1e-7f;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

AttitudeControlInput makeInput(Vec3 errorRad, Vec3 rateRadS, float c0, float c1, float c2, float c3,
                               float speed) {
  AttitudeControlInput in;
  in.attitudeError = errorRad;
  in.bodyRate = rateRadS;
  in.canardAngle[0] = c0;
  in.canardAngle[1] = c1;
  in.canardAngle[2] = c2;
  in.canardAngle[3] = c3;
  in.speed = speed;
  return in;
}

AttitudeControlInput errorOnly(float rollDeg, float pitchDeg, float yawDeg, float speed) {
  return makeInput({rad(rollDeg), rad(pitchDeg), rad(yawDeg)}, {}, 0.0f, 0.0f, 0.0f, 0.0f, speed);
}

// The old law on a team-frame input. This is the frame conversion under test,
// written out once: old [0] pitch = team y, old [1] yaw = team z, old [2] roll
// = team x. Canard angles and speed go through unchanged.
void runOld(const OldBuild& old, const AttitudeControlInput& in, float command[4]) {
  const float errorOld[3] = {in.attitudeError.y, in.attitudeError.z, in.attitudeError.x};
  const float rateOld[3] = {in.bodyRate.y, in.bodyRate.z, in.bodyRate.x};
  old.lqr(errorOld, rateOld, in.canardAngle, in.speed, command);
}

float largestDifference(const float a[4], const float b[4]) {
  float worst = 0.0f;
  for (int i = 0; i < 4; ++i) worst = std::fmax(worst, std::fabs(a[i] - b[i]));
  return worst;
}

float largestMagnitude(const float a[4]) {
  float worst = 0.0f;
  for (int i = 0; i < 4; ++i) worst = std::fmax(worst, std::fabs(a[i]));
  return worst;
}

// Fails the running test with everything needed to reproduce the case.
void failCase(const char* what, const char* build, const AttitudeControlInput& in,
              const float expected[4], const float got[4]) {
  char message[720];
  std::snprintf(message, sizeof(message),
                "%s [%s] error=(%.9g, %.9g, %.9g) rate=(%.9g, %.9g, %.9g) "
                "canard=(%.9g, %.9g, %.9g, %.9g) speed=%.9g "
                "expected=(%.9g, %.9g, %.9g, %.9g) got=(%.9g, %.9g, %.9g, %.9g)",
                what, build, double(in.attitudeError.x), double(in.attitudeError.y),
                double(in.attitudeError.z), double(in.bodyRate.x), double(in.bodyRate.y),
                double(in.bodyRate.z), double(in.canardAngle[0]), double(in.canardAngle[1]),
                double(in.canardAngle[2]), double(in.canardAngle[3]), double(in.speed),
                double(expected[0]), double(expected[1]), double(expected[2]), double(expected[3]),
                double(got[0]), double(got[1]), double(got[2]), double(got[3]));
  TEST_FAIL_MESSAGE(message);
}

void assertAllZero(const float command[4]) {
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_TRUE_MESSAGE(command[i] == 0.0f, "command must be exactly zero");
  }
}

// compute() must refuse: return false and write four zeros over whatever was
// in the output array.
void assertRejected(LqrAttitudeController& controller, const AttitudeControlInput& in) {
  float command[4] = {0.5f, -0.5f, 0.25f, -0.25f};
  TEST_ASSERT_FALSE(controller.compute(in, command));
  assertAllZero(command);
}

// Fixed-seed generator, so the sweep is the same on every run and machine.
struct Lcg {
  uint64_t state;

  uint32_t next() {
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return static_cast<uint32_t>(state >> 32);
  }
  float unit() { return static_cast<float>(next() >> 8) * (1.0f / 16777216.0f); }  // [0, 1)
  float symmetric() { return 2.0f * unit() - 1.0f; }                               // [-1, 1)
  int pick(int count) { return static_cast<int>(next() % static_cast<uint32_t>(count)); }
  float sign() { return pick(2) == 0 ? -1.0f : 1.0f; }
};

// ---------------------------------------------------------------------------
// The sweep
// ---------------------------------------------------------------------------

// What a sweep covered. The equivalence test asserts on these, so the sweep
// cannot quietly stop reaching a regime.
struct Coverage {
  int cases = 0;
  int unsaturated = 0;      // no canard at the limit
  int pitchYawScaled = 0;   // at the limit, roll below it: pitch/yaw were scaled
  int rollTakesAll = 0;     // roll alone at the limit: pitch/yaw got nothing
  int rollClampLive = 0;    // |roll error| above its limit AND roll not saturated
  int speedBelow = 0;
  int speedInside = 0;
  int speedAbove = 0;
  float worstDifference = 0.0f;
};

void classify(const OldBuild& old, const AttitudeControlInput& in, const float command[4],
              Coverage& coverage) {
  const float limit = old.canardLimitRad;
  const bool allEqual =
      command[0] == command[1] && command[1] == command[2] && command[2] == command[3];
  const bool rollTakesAll = allEqual && std::fabs(command[0]) == limit;
  if (rollTakesAll) {
    ++coverage.rollTakesAll;
  } else if (largestMagnitude(command) >= limit * (1.0f - 1e-6f)) {
    ++coverage.pitchYawScaled;
  } else {
    ++coverage.unsaturated;
  }
  if (std::fabs(in.attitudeError.x) > old.rollErrorLimitRad && !rollTakesAll) {
    ++coverage.rollClampLive;
  }
  if (in.speed < old.table[0][0]) {
    ++coverage.speedBelow;
  } else if (in.speed > old.table[old.rows - 1][0]) {
    ++coverage.speedAbove;
  } else {
    ++coverage.speedInside;
  }
}

// One comparison: same situation to the old code and to the port.
void compareOnce(const OldBuild& old, LqrAttitudeController& port, const AttitudeControlInput& in,
                 Coverage& coverage) {
  float oldCommand[4];
  runOld(old, in, oldCommand);

  float newCommand[4] = {9.0f, 9.0f, 9.0f, 9.0f};
  const bool ok = port.compute(in, newCommand);
  if (!ok) failCase("port refused a finite input", old.name, in, oldCommand, newCommand);

  const float difference = largestDifference(oldCommand, newCommand);
  // Written so that a NaN on either side fails.
  if (!(difference <= kEquivalenceTolRad)) {
    failCase("old and new disagree", old.name, in, oldCommand, newCommand);
  }
  // The final clamp makes this exact, with no tolerance.
  if (!(largestMagnitude(newCommand) <= old.canardLimitRad)) {
    failCase("port command beyond the limit", old.name, in, oldCommand, newCommand);
  }

  coverage.worstDifference = std::fmax(coverage.worstDifference, difference);
  ++coverage.cases;
  classify(old, in, oldCommand, coverage);
}

// Three sizes for each kind of input, after the mix in the old sim's parity
// test (scales 0.02, 0.3, 3.0): far below saturation, around it, far beyond.
// The roll error sizes are larger because the roll gain is about 8 times
// smaller than the pitch/yaw gain.
const float kRollErrorSizeRad[3] = {rad(0.2f), rad(3.0f), rad(30.0f)};
const float kTiltErrorSizeRad[3] = {rad(0.04f), rad(0.6f), rad(6.0f)};
const float kRateSizeRadS[3] = {rad(0.2f), rad(3.0f), rad(30.0f)};
const float kCanardSizeRad[3] = {rad(0.15f), rad(2.0f), rad(7.5f)};

// A speed chosen to hit every part of the table: anywhere from well below to
// well above it, exactly on a row, one float either side of a row, and far
// outside (including zero and negative).
float randomSpeed(Lcg& rng, const float (*table)[7], int rows) {
  const float first = table[0][0];
  const float last = table[rows - 1][0];
  const float span = last - first;
  const float onRow = table[rng.pick(rows)][0];
  switch (rng.pick(10)) {
    case 0:
    case 1:
      return onRow;
    case 2:
      return std::nextafter(onRow, rng.sign() * kInf);
    case 3: {
      const float farOut[5] = {0.0f, -25.0f, -1.0e6f, 1.0e3f, 1.0e6f};
      return farOut[rng.pick(5)];
    }
    default:
      return first - 0.5f * span + 2.0f * span * rng.unit();
  }
}

// Every component is drawn separately, so pitch, yaw and roll always differ in
// size and sign. That matters: an input with equal components would pass a
// port that mixed the axes up.
AttitudeControlInput randomInput(Lcg& rng, const float (*table)[7], int rows, int kind) {
  AttitudeControlInput in;
  if (kind == 0) {
    // General mix: each group of inputs picks its own size.
    const int rollSize = rng.pick(3);
    const int tiltSize = rng.pick(3);
    in.attitudeError = {rng.symmetric() * kRollErrorSizeRad[rollSize],
                        rng.symmetric() * kTiltErrorSizeRad[tiltSize],
                        rng.symmetric() * kTiltErrorSizeRad[tiltSize]};
    const int rollRateSize = rng.pick(3);
    const int tiltRateSize = rng.pick(3);
    in.bodyRate = {rng.symmetric() * kRateSizeRadS[rollRateSize],
                   rng.symmetric() * kRateSizeRadS[tiltRateSize],
                   rng.symmetric() * kRateSizeRadS[tiltRateSize]};
    const int canardSize = rng.pick(3);
    for (int i = 0; i < 4; ++i) in.canardAngle[i] = rng.symmetric() * kCanardSizeRad[canardSize];
  } else if (kind == 1) {
    // Roll error beyond its 10 deg limit, with a roll rate pushing the other
    // way hard enough that roll does not saturate. Only here does the
    // roll-error limit change the answer.
    const float side = rng.sign();
    in.attitudeError = {side * rad(10.0f + 20.0f * rng.unit()),
                        rng.symmetric() * kTiltErrorSizeRad[0],
                        rng.symmetric() * kTiltErrorSizeRad[0]};
    in.bodyRate = {-side * rad(35.0f + 60.0f * rng.unit()), rng.symmetric() * kRateSizeRadS[0],
                   rng.symmetric() * kRateSizeRadS[0]};
    for (int i = 0; i < 4; ++i) in.canardAngle[i] = rng.symmetric() * kCanardSizeRad[0];
  } else {
    // Everything small: the unsaturated, linear region.
    in.attitudeError = {rng.symmetric() * kRollErrorSizeRad[0],
                        rng.symmetric() * kTiltErrorSizeRad[0],
                        rng.symmetric() * kTiltErrorSizeRad[0]};
    in.bodyRate = {rng.symmetric() * kRateSizeRadS[0], rng.symmetric() * kRateSizeRadS[0],
                   rng.symmetric() * kRateSizeRadS[0]};
    for (int i = 0; i < 4; ++i) in.canardAngle[i] = rng.symmetric() * kCanardSizeRad[0];
  }
  in.speed = randomSpeed(rng, table, rows);
  return in;
}

constexpr int kRandomCases = 30000;

// Hand-picked values, all combined with each other. Errors and rates are
// (roll, pitch, yaw) and are deliberately lopsided.
void sweepHandPicked(const OldBuild& old, LqrAttitudeController& port, Coverage& coverage) {
  const float speeds[] = {0.0f,  5.0f,   13.0f,  29.99f,  30.0f,  35.0f,  40.0f,  50.0f, 60.0f,
                          80.0f, 100.0f, 120.0f, 130.0f, 130.01f, 200.0f, 300.0f, -25.0f};
  const float errorsDeg[][3] = {
      {0.0f, 0.0f, 0.0f},     {0.057f, 0.0f, 0.0f},    {0.0f, 0.057f, 0.0f},  {0.0f, 0.0f, 0.057f},
      {1.0f, 2.0f, -1.0f},    {8.0f, 2.0f, -1.0f},     {0.0f, 2.0f, -1.0f},   {-0.5f, 0.2f, 0.35f},
      {12.0f, -0.1f, 0.25f},  {-25.0f, 3.0f, -7.0f},   {30.0f, -30.0f, 15.0f}, {-9.9f, 0.01f, -0.02f},
      {10.1f, -0.03f, 0.01f}, {0.3f, -0.2f, 0.1f},
  };
  const float ratesRadS[][3] = {
      {0.0f, 0.0f, 0.0f},  {0.01f, -0.02f, 0.005f}, {-0.5f, 0.3f, 0.1f},     {5.0f, -2.0f, 1.0f},
      {-1.0f, 0.0f, 0.0f}, {1.1f, 0.02f, -0.01f},   {0.0f, 0.004f, -0.007f},
  };
  // Canard angles: none; the roll, pitch and yaw patterns; the pattern the
  // law neither sees nor commands; single canards; and a lopsided set.
  const float canardsDeg[][4] = {
      {0.0f, 0.0f, 0.0f, 0.0f},  {3.0f, 3.0f, 3.0f, 3.0f},   {-2.0f, 0.0f, 2.0f, 0.0f},
      {0.0f, 5.0f, 0.0f, -5.0f}, {7.5f, -7.5f, 7.5f, -7.5f}, {7.5f, 0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, -4.0f}, {1.0f, -2.0f, 3.5f, -0.5f},
  };
  for (const float speed : speeds) {
    for (const auto& e : errorsDeg) {
      for (const auto& w : ratesRadS) {
        for (const auto& c : canardsDeg) {
          const AttitudeControlInput in =
              makeInput({rad(e[0]), rad(e[1]), rad(e[2])}, {w[0], w[1], w[2]}, rad(c[0]), rad(c[1]),
                        rad(c[2]), rad(c[3]), speed);
          compareOnce(old, port, in, coverage);
        }
      }
    }
  }

  // Finite but absurd values that do not overflow: the saturation block has to
  // cope with a pitch demand of 1e31 rad.
  compareOnce(old, port, makeInput({1.0e30f, -1.0e30f, 3.0e29f}, {}, 0.0f, 0.0f, 0.0f, 0.0f, 80.0f),
              coverage);
  compareOnce(old, port,
              makeInput({rad(0.1f), 2.0e30f, -1.0e30f}, {-1.0e20f, 1.0e25f, 1.0e24f}, 0.0f, 0.0f, 0.0f,
                        0.0f, 45.0f),
              coverage);
  compareOnce(old, port,
              makeInput({}, {}, 1.0e30f, -2.0e30f, 3.0e30f, -1.0e29f, std::numeric_limits<float>::max()),
              coverage);
}

void sweepRandom(const OldBuild& old, LqrAttitudeController& port, uint64_t seed,
                 Coverage& coverage) {
  Lcg rng{seed};
  for (int i = 0; i < kRandomCases; ++i) {
    // 3 in 5 general, 1 in 5 roll-clamp, 1 in 5 small.
    const int slot = i % 5;
    const int kind = slot < 3 ? 0 : (slot == 3 ? 1 : 2);
    compareOnce(old, port, randomInput(rng, old.table, old.rows, kind), coverage);
  }
}

void checkEquivalence(const OldBuild& old) {
  // The port is given the old code's own table and the old code's own limit,
  // so this test does not move when a FlightConfig placeholder changes.
  LqrAttitudeController port(old.table, old.rows, old.canardLimitRad);
  TEST_ASSERT_TRUE(port.valid());

  // The roll-error limit is the one fixed number the port carries itself.
  TEST_ASSERT_FLOAT_WITHIN(1e-7f, old.rollErrorLimitRad, LqrAttitudeController::kRollErrorLimitRad);

  Coverage coverage;
  sweepHandPicked(old, port, coverage);
  sweepRandom(old, port, 20261120ull, coverage);

  char info[320];
  std::snprintf(info, sizeof(info),
                "[%s] %d cases: %d unsaturated, %d pitch/yaw scaled, %d roll takes all, "
                "%d roll clamp live; speed %d below / %d inside / %d above; worst difference %.3g rad",
                old.name, coverage.cases, coverage.unsaturated, coverage.pitchYawScaled,
                coverage.rollTakesAll, coverage.rollClampLive, coverage.speedBelow,
                coverage.speedInside, coverage.speedAbove, double(coverage.worstDifference));
  TEST_MESSAGE(info);

  // The sweep must really reach every regime. Saturation is the normal
  // operating state, so it gets the most cases, but the linear region and the
  // roll-error limit must be well covered too.
  TEST_ASSERT_GREATER_OR_EQUAL(30000, coverage.cases);
  TEST_ASSERT_GREATER_OR_EQUAL(coverage.cases / 10, coverage.unsaturated);
  // A fifth, not a quarter: the made-up two-row table reaches about 24 %.
  TEST_ASSERT_GREATER_OR_EQUAL(coverage.cases / 5, coverage.pitchYawScaled);
  TEST_ASSERT_GREATER_OR_EQUAL(coverage.cases / 10, coverage.rollTakesAll);
  TEST_ASSERT_GREATER_OR_EQUAL(1000, coverage.rollClampLive);
  TEST_ASSERT_GREATER_OR_EQUAL(coverage.cases / 10, coverage.speedBelow);
  TEST_ASSERT_GREATER_OR_EQUAL(coverage.cases / 4, coverage.speedInside);
  TEST_ASSERT_GREATER_OR_EQUAL(coverage.cases / 10, coverage.speedAbove);
}

// ---------------------------------------------------------------------------
// Equivalence with the old controller (design doc 8.1, the required test)
// ---------------------------------------------------------------------------

void test_port_matches_old_controller_as_shipped() { checkEquivalence(oldAsShipped()); }

void test_port_matches_old_controller_on_the_reference_table() {
  checkEquivalence(oldReferenceTable());
}

void test_port_matches_old_controller_on_a_two_row_table() {
  TEST_ASSERT_EQUAL_INT(2, oldTwoRowTable().rows);
  checkEquivalence(oldTwoRowTable());
}

// The sweep is only worth something if a wrong axis mapping would fail it.
// This feeds the port deliberately wrong mappings of one lopsided, unsaturated
// input and requires each to be far outside the equivalence tolerance.
void test_equivalence_probe_detects_a_wrong_axis_mapping() {
  const OldBuild& old = oldReferenceTable();
  LqrAttitudeController port(old.table, old.rows, old.canardLimitRad);

  const AttitudeControlInput right = makeInput({rad(0.6f), rad(-0.1f), rad(0.05f)},
                                               {0.004f, 0.002f, -0.001f}, rad(0.3f), rad(-0.2f),
                                               rad(0.1f), rad(0.25f), 70.0f);
  float oldCommand[4];
  runOld(old, right, oldCommand);
  TEST_ASSERT_TRUE(largestMagnitude(oldCommand) < 0.9f * old.canardLimitRad);  // unsaturated

  float command[4];
  TEST_ASSERT_TRUE(port.compute(right, command));
  TEST_ASSERT_TRUE(largestDifference(oldCommand, command) <= kEquivalenceTolRad);

  const float e[3] = {right.attitudeError.x, right.attitudeError.y, right.attitudeError.z};
  const float w[3] = {right.bodyRate.x, right.bodyRate.y, right.bodyRate.z};
  constexpr float kClearlyDifferentRad = 1e-3f;

  // The five wrong orderings of the three axes.
  const int orderings[5][3] = {{0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  for (const auto& order : orderings) {
    AttitudeControlInput wrong = right;
    wrong.attitudeError = {e[order[0]], e[order[1]], e[order[2]]};
    wrong.bodyRate = {w[order[0]], w[order[1]], w[order[2]]};
    TEST_ASSERT_TRUE(port.compute(wrong, command));
    TEST_ASSERT_TRUE_MESSAGE(largestDifference(oldCommand, command) > kClearlyDifferentRad,
                             "a wrong axis order went unnoticed");
  }

  // One axis with the wrong sign.
  for (int axis = 0; axis < 3; ++axis) {
    const float flip[3] = {axis == 0 ? -1.0f : 1.0f, axis == 1 ? -1.0f : 1.0f,
                           axis == 2 ? -1.0f : 1.0f};
    AttitudeControlInput wrong = right;
    wrong.attitudeError = {flip[0] * e[0], flip[1] * e[1], flip[2] * e[2]};
    wrong.bodyRate = {flip[0] * w[0], flip[1] * w[1], flip[2] * w[2]};
    TEST_ASSERT_TRUE(port.compute(wrong, command));
    TEST_ASSERT_TRUE_MESSAGE(largestDifference(oldCommand, command) > kClearlyDifferentRad,
                             "a flipped axis went unnoticed");
  }

  // Canard angles shifted round by one place.
  AttitudeControlInput shifted = right;
  for (int i = 0; i < 4; ++i) shifted.canardAngle[i] = right.canardAngle[(i + 1) % 4];
  TEST_ASSERT_TRUE(port.compute(shifted, command));
  TEST_ASSERT_TRUE_MESSAGE(largestDifference(oldCommand, command) > kClearlyDifferentRad,
                           "shifted canard angles went unnoticed");

  // Commands shifted round by one place.
  TEST_ASSERT_TRUE(port.compute(right, command));
  const float shiftedCommand[4] = {command[1], command[2], command[3], command[0]};
  TEST_ASSERT_TRUE(largestDifference(oldCommand, shiftedCommand) > kClearlyDifferentRad);
}

// ---------------------------------------------------------------------------
// The frame relation, checked from quaternions
// ---------------------------------------------------------------------------

// Turns team-frame body components into old-frame body components:
//   v_old = kOldFromTeam v_team kOldFromTeam*
// so an attitude (body -> NED) converts as  q_team = q_old (x) kOldFromTeam.
// It is a third of a turn about the body diagonal.
constexpr Quat kOldFromTeam{0.5f, -0.5f, -0.5f, -0.5f};

void assertVecNear(const Vec3& expected, const Vec3& got, float tol) {
  TEST_ASSERT_FLOAT_WITHIN(tol, expected.x, got.x);
  TEST_ASSERT_FLOAT_WITHIN(tol, expected.y, got.y);
  TEST_ASSERT_FLOAT_WITHIN(tol, expected.z, got.z);
}

Quat randomAttitude(Lcg& rng) {
  for (;;) {
    const Quat q{rng.symmetric(), rng.symmetric(), rng.symmetric(), rng.symmetric()};
    const float n = norm(q);
    if (n > 0.1f && n <= 1.0f) return normalized(q);
  }
}

// The relation the end-to-end test rests on, checked on its own first.
void test_quaternion_relation_between_old_and_team_frames() {
  TEST_ASSERT_TRUE(isUnit(kOldFromTeam, 1e-6f));

  // Team x (nose) is old z; team y (pitch) is old x; team z (yaw) is old y.
  assertVecNear({0.0f, 0.0f, 1.0f}, rotate(kOldFromTeam, {1.0f, 0.0f, 0.0f}), 1e-6f);
  assertVecNear({1.0f, 0.0f, 0.0f}, rotate(kOldFromTeam, {0.0f, 1.0f, 0.0f}), 1e-6f);
  assertVecNear({0.0f, 1.0f, 0.0f}, rotate(kOldFromTeam, {0.0f, 0.0f, 1.0f}), 1e-6f);

  // The old code's nose-up attitude (nose = old z, pointing up in NED) must
  // come out nose-up in the team frame.
  const Quat oldNoseUp{0.0f, 1.0f, 0.0f, 0.0f};
  assertVecNear(kUpNed, rotate(oldNoseUp, {0.0f, 0.0f, 1.0f}), 1e-6f);
  const Quat teamNoseUp = oldNoseUp * kOldFromTeam;
  assertVecNear(kUpNed, noseNed(teamNoseUp), 1e-6f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, tilt(teamNoseUp));

  // For any attitude and any body vector: the same physical vector, written
  // in each frame's components, lands on the same NED vector.
  Lcg rng{7ull};
  for (int i = 0; i < 500; ++i) {
    const Quat qOld = randomAttitude(rng);
    const Quat qTeam = normalized(qOld * kOldFromTeam);
    const Vec3 vTeam{rng.symmetric(), rng.symmetric(), rng.symmetric()};
    const Vec3 vOld{vTeam.y, vTeam.z, vTeam.x};
    assertVecNear(rotate(qOld, vOld), rotate(qTeam, vTeam), 2e-6f);
  }
}

// Log(conj(qRef) (x) q) in double precision, written here from scratch so the
// old side of the end-to-end test does not lean on lib/Frames.
void rotationVectorErrorDouble(const Quat& qRef, const Quat& q, double out[3]) {
  double a[4] = {qRef.w, -double(qRef.x), -double(qRef.y), -double(qRef.z)};
  double b[4] = {q.w, q.x, q.y, q.z};
  const double na = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
  const double nb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + b[3] * b[3]);
  for (int i = 0; i < 4; ++i) {
    a[i] /= na;
    b[i] /= nb;
  }
  double p[4] = {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                 a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                 a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                 a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
  if (p[0] < 0.0) {
    for (int i = 0; i < 4; ++i) p[i] = -p[i];
  }
  const double s = std::sqrt(p[1] * p[1] + p[2] * p[2] + p[3] * p[3]);
  const double factor = s > 1e-12 ? 2.0 * std::atan2(s, p[0]) / s : 2.0;
  for (int i = 0; i < 3; ++i) out[i] = factor * p[i + 1];
}

// End to end from attitudes. Old side: old-frame quaternions, an error worked
// out here in double precision, the old law. New side: the same attitudes
// converted to the team frame, lib/Frames attitudeError(), the port. Nothing
// on the new side is told which axis is which: it follows from the frames.
void test_port_matches_old_controller_starting_from_quaternions() {
  const OldBuild& old = oldReferenceTable();
  LqrAttitudeController port(old.table, old.rows, old.canardLimitRad);
  TEST_ASSERT_TRUE(port.valid());

  // Looser than the core test: the two sides reach the error by different
  // arithmetic (float against double), and the pitch/yaw gain is about 10.
  constexpr float kTolRad = 1e-5f;
  const float errorSizeRad[3] = {rad(0.05f), rad(1.0f), rad(15.0f)};
  const float rateSizeRadS[3] = {0.002f, 0.05f, 1.0f};
  const float canardSizeRad[3] = {rad(0.1f), rad(2.0f), rad(7.5f)};

  Lcg rng{424242ull};
  int unsaturated = 0;
  int saturated = 0;
  constexpr int kCases = 8000;
  for (int i = 0; i < kCases; ++i) {
    // Old frame: [0] pitch, [1] yaw, [2] roll.
    const Quat qRefOld = randomAttitude(rng);
    const int tiltSize = rng.pick(3);
    const Vec3 offsetOld{rng.symmetric() * errorSizeRad[tiltSize],
                         rng.symmetric() * errorSizeRad[tiltSize],
                         rng.symmetric() * errorSizeRad[rng.pick(3)]};
    const Quat qOld = normalized(qRefOld * fromRotationVector(offsetOld));
    const int rateSize = rng.pick(3);
    const float rateOld[3] = {rng.symmetric() * rateSizeRadS[rateSize],
                              rng.symmetric() * rateSizeRadS[rateSize],
                              rng.symmetric() * rateSizeRadS[rateSize]};
    const int canardSize = rng.pick(3);
    float canard[4];
    for (float& c : canard) c = rng.symmetric() * canardSizeRad[canardSize];
    const float speed = 160.0f * rng.unit();

    // Old side.
    double errorDouble[3];
    rotationVectorErrorDouble(qRefOld, qOld, errorDouble);
    const float errorOld[3] = {static_cast<float>(errorDouble[0]), static_cast<float>(errorDouble[1]),
                               static_cast<float>(errorDouble[2])};
    float oldCommand[4];
    old.lqr(errorOld, rateOld, canard, speed, oldCommand);

    // New side.
    const Quat qRefTeam = qRefOld * kOldFromTeam;
    const Quat qTeam = qOld * kOldFromTeam;
    AttitudeControlInput in;
    in.attitudeError = attitudeError(qRefTeam, qTeam);
    in.bodyRate = {rateOld[2], rateOld[0], rateOld[1]};
    for (int c = 0; c < 4; ++c) in.canardAngle[c] = canard[c];
    in.speed = speed;
    float newCommand[4];
    TEST_ASSERT_TRUE(port.compute(in, newCommand));

    if (!(largestDifference(oldCommand, newCommand) <= kTolRad)) {
      failCase("old and new disagree from quaternions", old.name, in, oldCommand, newCommand);
    }
    if (largestMagnitude(oldCommand) < 0.99f * old.canardLimitRad) {
      ++unsaturated;
    } else {
      ++saturated;
    }
  }
  // Both regimes must be present, or the comparison proves less than it seems.
  TEST_ASSERT_GREATER_OR_EQUAL(kCases / 50, unsaturated);
  TEST_ASSERT_GREATER_OR_EQUAL(kCases / 2, saturated);
}

// ---------------------------------------------------------------------------
// The old host tests (Voyager-launch-controller test/test_controller.cpp),
// moved to the team frame. All use the reference table and the old 4 deg
// limit, which is what their expected numbers assume.
// ---------------------------------------------------------------------------

LqrAttitudeController referencePort() {
  const OldBuild& old = oldReferenceTable();
  return LqrAttitudeController(old.table, old.rows, old.canardLimitRad);
}

// The whole 4 x 10 gain matrix at 80 m/s, one state nudged at a time:
// command = -K z with z = [e_roll, e_pitch, e_yaw, w_roll, w_pitch, w_yaw,
// canard 0..3]. The old test checked the rows of canards 0 and 2 only; the
// rows of canards 1 and 3 are added, since they are the ones that carry yaw.
void test_gain_matrix_at_80_m_s() {
  LqrAttitudeController port = referencePort();
  const float gainMatrix[4][10] = {
      {1.2439f, -9.662f, 0.0f, 0.2018f, -3.511f, 0.0f, 0.6202f, 0.3971f, 0.1740f, 0.3971f},
      {1.2439f, 0.0f, -9.662f, 0.2018f, 0.0f, -3.511f, 0.3971f, 0.6202f, 0.3971f, 0.1740f},
      {1.2439f, 9.662f, 0.0f, 0.2018f, 3.511f, 0.0f, 0.1740f, 0.3971f, 0.6202f, 0.3971f},
      {1.2439f, 0.0f, 9.662f, 0.2018f, 0.0f, 3.511f, 0.3971f, 0.1740f, 0.3971f, 0.6202f},
  };
  constexpr float kNudge = 1e-3f;
  for (int state = 0; state < 10; ++state) {
    float z[10] = {};
    z[state] = kNudge;
    const AttitudeControlInput in =
        makeInput({z[0], z[1], z[2]}, {z[3], z[4], z[5]}, z[6], z[7], z[8], z[9], 80.0f);
    float command[4];
    TEST_ASSERT_TRUE(port.compute(in, command));
    for (int canard = 0; canard < 4; ++canard) {
      TEST_ASSERT_FLOAT_WITHIN(1e-3f, gainMatrix[canard][state], -command[canard] / kNudge);
    }
  }
}

// Seen through a pure pitch-rate input: canard 0 = +k_rate * pitch rate.
void test_gain_interpolation_and_end_rows_held() {
  LqrAttitudeController port = referencePort();
  constexpr float kNudge = 1e-3f;
  AttitudeControlInput in = makeInput({}, {0.0f, kNudge, 0.0f}, 0.0f, 0.0f, 0.0f, 0.0f, 50.0f);
  float command[4];

  TEST_ASSERT_TRUE(port.compute(in, command));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f * (4.85f + 3.92f), command[0] / kNudge);  // half way, 40 to 60

  in.speed = 45.0f;  // a quarter of the way, 40 to 60
  TEST_ASSERT_TRUE(port.compute(in, command));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.85f + 0.25f * (3.92f - 4.85f), command[0] / kNudge);

  in.speed = 125.0f;  // half way along the last, shorter, pair of rows
  TEST_ASSERT_TRUE(port.compute(in, command));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f * (3.10f + 3.03f), command[0] / kNudge);

  in.speed = 60.0f;  // exactly on a row
  TEST_ASSERT_TRUE(port.compute(in, command));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.92f, command[0] / kNudge);

  in.speed = 5.0f;  // below the table: first row held
  TEST_ASSERT_TRUE(port.compute(in, command));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.88f, command[0] / kNudge);

  in.speed = -40.0f;  // a negative speed is still "below the table"
  TEST_ASSERT_TRUE(port.compute(in, command));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.88f, command[0] / kNudge);

  in.speed = 300.0f;  // above the table: last row held
  TEST_ASSERT_TRUE(port.compute(in, command));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.03f, command[0] / kNudge);
}

// Roll asks for less than the limit: it is delivered in full, and pitch and
// yaw are scaled by one factor into what is left.
void test_roll_delivered_in_full_and_pitch_yaw_scaled_into_the_rest() {
  LqrAttitudeController port = referencePort();
  const float limit = port.canardLimitRad();
  const AttitudeControlInput in = errorOnly(1.0f, 2.0f, -1.0f, 80.0f);
  float c[4];
  TEST_ASSERT_TRUE(port.compute(in, c));

  const float vRoll = -1.24385f * in.attitudeError.x;
  const float vPitch = -9.66241f * in.attitudeError.y;
  const float vYaw = -9.66241f * in.attitudeError.z;
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, vRoll, 0.25f * (c[0] + c[1] + c[2] + c[3]));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, vPitch / vYaw, (c[2] - c[0]) / (c[3] - c[1]));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, vPitch * (limit - std::fabs(vRoll)) / std::fabs(vPitch),
                           0.5f * (c[2] - c[0]));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, deg(limit), deg(largestMagnitude(c)));

  // The four numbers themselves, worked out by hand for the porting brief.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.5123f, deg(c[0]));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -2.6219f, deg(c[1]));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -4.0000f, deg(c[2]));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.1342f, deg(c[3]));
}

// Roll asks for more than the limit: all four go to the limit in roll, and
// pitch and yaw get nothing.
void test_roll_beyond_the_limit_takes_all_of_it() {
  LqrAttitudeController port = referencePort();
  float c[4];
  TEST_ASSERT_TRUE(port.compute(errorOnly(8.0f, 2.0f, -1.0f, 80.0f), c));
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -deg(port.canardLimitRad()), deg(c[i]));
  }
}

// Pitch and yaw alone: scaled so the largest canard is at the limit, with no
// roll part.
void test_pitch_and_yaw_alone_scale_to_the_limit() {
  LqrAttitudeController port = referencePort();
  const float limitDeg = deg(port.canardLimitRad());
  float c[4];
  TEST_ASSERT_TRUE(port.compute(errorOnly(0.0f, 2.0f, -1.0f, 80.0f), c));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, c[0] + c[1] + c[2] + c[3]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, limitDeg, deg(c[0]));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.5f * limitDeg, deg(c[1]));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -limitDeg, deg(c[2]));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f * limitDeg, deg(c[3]));
}

// The roll-error limit: an error beyond 10 deg behaves as 10 deg.
// The old test of this could not fail (at its settings the output saturated
// with or without the limit). Here a roll rate pushing the other way keeps
// the roll command off the output limit, so the 10 deg limit is the only
// thing that can make 25 deg and 10 deg agree.
void test_roll_error_beyond_10_deg_behaves_as_10_deg() {
  LqrAttitudeController port = referencePort();
  const Vec3 opposingRate{rad(-50.0f), 0.0f, 0.0f};
  float at25[4], at12[4], at10[4], at9[4];
  TEST_ASSERT_TRUE(port.compute(makeInput({rad(25.0f), 0.0f, 0.0f}, opposingRate, 0, 0, 0, 0, 80.0f), at25));
  TEST_ASSERT_TRUE(port.compute(makeInput({rad(12.0f), 0.0f, 0.0f}, opposingRate, 0, 0, 0, 0, 80.0f), at12));
  TEST_ASSERT_TRUE(port.compute(makeInput({rad(10.0f), 0.0f, 0.0f}, opposingRate, 0, 0, 0, 0, 80.0f), at10));
  TEST_ASSERT_TRUE(port.compute(makeInput({rad(9.0f), 0.0f, 0.0f}, opposingRate, 0, 0, 0, 0, 80.0f), at9));

  // -(1.24385 * 10 - 0.20184 * 50) = -2.3465 deg, well inside the 4 deg limit.
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -2.3465f, deg(at25[i]));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, deg(at10[i]), deg(at25[i]));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, deg(at10[i]), deg(at12[i]));
    // Below 10 deg the error is used as it is: -(1.24385 * 9 - 10.092) = -1.1027 deg.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -1.1027f, deg(at9[i]));
  }

  // The same on the other side.
  float negative[4];
  TEST_ASSERT_TRUE(port.compute(makeInput({rad(-25.0f), 0.0f, 0.0f}, {rad(50.0f), 0.0f, 0.0f}, 0, 0, 0, 0, 80.0f),
                                negative));
  for (int i = 0; i < 4; ++i) TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.3465f, deg(negative[i]));

  // Pitch and yaw errors are NOT limited this way: 25 deg and 10 deg of pitch
  // error with an opposing pitch rate give different commands.
  float pitch25[4], pitch10[4];
  const Vec3 pitchRate{0.0f, rad(-30.0f), 0.0f};
  TEST_ASSERT_TRUE(port.compute(makeInput({0.0f, rad(11.0f), 0.0f}, pitchRate, 0, 0, 0, 0, 80.0f), pitch25));
  TEST_ASSERT_TRUE(port.compute(makeInput({0.0f, rad(10.9f), 0.0f}, pitchRate, 0, 0, 0, 0, 80.0f), pitch10));
  // -(9.66241 * 11 - 3.51075 * 30) = -0.964 deg; at 10.9 deg, +0.002 deg.
  TEST_ASSERT_FLOAT_WITHIN(2e-2f, 0.964f, deg(pitch25[0]));
  TEST_ASSERT_FLOAT_WITHIN(2e-2f, -0.002f, deg(pitch10[0]));
}

// With the old servo model in the loop, a held 1 deg roll error settles at
// -k_angle / (1 + k_servo) deg on every canard.
void test_held_roll_error_settles_with_the_old_servo_model() {
  const OldBuild& old = oldReferenceTable();
  LqrAttitudeController port = referencePort();
  AttitudeControlInput in = errorOnly(1.0f, 0.0f, 0.0f, 80.0f);
  float command[4] = {};
  for (int step = 0; step < 400; ++step) {
    TEST_ASSERT_TRUE(port.compute(in, command));
    old.servoModelStep(in.canardAngle, command);
  }
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.24385f / 2.58833f, deg(command[i]));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, command[i], in.canardAngle[i]);
  }
}

// ---------------------------------------------------------------------------
// Physical signs in the team frame
// ---------------------------------------------------------------------------

// Net moment of a set of canard deflections about the centre of mass, up to a
// positive factor. The geometry is the one AttitudeController.h states:
// canard i sits a quarter turn * i from body +y towards body +z, and a
// positive deflection pushes it sideways so that all four positive roll the
// rocket about +x. That makes canards 0..3 push along +z, -y, -z, +y. The
// canards are ahead of the centre of mass.
Vec3 canardMoment(const float deflection[4]) {
  const float cosQuarter[4] = {1.0f, 0.0f, -1.0f, 0.0f};
  const float sinQuarter[4] = {0.0f, 1.0f, 0.0f, -1.0f};
  constexpr float kAheadOfCentreOfMass = 1.0f;  // any positive length
  constexpr float kRadius = 0.1f;               // any positive length
  Vec3 moment;
  for (int i = 0; i < 4; ++i) {
    const Vec3 position{kAheadOfCentreOfMass, kRadius * cosQuarter[i], kRadius * sinQuarter[i]};
    const Vec3 push{0.0f, -sinQuarter[i], cosQuarter[i]};
    moment = moment + cross(position, push) * deflection[i];
  }
  return moment;
}

// The moment model itself, so the tests below stand on something checked.
void test_canard_moment_model_matches_the_stated_geometry() {
  const float rollPattern[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  const float pitchPattern[4] = {-1.0f, 0.0f, 1.0f, 0.0f};
  const float yawPattern[4] = {0.0f, -1.0f, 0.0f, 1.0f};
  const Vec3 roll = canardMoment(rollPattern);
  const Vec3 pitch = canardMoment(pitchPattern);
  const Vec3 yaw = canardMoment(yawPattern);
  // All four positive roll about +x, and nothing else.
  TEST_ASSERT_TRUE(roll.x > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, roll.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, roll.z);
  // The pitch pattern turns the rocket about +y only, the yaw pattern about +z only.
  TEST_ASSERT_TRUE(pitch.y > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pitch.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pitch.z);
  TEST_ASSERT_TRUE(yaw.z > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, yaw.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, yaw.y);
}

// A reference attitude that is nose-up with some roll, so nothing lines up
// with the NED axes by luck.
Quat noseUpReference() {
  return verticalReference(normalized(Quat{0.3f, 0.5f, -0.4f, 0.7f}));
}

struct TiltCase {
  Vec3 axis;       // body axis the rocket is turned about
  float angleDeg;  // signed
};

// Runs the port on "the rocket is turned by `angle` about body `axis` away
// from the reference" and returns the commands. Also hands back where the
// nose went, in the reference attitude's body axes.
void commandsForTurn(LqrAttitudeController& port, const TiltCase& turn, float speed, float command[4],
                     Vec3& noseInReferenceAxes) {
  const Quat reference = noseUpReference();
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, tilt(reference));
  const Quat attitude = reference * fromAxisAngle(turn.axis, rad(turn.angleDeg));
  noseInReferenceAxes = rotateInverse(reference, noseNed(attitude));

  AttitudeControlInput in;
  in.attitudeError = attitudeError(reference, attitude);
  in.speed = speed;
  TEST_ASSERT_TRUE(port.compute(in, command));
}

// Small turns stay in the linear region; the large ones saturate. The signs
// must be the same in both.
const float kTurnSizesDeg[] = {0.05f, 0.3f, 5.0f, 25.0f};
constexpr float kNothingRad = 2e-5f;  // "this canard does not move"

// Nose tilted towards body +y. That is a positive turn about +z, so yaw must
// push back: canard 1 (on +z) positive, canard 3 (on -z) negative, and the
// pitch pair (canards 0 and 2) left alone.
void test_nose_tilted_towards_plus_y_is_pushed_back() {
  LqrAttitudeController port = referencePort();
  for (const float size : kTurnSizesDeg) {
    float c[4];
    Vec3 nose;
    commandsForTurn(port, {{0.0f, 0.0f, 1.0f}, size}, 80.0f, c, nose);
    TEST_ASSERT_TRUE_MESSAGE(nose.y > 0.0f && std::fabs(nose.z) < 1e-5f, "test set-up: nose not towards +y");
    TEST_ASSERT_TRUE(c[1] > 0.0f);
    TEST_ASSERT_TRUE(c[3] < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -c[1], c[3]);
    TEST_ASSERT_FLOAT_WITHIN(kNothingRad, 0.0f, c[0]);
    TEST_ASSERT_FLOAT_WITHIN(kNothingRad, 0.0f, c[2]);
    // The moment turns the rocket back: about -z.
    TEST_ASSERT_TRUE(canardMoment(c).z < 0.0f);
  }
}

// Nose tilted towards body +z. That is a NEGATIVE turn about +y, so pitch must
// push back: canard 0 (on +y) negative, canard 2 (on -y) positive, and the
// yaw pair left alone.
void test_nose_tilted_towards_plus_z_is_pushed_back() {
  LqrAttitudeController port = referencePort();
  for (const float size : kTurnSizesDeg) {
    float c[4];
    Vec3 nose;
    commandsForTurn(port, {{0.0f, 1.0f, 0.0f}, -size}, 80.0f, c, nose);
    TEST_ASSERT_TRUE_MESSAGE(nose.z > 0.0f && std::fabs(nose.y) < 1e-5f, "test set-up: nose not towards +z");
    TEST_ASSERT_TRUE(c[0] < 0.0f);
    TEST_ASSERT_TRUE(c[2] > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -c[0], c[2]);
    TEST_ASSERT_FLOAT_WITHIN(kNothingRad, 0.0f, c[1]);
    TEST_ASSERT_FLOAT_WITHIN(kNothingRad, 0.0f, c[3]);
    // The moment turns the rocket back: about +y.
    TEST_ASSERT_TRUE(canardMoment(c).y > 0.0f);
  }
}

// A positive roll error (rolled too far about the nose) gives all four
// canards the same negative angle.
void test_positive_roll_error_gives_all_four_negative() {
  LqrAttitudeController port = referencePort();
  for (const float size : kTurnSizesDeg) {
    float c[4];
    Vec3 nose;
    commandsForTurn(port, {{1.0f, 0.0f, 0.0f}, size}, 80.0f, c, nose);
    for (int i = 0; i < 4; ++i) {
      TEST_ASSERT_TRUE(c[i] < 0.0f);
      TEST_ASSERT_FLOAT_WITHIN(kNothingRad, c[0], c[i]);
    }
    TEST_ASSERT_TRUE(canardMoment(c).x < 0.0f);

    // And the mirror image.
    commandsForTurn(port, {{1.0f, 0.0f, 0.0f}, -size}, 80.0f, c, nose);
    for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(c[i] > 0.0f);
  }
}

// Negative feedback on every axis, for errors and for rates, at every speed
// in the table: the canard moment always opposes what it responds to, and
// has no part about the other two axes.
void test_every_axis_is_negative_feedback_at_every_table_speed() {
  const OldBuild& old = oldReferenceTable();
  LqrAttitudeController port = referencePort();
  const Vec3 axes[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
  for (int row = 0; row < old.rows; ++row) {
    const float speed = old.table[row][0];
    for (int axis = 0; axis < 3; ++axis) {
      for (const float sign : {-1.0f, 1.0f}) {
        for (const bool useRate : {false, true}) {
          AttitudeControlInput in;
          const Vec3 disturbance = axes[axis] * (sign * 2e-3f);  // small: linear region
          if (useRate) {
            in.bodyRate = disturbance;
          } else {
            in.attitudeError = disturbance;
          }
          in.speed = speed;
          float c[4];
          TEST_ASSERT_TRUE(port.compute(in, c));
          const Vec3 moment = canardMoment(c);
          const float along = dot(moment, axes[axis]) * sign;
          TEST_ASSERT_TRUE_MESSAGE(along < 0.0f, "the canards push the same way as the disturbance");
          const Vec3 across = moment - axes[axis] * dot(moment, axes[axis]);
          TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, norm(across));
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Roll priority, at several limits
// ---------------------------------------------------------------------------

// Gains of the reference table's 80 m/s row, read from the table.
struct Gains80 {
  float rollAngle;
  float tiltAngle;
};

Gains80 referenceGainsAt80() {
  const OldBuild& old = oldReferenceTable();
  for (int row = 0; row < old.rows; ++row) {
    if (old.table[row][0] == 80.0f) return {old.table[row][1], old.table[row][4]};
  }
  TEST_FAIL_MESSAGE("the reference table has no 80 m/s row");
  return {1.0f, 1.0f};
}

void checkRollPriorityAtLimit(float limit) {
  const OldBuild& old = oldReferenceTable();
  LqrAttitudeController port(old.table, old.rows, limit);
  TEST_ASSERT_TRUE(port.valid());
  const Gains80 k = referenceGainsAt80();
  const float tol = 2e-5f * limit;
  float c[4];

  // 1. A roll demand beyond the limit leaves pitch and yaw no share at all,
  //    however large their errors: all four sit exactly on the limit.
  const float bigRoll = 1.25f * limit / k.rollAngle;
  TEST_ASSERT_TRUE_MESSAGE(bigRoll < LqrAttitudeController::kRollErrorLimitRad,
                           "test set-up: roll error would be clamped");
  TEST_ASSERT_TRUE(port.compute(makeInput({bigRoll, rad(2.0f), rad(-1.0f)}, {}, 0, 0, 0, 0, 80.0f), c));
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(c[i] == -limit);
  TEST_ASSERT_TRUE(port.compute(makeInput({-bigRoll, rad(2.0f), rad(-1.0f)}, {}, 0, 0, 0, 0, 80.0f), c));
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(c[i] == limit);

  // 2. A roll demand of a quarter of the limit is delivered in full, and
  //    pitch and yaw share the other three quarters in their own 2 : -1
  //    proportion. Pitch error +limit and yaw error -limit/2 both ask for far
  //    more than is left.
  const float smallRoll = 0.25f * limit / k.rollAngle;
  TEST_ASSERT_TRUE(port.compute(makeInput({smallRoll, limit, -0.5f * limit}, {}, 0, 0, 0, 0, 80.0f), c));
  TEST_ASSERT_FLOAT_WITHIN(tol, -0.25f * limit, 0.25f * (c[0] + c[1] + c[2] + c[3]));  // roll part
  TEST_ASSERT_FLOAT_WITHIN(tol, -0.75f * limit, 0.5f * (c[2] - c[0]));                 // pitch part
  TEST_ASSERT_FLOAT_WITHIN(tol, 0.375f * limit, 0.5f * (c[3] - c[1]));                 // yaw part
  TEST_ASSERT_FLOAT_WITHIN(tol, 0.5f * limit, c[0]);
  TEST_ASSERT_FLOAT_WITHIN(tol, -0.625f * limit, c[1]);
  TEST_ASSERT_FLOAT_WITHIN(tol, -limit, c[2]);
  TEST_ASSERT_FLOAT_WITHIN(tol, 0.125f * limit, c[3]);

  // 3. No roll demand: pitch and yaw get the whole limit.
  TEST_ASSERT_TRUE(port.compute(makeInput({0.0f, limit, -0.5f * limit}, {}, 0, 0, 0, 0, 80.0f), c));
  TEST_ASSERT_FLOAT_WITHIN(tol, limit, c[0]);
  TEST_ASSERT_FLOAT_WITHIN(tol, -0.5f * limit, c[1]);
  TEST_ASSERT_FLOAT_WITHIN(tol, -limit, c[2]);
  TEST_ASSERT_FLOAT_WITHIN(tol, 0.5f * limit, c[3]);

  // 4. Everything small: nothing is scaled. Roll asks for -0.1, pitch for
  //    -0.2 and yaw for +0.1 of the limit.
  TEST_ASSERT_TRUE(port.compute(makeInput({0.1f * limit / k.rollAngle, 0.2f * limit / k.tiltAngle,
                                           -0.1f * limit / k.tiltAngle},
                                          {}, 0, 0, 0, 0, 80.0f),
                                c));
  TEST_ASSERT_FLOAT_WITHIN(tol, 0.1f * limit, c[0]);
  TEST_ASSERT_FLOAT_WITHIN(tol, -0.2f * limit, c[1]);
  TEST_ASSERT_FLOAT_WITHIN(tol, -0.3f * limit, c[2]);
  TEST_ASSERT_FLOAT_WITHIN(tol, 0.0f, c[3]);
}

void test_roll_priority_at_the_flight_limit() { checkRollPriorityAtLimit(FlightConfig::kCanardLimitRad); }

void test_roll_priority_at_the_old_limit() { checkRollPriorityAtLimit(oldReferenceTable().canardLimitRad); }

// The limit really is the constructor's argument everywhere it is used: the
// same properties hold at half the flight limit and at the ground-test limit.
void test_roll_priority_follows_whatever_limit_is_given() {
  checkRollPriorityAtLimit(0.5f * FlightConfig::kCanardLimitRad);
  checkRollPriorityAtLimit(FlightConfig::kGroundTestLimitRad);
}

// ---------------------------------------------------------------------------
// Bad inputs
// ---------------------------------------------------------------------------

constexpr int kInputSlots = 11;

float& inputSlot(AttitudeControlInput& in, int slot) {
  switch (slot) {
    case 0: return in.attitudeError.x;
    case 1: return in.attitudeError.y;
    case 2: return in.attitudeError.z;
    case 3: return in.bodyRate.x;
    case 4: return in.bodyRate.y;
    case 5: return in.bodyRate.z;
    case 6: return in.canardAngle[0];
    case 7: return in.canardAngle[1];
    case 8: return in.canardAngle[2];
    case 9: return in.canardAngle[3];
    default: return in.speed;
  }
}

const char* const kInputSlotNames[kInputSlots] = {
    "roll error", "pitch error", "yaw error", "roll rate", "pitch rate", "yaw rate",
    "canard 0",   "canard 1",    "canard 2",  "canard 3",  "speed",
};

// A healthy, lopsided input that gives a non-zero command on every canard.
AttitudeControlInput healthyInput() {
  return makeInput({rad(0.5f), rad(-0.1f), rad(0.05f)}, {0.02f, -0.01f, 0.005f}, rad(0.5f), rad(-0.3f),
                   rad(0.2f), rad(0.1f), 70.0f);
}

void checkEveryNonFiniteInputIsRejected(LqrAttitudeController& controller) {
  const AttitudeControlInput good = healthyInput();
  float expected[4];
  TEST_ASSERT_TRUE(controller.compute(good, expected));
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(expected[i] != 0.0f);

  const float badValues[3] = {kNaN, kInf, -kInf};
  for (int slot = 0; slot < kInputSlots; ++slot) {
    for (const float bad : badValues) {
      AttitudeControlInput in = good;
      inputSlot(in, slot) = bad;
      float command[4] = {0.5f, -0.5f, 0.25f, -0.25f};
      const bool ok = controller.compute(in, command);
      TEST_ASSERT_FALSE_MESSAGE(ok, kInputSlotNames[slot]);
      for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE_MESSAGE(command[i] == 0.0f, kInputSlotNames[slot]);

      // Nothing sticks: the next healthy input gets the healthy answer.
      float after[4];
      TEST_ASSERT_TRUE(controller.compute(good, after));
      for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(after[i] == expected[i]);
    }
  }

  // Every input bad at once.
  AttitudeControlInput allBad = good;
  for (int slot = 0; slot < kInputSlots; ++slot) inputSlot(allBad, slot) = kNaN;
  assertRejected(controller, allBad);
}

void test_any_non_finite_input_gives_zero_and_false() {
  LqrAttitudeController reference = referencePort();
  checkEveryNonFiniteInputIsRejected(reference);

  LqrAttitudeController flight(FlightConfig::kCanardLimitRad);
  checkEveryNonFiniteInputIsRejected(flight);
}

// The port is deliberately stricter than the old code in two places. The old
// code only checked its three axis commands, so an infinite roll error was
// limited to 10 deg and flown, and an infinite speed picked an end row of the
// table and was flown. This test pins both facts: what the old code does, and
// that the port refuses.
void test_port_refuses_infinities_the_old_code_flew() {
  const OldBuild& old = oldReferenceTable();
  LqrAttitudeController port = referencePort();

  for (const float infinite : {kInf, -kInf}) {
    AttitudeControlInput rollError = healthyInput();
    rollError.attitudeError.x = infinite;
    float oldCommand[4];
    runOld(old, rollError, oldCommand);
    TEST_ASSERT_TRUE_MESSAGE(largestMagnitude(oldCommand) > 0.0f,
                             "the old code no longer flies an infinite roll error");
    assertRejected(port, rollError);

    AttitudeControlInput speed = healthyInput();
    speed.speed = infinite;
    runOld(old, speed, oldCommand);
    TEST_ASSERT_TRUE_MESSAGE(largestMagnitude(oldCommand) > 0.0f,
                             "the old code no longer flies an infinite speed");
    assertRejected(port, speed);
  }
}

// Finite inputs so large that the arithmetic overflows are refused too.
void test_overflow_from_finite_inputs_gives_zero_and_false() {
  LqrAttitudeController port = referencePort();
  const float huge = std::numeric_limits<float>::max();

  for (int slot = 0; slot < 6; ++slot) {  // each error and each rate: huge * gain overflows
    for (const float sign : {-1.0f, 1.0f}) {
      AttitudeControlInput in = healthyInput();
      inputSlot(in, slot) = sign * huge;
      // The roll error is limited to 10 deg before its gain, so a huge one
      // is simply a large error, not an overflow. The roll-rate gain is
      // below 1 on every row, so huge * gain stays finite too: a large but
      // finite demand that saturates, exactly as in the old controller.
      if (slot == 0 || slot == 3) {
        float command[4];
        TEST_ASSERT_TRUE(port.compute(in, command));
        continue;
      }
      assertRejected(port, in);
    }
  }

  // Canard angles: the roll sum, and the pitch and yaw differences.
  assertRejected(port, makeInput({}, {}, huge, huge, huge, huge, 80.0f));
  assertRejected(port, makeInput({}, {}, -huge, 0.0f, huge, 0.0f, 80.0f));
  assertRejected(port, makeInput({}, {}, 0.0f, huge, 0.0f, -huge, 80.0f));

  // A huge finite speed is just "above the table".
  AttitudeControlInput fast = healthyInput();
  fast.speed = huge;
  float command[4];
  TEST_ASSERT_TRUE(port.compute(fast, command));
  AttitudeControlInput top = healthyInput();
  top.speed = oldReferenceTable().table[oldReferenceTable().rows - 1][0];
  float topCommand[4];
  TEST_ASSERT_TRUE(port.compute(top, topCommand));
  TEST_ASSERT_TRUE(largestDifference(command, topCommand) <= kEquivalenceTolRad);
}

// ---------------------------------------------------------------------------
// Bad tables and limits
// ---------------------------------------------------------------------------

constexpr int kTestRows = 3;

// A small valid table for the tests below to break one thing at a time in.
// Made-up numbers.
void fillValidTable(float table[kTestRows][7]) {
  const float valid[kTestRows][7] = {
      {25.0f, 1.5f, 0.30f, 0.4f, 9.0f, 5.0f, 0.10f},
      {70.0f, 1.3f, 0.25f, 1.2f, 9.5f, 4.0f, 0.35f},
      {140.0f, 1.1f, 0.15f, 2.6f, 9.9f, 3.0f, 0.85f},
  };
  for (int row = 0; row < kTestRows; ++row) {
    for (int column = 0; column < 7; ++column) table[row][column] = valid[row][column];
  }
}

// An unusable controller must say so and must command nothing, even for a
// perfectly healthy input.
void assertUnusable(const float (*table)[7], int rows, float limit) {
  LqrAttitudeController controller(table, rows, limit);
  TEST_ASSERT_FALSE(controller.valid());
  assertRejected(controller, healthyInput());
}

void test_valid_table_and_limit_are_accepted() {
  float table[kTestRows][7];
  fillValidTable(table);
  const float limit = FlightConfig::kCanardLimitRad;

  LqrAttitudeController three(table, kTestRows, limit);
  TEST_ASSERT_TRUE(three.valid());
  TEST_ASSERT_TRUE(three.canardLimitRad() == limit);
  float command[4];
  TEST_ASSERT_TRUE(three.compute(healthyInput(), command));
  TEST_ASSERT_TRUE(largestMagnitude(command) > 0.0f);

  // Two rows is the smallest table allowed.
  LqrAttitudeController two(table, 2, limit);
  TEST_ASSERT_TRUE(two.valid());
  TEST_ASSERT_TRUE(two.compute(healthyInput(), command));
}

void test_table_with_too_few_rows_is_refused() {
  float table[kTestRows][7];
  fillValidTable(table);
  const float limit = FlightConfig::kCanardLimitRad;
  assertUnusable(table, 1, limit);
  assertUnusable(table, 0, limit);
  assertUnusable(table, -3, limit);
  assertUnusable(nullptr, kTestRows, limit);
  assertUnusable(nullptr, 0, limit);
}

void test_table_with_speeds_not_increasing_is_refused() {
  const float limit = FlightConfig::kCanardLimitRad;
  float table[kTestRows][7];

  fillValidTable(table);
  table[1][0] = table[0][0];  // equal: the interpolation would divide by zero
  assertUnusable(table, kTestRows, limit);

  fillValidTable(table);
  table[2][0] = table[1][0];  // equal, last pair
  assertUnusable(table, kTestRows, limit);

  fillValidTable(table);
  table[1][0] = table[0][0] - 1.0f;  // going down
  assertUnusable(table, kTestRows, limit);

  fillValidTable(table);
  table[2][0] = 0.5f * (table[0][0] + table[1][0]);  // last row out of order
  assertUnusable(table, kTestRows, limit);

  // A bad pair beyond the rows in use does not matter.
  fillValidTable(table);
  table[2][0] = table[1][0];
  LqrAttitudeController firstTwoRows(table, 2, limit);
  TEST_ASSERT_TRUE(firstTwoRows.valid());
}

void test_table_with_a_non_finite_number_is_refused() {
  const float limit = FlightConfig::kCanardLimitRad;
  const float badValues[3] = {kNaN, kInf, -kInf};
  float table[kTestRows][7];
  // Column 0 is the speed, columns 1 to 6 the gains. Every cell, every kind
  // of bad value.
  for (int row = 0; row < kTestRows; ++row) {
    for (int column = 0; column < 7; ++column) {
      for (const float bad : badValues) {
        fillValidTable(table);
        table[row][column] = bad;
        assertUnusable(table, kTestRows, limit);
      }
    }
  }
}

void test_bad_limit_is_refused() {
  float table[kTestRows][7];
  fillValidTable(table);
  assertUnusable(table, kTestRows, 0.0f);
  assertUnusable(table, kTestRows, -0.0f);
  assertUnusable(table, kTestRows, -FlightConfig::kCanardLimitRad);
  assertUnusable(table, kTestRows, kNaN);
  assertUnusable(table, kTestRows, kInf);
  assertUnusable(table, kTestRows, -kInf);

  // The flight-table constructor checks its limit too.
  LqrAttitudeController noLimit(0.0f);
  TEST_ASSERT_FALSE(noLimit.valid());
  assertRejected(noLimit, healthyInput());
  LqrAttitudeController nanLimit(kNaN);
  TEST_ASSERT_FALSE(nanLimit.valid());
  assertRejected(nanLimit, healthyInput());
}

// ---------------------------------------------------------------------------
// The controller as the flight program builds it
// ---------------------------------------------------------------------------

// The flight table and FlightConfig's limit. This does not assume the flight
// table's numbers (GNC will replace them), only that it is usable and that
// the output respects the limit everywhere.
void test_flight_table_and_flight_limit_are_usable() {
  LqrAttitudeController flight(FlightConfig::kCanardLimitRad);
  TEST_ASSERT_TRUE_MESSAGE(flight.valid(), "the flight gain table or FlightConfig::kCanardLimitRad is unusable");
  TEST_ASSERT_TRUE(flight.canardLimitRad() == FlightConfig::kCanardLimitRad);

  // Speeds around the range the flight uses the law in.
  float speedTable[2][7] = {};
  speedTable[0][0] = 0.5f * FlightConfig::kMinControlSpeed;
  speedTable[1][0] = 5.0f * FlightConfig::kMinControlSpeed;

  Lcg rng{99ull};
  int atTheLimit = 0;
  for (int i = 0; i < 20000; ++i) {
    const AttitudeControlInput in = randomInput(rng, speedTable, 2, i % 3);
    float command[4] = {9.0f, 9.0f, 9.0f, 9.0f};
    TEST_ASSERT_TRUE(flight.compute(in, command));
    for (int c = 0; c < 4; ++c) TEST_ASSERT_TRUE(std::isfinite(command[c]));
    if (!(largestMagnitude(command) <= FlightConfig::kCanardLimitRad)) {
      const float limits[4] = {FlightConfig::kCanardLimitRad, FlightConfig::kCanardLimitRad,
                               FlightConfig::kCanardLimitRad, FlightConfig::kCanardLimitRad};
      failCase("flight controller beyond FlightConfig::kCanardLimitRad", "flight table", in, limits,
               command);
    }
    if (largestMagnitude(command) == FlightConfig::kCanardLimitRad) ++atTheLimit;
  }
  TEST_ASSERT_GREATER_THAN(1000, atTheLimit);  // the limit was really reached, often

  // At the lowest speed the flight controls at, a roll error still gets the
  // right-signed answer.
  float command[4];
  TEST_ASSERT_TRUE(flight.compute(errorOnly(1.0f, 0.0f, 0.0f, FlightConfig::kMinControlSpeed), command));
  for (int c = 0; c < 4; ++c) TEST_ASSERT_TRUE(command[c] < 0.0f);
}

// The output never leaves the limit, at limits other than the old one, over a
// sweep that is mostly saturated.
void test_output_is_always_within_the_given_limit() {
  const OldBuild& old = oldReferenceTable();
  const float limits[] = {0.5f * FlightConfig::kCanardLimitRad, FlightConfig::kCanardLimitRad,
                          FlightConfig::kGroundTestLimitRad};
  for (const float limit : limits) {
    LqrAttitudeController port(old.table, old.rows, limit);
    Lcg rng{31337ull};
    int atTheLimit = 0;
    for (int i = 0; i < 20000; ++i) {
      const AttitudeControlInput in = randomInput(rng, old.table, old.rows, i % 3);
      float command[4];
      TEST_ASSERT_TRUE(port.compute(in, command));
      TEST_ASSERT_TRUE(largestMagnitude(command) <= limit);
      if (largestMagnitude(command) == limit) ++atTheLimit;
    }
    TEST_ASSERT_GREATER_THAN(1000, atTheLimit);
  }
}

// ---------------------------------------------------------------------------
// No state, and the output array may be the input's
// ---------------------------------------------------------------------------

void test_controller_keeps_no_state_between_calls() {
  LqrAttitudeController port = referencePort();
  const AttitudeControlInput a = healthyInput();
  const AttitudeControlInput b = errorOnly(9.0f, -3.0f, 4.0f, 120.0f);  // saturating, different speed
  float first[4], again[4], other[4];
  TEST_ASSERT_TRUE(port.compute(a, first));
  TEST_ASSERT_TRUE(port.compute(b, other));
  TEST_ASSERT_TRUE(port.compute(a, again));
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(first[i] == again[i]);

  // A second controller on the same table gives the same answers.
  LqrAttitudeController twin = referencePort();
  TEST_ASSERT_TRUE(twin.compute(a, again));
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(first[i] == again[i]);
}

void test_output_array_may_be_the_input_canard_array() {
  LqrAttitudeController port = referencePort();
  const AttitudeControlInput original = healthyInput();
  float expected[4];
  TEST_ASSERT_TRUE(port.compute(original, expected));

  AttitudeControlInput shared = original;
  TEST_ASSERT_TRUE(port.compute(shared, shared.canardAngle));
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(shared.canardAngle[i] == expected[i]);

  // And on a refusal the shared array is zeroed, not left half-written.
  AttitudeControlInput bad = original;
  bad.attitudeError.z = kNaN;
  TEST_ASSERT_FALSE(port.compute(bad, bad.canardAngle));
  assertAllZero(bad.canardAngle);
}

// The flight program reaches the law through the AttitudeController
// interface. The answer must be the same that way.
void test_works_through_the_attitude_controller_interface() {
  LqrAttitudeController port = referencePort();
  AttitudeController& asInterface = port;
  float direct[4], viaInterface[4];
  TEST_ASSERT_TRUE(port.compute(healthyInput(), direct));
  TEST_ASSERT_TRUE(asInterface.compute(healthyInput(), viaInterface));
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(direct[i] == viaInterface[i]);

  AttitudeControlInput bad = healthyInput();
  bad.speed = kNaN;
  float command[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  TEST_ASSERT_FALSE(asInterface.compute(bad, command));
  assertAllZero(command);
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();

  // Equivalence with the old controller.
  RUN_TEST(test_port_matches_old_controller_as_shipped);
  RUN_TEST(test_port_matches_old_controller_on_the_reference_table);
  RUN_TEST(test_port_matches_old_controller_on_a_two_row_table);
  RUN_TEST(test_equivalence_probe_detects_a_wrong_axis_mapping);
  RUN_TEST(test_quaternion_relation_between_old_and_team_frames);
  RUN_TEST(test_port_matches_old_controller_starting_from_quaternions);

  // The old host tests, in the team frame.
  RUN_TEST(test_gain_matrix_at_80_m_s);
  RUN_TEST(test_gain_interpolation_and_end_rows_held);
  RUN_TEST(test_roll_delivered_in_full_and_pitch_yaw_scaled_into_the_rest);
  RUN_TEST(test_roll_beyond_the_limit_takes_all_of_it);
  RUN_TEST(test_pitch_and_yaw_alone_scale_to_the_limit);
  RUN_TEST(test_roll_error_beyond_10_deg_behaves_as_10_deg);
  RUN_TEST(test_held_roll_error_settles_with_the_old_servo_model);

  // Physical signs.
  RUN_TEST(test_canard_moment_model_matches_the_stated_geometry);
  RUN_TEST(test_nose_tilted_towards_plus_y_is_pushed_back);
  RUN_TEST(test_nose_tilted_towards_plus_z_is_pushed_back);
  RUN_TEST(test_positive_roll_error_gives_all_four_negative);
  RUN_TEST(test_every_axis_is_negative_feedback_at_every_table_speed);

  // Roll priority.
  RUN_TEST(test_roll_priority_at_the_flight_limit);
  RUN_TEST(test_roll_priority_at_the_old_limit);
  RUN_TEST(test_roll_priority_follows_whatever_limit_is_given);

  // Bad inputs, tables and limits.
  RUN_TEST(test_any_non_finite_input_gives_zero_and_false);
  RUN_TEST(test_port_refuses_infinities_the_old_code_flew);
  RUN_TEST(test_overflow_from_finite_inputs_gives_zero_and_false);
  RUN_TEST(test_valid_table_and_limit_are_accepted);
  RUN_TEST(test_table_with_too_few_rows_is_refused);
  RUN_TEST(test_table_with_speeds_not_increasing_is_refused);
  RUN_TEST(test_table_with_a_non_finite_number_is_refused);
  RUN_TEST(test_bad_limit_is_refused);

  // As the flight program uses it.
  RUN_TEST(test_flight_table_and_flight_limit_are_usable);
  RUN_TEST(test_output_is_always_within_the_given_limit);
  RUN_TEST(test_controller_keeps_no_state_between_calls);
  RUN_TEST(test_output_array_may_be_the_input_canard_array);
  RUN_TEST(test_works_through_the_attitude_controller_interface);

  return UNITY_END();
}
