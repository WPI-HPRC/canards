// lib/Frames: the team's frame and quaternion conventions
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 4, and step 2 of
// section 10). These tests pin the conventions themselves, so a sign or
// ordering mistake anywhere in Frames shows up here and not in flight.
//
// Tolerances: a float quaternion holds an attitude to about 1e-7 rad per
// operation, so angles are compared to a few microradians unless a test says
// otherwise.

#include <unity.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>

#include "FlightConfig.h"
#include "Frames.h"
#include "FramesDisplay.h"
#include "VehicleConfig.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kHalfPi = 0.5f * kPi;
constexpr float kDeg = kPi / 180.0f;
const float kNan = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();

constexpr Vec3 kBodyX{1.0f, 0.0f, 0.0f};  // nose
constexpr Vec3 kBodyY{0.0f, 1.0f, 0.0f};
constexpr Vec3 kBodyZ{0.0f, 0.0f, 1.0f};
constexpr Vec3 kNorth{1.0f, 0.0f, 0.0f};
constexpr Vec3 kEast{0.0f, 1.0f, 0.0f};
constexpr Vec3 kDown{0.0f, 0.0f, 1.0f};
constexpr Vec3 kUp{0.0f, 0.0f, -1.0f};

// Rolls about the nose and compass headings used by the sweeps.
const float kRolls[] = {0.0f, 0.3f, -1.2f, 2.5f, -3.0f};
const float kHeadings[] = {0.0f, 0.7f, -2.0f, 3.1f, kHalfPi};

// --- assertion helpers: every failure names the case that was running -------

char gCase[200] = "";

void setCase(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(gCase, sizeof(gCase), format, args);
  va_end(args);
}

void expectNear(float tolerance, float expected, float actual, int line) {
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual, line, gCase);
}

void expectVecNear(float tolerance, const Vec3& expected, const Vec3& actual, int line) {
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.x, actual.x, line, gCase);
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.y, actual.y, line, gCase);
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.z, actual.z, line, gCase);
}

// Element by element: q and -q do NOT compare equal here.
void expectQuatNear(float tolerance, const Quat& expected, const Quat& actual, int line) {
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.w, actual.w, line, gCase);
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.x, actual.x, line, gCase);
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.y, actual.y, line, gCase);
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.z, actual.z, line, gCase);
}

void expectTrue(bool condition, int line) { UNITY_TEST_ASSERT(condition, line, gCase); }

#define EXPECT_NEAR(tolerance, expected, actual) expectNear((tolerance), (expected), (actual), __LINE__)
#define EXPECT_VEC_NEAR(tolerance, expected, actual) expectVecNear((tolerance), (expected), (actual), __LINE__)
#define EXPECT_QUAT_NEAR(tolerance, expected, actual) expectQuatNear((tolerance), (expected), (actual), __LINE__)
#define EXPECT_TRUE(condition) expectTrue((condition), __LINE__)

// --- attitude builders ------------------------------------------------------

Quat negated(const Quat& q) { return {-q.w, -q.x, -q.y, -q.z}; }

Quat scaled(const Quat& q, float s) { return {q.w * s, q.x * s, q.y * s, q.z * s}; }

// `base` turned by `angle` about one of the rocket's own axes.
Quat rotatedInBody(const Quat& base, const Vec3& bodyAxis, float angle) {
  return base * fromAxisAngle(bodyAxis, angle);
}

// `base` turned by `angle` about an axis fixed to the ground.
Quat rotatedInNed(const Quat& base, const Vec3& nedAxis, float angle) {
  return fromAxisAngle(nedAxis, angle) * base;
}

// Nose straight up: pitch +90 deg, then a roll about the nose, all turned to
// a compass heading. Every (roll, heading) pair is a nose-up attitude.
Quat noseUp(float roll, float heading) {
  return fromAxisAngle(kDown, heading) * fromAxisAngle(kBodyY, kHalfPi) * fromAxisAngle(kBodyX, roll);
}

// A horizontal direction, `azimuth` from north towards east.
Vec3 horizontal(float azimuth) { return {std::cos(azimuth), std::sin(azimuth), 0.0f}; }

float wrapPi(float angle) {
  while (angle > kPi) angle -= 2.0f * kPi;
  while (angle <= -kPi) angle += 2.0f * kPi;
  return angle;
}

float determinant(const Mat3& a) {
  return a.m[0][0] * (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1]) -
         a.m[0][1] * (a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0]) +
         a.m[0][2] * (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]);
}

// --- compile-time checks: the constexpr parts are usable in constants --------

static_assert((Quat{0.0f, 1.0f, 0.0f, 0.0f} * Quat{0.0f, 0.0f, 1.0f, 0.0f}).z == 1.0f, "i * j = k");
static_assert(cross(Vec3{1.0f, 0.0f, 0.0f}, Vec3{0.0f, 1.0f, 0.0f}).z == 1.0f, "x cross y = z");
static_assert(dot(kNoseBody, kNoseBody) == 1.0f, "the nose is a unit vector");
static_assert(kNoseBody.x == 1.0f, "the nose is body +x");
static_assert(kUpNed.z == -1.0f, "NED z points down");
static_assert((Mat3{} * Vec3{1.0f, 2.0f, 3.0f}).y == 2.0f, "the default Mat3 is the identity");

// ===========================================================================
// Vectors
// ===========================================================================

void test_vector_operators() {
  const Vec3 a{1.0f, -2.0f, 3.0f};
  const Vec3 b{0.5f, 4.0f, -6.0f};
  EXPECT_VEC_NEAR(0.0f, (Vec3{1.5f, 2.0f, -3.0f}), a + b);
  EXPECT_VEC_NEAR(0.0f, (Vec3{0.5f, -6.0f, 9.0f}), a - b);
  EXPECT_VEC_NEAR(0.0f, (Vec3{-1.0f, 2.0f, -3.0f}), -a);
  EXPECT_VEC_NEAR(0.0f, (Vec3{2.0f, -4.0f, 6.0f}), a * 2.0f);
  EXPECT_VEC_NEAR(0.0f, (Vec3{2.0f, -4.0f, 6.0f}), 2.0f * a);
  EXPECT_NEAR(0.0f, 0.5f - 8.0f - 18.0f, dot(a, b));
  const Vec3 zero;
  EXPECT_VEC_NEAR(0.0f, (Vec3{0.0f, 0.0f, 0.0f}), zero);  // a default Vec3 is zero
}

void test_cross_is_right_handed() {
  EXPECT_VEC_NEAR(0.0f, kBodyZ, cross(kBodyX, kBodyY));
  EXPECT_VEC_NEAR(0.0f, kBodyX, cross(kBodyY, kBodyZ));
  EXPECT_VEC_NEAR(0.0f, kBodyY, cross(kBodyZ, kBodyX));
  EXPECT_VEC_NEAR(0.0f, -kBodyZ, cross(kBodyY, kBodyX));
  // NED is right-handed too: north cross east is down.
  EXPECT_VEC_NEAR(0.0f, kDown, cross(kNorth, kEast));
  // General case against the component formula.
  EXPECT_VEC_NEAR(1e-6f, (Vec3{0.0f, 7.5f, 5.0f}), cross(Vec3{1.0f, -2.0f, 3.0f}, Vec3{0.5f, 4.0f, -6.0f}));
}

void test_norm_of_vectors_and_quaternions() {
  EXPECT_NEAR(1e-6f, 5.0f, norm(Vec3{3.0f, -4.0f, 0.0f}));
  EXPECT_NEAR(1e-6f, 13.0f, norm(Vec3{3.0f, 4.0f, -12.0f}));
  EXPECT_NEAR(0.0f, 0.0f, norm(Vec3{}));
  EXPECT_NEAR(1e-6f, 5.0f, norm(Quat{1.0f, -2.0f, 2.0f, 4.0f}));
  EXPECT_NEAR(0.0f, 1.0f, norm(Quat{}));  // a default Quat is the identity
}

void test_normalized_vector() {
  EXPECT_VEC_NEAR(1e-6f, (Vec3{0.6f, 0.0f, -0.8f}), normalized(Vec3{3.0f, 0.0f, -4.0f}));
  const Vec3 directions[] = {{1.0f, 2.0f, 3.0f}, {-0.001f, 0.0f, 0.0f}, {250.0f, -30.0f, 4.0f}, {0.0f, 1e-6f, 1e-6f}};
  for (const Vec3& v : directions) {
    setCase("v = (%g, %g, %g)", v.x, v.y, v.z);
    const Vec3 unit = normalized(v);
    EXPECT_NEAR(1e-6f, 1.0f, norm(unit));
    // Same direction: parallel and pointing the same way.
    EXPECT_NEAR(1e-6f * norm(v), norm(v), dot(unit, v));
  }
}

void test_normalized_vector_falls_back() {
  const Vec3 other{0.0f, 0.0f, -1.0f};
  // Too short to have a direction.
  EXPECT_VEC_NEAR(0.0f, kBodyX, normalized(Vec3{0.0f, 0.0f, 0.0f}));  // the default fallback
  EXPECT_VEC_NEAR(0.0f, other, normalized(Vec3{0.0f, 0.0f, 0.0f}, other));
  EXPECT_VEC_NEAR(0.0f, other, normalized(Vec3{1e-30f, -1e-30f, 0.0f}, other));
  // Not finite.
  EXPECT_VEC_NEAR(0.0f, other, normalized(Vec3{kNan, 0.0f, 0.0f}, other));
  EXPECT_VEC_NEAR(0.0f, other, normalized(Vec3{1.0f, kNan, 0.0f}, other));
  EXPECT_VEC_NEAR(0.0f, other, normalized(Vec3{1.0f, 2.0f, kInf}, other));
  EXPECT_VEC_NEAR(0.0f, other, normalized(Vec3{-kInf, kInf, 0.0f}, other));
}

void test_normalized_vector_handles_very_long_vectors() {
  // Finite, but its squares overflow a float. It still has a direction, so it
  // must not be mistaken for "not finite".
  const Vec3 other{0.0f, 0.0f, -1.0f};
  EXPECT_VEC_NEAR(1e-6f, (Vec3{0.6f, 0.0f, -0.8f}), normalized(Vec3{3e25f, 0.0f, -4e25f}, other));
  const float most = std::numeric_limits<float>::max();
  const float third = 1.0f / std::sqrt(3.0f);
  EXPECT_VEC_NEAR(1e-6f, (Vec3{third, -third, third}), normalized(Vec3{most, -most, most}, other));
}

void test_is_finite_vector() {
  TEST_ASSERT_TRUE(isFinite(Vec3{1.0f, -2.0f, 3e30f}));
  TEST_ASSERT_TRUE(isFinite(Vec3{}));
  TEST_ASSERT_FALSE(isFinite(Vec3{kNan, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isFinite(Vec3{0.0f, kNan, 0.0f}));
  TEST_ASSERT_FALSE(isFinite(Vec3{0.0f, 0.0f, kNan}));
  TEST_ASSERT_FALSE(isFinite(Vec3{kInf, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isFinite(Vec3{0.0f, -kInf, 0.0f}));
  TEST_ASSERT_FALSE(isFinite(Vec3{0.0f, 0.0f, kInf}));
}

void test_mat3_times_vec3() {
  // Row-major, out = m * v: each output is one ROW of the matrix dotted with v.
  const Mat3 m = {{{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 10.0f}}};
  EXPECT_VEC_NEAR(0.0f, (Vec3{1.0f, 4.0f, 7.0f}), (m * Vec3{1.0f, 0.0f, 0.0f}));
  EXPECT_VEC_NEAR(0.0f, (Vec3{2.0f, 5.0f, 8.0f}), (m * Vec3{0.0f, 1.0f, 0.0f}));
  EXPECT_VEC_NEAR(0.0f, (Vec3{3.0f, 6.0f, 10.0f}), (m * Vec3{0.0f, 0.0f, 1.0f}));
  EXPECT_VEC_NEAR(1e-5f, (Vec3{-6.0f, -12.0f, -19.0f}), (m * Vec3{1.0f, -2.0f, -1.0f}));
  const Mat3 identity;
  EXPECT_VEC_NEAR(0.0f, (Vec3{1.5f, -2.5f, 3.5f}), (identity * Vec3{1.5f, -2.5f, 3.5f}));
}

// A sensor mount map only relabels and flips axes, so it must be a proper
// rotation: orthonormal with determinant +1. A determinant of -1 would turn
// the right-handed sensor into a left-handed body frame and reverse one gyro
// axis relative to the accelerometer.
void expectProperRotation(const Mat3& a, int line) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const float rowDot = a.m[i][0] * a.m[j][0] + a.m[i][1] * a.m[j][1] + a.m[i][2] * a.m[j][2];
      const float colDot = a.m[0][i] * a.m[0][j] + a.m[1][i] * a.m[1][j] + a.m[2][i] * a.m[2][j];
      expectNear(1e-6f, i == j ? 1.0f : 0.0f, rowDot, line);
      expectNear(1e-6f, i == j ? 1.0f : 0.0f, colDot, line);
    }
  }
  expectNear(1e-6f, 1.0f, determinant(a), line);
}

void test_mount_maps_are_proper_rotations() {
  setCase("kImuToBody");
  expectProperRotation(VehicleConfig::kImuToBody, __LINE__);
  setCase("kMagToBody");
  expectProperRotation(VehicleConfig::kMagToBody, __LINE__);

  // The check itself must be able to fail: a mirror is orthonormal but has
  // determinant -1.
  const Mat3 mirror = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, -1.0f}}};
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, determinant(mirror));

  // A proper rotation keeps lengths and handedness of what it maps.
  const Vec3 a{0.3f, -1.1f, 2.0f};
  const Vec3 b{-0.7f, 0.2f, 0.5f};
  const Mat3& imu = VehicleConfig::kImuToBody;
  EXPECT_NEAR(1e-6f, norm(a), norm(imu * a));
  EXPECT_VEC_NEAR(1e-6f, imu * cross(a, b), cross(imu * a, imu * b));
}

// ===========================================================================
// Quaternion basics
// ===========================================================================

void test_hamilton_product() {
  const Quat i{0.0f, 1.0f, 0.0f, 0.0f};
  const Quat j{0.0f, 0.0f, 1.0f, 0.0f};
  const Quat k{0.0f, 0.0f, 0.0f, 1.0f};
  // Hamilton: ij = k, jk = i, ki = j, and the reverse order flips the sign.
  // (The other common convention, JPL, has ij = -k.)
  EXPECT_QUAT_NEAR(0.0f, k, i * j);
  EXPECT_QUAT_NEAR(0.0f, i, j * k);
  EXPECT_QUAT_NEAR(0.0f, j, k * i);
  EXPECT_QUAT_NEAR(0.0f, negated(k), j * i);
  EXPECT_QUAT_NEAR(0.0f, (Quat{-1.0f, 0.0f, 0.0f, 0.0f}), i * i);
  // The identity changes nothing, on either side.
  const Quat q{0.5f, -0.1f, 0.7f, 0.2f};
  EXPECT_QUAT_NEAR(0.0f, q, kIdentityQuat * q);
  EXPECT_QUAT_NEAR(0.0f, q, q * kIdentityQuat);
  // A general product against values worked by hand.
  EXPECT_QUAT_NEAR(1e-6f, (Quat{16.0f, -30.0f, 8.0f, 20.0f}),
                   (Quat{1.0f, 2.0f, 3.0f, 4.0f}) * (Quat{2.0f, -3.0f, 4.0f, -5.0f}));
}

void test_conjugate_is_the_inverse_rotation() {
  EXPECT_QUAT_NEAR(0.0f, (Quat{0.5f, 0.1f, -0.7f, -0.2f}), conjugate(Quat{0.5f, -0.1f, 0.7f, 0.2f}));
  for (float roll : kRolls) {
    for (float heading : kHeadings) {
      setCase("roll %g heading %g", roll, heading);
      const Quat q = rotatedInBody(noseUp(roll, heading), Vec3{0.2f, 0.9f, -0.4f}, 0.8f);
      EXPECT_QUAT_NEAR(1e-6f, kIdentityQuat, q * conjugate(q));
      EXPECT_QUAT_NEAR(1e-6f, kIdentityQuat, conjugate(q) * q);
    }
  }
}

void test_normalized_quaternion() {
  EXPECT_QUAT_NEAR(1e-6f, (Quat{0.2f, -0.4f, 0.4f, 0.8f}), normalized(Quat{1.0f, -2.0f, 2.0f, 4.0f}));
  EXPECT_QUAT_NEAR(1e-6f, (Quat{0.0f, 0.0f, -1.0f, 0.0f}), normalized(Quat{0.0f, 0.0f, -1e-5f, 0.0f}));
  // It must not flip the sign: the caller decides between q and -q.
  EXPECT_QUAT_NEAR(1e-6f, (Quat{-0.6f, 0.0f, 0.8f, 0.0f}), normalized(Quat{-3.0f, 0.0f, 4.0f, 0.0f}));
  // An already-unit quaternion comes back as it was.
  const Quat unit = noseUp(0.4f, -1.0f);
  EXPECT_QUAT_NEAR(1e-6f, unit, normalized(unit));
}

void test_normalized_quaternion_falls_back_to_identity() {
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, normalized(Quat{0.0f, 0.0f, 0.0f, 0.0f}));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, normalized(Quat{1e-30f, 0.0f, -1e-30f, 0.0f}));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, normalized(Quat{kNan, 0.0f, 0.0f, 0.0f}));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, normalized(Quat{0.0f, 1.0f, kNan, 0.0f}));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, normalized(Quat{0.0f, 0.0f, 0.0f, kInf}));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, normalized(Quat{-kInf, kInf, 1.0f, 0.0f}));
}

void test_normalized_quaternion_handles_very_long_quaternions() {
  // Finite, but the squares overflow. It still has a direction.
  EXPECT_QUAT_NEAR(1e-6f, (Quat{0.6f, 0.0f, 0.0f, -0.8f}), normalized(Quat{3e25f, 0.0f, 0.0f, -4e25f}));
  const float most = std::numeric_limits<float>::max();
  EXPECT_QUAT_NEAR(1e-6f, (Quat{0.5f, -0.5f, 0.5f, 0.5f}), normalized(Quat{most, -most, most, most}));
}

void test_is_finite_and_is_unit_quaternion() {
  TEST_ASSERT_TRUE(isFinite(kIdentityQuat));
  TEST_ASSERT_TRUE(isFinite(Quat{0.0f, 0.0f, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isFinite(Quat{kNan, 0.0f, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isFinite(Quat{1.0f, kNan, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isFinite(Quat{1.0f, 0.0f, kInf, 0.0f}));
  TEST_ASSERT_FALSE(isFinite(Quat{1.0f, 0.0f, 0.0f, -kInf}));

  TEST_ASSERT_TRUE(isUnit(kIdentityQuat));
  TEST_ASSERT_TRUE(isUnit(noseUp(1.0f, 2.0f)));
  TEST_ASSERT_TRUE(isUnit(negated(noseUp(1.0f, 2.0f))));
  TEST_ASSERT_TRUE(isUnit(Quat{1.0005f, 0.0f, 0.0f, 0.0f}));  // inside the default tolerance
  TEST_ASSERT_FALSE(isUnit(Quat{1.01f, 0.0f, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isUnit(Quat{0.99f, 0.0f, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isUnit(Quat{0.0f, 0.0f, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isUnit(Quat{1.0f, 1.0f, 0.0f, 0.0f}));
  // The tolerance is the caller's to choose.
  TEST_ASSERT_TRUE(isUnit(Quat{1.01f, 0.0f, 0.0f, 0.0f}, 0.02f));
  TEST_ASSERT_FALSE(isUnit(Quat{1.0005f, 0.0f, 0.0f, 0.0f}, 1e-4f));
  // Never unit if not finite, whatever the tolerance.
  TEST_ASSERT_FALSE(isUnit(Quat{kNan, 0.0f, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isUnit(Quat{1.0f, kNan, 0.0f, 0.0f}));
  TEST_ASSERT_FALSE(isUnit(Quat{kInf, 0.0f, 0.0f, 0.0f}, kInf));
  TEST_ASSERT_FALSE(isUnit(Quat{1e30f, 0.0f, 0.0f, 0.0f}));
}

// ===========================================================================
// rotate(): the quaternion takes BODY vectors into NED
// ===========================================================================

void test_rotate_takes_body_vectors_into_ned() {
  // The identity attitude: nose north, body y east, body z down.
  EXPECT_VEC_NEAR(0.0f, kNorth, rotate(kIdentityQuat, kBodyX));
  EXPECT_VEC_NEAR(0.0f, kEast, rotate(kIdentityQuat, kBodyY));
  EXPECT_VEC_NEAR(0.0f, kDown, rotate(kIdentityQuat, kBodyZ));

  // Yawed +90 deg (about down): the nose swings from north to EAST.
  const Quat yaw90{std::sqrt(0.5f), 0.0f, 0.0f, std::sqrt(0.5f)};
  EXPECT_VEC_NEAR(1e-6f, kEast, rotate(yaw90, kBodyX));
  EXPECT_VEC_NEAR(1e-6f, -kNorth, rotate(yaw90, kBodyY));
  EXPECT_VEC_NEAR(1e-6f, kDown, rotate(yaw90, kBodyZ));

  // Pitched +90 deg (about body y): the nose swings from north to UP.
  const Quat pitch90{std::sqrt(0.5f), 0.0f, std::sqrt(0.5f), 0.0f};
  EXPECT_VEC_NEAR(1e-6f, kUp, rotate(pitch90, kBodyX));
  EXPECT_VEC_NEAR(1e-6f, kEast, rotate(pitch90, kBodyY));
  EXPECT_VEC_NEAR(1e-6f, kNorth, rotate(pitch90, kBodyZ));

  // Rolled +90 deg (about the nose): body y swings from east to DOWN.
  const Quat roll90{std::sqrt(0.5f), std::sqrt(0.5f), 0.0f, 0.0f};
  EXPECT_VEC_NEAR(1e-6f, kNorth, rotate(roll90, kBodyX));
  EXPECT_VEC_NEAR(1e-6f, kDown, rotate(roll90, kBodyY));
  EXPECT_VEC_NEAR(1e-6f, -kEast, rotate(roll90, kBodyZ));
}

void test_rotate_inverse_takes_ned_vectors_into_body() {
  // Sitting on the pad nose-up, an accelerometer feels the pad pushing UP,
  // which is along the nose: +x in body axes, whatever the roll or heading.
  for (float roll : kRolls) {
    for (float heading : kHeadings) {
      setCase("roll %g heading %g", roll, heading);
      EXPECT_VEC_NEAR(1e-6f, kBodyX, rotateInverse(noseUp(roll, heading), kUp));
    }
  }
  // Yawed +90 deg, north is on the rocket's left: body -y.
  EXPECT_VEC_NEAR(1e-6f, -kBodyY, rotateInverse(fromAxisAngle(kDown, kHalfPi), kNorth));
}

void test_rotate_matches_the_axis_angle_formula() {
  // Rodrigues: v' = v cos(a) + (k x v) sin(a) + k (k.v)(1 - cos(a)), right-handed.
  const Vec3 axes[] = {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.6f, -0.48f, 0.64f}, {-1.0f, 2.0f, 2.0f}};
  const float angles[] = {0.0f, 1e-3f, 0.5f, -1.1f, 2.0f, 3.0f, -3.1f, 5.0f};
  const Vec3 vectors[] = {{1.0f, 0.0f, 0.0f}, {0.3f, -1.1f, 2.0f}, {-40.0f, 2.0f, 9.5f}};
  for (const Vec3& axis : axes) {
    const Vec3 k = normalized(axis);
    for (float a : angles) {
      for (const Vec3& v : vectors) {
        setCase("axis (%g, %g, %g) angle %g v (%g, %g, %g)", axis.x, axis.y, axis.z, a, v.x, v.y, v.z);
        const Vec3 want = v * std::cos(a) + cross(k, v) * std::sin(a) + k * (dot(k, v) * (1.0f - std::cos(a)));
        const float tolerance = 2e-6f * (1.0f + norm(v));
        EXPECT_VEC_NEAR(tolerance, want, rotate(fromAxisAngle(axis, a), v));
      }
    }
  }
}

void test_rotate_and_rotate_inverse_round_trip() {
  const Vec3 vectors[] = {{1.0f, 0.0f, 0.0f}, {0.3f, -1.1f, 2.0f}, {-40.0f, 2.0f, 9.5f}, {0.0f, 0.0f, 0.0f}};
  for (float roll : kRolls) {
    for (float heading : kHeadings) {
      const Quat q = rotatedInNed(noseUp(roll, heading), horizontal(heading + 1.0f), 0.2f + roll);
      for (const Vec3& v : vectors) {
        setCase("roll %g heading %g v (%g, %g, %g)", roll, heading, v.x, v.y, v.z);
        const float tolerance = 2e-6f * (1.0f + norm(v));
        EXPECT_VEC_NEAR(tolerance, v, rotateInverse(q, rotate(q, v)));
        EXPECT_VEC_NEAR(tolerance, v, rotate(q, rotateInverse(q, v)));
        EXPECT_NEAR(tolerance, norm(v), norm(rotate(q, v)));  // a rotation keeps lengths
        // q and -q are the same rotation.
        EXPECT_VEC_NEAR(tolerance, rotate(q, v), rotate(negated(q), v));
      }
    }
  }
}

void test_product_applies_the_right_hand_factor_first() {
  const Quat yaw90 = fromAxisAngle(kDown, kHalfPi);
  const Quat pitch90 = fromAxisAngle(kBodyY, kHalfPi);
  // yaw (x) pitch: pitch first takes the nose north -> up, then the yaw about
  // the vertical leaves it there.
  EXPECT_VEC_NEAR(1e-6f, kUp, rotate(yaw90 * pitch90, kBodyX));
  // pitch (x) yaw: yaw first takes the nose north -> east, then the pitch
  // about east leaves it there.
  EXPECT_VEC_NEAR(1e-6f, kEast, rotate(pitch90 * yaw90, kBodyX));

  // In general (a (x) b) v = a (b v).
  const Quat a = fromAxisAngle(Vec3{0.6f, -0.48f, 0.64f}, 1.1f);
  const Quat b = fromAxisAngle(Vec3{-1.0f, 2.0f, 2.0f}, -2.3f);
  const Vec3 v{0.3f, -1.1f, 2.0f};
  EXPECT_VEC_NEAR(3e-6f, rotate(a, rotate(b, v)), rotate(a * b, v));
}

void test_right_factor_turns_in_body_left_factor_turns_in_ned() {
  for (float roll : kRolls) {
    for (float heading : kHeadings) {
      setCase("roll %g heading %g", roll, heading);
      const Quat q = rotatedInNed(noseUp(roll, heading), horizontal(0.4f), 0.5f);
      // A turn about the rocket's own nose (right factor) cannot move the nose.
      EXPECT_VEC_NEAR(2e-6f, noseNed(q), noseNed(rotatedInBody(q, kBodyX, 1.3f)));
      // A turn about the ground's vertical (left factor) changes the heading
      // but not how far the nose is from vertical.
      const Quat turned = rotatedInNed(q, kDown, 1.3f);
      EXPECT_NEAR(2e-6f, noseNed(q).z, noseNed(turned).z);
      EXPECT_NEAR(2e-6f, tilt(q), tilt(turned));
      TEST_ASSERT_TRUE(norm(noseNed(q) - noseNed(turned)) > 0.1f);
    }
  }
}

// ===========================================================================
// fromAxisAngle, fromRotationVector, toRotationVector
// ===========================================================================

void test_from_axis_angle() {
  EXPECT_QUAT_NEAR(1e-6f, (Quat{std::cos(0.35f), 0.0f, 0.0f, std::sin(0.35f)}), fromAxisAngle(kBodyZ, 0.7f));
  // The axis does not need to be unit length.
  EXPECT_QUAT_NEAR(1e-6f, fromAxisAngle(Vec3{0.6f, 0.0f, -0.8f}, 1.2f), fromAxisAngle(Vec3{30.0f, 0.0f, -40.0f}, 1.2f));
  EXPECT_QUAT_NEAR(1e-6f, fromAxisAngle(Vec3{0.6f, 0.0f, -0.8f}, 1.2f), fromAxisAngle(Vec3{6e-7f, 0.0f, -8e-7f}, 1.2f));
  EXPECT_QUAT_NEAR(1e-6f, fromAxisAngle(Vec3{0.6f, 0.0f, -0.8f}, 1.2f), fromAxisAngle(Vec3{3e25f, 0.0f, -4e25f}, 1.2f));
  // A negative angle is the inverse rotation, and so is the opposite axis.
  EXPECT_QUAT_NEAR(1e-6f, conjugate(fromAxisAngle(Vec3{1.0f, 2.0f, 3.0f}, 0.9f)), fromAxisAngle(Vec3{1.0f, 2.0f, 3.0f}, -0.9f));
  EXPECT_QUAT_NEAR(1e-6f, conjugate(fromAxisAngle(Vec3{1.0f, 2.0f, 3.0f}, 0.9f)), fromAxisAngle(Vec3{-1.0f, -2.0f, -3.0f}, 0.9f));
  // Zero angle is the identity; the result is always unit length.
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromAxisAngle(Vec3{1.0f, 2.0f, 3.0f}, 0.0f));
  const float angles[] = {-7.0f, -kPi, -1.0f, 1e-4f, 2.0f, kPi, 6.0f, 100.0f};
  for (float a : angles) {
    setCase("angle %g", a);
    EXPECT_NEAR(1e-6f, 1.0f, norm(fromAxisAngle(Vec3{-3.0f, 0.5f, 2.0f}, a)));
  }
}

void test_from_axis_angle_bad_inputs_give_identity() {
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromAxisAngle(Vec3{0.0f, 0.0f, 0.0f}, 1.0f));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromAxisAngle(Vec3{kNan, 0.0f, 0.0f}, 1.0f));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromAxisAngle(Vec3{0.0f, kInf, 0.0f}, 1.0f));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromAxisAngle(kBodyX, kNan));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromAxisAngle(kBodyX, kInf));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromAxisAngle(kBodyX, -kInf));
}

void test_rotation_vector_round_trip() {
  const Vec3 directions[] = {{1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
                             {0.6f, -0.48f, 0.64f}, {-0.3f, -0.5f, 0.81f}};
  // From far below the first-order switch, across it, up to just short of pi.
  const float angles[] = {1e-20f, 1e-12f, 1e-9f, 1e-7f, 9e-7f, 1.1e-6f, 1e-5f, 1e-3f, 0.1f,
                          1.0f, kHalfPi, 3.0f, kPi - 1e-2f, kPi - 1e-3f, kPi - 1e-5f};
  for (const Vec3& direction : directions) {
    const Vec3 unit = normalized(direction);
    for (float angle : angles) {
      setCase("direction (%g, %g, %g) angle %.9g", direction.x, direction.y, direction.z, angle);
      const Vec3 rv = unit * angle;
      const Quat q = fromRotationVector(rv);
      EXPECT_NEAR(1e-6f, 1.0f, norm(q));
      EXPECT_TRUE(q.w > 0.0f);
      // Relative tolerance: a 1e-7 rad rotation must come back as 1e-7 rad,
      // not as zero and not as 3e-4.
      EXPECT_VEC_NEAR(3e-6f * angle, rv, toRotationVector(q));
      // And it is the same rotation as the axis-angle form.
      EXPECT_QUAT_NEAR(1e-6f, fromAxisAngle(unit, angle), q);
    }
  }
  // Zero both ways.
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromRotationVector(Vec3{0.0f, 0.0f, 0.0f}));
  EXPECT_VEC_NEAR(0.0f, (Vec3{0.0f, 0.0f, 0.0f}), toRotationVector(kIdentityQuat));
}

void test_rotation_vector_takes_the_shorter_way() {
  const Vec3 unit = normalized(Vec3{0.6f, -0.48f, 0.64f});
  // 270 deg one way is 90 deg the other way.
  EXPECT_VEC_NEAR(3e-6f, unit * -kHalfPi, toRotationVector(fromRotationVector(unit * (1.5f * kPi))));
  // A full turn is no rotation at all.
  EXPECT_VEC_NEAR(3e-6f, (Vec3{0.0f, 0.0f, 0.0f}), toRotationVector(fromRotationVector(unit * (2.0f * kPi))));
  // q and -q are the same rotation, and so is any positive multiple of q.
  const float angles[] = {1e-7f, 1e-3f, 0.4f, 2.0f, 3.0f, kPi - 1e-3f};
  for (float angle : angles) {
    setCase("angle %.9g", angle);
    const Quat q = fromAxisAngle(unit, angle);
    EXPECT_VEC_NEAR(3e-6f * angle, unit * angle, toRotationVector(q));
    EXPECT_VEC_NEAR(3e-6f * angle, unit * angle, toRotationVector(negated(q)));
    EXPECT_VEC_NEAR(3e-6f * angle, unit * angle, toRotationVector(scaled(q, 7.0f)));
    EXPECT_VEC_NEAR(3e-6f * angle, unit * angle, toRotationVector(scaled(q, -0.01f)));
  }
  // Exactly 180 deg is the one ambiguous case: either axis sign is right, but
  // the length must be pi.
  const Vec3 half = toRotationVector(Quat{0.0f, 0.6f, 0.0f, -0.8f});
  EXPECT_NEAR(2e-6f, kPi, norm(half));
  EXPECT_NEAR(2e-6f, kPi, std::fabs(dot(half, Vec3{0.6f, 0.0f, -0.8f})));
  // The length never exceeds pi.
  for (float roll : kRolls) {
    for (float heading : kHeadings) {
      for (float extra : angles) {
        setCase("roll %g heading %g extra %g", roll, heading, extra);
        const Quat q = rotatedInBody(noseUp(roll, heading), Vec3{0.1f, -0.7f, 0.7f}, 2.0f * extra);
        EXPECT_TRUE(norm(toRotationVector(q)) <= kPi + 2e-6f);
      }
    }
  }
}

void test_rotation_vector_bad_inputs() {
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromRotationVector(Vec3{kNan, 0.0f, 0.0f}));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromRotationVector(Vec3{0.0f, kInf, 0.0f}));
  EXPECT_QUAT_NEAR(0.0f, kIdentityQuat, fromRotationVector(Vec3{0.0f, 1.0f, -kInf}));
  // A finite but absurd vector must still give a usable unit quaternion.
  const Quat absurd = fromRotationVector(Vec3{1e30f, -1e30f, 0.0f});
  TEST_ASSERT_TRUE(isUnit(absurd));
  // Invalid quaternions read as "no rotation", never as NaN.
  const Quat bad[] = {{kNan, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, kNan, 0.0f}, {kInf, 0.0f, 0.0f, 0.0f},
                      {1.0f, -kInf, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}};
  for (const Quat& q : bad) {
    setCase("q = [%g, %g, %g, %g]", q.w, q.x, q.y, q.z);
    EXPECT_VEC_NEAR(0.0f, (Vec3{0.0f, 0.0f, 0.0f}), toRotationVector(q));
  }
}

// ===========================================================================
// Tilt
// ===========================================================================

void test_nose_up_has_zero_tilt_at_the_euler_singularity() {
  // Every roll about the nose and every heading: the nose is up, tilt is 0,
  // and the display pitch confirms this is the pitch = +90 deg singularity.
  for (float roll : kRolls) {
    for (float heading : kHeadings) {
      setCase("roll %g heading %g", roll, heading);
      const Quat q = noseUp(roll, heading);
      EXPECT_VEC_NEAR(1e-6f, kUp, noseNed(q));
      EXPECT_NEAR(1e-6f, 0.0f, tilt(q));
      EXPECT_NEAR(1e-6f, 0.0f, tilt(negated(q)));
      EXPECT_NEAR(2e-6f, kHalfPi, eulerZyxForDisplay(q).pitch);
    }
  }
  // Reached a different way: lying on its side at any heading, then raised
  // 90 deg about the horizontal axis square to the nose.
  for (float heading : kHeadings) {
    for (float roll : kRolls) {
      setCase("raised: heading %g roll %g", heading, roll);
      const Quat lying = fromAxisAngle(kDown, heading) * fromAxisAngle(kBodyX, roll);
      const Quat raised = rotatedInNed(lying, cross(noseNed(lying), kUp), kHalfPi);
      EXPECT_NEAR(1e-6f, 0.0f, tilt(raised));
    }
  }
  // And the two nose-up quaternions written out in the porting brief.
  setCase("literal quaternions");
  EXPECT_NEAR(1e-6f, 0.0f, tilt(Quat{std::sqrt(0.5f), 0.0f, std::sqrt(0.5f), 0.0f}));
  EXPECT_NEAR(1e-6f, 0.0f, tilt(Quat{0.5f, 0.5f, 0.5f, -0.5f}));
  // The identity is NOT nose-up: it lies on its side pointing north.
  EXPECT_NEAR(1e-6f, kHalfPi, tilt(kIdentityQuat));
}

void test_tilt_is_the_angle_between_nose_and_up() {
  const float tilts[] = {0.0f, 1e-3f, 1.0f * kDeg, 8.0f * kDeg, 30.0f * kDeg, 60.0f * kDeg, kHalfPi,
                         120.0f * kDeg, 170.0f * kDeg, 179.9f * kDeg, kPi};
  const float azimuths[] = {0.0f, 1.0f, 2.2f, -2.9f, -0.6f};
  for (float want : tilts) {
    for (float azimuth : azimuths) {
      for (float roll : kRolls) {
        for (float heading : kHeadings) {
          setCase("tilt %g azimuth %g roll %g heading %g", want, azimuth, roll, heading);
          const Vec3 axis = horizontal(azimuth);
          const Quat q = rotatedInNed(noseUp(roll, heading), axis, want);
          EXPECT_NEAR(2e-6f, want, tilt(q));
          EXPECT_NEAR(2e-6f, want, tilt(negated(q)));
          // The nose leans over towards (axis x up).
          const Vec3 nose = kUp * std::cos(want) + cross(axis, kUp) * std::sin(want);
          EXPECT_VEC_NEAR(2e-6f, nose, noseNed(q));
          // Rolling about the nose cannot change the tilt.
          EXPECT_NEAR(2e-6f, want, tilt(rotatedInBody(q, kBodyX, 0.9f)));
        }
      }
    }
  }
  // Leaning over in body axes gives the same angle: pitch and yaw both tilt.
  EXPECT_NEAR(2e-6f, 0.25f, tilt(rotatedInBody(noseUp(0.4f, 1.0f), kBodyY, 0.25f)));
  EXPECT_NEAR(2e-6f, 0.25f, tilt(rotatedInBody(noseUp(0.4f, 1.0f), kBodyZ, -0.25f)));
  // Straight down.
  EXPECT_NEAR(1e-6f, kPi, tilt(fromAxisAngle(kBodyY, -kHalfPi)));
}

void test_small_tilts_are_resolved() {
  // acos(dot(nose, up)) in float cannot tell anything below about 3e-4 rad
  // from zero. A 1e-4 rad tilt must be resolved to better than 1e-6 rad.
  const float tilts[] = {1e-6f, 1e-5f, 1e-4f, 3e-4f, 1e-3f};
  const float azimuths[] = {0.0f, 1.0f, 2.2f, -2.9f, -0.6f};
  for (float want : tilts) {
    for (float azimuth : azimuths) {
      for (float roll : kRolls) {
        for (float heading : kHeadings) {
          setCase("tilt %g azimuth %g roll %g heading %g", want, azimuth, roll, heading);
          EXPECT_NEAR(1e-6f, want, tilt(rotatedInNed(noseUp(roll, heading), horizontal(azimuth), want)));
          EXPECT_NEAR(1e-6f, want, tilt(rotatedInBody(noseUp(roll, heading), Vec3{0.0f, std::cos(azimuth), std::sin(azimuth)}, want)));
        }
      }
    }
  }
}

// ===========================================================================
// Attitude error
// ===========================================================================

// References used by the error tests: nose-up at several rolls and headings,
// the identity, and two tilted, rolled attitudes.
constexpr int kReferenceCount = 9;
Quat referenceAttitude(int index) {
  switch (index) {
    case 0: return noseUp(0.0f, 0.0f);
    case 1: return noseUp(1.1f, 0.0f);
    case 2: return noseUp(-2.6f, 2.0f);
    case 3: return noseUp(3.0f, -1.3f);
    case 4: return Quat{0.5f, 0.5f, 0.5f, -0.5f};  // the brief's nose-up quaternion
    case 5: return kIdentityQuat;
    case 6: return fromAxisAngle(Vec3{0.6f, -0.48f, 0.64f}, 1.1f);
    case 7: return rotatedInNed(noseUp(0.7f, 0.2f), horizontal(1.9f), 0.35f);
    default: return negated(noseUp(0.5f, 1.0f));  // a reference with w < 0
  }
}

void test_error_is_zero_at_the_reference() {
  for (int i = 0; i < kReferenceCount; ++i) {
    setCase("reference %d", i);
    const Quat ref = referenceAttitude(i);
    EXPECT_VEC_NEAR(1e-7f, (Vec3{0.0f, 0.0f, 0.0f}), attitudeError(ref, ref));
    EXPECT_VEC_NEAR(1e-7f, (Vec3{0.0f, 0.0f, 0.0f}), attitudeError(ref, negated(ref)));
  }
}

void test_error_axes_and_signs() {
  // A small positive turn about one body axis, applied in the body frame,
  // must show up as +angle on exactly that component: [x] roll, [y] pitch,
  // [z] yaw. A sign or axis mix-up here drives the canards the wrong way.
  const float angles[] = {1e-4f, -1e-3f, 0.01f, 5.0f * kDeg, -3.0f * kDeg, 20.0f * kDeg, -0.6f};
  for (int i = 0; i < kReferenceCount; ++i) {
    const Quat ref = referenceAttitude(i);
    for (float a : angles) {
      setCase("reference %d angle %g about body x (roll)", i, a);
      EXPECT_VEC_NEAR(2e-6f, (Vec3{a, 0.0f, 0.0f}), attitudeError(ref, rotatedInBody(ref, kBodyX, a)));
      setCase("reference %d angle %g about body y (pitch)", i, a);
      EXPECT_VEC_NEAR(2e-6f, (Vec3{0.0f, a, 0.0f}), attitudeError(ref, rotatedInBody(ref, kBodyY, a)));
      setCase("reference %d angle %g about body z (yaw)", i, a);
      EXPECT_VEC_NEAR(2e-6f, (Vec3{0.0f, 0.0f, a}), attitudeError(ref, rotatedInBody(ref, kBodyZ, a)));
    }
  }
}

void test_error_signs_in_physical_terms() {
  // Nose-up, rolled so that body z points north and body y points east.
  const Quat ref = Quat{std::sqrt(0.5f), 0.0f, std::sqrt(0.5f), 0.0f};
  EXPECT_VEC_NEAR(1e-6f, kNorth, rotate(ref, kBodyZ));
  EXPECT_VEC_NEAR(1e-6f, kEast, rotate(ref, kBodyY));
  // The nose leaning towards body +z (north here) is a turn about body -y:
  // a NEGATIVE pitch error.
  const Quat leanNorth = rotatedInNed(ref, kEast, -0.1f);
  TEST_ASSERT_TRUE(noseNed(leanNorth).x > 0.05f);
  EXPECT_VEC_NEAR(2e-6f, (Vec3{0.0f, -0.1f, 0.0f}), attitudeError(ref, leanNorth));
  // The nose leaning towards body +y (east here) is a turn about body +z:
  // a POSITIVE yaw error.
  const Quat leanEast = rotatedInNed(ref, kNorth, 0.1f);
  TEST_ASSERT_TRUE(noseNed(leanEast).y > 0.05f);
  EXPECT_VEC_NEAR(2e-6f, (Vec3{0.0f, 0.0f, 0.1f}), attitudeError(ref, leanEast));
  // Seen from the tail, a positive roll error is clockwise: body y (east)
  // moves towards body z (north).
  const Quat rolled = rotatedInBody(ref, kBodyX, 0.1f);
  TEST_ASSERT_TRUE(rotate(rolled, kBodyY).x > 0.05f);
  EXPECT_VEC_NEAR(2e-6f, (Vec3{0.1f, 0.0f, 0.0f}), attitudeError(ref, rolled));
}

void test_error_is_the_body_rotation_vector() {
  // Any rotation vector applied in body axes comes back element for element.
  const Vec3 errors[] = {{4.0f * kDeg, -7.0f * kDeg, 12.0f * kDeg}, {-0.3f, 0.2f, 0.1f}, {1e-5f, -2e-5f, 3e-5f},
                         {1.0f, 1.0f, -1.0f}, {-2.0f, 0.5f, 1.5f}, {0.0f, 0.0f, 3.0f}};
  for (int i = 0; i < kReferenceCount; ++i) {
    const Quat ref = referenceAttitude(i);
    for (const Vec3& want : errors) {
      setCase("reference %d error (%g, %g, %g)", i, want.x, want.y, want.z);
      const Quat q = ref * fromRotationVector(want);
      EXPECT_VEC_NEAR(3e-6f, want, attitudeError(ref, q));
      // Swapping the two attitudes reverses the error.
      EXPECT_VEC_NEAR(3e-6f, -want, attitudeError(q, ref));
    }
  }
}

void test_error_is_the_same_for_q_and_minus_q() {
  const Vec3 errors[] = {{0.02f, -0.01f, 0.03f}, {-0.3f, 0.2f, 0.1f}, {170.0f * kDeg, 0.0f, 0.0f},
                         {0.0f, 2.5f, 0.0f}, {1.0f, -2.0f, 1.5f}};
  for (int i = 0; i < kReferenceCount; ++i) {
    const Quat ref = referenceAttitude(i);
    for (const Vec3& want : errors) {
      setCase("reference %d error (%g, %g, %g)", i, want.x, want.y, want.z);
      const Quat q = ref * fromRotationVector(want);
      const Vec3 error = attitudeError(ref, q);
      EXPECT_VEC_NEAR(3e-6f, want, error);
      // The estimator may hand over either sign of the same attitude, and the
      // sign may flip mid-flight. The controller must not see a 360 deg jump.
      EXPECT_VEC_NEAR(1e-6f, error, attitudeError(ref, negated(q)));
      EXPECT_VEC_NEAR(1e-6f, error, attitudeError(negated(ref), q));
      EXPECT_VEC_NEAR(1e-6f, error, attitudeError(negated(ref), negated(q)));
    }
  }
}

void test_error_takes_the_shorter_way_round() {
  // 190 deg of roll one way is 170 deg the other way.
  for (int i = 0; i < kReferenceCount; ++i) {
    setCase("reference %d", i);
    const Quat ref = referenceAttitude(i);
    const Vec3 error = attitudeError(ref, rotatedInBody(ref, kBodyX, 190.0f * kDeg));
    EXPECT_VEC_NEAR(5e-6f, (Vec3{-170.0f * kDeg, 0.0f, 0.0f}), error);
  }
}

void test_error_ignores_quaternion_length() {
  const Quat ref = referenceAttitude(7);
  const Vec3 want{0.05f, -0.2f, 0.1f};
  const Quat q = ref * fromRotationVector(want);
  EXPECT_VEC_NEAR(3e-6f, want, attitudeError(scaled(ref, 1.02f), scaled(q, 0.97f)));
  EXPECT_VEC_NEAR(3e-6f, want, attitudeError(scaled(ref, 30.0f), scaled(q, 1e-3f)));
}

// ===========================================================================
// Vertical reference
// ===========================================================================

void test_vertical_reference_removes_tilt_and_keeps_roll() {
  const float tilts[] = {1e-4f, 1.0f * kDeg, 8.0f * kDeg, 30.0f * kDeg, 60.0f * kDeg, kHalfPi,
                         120.0f * kDeg, 150.0f * kDeg, 170.0f * kDeg, 179.0f * kDeg};
  const float azimuths[] = {0.0f, 1.0f, 2.2f, -2.9f, -0.6f};
  for (float tiltAngle : tilts) {
    for (float azimuth : azimuths) {
      for (float roll : kRolls) {
        for (float heading : kHeadings) {
          setCase("tilt %g azimuth %g roll %g heading %g", tiltAngle, azimuth, roll, heading);
          // An upright attitude, leaned over about a horizontal axis.
          const Quat upright = noseUp(roll, heading);
          const Quat q = rotatedInNed(upright, horizontal(azimuth), tiltAngle);
          const Quat ref = verticalReference(q);

          EXPECT_TRUE(isUnit(ref, 1e-5f));
          // The tilt is gone.
          EXPECT_NEAR(2e-6f, 0.0f, tilt(ref));
          // The roll is kept: getting from the reference back to q takes no
          // turn about the nose at all, only the tilt itself.
          const Vec3 error = attitudeError(ref, q);
          EXPECT_NEAR(3e-6f, 0.0f, error.x);
          EXPECT_NEAR(3e-6f, tilt(q), norm(error));
          // q and -q give the same reference attitude.
          EXPECT_NEAR(3e-6f, 0.0f, norm(attitudeError(ref, verticalReference(negated(q)))));

          // It is the smallest rotation, so it undoes the lean exactly and
          // lands on the upright attitude we started from. (Close to straight
          // down the lean axis is poorly determined by the nose direction, so
          // this comparison stops at 150 deg.)
          if (tiltAngle <= 150.5f * kDeg) {
            EXPECT_NEAR(5e-6f, 0.0f, norm(attitudeError(upright, ref)));
          }
        }
      }
    }
  }
}

void test_vertical_reference_keeps_roll_when_rolled_first() {
  // Spin the rocket about its nose by `spin`, lean it over, take the
  // reference: the reference must have spun by exactly `spin` too.
  const float spins[] = {0.2f, -1.0f, 2.8f};
  for (float spin : spins) {
    for (float heading : kHeadings) {
      setCase("spin %g heading %g", spin, heading);
      const Quat upright = noseUp(0.3f, heading);
      const Vec3 axis = horizontal(heading - 0.8f);
      const Quat refBefore = verticalReference(rotatedInNed(upright, axis, 0.3f));
      const Quat refAfter = verticalReference(rotatedInNed(rotatedInBody(upright, kBodyX, spin), axis, 0.3f));
      EXPECT_VEC_NEAR(4e-6f, (Vec3{spin, 0.0f, 0.0f}), attitudeError(refBefore, refAfter));
      EXPECT_NEAR(4e-6f, spin, wrapPi(rollAboutNose(refAfter) - rollAboutNose(refBefore)));
    }
  }
}

void test_vertical_reference_of_nose_up_is_unchanged() {
  for (float roll : kRolls) {
    for (float heading : kHeadings) {
      setCase("roll %g heading %g", roll, heading);
      const Quat q = noseUp(roll, heading);
      EXPECT_QUAT_NEAR(1e-6f, q, verticalReference(q));
      EXPECT_QUAT_NEAR(1e-6f, negated(q), verticalReference(negated(q)));
    }
  }
}

void test_vertical_reference_straight_down_returns_q() {
  // Straight down there is no smallest rotation to vertical (every horizontal
  // axis is as good as any other), so the attitude is handed back as it is.
  for (float roll : kRolls) {
    for (float heading : kHeadings) {
      setCase("pitched down: roll %g heading %g", roll, heading);
      const Quat down = fromAxisAngle(kDown, heading) * fromAxisAngle(kBodyY, -kHalfPi) * fromAxisAngle(kBodyX, roll);
      EXPECT_VEC_NEAR(1e-6f, kDown, noseNed(down));
      EXPECT_NEAR(1e-6f, kPi, tilt(down));
      EXPECT_QUAT_NEAR(1e-6f, down, verticalReference(down));
      EXPECT_QUAT_NEAR(1e-6f, negated(down), verticalReference(negated(down)));

      setCase("flipped over: roll %g heading %g", roll, heading);
      const Quat flipped = rotatedInNed(noseUp(roll, heading), horizontal(heading + 0.5f), kPi);
      EXPECT_QUAT_NEAR(1e-6f, flipped, verticalReference(flipped));

      // A hair away from straight down is still "almost straight down".
      setCase("nearly down: roll %g heading %g", roll, heading);
      const Quat nearly = rotatedInNed(noseUp(roll, heading), horizontal(heading + 0.5f), kPi - 1e-4f);
      EXPECT_QUAT_NEAR(1e-6f, nearly, verticalReference(nearly));
    }
  }
  // The result is always unit length, even if the input was not.
  const Quat down = fromAxisAngle(kBodyY, -kHalfPi);
  const Quat ref = verticalReference(scaled(down, 3.0f));
  TEST_ASSERT_TRUE(isUnit(ref, 1e-5f));
  EXPECT_QUAT_NEAR(1e-6f, down, ref);
}

void test_vertical_reference_ignores_quaternion_length() {
  const Quat q = rotatedInNed(noseUp(0.9f, -0.4f), horizontal(2.0f), 0.4f);
  const Quat ref = verticalReference(q);
  const float scales[] = {0.5f, 1.01f, 20.0f};
  for (float s : scales) {
    setCase("scale %g", s);
    const Quat refScaled = verticalReference(scaled(q, s));
    TEST_ASSERT_TRUE(isUnit(refScaled, 1e-5f));
    EXPECT_QUAT_NEAR(2e-6f, ref, refScaled);
  }
}

// ===========================================================================
// Roll about the nose
// ===========================================================================

void test_roll_about_nose_differences_equal_the_applied_roll() {
  const float applied[] = {-3.0f, -1.5f, -0.2f, 1e-3f, 0.7f, 2.9f};
  const Quat bases[] = {kIdentityQuat,
                        noseUp(0.0f, 0.0f),
                        noseUp(1.1f, 0.6f),
                        noseUp(-2.0f, -2.5f),
                        rotatedInNed(noseUp(0.4f, 1.0f), horizontal(0.3f), 0.5f),
                        rotatedInNed(noseUp(-0.9f, 2.0f), horizontal(-1.2f), 1.2f),
                        fromAxisAngle(Vec3{0.6f, -0.48f, 0.64f}, 1.1f)};
  int index = 0;
  for (const Quat& base : bases) {
    for (float a : applied) {
      setCase("base %d applied %g", index, a);
      const Quat rolled = rotatedInBody(base, kBodyX, a);
      EXPECT_NEAR(3e-6f, a, wrapPi(rollAboutNose(rolled) - rollAboutNose(base)));
      // Same answer whichever sign the estimator hands over.
      EXPECT_NEAR(3e-6f, a, wrapPi(rollAboutNose(negated(rolled)) - rollAboutNose(base)));
      EXPECT_NEAR(3e-6f, a, wrapPi(rollAboutNose(scaled(rolled, 4.0f)) - rollAboutNose(base)));
    }
    ++index;
  }
}

void test_roll_about_nose_of_pure_rolls_and_pure_swings() {
  // A pure roll from the identity reads as that roll.
  const float rolls[] = {-3.1f, -kHalfPi, -0.01f, 0.0f, 1e-4f, 1.0f, kHalfPi, 3.1f};
  for (float roll : rolls) {
    setCase("roll %g", roll);
    EXPECT_NEAR(2e-6f, roll, rollAboutNose(fromAxisAngle(kBodyX, roll)));
  }
  // A turn about any axis square to the nose has no roll in it.
  const float swings[] = {-3.0f, -1.0f, 0.3f, 2.0f, 3.1f, 4.0f};
  for (float swing : swings) {
    setCase("swing %g", swing);
    EXPECT_NEAR(2e-6f, 0.0f, rollAboutNose(fromAxisAngle(kBodyY, swing)));
    EXPECT_NEAR(2e-6f, 0.0f, rollAboutNose(fromAxisAngle(Vec3{0.0f, 0.6f, -0.8f}, swing)));
  }
}

void test_roll_about_nose_range() {
  // The result lies in (-pi, pi]: half a turn either way reads as +pi.
  EXPECT_NEAR(1e-6f, kPi, rollAboutNose(Quat{0.0f, 1.0f, 0.0f, 0.0f}));
  EXPECT_NEAR(1e-6f, kPi, rollAboutNose(Quat{0.0f, -1.0f, 0.0f, 0.0f}));
  for (int step = -40; step <= 40; ++step) {
    const float a = 0.37f * static_cast<float>(step);
    setCase("applied %g", a);
    const float roll = rollAboutNose(rotatedInBody(noseUp(0.3f, 0.9f), kBodyX, a));
    EXPECT_TRUE(roll > -kPi - 1e-6f && roll <= kPi + 1e-6f);
  }
  // A 180 deg swing has no defined roll. It must still give a finite number.
  TEST_ASSERT_TRUE(std::isfinite(rollAboutNose(Quat{0.0f, 0.0f, 1.0f, 0.0f})));
  TEST_ASSERT_TRUE(std::isfinite(rollAboutNose(Quat{0.0f, 0.0f, 0.6f, -0.8f})));
}

// ===========================================================================
// Bad input never produces NaN
// ===========================================================================

void test_flight_helpers_stay_finite_on_bad_quaternions() {
  // Navigation rejects a non-finite or non-unit attitude before anything uses
  // it. If one slips through anyway, these helpers must not spread NaN.
  const Quat bad[] = {{kNan, 0.0f, 0.0f, 0.0f}, {1.0f, kNan, 0.0f, 0.0f}, {0.5f, 0.5f, kNan, 0.5f},
                      {1.0f, 0.0f, 0.0f, kNan}, {kInf, 0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, -kInf, 0.0f},
                      {kNan, kNan, kNan, kNan}, {0.0f, 0.0f, 0.0f, 0.0f}, {1e-30f, 0.0f, 1e-30f, 0.0f}};
  const Quat good = rotatedInNed(noseUp(0.4f, 1.0f), horizontal(0.3f), 0.2f);
  for (const Quat& q : bad) {
    setCase("q = [%g, %g, %g, %g]", q.w, q.x, q.y, q.z);
    const float t = tilt(q);
    EXPECT_TRUE(std::isfinite(t) && t >= 0.0f && t <= kPi + 1e-6f);
    EXPECT_TRUE(isFinite(attitudeError(q, good)));
    EXPECT_TRUE(isFinite(attitudeError(good, q)));
    EXPECT_TRUE(isFinite(attitudeError(q, q)));
    EXPECT_TRUE(isUnit(verticalReference(q), 1e-5f));
    EXPECT_TRUE(std::isfinite(rollAboutNose(q)));
    const EulerZyx euler = eulerZyxForDisplay(q);
    EXPECT_TRUE(std::isfinite(euler.yaw) && std::isfinite(euler.pitch) && std::isfinite(euler.roll));
  }
}

// ===========================================================================
// The 8 "Math tests" of Canards-Rocket test/test_desktop/test_canards.cpp,
// ported to the new frame.
//
// The old code used the same quaternion convention but labelled the body axes
// differently: old x = pitch, old y = yaw, old z = roll (nose). The new frame
// has the nose on x. Each test below builds the old test's attitude with the
// OLD axis labels, converts it, and expects the old answer re-labelled.
// ===========================================================================

// Old body axes in terms of the new ones: old x = new y, old y = new z,
// old z = new x. So a vector's old components (a, b, c) are (c, a, b) in the
// new frame.
Vec3 fromOldAxes(const Vec3& vOld) { return {vOld.z, vOld.x, vOld.y}; }

// The rotation that takes new-frame body components to old-frame ones: -120
// deg about (1, 1, 1). The same physical attitude is  qNew = qOld (x) this.
Quat oldBodyFromNewBody() { return fromAxisAngle(Vec3{1.0f, 1.0f, 1.0f}, -120.0f * kDeg); }

Quat fromOldFrame(const Quat& qOld) { return qOld * oldBodyFromNewBody(); }

// The old tests' "nose (+z body) straight up" attitude, in the OLD frame.
const Quat kOldVertical{0.0f, 1.0f, 0.0f, 0.0f};

void test_old_frame_conversion_is_right() {
  // The porting brief derived [0.5, -0.5, -0.5, -0.5]; check it rather than
  // trust it.
  EXPECT_QUAT_NEAR(1e-6f, (Quat{0.5f, -0.5f, -0.5f, -0.5f}), oldBodyFromNewBody());
  // It maps each new body axis onto the old label for the same direction.
  EXPECT_VEC_NEAR(1e-6f, (Vec3{0.0f, 0.0f, 1.0f}), rotate(oldBodyFromNewBody(), kBodyX));  // nose: old z
  EXPECT_VEC_NEAR(1e-6f, (Vec3{1.0f, 0.0f, 0.0f}), rotate(oldBodyFromNewBody(), kBodyY));  // pitch: old x
  EXPECT_VEC_NEAR(1e-6f, (Vec3{0.0f, 1.0f, 0.0f}), rotate(oldBodyFromNewBody(), kBodyZ));  // yaw: old y
  // The defining property: one physical vector, written in old axes with the
  // old quaternion or in new axes with the new one, lands on the same NED
  // vector.
  const Quat olds[] = {kOldVertical, fromAxisAngle(Vec3{0.6f, -0.48f, 0.64f}, 1.1f), kIdentityQuat};
  const Vec3 vectors[] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.3f, -1.1f, 2.0f}};
  for (const Quat& qOld : olds) {
    for (const Vec3& vOld : vectors) {
      setCase("qOld [%g, %g, %g, %g] vOld (%g, %g, %g)", qOld.w, qOld.x, qOld.y, qOld.z, vOld.x, vOld.y, vOld.z);
      EXPECT_VEC_NEAR(2e-6f, rotate(qOld, vOld), rotate(fromOldFrame(qOld), fromOldAxes(vOld)));
    }
  }
  // The brief's worked example: the old vertical becomes [0.5, 0.5, 0.5, -0.5].
  setCase("old vertical");
  EXPECT_QUAT_NEAR(1e-6f, (Quat{0.5f, 0.5f, 0.5f, -0.5f}), fromOldFrame(kOldVertical));
}

// Old #1: test_vertical_attitude_has_nose_up
void test_ported_vertical_attitude_has_nose_up() {
  const Quat vertical = fromOldFrame(kOldVertical);
  const Vec3 nose = rotate(vertical, fromOldAxes(Vec3{0.0f, 0.0f, 1.0f}));
  EXPECT_VEC_NEAR(1e-6f, (Vec3{0.0f, 0.0f, -1.0f}), nose);
  EXPECT_VEC_NEAR(0.0f, kNoseBody, fromOldAxes(Vec3{0.0f, 0.0f, 1.0f}));  // the old nose is the new nose
  EXPECT_NEAR(1e-6f, 0.0f, tilt(vertical));
}

// Old #2: test_error_is_the_body_rotation_vector
void test_ported_error_is_the_body_rotation_vector() {
  const Quat referenceOld = fromAxisAngle(Vec3{0.6f, -0.48f, 0.64f}, 1.1f);
  const Vec3 wantOld{4.0f * kDeg, -7.0f * kDeg, 12.0f * kDeg};
  const Quat qOld = rotatedInBody(referenceOld, wantOld, norm(wantOld));
  const Vec3 error = attitudeError(fromOldFrame(referenceOld), fromOldFrame(qOld));
  // Old [pitch, yaw, roll] = (4, -7, 12) deg is new [roll, pitch, yaw] = (12, 4, -7) deg.
  EXPECT_VEC_NEAR(1e-5f, (Vec3{12.0f * kDeg, 4.0f * kDeg, -7.0f * kDeg}), error);
  EXPECT_VEC_NEAR(1e-5f, fromOldAxes(wantOld), error);
}

// Old #3: test_error_axes_and_signs
void test_ported_error_axes_and_signs() {
  const Quat vertical = fromOldFrame(kOldVertical);
  // Old: +5 deg about old x (pitch) -> error (5 deg, 0, 0). New: pitch is y.
  Quat q = fromOldFrame(rotatedInBody(kOldVertical, Vec3{1.0f, 0.0f, 0.0f}, 5.0f * kDeg));
  EXPECT_VEC_NEAR(1e-5f, (Vec3{0.0f, 5.0f * kDeg, 0.0f}), attitudeError(vertical, q));
  // Old: -3 deg about old y (yaw) -> error[1] = -3 deg. New: yaw is z.
  q = fromOldFrame(rotatedInBody(kOldVertical, Vec3{0.0f, 1.0f, 0.0f}, -3.0f * kDeg));
  EXPECT_VEC_NEAR(1e-5f, (Vec3{0.0f, 0.0f, -3.0f * kDeg}), attitudeError(vertical, q));
  // Old: +20 deg about old z (roll) -> error[2] = +20 deg. New: roll is x.
  q = fromOldFrame(rotatedInBody(kOldVertical, Vec3{0.0f, 0.0f, 1.0f}, 20.0f * kDeg));
  EXPECT_VEC_NEAR(1e-5f, (Vec3{20.0f * kDeg, 0.0f, 0.0f}), attitudeError(vertical, q));

  // The same three turns made directly about the NEW axes agree.
  EXPECT_VEC_NEAR(1e-5f, (Vec3{0.0f, 5.0f * kDeg, 0.0f}), attitudeError(vertical, rotatedInBody(vertical, kBodyY, 5.0f * kDeg)));
  EXPECT_VEC_NEAR(1e-5f, (Vec3{0.0f, 0.0f, -3.0f * kDeg}), attitudeError(vertical, rotatedInBody(vertical, kBodyZ, -3.0f * kDeg)));
  EXPECT_VEC_NEAR(1e-5f, (Vec3{20.0f * kDeg, 0.0f, 0.0f}), attitudeError(vertical, rotatedInBody(vertical, kBodyX, 20.0f * kDeg)));
}

// Old #4: test_error_is_the_same_for_q_and_minus_q
void test_ported_error_is_the_same_for_q_and_minus_q() {
  const Quat vertical = fromOldFrame(kOldVertical);
  const Quat q = fromOldFrame(rotatedInBody(kOldVertical, Vec3{0.0f, 0.0f, 1.0f}, 170.0f * kDeg));
  const Vec3 error = attitudeError(vertical, q);
  const Vec3 flipped = attitudeError(vertical, negated(q));
  EXPECT_NEAR(1e-4f, 170.0f * kDeg, error.x);  // old error[2] (roll) is new x
  EXPECT_NEAR(1e-5f, error.x, flipped.x);
}

// Old #5: test_vertical_reference_removes_tilt_and_keeps_roll
void test_ported_vertical_reference_removes_tilt_and_keeps_roll() {
  // Rolled 40 degrees, then tilted 8 degrees about an oblique body axis.
  const Quat rolledOld = rotatedInBody(kOldVertical, Vec3{0.0f, 0.0f, 1.0f}, 40.0f * kDeg);
  const Quat qOld = rotatedInBody(rolledOld, Vec3{0.6f, 0.8f, 0.0f}, 8.0f * kDeg);
  const Quat rolled = fromOldFrame(rolledOld);
  const Quat q = fromOldFrame(qOld);
  EXPECT_NEAR(1e-4f, 8.0f * kDeg, tilt(q));

  const Quat reference = verticalReference(q);
  EXPECT_NEAR(1e-4f, 0.0f, tilt(reference));

  // The error back to the reference is exactly the tilt, with no roll in it.
  // Old (0.6, 0.8, 0) * 8 deg in [pitch, yaw, roll] is new (0, 0.6, 0.8) * 8 deg.
  const Vec3 error = attitudeError(reference, q);
  EXPECT_NEAR(1e-5f, 0.0f, error.x);
  EXPECT_NEAR(1e-4f, 0.6f * 8.0f * kDeg, error.y);
  EXPECT_NEAR(1e-4f, 0.8f * 8.0f * kDeg, error.z);

  // And the reference is the rolled, untilted attitude.
  EXPECT_VEC_NEAR(1e-4f, (Vec3{0.0f, 0.0f, 0.0f}), attitudeError(rolled, reference));
}

// Old #6: test_vertical_reference_of_vertical_is_unchanged
void test_ported_vertical_reference_of_vertical_is_unchanged() {
  const Quat vertical = fromOldFrame(kOldVertical);
  EXPECT_QUAT_NEAR(1e-6f, vertical, verticalReference(vertical));
}

// Old #7: test_velocity_integration. The old integrateVelocity() is not part
// of lib/Frames (the estimator owns velocity now), but it is one line of
// rotate(): velNed += (R(q) * accelBody + gravity) * dt. Ported with that
// line here, because it checks which body axis rotate() treats as the nose.
void integrateVelocity(const Quat& q, const Vec3& accelBody, float dt, Vec3& velocityNed) {
  const Vec3 gravityNed{0.0f, 0.0f, FlightConfig::kGravity};
  velocityNed = velocityNed + (rotate(q, accelBody) + gravityNed) * dt;
}

void test_ported_velocity_integration() {
  const Quat vertical = fromOldFrame(kOldVertical);
  // Sitting on the pad, nose up: the accelerometer reads +1 g along the nose.
  Vec3 velocity;
  const Vec3 atRest = fromOldAxes(Vec3{0.0f, 0.0f, FlightConfig::kGravity});
  EXPECT_VEC_NEAR(0.0f, (Vec3{FlightConfig::kGravity, 0.0f, 0.0f}), atRest);  // along the new nose, body +x
  for (int i = 0; i < 200; ++i) integrateVelocity(vertical, atRest, 0.005f, velocity);
  EXPECT_NEAR(1e-3f, 0.0f, velocity.z);

  // 3 g along the nose for one second is 2 g of climb (NED: up is -z).
  const Vec3 thrust = fromOldAxes(Vec3{0.0f, 0.0f, 3.0f * FlightConfig::kGravity});
  for (int i = 0; i < 200; ++i) integrateVelocity(vertical, thrust, 0.005f, velocity);
  EXPECT_NEAR(1e-2f, -2.0f * FlightConfig::kGravity, velocity.z);
  EXPECT_NEAR(1e-3f, 0.0f, velocity.x);
}

// Old #8: test_axis_map (applyAxisMap is now Mat3 * Vec3)
void test_ported_axis_map() {
  const Mat3 map = {{{0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}}};
  const Vec3 out = map * Vec3{1.0f, 2.0f, 3.0f};
  TEST_ASSERT_EQUAL_FLOAT(-3.0f, out.x);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, out.y);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, out.z);
}

// ===========================================================================
// FramesDisplay.h: Euler angles for display only
// ===========================================================================

void test_display_euler_matches_a_yaw_pitch_roll_build() {
  // Away from the singularity: pitch up to +/-80 deg.
  const float yaws[] = {-3.0f, -1.0f, 0.0f, 0.5f, 2.8f};
  const float pitches[] = {-80.0f * kDeg, -0.5f, 0.0f, 0.3f, 1.2f, 80.0f * kDeg};
  const float rolls[] = {-3.0f, -0.7f, 0.0f, 1.0f, 3.1f};
  for (float yaw : yaws) {
    for (float pitch : pitches) {
      for (float roll : rolls) {
        setCase("yaw %g pitch %g roll %g", yaw, pitch, roll);
        const Quat q = fromAxisAngle(kDown, yaw) * fromAxisAngle(kBodyY, pitch) * fromAxisAngle(kBodyX, roll);
        const EulerZyx euler = eulerZyxForDisplay(q);
        EXPECT_NEAR(5e-6f, 0.0f, wrapPi(euler.yaw - yaw));
        EXPECT_NEAR(5e-6f, pitch, euler.pitch);
        EXPECT_NEAR(5e-6f, 0.0f, wrapPi(euler.roll - roll));
        // q and -q read the same.
        const EulerZyx flipped = eulerZyxForDisplay(negated(q));
        EXPECT_NEAR(1e-6f, euler.yaw, flipped.yaw);
        EXPECT_NEAR(1e-6f, euler.pitch, flipped.pitch);
        EXPECT_NEAR(1e-6f, euler.roll, flipped.roll);
        // The display pitch and the flight tilt describe the same nose.
        EXPECT_NEAR(3e-6f, kHalfPi - tilt(q), euler.pitch);
      }
    }
  }
}

void test_display_euler_sign_conventions() {
  EulerZyx euler = eulerZyxForDisplay(kIdentityQuat);
  EXPECT_NEAR(0.0f, 0.0f, euler.yaw);
  EXPECT_NEAR(0.0f, 0.0f, euler.pitch);
  EXPECT_NEAR(0.0f, 0.0f, euler.roll);
  // Nose pointing east is a heading of +90 deg.
  euler = eulerZyxForDisplay(Quat{std::sqrt(0.5f), 0.0f, 0.0f, std::sqrt(0.5f)});
  EXPECT_NEAR(1e-6f, kHalfPi, euler.yaw);
  EXPECT_NEAR(1e-6f, 0.0f, euler.pitch);
  EXPECT_NEAR(1e-6f, 0.0f, euler.roll);
  // Nose 30 deg above the horizon is a pitch of +30 deg.
  const Quat pitched = fromAxisAngle(kBodyY, 30.0f * kDeg);
  TEST_ASSERT_TRUE(noseNed(pitched).z < 0.0f);
  euler = eulerZyxForDisplay(pitched);
  EXPECT_NEAR(1e-6f, 0.0f, euler.yaw);
  EXPECT_NEAR(1e-6f, 30.0f * kDeg, euler.pitch);
  EXPECT_NEAR(1e-6f, 0.0f, euler.roll);
  // Body y dipping below the horizon is a positive roll.
  const Quat rolled = fromAxisAngle(kBodyX, 20.0f * kDeg);
  TEST_ASSERT_TRUE(rotate(rolled, kBodyY).z > 0.0f);
  euler = eulerZyxForDisplay(rolled);
  EXPECT_NEAR(1e-6f, 0.0f, euler.yaw);
  EXPECT_NEAR(1e-6f, 0.0f, euler.pitch);
  EXPECT_NEAR(1e-6f, 20.0f * kDeg, euler.roll);
  // An unnormalised quaternion reads the same.
  const Quat q = fromAxisAngle(kDown, 0.4f) * fromAxisAngle(kBodyY, -0.3f) * fromAxisAngle(kBodyX, 1.0f);
  euler = eulerZyxForDisplay(scaled(q, 5.0f));
  EXPECT_NEAR(3e-6f, 0.4f, euler.yaw);
  EXPECT_NEAR(3e-6f, -0.3f, euler.pitch);
  EXPECT_NEAR(3e-6f, 1.0f, euler.roll);
}

void test_display_euler_is_finite_at_the_singularity() {
  // Nose-up and nose-down: yaw and roll mean nothing there, but the pitch is
  // right and nothing is NaN.
  for (float roll : kRolls) {
    for (float heading : kHeadings) {
      setCase("roll %g heading %g", roll, heading);
      const EulerZyx up = eulerZyxForDisplay(noseUp(roll, heading));
      EXPECT_TRUE(std::isfinite(up.yaw) && std::isfinite(up.roll));
      EXPECT_NEAR(2e-6f, kHalfPi, up.pitch);
      const Quat down = fromAxisAngle(kDown, heading) * fromAxisAngle(kBodyY, -kHalfPi) * fromAxisAngle(kBodyX, roll);
      const EulerZyx below = eulerZyxForDisplay(down);
      EXPECT_TRUE(std::isfinite(below.yaw) && std::isfinite(below.roll));
      EXPECT_NEAR(2e-6f, -kHalfPi, below.pitch);
    }
  }
}

}  // namespace

void setUp() { gCase[0] = '\0'; }
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();

  RUN_TEST(test_vector_operators);
  RUN_TEST(test_cross_is_right_handed);
  RUN_TEST(test_norm_of_vectors_and_quaternions);
  RUN_TEST(test_normalized_vector);
  RUN_TEST(test_normalized_vector_falls_back);
  RUN_TEST(test_normalized_vector_handles_very_long_vectors);
  RUN_TEST(test_is_finite_vector);
  RUN_TEST(test_mat3_times_vec3);
  RUN_TEST(test_mount_maps_are_proper_rotations);

  RUN_TEST(test_hamilton_product);
  RUN_TEST(test_conjugate_is_the_inverse_rotation);
  RUN_TEST(test_normalized_quaternion);
  RUN_TEST(test_normalized_quaternion_falls_back_to_identity);
  RUN_TEST(test_normalized_quaternion_handles_very_long_quaternions);
  RUN_TEST(test_is_finite_and_is_unit_quaternion);

  RUN_TEST(test_rotate_takes_body_vectors_into_ned);
  RUN_TEST(test_rotate_inverse_takes_ned_vectors_into_body);
  RUN_TEST(test_rotate_matches_the_axis_angle_formula);
  RUN_TEST(test_rotate_and_rotate_inverse_round_trip);
  RUN_TEST(test_product_applies_the_right_hand_factor_first);
  RUN_TEST(test_right_factor_turns_in_body_left_factor_turns_in_ned);

  RUN_TEST(test_from_axis_angle);
  RUN_TEST(test_from_axis_angle_bad_inputs_give_identity);
  RUN_TEST(test_rotation_vector_round_trip);
  RUN_TEST(test_rotation_vector_takes_the_shorter_way);
  RUN_TEST(test_rotation_vector_bad_inputs);

  RUN_TEST(test_nose_up_has_zero_tilt_at_the_euler_singularity);
  RUN_TEST(test_tilt_is_the_angle_between_nose_and_up);
  RUN_TEST(test_small_tilts_are_resolved);

  RUN_TEST(test_error_is_zero_at_the_reference);
  RUN_TEST(test_error_axes_and_signs);
  RUN_TEST(test_error_signs_in_physical_terms);
  RUN_TEST(test_error_is_the_body_rotation_vector);
  RUN_TEST(test_error_is_the_same_for_q_and_minus_q);
  RUN_TEST(test_error_takes_the_shorter_way_round);
  RUN_TEST(test_error_ignores_quaternion_length);

  RUN_TEST(test_vertical_reference_removes_tilt_and_keeps_roll);
  RUN_TEST(test_vertical_reference_keeps_roll_when_rolled_first);
  RUN_TEST(test_vertical_reference_of_nose_up_is_unchanged);
  RUN_TEST(test_vertical_reference_straight_down_returns_q);
  RUN_TEST(test_vertical_reference_ignores_quaternion_length);

  RUN_TEST(test_roll_about_nose_differences_equal_the_applied_roll);
  RUN_TEST(test_roll_about_nose_of_pure_rolls_and_pure_swings);
  RUN_TEST(test_roll_about_nose_range);

  RUN_TEST(test_flight_helpers_stay_finite_on_bad_quaternions);

  RUN_TEST(test_old_frame_conversion_is_right);
  RUN_TEST(test_ported_vertical_attitude_has_nose_up);
  RUN_TEST(test_ported_error_is_the_body_rotation_vector);
  RUN_TEST(test_ported_error_axes_and_signs);
  RUN_TEST(test_ported_error_is_the_same_for_q_and_minus_q);
  RUN_TEST(test_ported_vertical_reference_removes_tilt_and_keeps_roll);
  RUN_TEST(test_ported_vertical_reference_of_vertical_is_unchanged);
  RUN_TEST(test_ported_velocity_integration);
  RUN_TEST(test_ported_axis_map);

  RUN_TEST(test_display_euler_matches_a_yaw_pitch_roll_build);
  RUN_TEST(test_display_euler_sign_conventions);
  RUN_TEST(test_display_euler_is_finite_at_the_singularity);

  return UNITY_END();
}
