// Navigation and NullEstimator
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 5, 6.2, 6.5 and step 4
// of section 10).
//
// Navigation is tested against a FakeEstimator that records what it was given
// and returns whatever the test scripted.
#include <unity.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "Estimator/NullEstimator.h"
#include "FakeEstimator.h"
#include "FlightConfig.h"
#include "Frames.h"
#include "SensorSource.h"
#include "Subsystems/Navigation/Navigation.h"
#include "VehicleConfig.h"

void setUp() {}
void tearDown() {}

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kHalfPi = 1.57079632679489661923f;

constexpr uint64_t kStepUs = 5000;
constexpr float kStepS = 0.005f;
// Late enough that "one stale time ago" is still a positive time.
constexpr uint64_t kT0 = FlightConfig::kNavStaleUs + 10 * FlightConfig::kUsPerSecond;

Mat3 transposed(const Mat3& a) {
  Mat3 t;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) t.m[r][c] = a.m[c][r];
  }
  return t;
}

bool isIdentity(const Mat3& a) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (a.m[r][c] != (r == c ? 1.0f : 0.0f)) return false;
    }
  }
  return true;
}

// What the IMU reads, in its own axes, for a given body-axis vector. The
// mount map is a rotation, so its inverse is its transpose.
Vec3 imuSensorAxes(const Vec3& body) { return transposed(VehicleConfig::kImuToBody) * body; }

// The rocket at rest on the pad, nose up: the accelerometer feels 1 g pushing
// up through the airframe, which is along the nose (body +x).
SensorFrame padFrame(uint64_t timeUs) {
  SensorFrame f;
  f.timeUs = timeUs;
  f.imuOk = true;
  f.accel = imuSensorAxes(Vec3{FlightConfig::kGravity, 0.0f, 0.0f});
  f.gyro = Vec3{0.0f, 0.0f, 0.0f};
  return f;
}

// Body -> NED attitude with the nose `tiltRad` away from straight up.
// Rotating body x about NED y by +90 deg points it at NED -z, which is up.
Quat noseUpTilted(float tiltRad) { return fromAxisAngle(Vec3{0.0f, 1.0f, 0.0f}, kHalfPi - tiltRad); }

void expectVec3At(int line, const Vec3& expected, const Vec3& actual, float tolerance) {
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.x, actual.x, line, "x");
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.y, actual.y, line, "y");
  UNITY_TEST_ASSERT_FLOAT_WITHIN(tolerance, expected.z, actual.z, line, "z");
}

#define EXPECT_VEC3(expected, actual, tolerance) \
  expectVec3At(__LINE__, (expected), (actual), (tolerance))

// A FakeEstimator scripted with a fully valid, sensible estimate.
void scriptValid(FakeEstimator& estimator) {
  estimator.setValid(noseUpTilted(0.0f), Vec3{0.1f, 0.2f, 0.3f}, Vec3{3.0f, 4.0f, -12.0f}, 250.0f);
}

void expectNothingValid(int line, const Navigation& nav) {
  UNITY_TEST_ASSERT(!nav.attitudeValid(), line, "attitude must not be valid");
  UNITY_TEST_ASSERT(!nav.speedValid(), line, "speed must not be valid");
  UNITY_TEST_ASSERT(!nav.altitudeValid(), line, "altitude must not be valid");
  UNITY_TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.tilt(), line, "tilt must read 0 when invalid");
  UNITY_TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.speed(), line, "speed must read 0 when invalid");
}

#define EXPECT_NOTHING_VALID(nav) expectNothingValid(__LINE__, (nav))

void expectAllValid(int line, const Navigation& nav) {
  UNITY_TEST_ASSERT(nav.attitudeValid(), line, "attitude must be valid");
  UNITY_TEST_ASSERT(nav.speedValid(), line, "speed must be valid");
  UNITY_TEST_ASSERT(nav.altitudeValid(), line, "altitude must be valid");
}

#define EXPECT_ALL_VALID(nav) expectAllValid(__LINE__, (nav))

}  // namespace

// ---------------------------------------------------------------------------
// Before any data
// ---------------------------------------------------------------------------

static void test_nothing_is_valid_before_the_first_update() {
  FakeEstimator estimator;
  scriptValid(estimator);
  Navigation nav(estimator);

  TEST_ASSERT_FALSE(nav.imuValid());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.specificForce());
  EXPECT_NOTHING_VALID(nav);
  TEST_ASSERT_EQUAL_UINT32(0, nav.reinitializeCount());
  // Constructing Navigation must not touch the estimator.
  TEST_ASSERT_EQUAL_UINT32(0, estimator.callCount());
}

// ---------------------------------------------------------------------------
// Mount maps
// ---------------------------------------------------------------------------

static void expectProperRotation(const Mat3& a) {
  const Vec3 r0{a.m[0][0], a.m[0][1], a.m[0][2]};
  const Vec3 r1{a.m[1][0], a.m[1][1], a.m[1][2]};
  const Vec3 r2{a.m[2][0], a.m[2][1], a.m[2][2]};
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, dot(r0, r0));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, dot(r1, r1));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, dot(r2, r2));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, dot(r0, r1));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, dot(r0, r2));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, dot(r1, r2));
  // Determinant +1: a rotation, not a mirror. A mirrored map flips the sense
  // of every rotation the estimator sees.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, dot(cross(r0, r1), r2));
}

static void test_mount_maps_are_proper_rotations() {
  expectProperRotation(VehicleConfig::kImuToBody);
  expectProperRotation(VehicleConfig::kMagToBody);
}

static void test_imu_mount_map_is_applied_to_accel_and_gyro() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  SensorFrame frame;
  frame.timeUs = kT0;
  frame.imuOk = true;
  frame.accel = Vec3{1.0f, 2.0f, 3.0f};     // three different numbers, so a
  frame.gyro = Vec3{0.25f, -0.5f, 0.75f};   // swapped or flipped axis shows
  nav.update(frame, kStepS, true);

  TEST_ASSERT_EQUAL_UINT32(1, estimator.updateCount());
  const Mat3& map = VehicleConfig::kImuToBody;
  for (int row = 0; row < 3; ++row) {
    // Written out by hand: body[row] = sum over columns of map[row][col] * sensor[col].
    const float accelExpected = map.m[row][0] * 1.0f + map.m[row][1] * 2.0f + map.m[row][2] * 3.0f;
    const float gyroExpected =
        map.m[row][0] * 0.25f + map.m[row][1] * -0.5f + map.m[row][2] * 0.75f;
    const Vec3& accelGot = estimator.lastInput().accelBody;
    const Vec3& gyroGot = estimator.lastInput().gyroBody;
    const float accelComponent = row == 0 ? accelGot.x : (row == 1 ? accelGot.y : accelGot.z);
    const float gyroComponent = row == 0 ? gyroGot.x : (row == 1 ? gyroGot.y : gyroGot.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, accelExpected, accelComponent);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, gyroExpected, gyroComponent);
  }
  // The accessors give the same body-axis vectors.
  EXPECT_VEC3(estimator.lastInput().accelBody, nav.accelBody(), 0.0f);
  EXPECT_VEC3(estimator.lastInput().gyroBody, nav.gyroBody(), 0.0f);

  if (!isIdentity(map)) {
    // The estimator must not be handed raw sensor axes.
    const Vec3& got = estimator.lastInput().accelBody;
    TEST_ASSERT_FALSE(got.x == 1.0f && got.y == 2.0f && got.z == 3.0f);
  }
}

static void test_gravity_on_the_pad_arrives_along_the_nose() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, true);

  const Vec3 expected{FlightConfig::kGravity, 0.0f, 0.0f};
  EXPECT_VEC3(expected, estimator.lastInput().accelBody, 1e-4f);
  EXPECT_VEC3(expected, nav.accelBody(), 1e-4f);
}

static void test_roll_rate_about_the_nose_arrives_on_body_x() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  SensorFrame frame = padFrame(kT0);
  frame.gyro = imuSensorAxes(Vec3{2.0f, 0.0f, 0.0f});
  nav.update(frame, kStepS, false);

  EXPECT_VEC3((Vec3{2.0f, 0.0f, 0.0f}), estimator.lastInput().gyroBody, 1e-5f);
  EXPECT_VEC3((Vec3{2.0f, 0.0f, 0.0f}), nav.gyroBody(), 1e-5f);
}

static void test_mag_goes_through_the_mag_map() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  SensorFrame frame = padFrame(kT0);
  frame.magFresh = true;
  frame.mag = Vec3{10.0f, -20.0f, 45.0f};
  nav.update(frame, kStepS, true);

  const Mat3& map = VehicleConfig::kMagToBody;
  const Vec3 expected{
      map.m[0][0] * 10.0f + map.m[0][1] * -20.0f + map.m[0][2] * 45.0f,
      map.m[1][0] * 10.0f + map.m[1][1] * -20.0f + map.m[1][2] * 45.0f,
      map.m[2][0] * 10.0f + map.m[2][1] * -20.0f + map.m[2][2] * 45.0f,
  };
  EXPECT_VEC3(expected, estimator.lastInput().magBody, 1e-5f);
  TEST_ASSERT_TRUE(estimator.lastInput().magFresh);
}

// ---------------------------------------------------------------------------
// Specific force: straight from the IMU
// ---------------------------------------------------------------------------

static void test_specific_force_is_the_accel_magnitude() {
  NullEstimator estimator;
  Navigation nav(estimator);

  SensorFrame frame;
  frame.timeUs = kT0;
  frame.imuOk = true;
  frame.accel = Vec3{3.0f, -4.0f, 12.0f};
  nav.update(frame, kStepS, true);

  TEST_ASSERT_TRUE(nav.imuValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 13.0f, nav.specificForce());
}

static void test_specific_force_works_with_the_null_estimator() {
  NullEstimator estimator;
  Navigation nav(estimator);

  nav.update(padFrame(kT0), 0.0f, true);
  TEST_ASSERT_TRUE(nav.imuValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, FlightConfig::kGravity, nav.specificForce());
  TEST_ASSERT_TRUE(nav.specificForce() < FlightConfig::kLaunchAccel);
  EXPECT_NOTHING_VALID(nav);

  // Launch: well over the detection threshold, with no estimator at all.
  SensorFrame boost = padFrame(kT0 + kStepUs);
  boost.accel = imuSensorAxes(Vec3{2.0f * FlightConfig::kLaunchAccel, 0.0f, 0.0f});
  nav.update(boost, kStepS, true);
  TEST_ASSERT_TRUE(nav.imuValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f * FlightConfig::kLaunchAccel, nav.specificForce());
  TEST_ASSERT_TRUE(nav.specificForce() >= FlightConfig::kLaunchAccel);
  EXPECT_NOTHING_VALID(nav);
}

static void test_specific_force_ignores_a_broken_estimator() {
  FakeEstimator estimator;
  estimator.setValid(Quat{kNaN, kNaN, kNaN, kNaN}, Vec3{kNaN, kNaN, kNaN}, Vec3{kInf, kNaN, kNaN},
                     kNaN);
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, false);

  TEST_ASSERT_TRUE(nav.imuValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, FlightConfig::kGravity, nav.specificForce());
  EXPECT_NOTHING_VALID(nav);
}

static void test_failed_imu_read_gives_zero_specific_force() {
  FakeEstimator estimator;
  scriptValid(estimator);
  Navigation nav(estimator);

  // The numbers look like a launch, but the read failed: they are not data.
  SensorFrame frame = padFrame(kT0);
  frame.imuOk = false;
  frame.accel = imuSensorAxes(Vec3{3.0f * FlightConfig::kLaunchAccel, 0.0f, 0.0f});
  nav.update(frame, kStepS, true);

  TEST_ASSERT_FALSE(nav.imuValid());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.specificForce());
}

static void test_non_finite_accel_or_gyro_makes_the_imu_invalid() {
  const float badValues[] = {kNaN, kInf, -kInf};
  for (float bad : badValues) {
    for (int field = 0; field < 6; ++field) {
      FakeEstimator estimator;
      scriptValid(estimator);
      Navigation nav(estimator);

      SensorFrame frame = padFrame(kT0);
      frame.gyro = Vec3{0.1f, 0.2f, 0.3f};
      float* fields[6] = {&frame.accel.x, &frame.accel.y, &frame.accel.z,
                          &frame.gyro.x,  &frame.gyro.y,  &frame.gyro.z};
      *fields[field] = bad;
      nav.update(frame, kStepS, true);

      TEST_ASSERT_FALSE(nav.imuValid());
      TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.specificForce());
      TEST_ASSERT_EQUAL_UINT32(0, estimator.updateCount());
    }
  }
}

static void test_accel_too_large_to_take_a_magnitude_of_is_invalid() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  // Each component is a finite float, but the sum of squares overflows. An
  // infinite |accel| must not read as "above the launch threshold".
  SensorFrame frame = padFrame(kT0);
  frame.accel = Vec3{3.0e38f, 3.0e38f, 0.0f};
  nav.update(frame, kStepS, true);

  TEST_ASSERT_FALSE(nav.imuValid());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.specificForce());
  TEST_ASSERT_EQUAL_UINT32(0, estimator.updateCount());
}

static void test_imu_recovers_after_a_bad_frame() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  SensorFrame bad = padFrame(kT0);
  bad.accel.y = kNaN;
  nav.update(bad, kStepS, true);
  TEST_ASSERT_FALSE(nav.imuValid());

  nav.update(padFrame(kT0 + kStepUs), kStepS, true);
  TEST_ASSERT_TRUE(nav.imuValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, FlightConfig::kGravity, nav.specificForce());
  TEST_ASSERT_EQUAL_UINT32(1, estimator.updateCount());
}

// ---------------------------------------------------------------------------
// What the estimator is given
// ---------------------------------------------------------------------------

static void test_estimator_is_called_once_per_good_frame() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  for (uint32_t i = 0; i < 5; ++i) nav.update(padFrame(kT0 + i * kStepUs), kStepS, true);

  TEST_ASSERT_EQUAL_UINT32(5, estimator.updateCount());
  TEST_ASSERT_EQUAL_UINT32(0, estimator.reinitializeCount());
  for (uint32_t i = 0; i < 5; ++i) {
    TEST_ASSERT_EQUAL_UINT64(kT0 + i * kStepUs, estimator.inputAt(i).timeUs);
  }
}

static void test_estimator_is_not_called_on_a_bad_imu_frame() {
  FakeEstimator estimator;
  scriptValid(estimator);
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, true);
  TEST_ASSERT_EQUAL_UINT32(1, estimator.updateCount());

  SensorFrame failed = padFrame(kT0 + kStepUs);
  failed.imuOk = false;
  nav.update(failed, kStepS, true);
  TEST_ASSERT_EQUAL_UINT32(1, estimator.updateCount());

  SensorFrame nan = padFrame(kT0 + 2 * kStepUs);
  nan.gyro.z = kNaN;
  nav.update(nan, kStepS, true);
  TEST_ASSERT_EQUAL_UINT32(1, estimator.updateCount());

  // The previous estimate is kept as it was, with its old time.
  TEST_ASSERT_EQUAL_UINT64(kT0, nav.state().timeUs);
  TEST_ASSERT_EQUAL_FLOAT(250.0f, nav.altitudeAgl());
}

static void test_time_dt_and_on_pad_are_passed_through() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  nav.update(padFrame(kT0), 0.0f, true);
  TEST_ASSERT_EQUAL_UINT64(kT0, estimator.lastInput().timeUs);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, estimator.lastInput().dt);
  TEST_ASSERT_TRUE(estimator.lastInput().onPad);

  nav.update(padFrame(kT0 + 4800), 0.0048f, false);
  TEST_ASSERT_EQUAL_UINT64(kT0 + 4800, estimator.lastInput().timeUs);
  TEST_ASSERT_EQUAL_FLOAT(0.0048f, estimator.lastInput().dt);
  TEST_ASSERT_FALSE(estimator.lastInput().onPad);

  // A sample time that does not fit in 32 bits.
  const uint64_t late = (5ull << 32) + 17;
  nav.update(padFrame(late), kStepS, true);
  TEST_ASSERT_EQUAL_UINT64(late, estimator.lastInput().timeUs);
}

static void test_slow_sensor_flags_and_values_are_passed_through() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  SensorFrame frame = padFrame(kT0);
  nav.update(frame, kStepS, true);
  TEST_ASSERT_FALSE(estimator.lastInput().magFresh);
  TEST_ASSERT_FALSE(estimator.lastInput().baroFresh);

  frame.timeUs += kStepUs;
  frame.baroFresh = true;
  frame.pressurePa = 98765.0f;
  nav.update(frame, kStepS, true);
  TEST_ASSERT_FALSE(estimator.lastInput().magFresh);
  TEST_ASSERT_TRUE(estimator.lastInput().baroFresh);
  TEST_ASSERT_EQUAL_FLOAT(98765.0f, estimator.lastInput().pressurePa);

  frame.timeUs += kStepUs;
  frame.baroFresh = false;
  frame.magFresh = true;
  frame.mag = Vec3{1.0f, 2.0f, 3.0f};
  nav.update(frame, kStepS, true);
  TEST_ASSERT_TRUE(estimator.lastInput().magFresh);
  TEST_ASSERT_FALSE(estimator.lastInput().baroFresh);

  frame.timeUs += kStepUs;
  frame.baroFresh = true;
  nav.update(frame, kStepS, true);
  TEST_ASSERT_TRUE(estimator.lastInput().magFresh);
  TEST_ASSERT_TRUE(estimator.lastInput().baroFresh);
}

static void test_non_finite_slow_sensor_sample_is_not_passed_on_as_fresh() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  SensorFrame frame = padFrame(kT0);
  frame.magFresh = true;
  frame.mag = Vec3{20.0f, kNaN, 5.0f};
  frame.baroFresh = true;
  frame.pressurePa = kInf;
  nav.update(frame, kStepS, true);

  // The IMU part of the frame is fine, so the estimator still runs.
  TEST_ASSERT_TRUE(nav.imuValid());
  TEST_ASSERT_EQUAL_UINT32(1, estimator.updateCount());
  TEST_ASSERT_FALSE(estimator.lastInput().magFresh);
  TEST_ASSERT_FALSE(estimator.lastInput().baroFresh);

  // One bad sensor does not hide the other.
  frame.timeUs += kStepUs;
  frame.mag = Vec3{20.0f, 1.0f, 5.0f};
  nav.update(frame, kStepS, true);
  TEST_ASSERT_TRUE(estimator.lastInput().magFresh);
  TEST_ASSERT_FALSE(estimator.lastInput().baroFresh);
}

// ---------------------------------------------------------------------------
// Re-initialising the estimator
// ---------------------------------------------------------------------------

static void test_no_reinitialize_without_a_request() {
  FakeEstimator estimator;
  Navigation nav(estimator);
  for (uint32_t i = 0; i < 10; ++i) nav.update(padFrame(kT0 + i * kStepUs), kStepS, true);
  TEST_ASSERT_EQUAL_UINT32(0, estimator.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(0, nav.reinitializeCount());
}

static void test_reinitialize_runs_before_that_cycles_update() {
  FakeEstimator estimator;
  Navigation nav(estimator);
  nav.update(padFrame(kT0), kStepS, true);

  nav.requestReinitialize();
  // Asking does nothing by itself: it happens at the start of the next update.
  TEST_ASSERT_EQUAL_UINT32(0, estimator.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(0, nav.reinitializeCount());

  nav.update(padFrame(kT0 + kStepUs), kStepS, true);

  TEST_ASSERT_EQUAL_UINT32(3, estimator.callCount());
  TEST_ASSERT_TRUE(estimator.callAt(0) == FakeEstimator::Call::Update);
  TEST_ASSERT_TRUE(estimator.callAt(1) == FakeEstimator::Call::Reinitialize);
  TEST_ASSERT_TRUE(estimator.callAt(2) == FakeEstimator::Call::Update);
  TEST_ASSERT_EQUAL_UINT32(1, nav.reinitializeCount());
  // The update right after the reinitialise is this cycle's sample.
  TEST_ASSERT_EQUAL_UINT64(kT0 + kStepUs, estimator.inputAt(1).timeUs);
}

static void test_reinitialize_on_the_very_first_cycle() {
  FakeEstimator estimator;
  Navigation nav(estimator);
  nav.requestReinitialize();
  nav.update(padFrame(kT0), 0.0f, true);

  TEST_ASSERT_EQUAL_UINT32(2, estimator.callCount());
  TEST_ASSERT_TRUE(estimator.callAt(0) == FakeEstimator::Call::Reinitialize);
  TEST_ASSERT_TRUE(estimator.callAt(1) == FakeEstimator::Call::Update);
}

static void test_one_request_reinitializes_once() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  nav.requestReinitialize();
  nav.requestReinitialize();  // asked twice before the update: still one
  for (uint32_t i = 0; i < 5; ++i) nav.update(padFrame(kT0 + i * kStepUs), kStepS, true);

  TEST_ASSERT_EQUAL_UINT32(1, estimator.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(1, nav.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(5, estimator.updateCount());
}

static void test_each_later_request_reinitializes_again() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  for (uint32_t i = 1; i <= 3; ++i) {
    nav.requestReinitialize();
    nav.update(padFrame(kT0 + i * kStepUs), kStepS, true);
    TEST_ASSERT_EQUAL_UINT32(i, estimator.reinitializeCount());
    TEST_ASSERT_EQUAL_UINT32(i, nav.reinitializeCount());
  }
}

static void test_reinitialize_still_happens_on_a_bad_imu_frame() {
  FakeEstimator estimator;
  Navigation nav(estimator);

  nav.requestReinitialize();
  SensorFrame bad = padFrame(kT0);
  bad.imuOk = false;
  nav.update(bad, kStepS, true);

  TEST_ASSERT_EQUAL_UINT32(1, estimator.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(1, nav.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(0, estimator.updateCount());

  // The request was used up: the next good frame does not repeat it.
  nav.update(padFrame(kT0 + kStepUs), kStepS, true);
  TEST_ASSERT_EQUAL_UINT32(1, estimator.reinitializeCount());
  TEST_ASSERT_EQUAL_UINT32(1, estimator.updateCount());
  TEST_ASSERT_TRUE(estimator.callAt(0) == FakeEstimator::Call::Reinitialize);
  TEST_ASSERT_TRUE(estimator.callAt(1) == FakeEstimator::Call::Update);
}

// ---------------------------------------------------------------------------
// Validity of the estimate
// ---------------------------------------------------------------------------

static void test_a_good_estimate_is_valid_and_its_values_come_through() {
  FakeEstimator estimator;
  const Quat attitude = noseUpTilted(0.3f);
  estimator.setValid(attitude, Vec3{0.1f, -0.2f, 0.3f}, Vec3{3.0f, 4.0f, -12.0f}, 250.0f);
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, false);

  EXPECT_ALL_VALID(nav);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, nav.tilt());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 13.0f, nav.speed());
  TEST_ASSERT_EQUAL_FLOAT(250.0f, nav.altitudeAgl());
  TEST_ASSERT_EQUAL_FLOAT(attitude.w, nav.attitude().w);
  TEST_ASSERT_EQUAL_FLOAT(attitude.x, nav.attitude().x);
  TEST_ASSERT_EQUAL_FLOAT(attitude.y, nav.attitude().y);
  TEST_ASSERT_EQUAL_FLOAT(attitude.z, nav.attitude().z);
  EXPECT_VEC3((Vec3{0.1f, -0.2f, 0.3f}), nav.bodyRate(), 0.0f);
  TEST_ASSERT_EQUAL_UINT64(kT0, nav.state().timeUs);
}

static void test_tilt_is_zero_nose_up_and_a_right_angle_on_its_side() {
  FakeEstimator estimator;
  scriptValid(estimator);
  Navigation nav(estimator);

  // Nose-up is pitch = +90 deg, the Euler singularity. Tilt must be clean there.
  estimator.output.attitude = noseUpTilted(0.0f);
  nav.update(padFrame(kT0), kStepS, false);
  TEST_ASSERT_TRUE(nav.attitudeValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, nav.tilt());

  // The identity attitude has the nose pointing north: lying on its side.
  estimator.output.attitude = kIdentityQuat;
  nav.update(padFrame(kT0 + kStepUs), kStepS, false);
  TEST_ASSERT_TRUE(nav.attitudeValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, kHalfPi, nav.tilt());

  // q and -q are the same attitude.
  const Quat q = noseUpTilted(0.2f);
  estimator.output.attitude = Quat{-q.w, -q.x, -q.y, -q.z};
  nav.update(padFrame(kT0 + 2 * kStepUs), kStepS, false);
  TEST_ASSERT_TRUE(nav.attitudeValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, nav.tilt());
}

static void test_each_estimator_flag_controls_only_its_own_value() {
  for (int cleared = 0; cleared < 3; ++cleared) {
    FakeEstimator estimator;
    scriptValid(estimator);
    estimator.output.attitude = noseUpTilted(0.3f);
    if (cleared == 0) estimator.output.attitudeValid = false;
    if (cleared == 1) estimator.output.velocityValid = false;
    if (cleared == 2) estimator.output.altitudeValid = false;
    Navigation nav(estimator);

    nav.update(padFrame(kT0), kStepS, false);

    TEST_ASSERT_EQUAL(cleared != 0, nav.attitudeValid());
    TEST_ASSERT_EQUAL(cleared != 1, nav.speedValid());
    TEST_ASSERT_EQUAL(cleared != 2, nav.altitudeValid());
    // A value that is not valid reads as 0, even though the estimator sent numbers.
    if (cleared == 0) TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.tilt());
    if (cleared == 1) TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.speed());
  }
}

static void test_estimate_is_fresh_at_exactly_the_stale_limit() {
  FakeEstimator estimator;
  scriptValid(estimator);
  estimator.stampTime = false;
  estimator.output.timeUs = kT0;
  Navigation nav(estimator);

  nav.update(padFrame(kT0 + FlightConfig::kNavStaleUs), kStepS, false);

  EXPECT_ALL_VALID(nav);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 13.0f, nav.speed());
}

static void test_estimate_is_stale_one_microsecond_past_the_limit() {
  FakeEstimator estimator;
  scriptValid(estimator);
  estimator.output.attitude = noseUpTilted(0.3f);
  estimator.stampTime = false;
  estimator.output.timeUs = kT0;
  Navigation nav(estimator);

  nav.update(padFrame(kT0 + FlightConfig::kNavStaleUs + 1), kStepS, false);

  EXPECT_NOTHING_VALID(nav);
  // The estimator's own output is still there for the log, flags and all.
  TEST_ASSERT_TRUE(nav.state().attitudeValid);
  TEST_ASSERT_TRUE(nav.state().velocityValid);
  TEST_ASSERT_TRUE(nav.state().altitudeValid);
  TEST_ASSERT_EQUAL_UINT64(kT0, nav.state().timeUs);
}

static void test_estimate_from_the_future_is_not_valid() {
  FakeEstimator estimator;
  scriptValid(estimator);
  estimator.stampTime = false;
  estimator.output.timeUs = kT0 + 1;
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, false);
  EXPECT_NOTHING_VALID(nav);

  // Far in the future: the unsigned age must not wrap round to "fresh".
  estimator.output.timeUs = ~0ull;
  nav.update(padFrame(kT0 + kStepUs), kStepS, false);
  EXPECT_NOTHING_VALID(nav);
}

static void test_kept_estimate_goes_stale_when_the_imu_drops_out() {
  FakeEstimator estimator;
  scriptValid(estimator);
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, false);
  EXPECT_ALL_VALID(nav);

  // The IMU fails, so the estimator is not run and the estimate stays at kT0.
  SensorFrame bad = padFrame(kT0 + FlightConfig::kNavStaleUs);
  bad.imuOk = false;
  nav.update(bad, kStepS, false);
  TEST_ASSERT_EQUAL_UINT32(1, estimator.updateCount());
  EXPECT_ALL_VALID(nav);  // exactly at the limit: still usable

  bad.timeUs = kT0 + FlightConfig::kNavStaleUs + 1;
  nav.update(bad, kStepS, false);
  TEST_ASSERT_EQUAL_UINT32(1, estimator.updateCount());
  EXPECT_NOTHING_VALID(nav);

  // The IMU comes back: a new estimate, valid again.
  nav.update(padFrame(kT0 + FlightConfig::kNavStaleUs + 2), kStepS, false);
  TEST_ASSERT_EQUAL_UINT32(2, estimator.updateCount());
  EXPECT_ALL_VALID(nav);
}

static void test_non_unit_quaternion_is_not_a_valid_attitude() {
  const Quat good = noseUpTilted(0.3f);
  const float scales[] = {0.0f, 0.5f, 0.99f, 1.01f, 2.0f};
  for (float s : scales) {
    FakeEstimator estimator;
    scriptValid(estimator);
    estimator.output.attitude = Quat{good.w * s, good.x * s, good.y * s, good.z * s};
    Navigation nav(estimator);

    nav.update(padFrame(kT0), kStepS, false);

    TEST_ASSERT_FALSE(nav.attitudeValid());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.tilt());
    // Speed and altitude do not depend on the attitude.
    TEST_ASSERT_TRUE(nav.speedValid());
    TEST_ASSERT_TRUE(nav.altitudeValid());
  }
}

static void test_quaternion_with_float_rounding_error_is_still_valid() {
  const Quat good = noseUpTilted(0.3f);
  const float s = 1.0001f;  // the kind of drift an integrating filter has
  FakeEstimator estimator;
  scriptValid(estimator);
  estimator.output.attitude = Quat{good.w * s, good.x * s, good.y * s, good.z * s};
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, false);

  TEST_ASSERT_TRUE(nav.attitudeValid());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.3f, nav.tilt());
}

static void test_non_finite_attitude_or_rate_is_not_a_valid_attitude() {
  const float badValues[] = {kNaN, kInf, -kInf};
  for (float bad : badValues) {
    for (int field = 0; field < 7; ++field) {
      FakeEstimator estimator;
      scriptValid(estimator);
      NavState& out = estimator.output;
      float* fields[7] = {&out.attitude.w, &out.attitude.x, &out.attitude.y, &out.attitude.z,
                          &out.bodyRate.x, &out.bodyRate.y, &out.bodyRate.z};
      *fields[field] = bad;
      Navigation nav(estimator);

      nav.update(padFrame(kT0), kStepS, false);

      TEST_ASSERT_FALSE(nav.attitudeValid());
      TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.tilt());
      TEST_ASSERT_TRUE(nav.speedValid());
      TEST_ASSERT_TRUE(nav.altitudeValid());
    }
  }
}

static void test_non_finite_velocity_is_not_a_valid_speed() {
  const float badValues[] = {kNaN, kInf, -kInf};
  for (float bad : badValues) {
    for (int field = 0; field < 3; ++field) {
      FakeEstimator estimator;
      scriptValid(estimator);
      NavState& out = estimator.output;
      float* fields[3] = {&out.velocityNed.x, &out.velocityNed.y, &out.velocityNed.z};
      *fields[field] = bad;
      Navigation nav(estimator);

      nav.update(padFrame(kT0), kStepS, false);

      TEST_ASSERT_FALSE(nav.speedValid());
      TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.speed());
      TEST_ASSERT_TRUE(nav.attitudeValid());
      TEST_ASSERT_TRUE(nav.altitudeValid());
    }
  }
}

static void test_velocity_too_large_to_take_a_magnitude_of_is_not_valid() {
  FakeEstimator estimator;
  scriptValid(estimator);
  estimator.output.velocityNed = Vec3{3.0e38f, 0.0f, 3.0e38f};
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, false);

  TEST_ASSERT_FALSE(nav.speedValid());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, nav.speed());
}

static void test_non_finite_altitude_is_not_valid() {
  const float badValues[] = {kNaN, kInf, -kInf};
  for (float bad : badValues) {
    FakeEstimator estimator;
    scriptValid(estimator);
    estimator.output.altitudeAgl = bad;
    Navigation nav(estimator);

    nav.update(padFrame(kT0), kStepS, false);

    TEST_ASSERT_FALSE(nav.altitudeValid());
    TEST_ASSERT_TRUE(nav.attitudeValid());
    TEST_ASSERT_TRUE(nav.speedValid());
  }
}

static void test_validity_follows_the_estimator_cycle_by_cycle() {
  FakeEstimator estimator;
  scriptValid(estimator);
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, false);
  EXPECT_ALL_VALID(nav);

  estimator.setInvalid();
  nav.update(padFrame(kT0 + kStepUs), kStepS, false);
  EXPECT_NOTHING_VALID(nav);

  scriptValid(estimator);
  nav.update(padFrame(kT0 + 2 * kStepUs), kStepS, false);
  EXPECT_ALL_VALID(nav);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 13.0f, nav.speed());
}

static void test_state_is_the_estimators_output_as_it_arrived() {
  FakeEstimator estimator;
  estimator.output.attitude = Quat{0.5f, 0.0f, 0.0f, 0.0f};  // not unit: Navigation rejects it
  estimator.output.attitudeValid = true;
  estimator.output.velocityNed = Vec3{1.0f, 2.0f, 3.0f};
  estimator.output.velocityValid = false;
  estimator.output.altitudeAgl = 77.0f;
  estimator.output.altitudeValid = true;
  Navigation nav(estimator);

  nav.update(padFrame(kT0), kStepS, false);

  TEST_ASSERT_FALSE(nav.attitudeValid());
  // The log gets what the estimator said, not Navigation's verdict.
  TEST_ASSERT_TRUE(nav.state().attitudeValid);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, nav.state().attitude.w);
  TEST_ASSERT_FALSE(nav.state().velocityValid);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, nav.state().velocityNed.y);
  TEST_ASSERT_TRUE(nav.state().altitudeValid);
  TEST_ASSERT_EQUAL_FLOAT(77.0f, nav.state().altitudeAgl);
  TEST_ASSERT_EQUAL_UINT64(kT0, nav.state().timeUs);
}

// ---------------------------------------------------------------------------
// NullEstimator
// ---------------------------------------------------------------------------

static void test_null_estimator_writes_a_default_state_with_the_input_time() {
  NullEstimator estimator;

  // Start from a NavState full of plausible-looking leftovers.
  NavState out;
  out.timeUs = 1;
  out.attitude = Quat{0.0f, 1.0f, 0.0f, 0.0f};
  out.bodyRate = Vec3{1.0f, 2.0f, 3.0f};
  out.velocityNed = Vec3{4.0f, 5.0f, 6.0f};
  out.altitudeAgl = 1234.0f;
  out.attitudeValid = true;
  out.velocityValid = true;
  out.altitudeValid = true;

  EstimatorInput in;
  in.timeUs = (3ull << 32) + 99;
  in.accelBody = Vec3{FlightConfig::kGravity, 0.0f, 0.0f};
  estimator.update(in, out);

  TEST_ASSERT_EQUAL_UINT64((3ull << 32) + 99, out.timeUs);
  TEST_ASSERT_FALSE(out.attitudeValid);
  TEST_ASSERT_FALSE(out.velocityValid);
  TEST_ASSERT_FALSE(out.altitudeValid);
  const NavState defaults;
  TEST_ASSERT_EQUAL_FLOAT(defaults.attitude.w, out.attitude.w);
  TEST_ASSERT_EQUAL_FLOAT(defaults.attitude.x, out.attitude.x);
  TEST_ASSERT_EQUAL_FLOAT(defaults.attitude.y, out.attitude.y);
  TEST_ASSERT_EQUAL_FLOAT(defaults.attitude.z, out.attitude.z);
  EXPECT_VEC3(defaults.bodyRate, out.bodyRate, 0.0f);
  EXPECT_VEC3(defaults.velocityNed, out.velocityNed, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(defaults.altitudeAgl, out.altitudeAgl);
}

static void test_null_estimator_reinitialize_changes_nothing() {
  NullEstimator estimator;
  estimator.reinitialize();
  estimator.reinitialize();

  NavState out;
  out.attitudeValid = true;
  EstimatorInput in;
  in.timeUs = 42;
  estimator.update(in, out);
  TEST_ASSERT_EQUAL_UINT64(42, out.timeUs);
  TEST_ASSERT_FALSE(out.attitudeValid);
}

static void test_navigation_with_the_null_estimator_never_reports_anything_valid() {
  NullEstimator estimator;
  Navigation nav(estimator);

  nav.requestReinitialize();
  for (uint32_t i = 0; i < 200; ++i) {
    const uint64_t t = kT0 + i * kStepUs;
    nav.update(padFrame(t), i == 0 ? 0.0f : kStepS, i < 100);
    EXPECT_NOTHING_VALID(nav);
    TEST_ASSERT_TRUE(nav.imuValid());
    // The estimator was run: its output carries this cycle's time.
    TEST_ASSERT_EQUAL_UINT64(t, nav.state().timeUs);
  }
  TEST_ASSERT_EQUAL_UINT32(1, nav.reinitializeCount());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_nothing_is_valid_before_the_first_update);
  RUN_TEST(test_mount_maps_are_proper_rotations);
  RUN_TEST(test_imu_mount_map_is_applied_to_accel_and_gyro);
  RUN_TEST(test_gravity_on_the_pad_arrives_along_the_nose);
  RUN_TEST(test_roll_rate_about_the_nose_arrives_on_body_x);
  RUN_TEST(test_mag_goes_through_the_mag_map);
  RUN_TEST(test_specific_force_is_the_accel_magnitude);
  RUN_TEST(test_specific_force_works_with_the_null_estimator);
  RUN_TEST(test_specific_force_ignores_a_broken_estimator);
  RUN_TEST(test_failed_imu_read_gives_zero_specific_force);
  RUN_TEST(test_non_finite_accel_or_gyro_makes_the_imu_invalid);
  RUN_TEST(test_accel_too_large_to_take_a_magnitude_of_is_invalid);
  RUN_TEST(test_imu_recovers_after_a_bad_frame);
  RUN_TEST(test_estimator_is_called_once_per_good_frame);
  RUN_TEST(test_estimator_is_not_called_on_a_bad_imu_frame);
  RUN_TEST(test_time_dt_and_on_pad_are_passed_through);
  RUN_TEST(test_slow_sensor_flags_and_values_are_passed_through);
  RUN_TEST(test_non_finite_slow_sensor_sample_is_not_passed_on_as_fresh);
  RUN_TEST(test_no_reinitialize_without_a_request);
  RUN_TEST(test_reinitialize_runs_before_that_cycles_update);
  RUN_TEST(test_reinitialize_on_the_very_first_cycle);
  RUN_TEST(test_one_request_reinitializes_once);
  RUN_TEST(test_each_later_request_reinitializes_again);
  RUN_TEST(test_reinitialize_still_happens_on_a_bad_imu_frame);
  RUN_TEST(test_a_good_estimate_is_valid_and_its_values_come_through);
  RUN_TEST(test_tilt_is_zero_nose_up_and_a_right_angle_on_its_side);
  RUN_TEST(test_each_estimator_flag_controls_only_its_own_value);
  RUN_TEST(test_estimate_is_fresh_at_exactly_the_stale_limit);
  RUN_TEST(test_estimate_is_stale_one_microsecond_past_the_limit);
  RUN_TEST(test_estimate_from_the_future_is_not_valid);
  RUN_TEST(test_kept_estimate_goes_stale_when_the_imu_drops_out);
  RUN_TEST(test_non_unit_quaternion_is_not_a_valid_attitude);
  RUN_TEST(test_quaternion_with_float_rounding_error_is_still_valid);
  RUN_TEST(test_non_finite_attitude_or_rate_is_not_a_valid_attitude);
  RUN_TEST(test_non_finite_velocity_is_not_a_valid_speed);
  RUN_TEST(test_velocity_too_large_to_take_a_magnitude_of_is_not_valid);
  RUN_TEST(test_non_finite_altitude_is_not_valid);
  RUN_TEST(test_validity_follows_the_estimator_cycle_by_cycle);
  RUN_TEST(test_state_is_the_estimators_output_as_it_arrived);
  RUN_TEST(test_null_estimator_writes_a_default_state_with_the_input_time);
  RUN_TEST(test_null_estimator_reinitialize_changes_nothing);
  RUN_TEST(test_navigation_with_the_null_estimator_never_reports_anything_valid);
  return UNITY_END();
}
