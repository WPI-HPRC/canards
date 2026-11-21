// PadAlignedEstimator: the sequencing (collect, align, run), velocity by
// integrating the accelerometer off the pad, altitude from a fresh barometer
// sample, and when each NavState field may be trusted.

#include <unity.h>

#include <cmath>
#include <cstddef>
#include <limits>

#include "Estimator/PadAlignedEstimator.h"
#include "FlightConfig.h"
#include "Frames.h"

void setUp() {}
void tearDown() {}

namespace {

namespace fc = FlightConfig;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kHalfPi = 1.57079632679489661923f;
constexpr uint64_t kPeriodUs = 5000;
constexpr float kDt = 0.005f;

// Nose up: body x along NED up (0, 0, -1). A rotation of +90 deg about body y
// takes body x to NED -z.
const Quat kNoseUp = fromAxisAngle(Vec3{0.0f, 1.0f, 0.0f}, kHalfPi);

// A backend that does real quaternion propagation with lib/Frames, so the
// adapter's velocity integration can be checked against known attitudes.
class FakeBackend final : public AttitudeBackend {
 public:
  size_t needed = 4;
  Quat alignedAttitude = kNoseUp;
  Vec3 bias{0.01f, -0.02f, 0.03f};
  bool refuseCollect = false;
  bool refuseAlign = false;
  bool failPropagate = false;

  size_t collectCalls = 0;
  size_t alignCalls = 0;
  size_t propagateCalls = 0;
  Vec3 alignedAccel, alignedMag;
  float alignedPressure = 0.0f;
  float lastDt = 0.0f;
  Quat q;

  size_t gyroSamplesNeeded() const override { return needed; }
  bool collectGyro(const Vec3&) override {
    ++collectCalls;
    return !refuseCollect;
  }
  bool align(const Vec3& accel, const Vec3& mag, float pressurePa) override {
    ++alignCalls;
    alignedAccel = accel;
    alignedMag = mag;
    alignedPressure = pressurePa;
    if (refuseAlign) return false;
    q = alignedAttitude;
    return true;
  }
  bool propagate(const Vec3& gyro, float dt) override {
    ++propagateCalls;
    lastDt = dt;
    if (failPropagate) return false;
    q = normalized(q * fromRotationVector((gyro - bias) * dt));
    return true;
  }
  Quat attitude() const override { return q; }
  Vec3 unbiasedGyro(const Vec3& gyro) const override { return gyro - bias; }
  // 1 m per 12 Pa below the reference: a simple, exact stand-in.
  float heightAgl(float pressurePa) const override { return (alignedPressure - pressurePa) / 12.0f; }
};

struct Rig {
  FakeBackend backend;
  PadAlignedEstimator estimator{backend};
  NavState out;
  uint64_t timeUs = 1000000;
  bool first = true;

  // One sample. At rest on the pad by default: gravity reads as specific
  // force up, i.e. along the nose for a nose-up rocket.
  void step(bool onPad, const Vec3& accelBody, const Vec3& gyroBody = Vec3{}, bool magFresh = true,
            bool baroFresh = true, float pressurePa = 101000.0f) {
    EstimatorInput in;
    in.timeUs = timeUs;
    in.dt = first ? 0.0f : kDt;
    in.onPad = onPad;
    in.accelBody = accelBody;
    in.gyroBody = gyroBody;
    in.magFresh = magFresh;
    in.magBody = Vec3{0.0f, 20.0f, 40.0f};
    in.baroFresh = baroFresh;
    in.pressurePa = pressurePa;
    estimator.update(in, out);
    first = false;
    timeUs += kPeriodUs;
  }
  // A gyro at rest reads its bias, so the attitude does not move.
  void atRest(int n = 1) {
    for (int i = 0; i < n; ++i) step(true, Vec3{fc::kGravity, 0.0f, 0.0f}, backend.bias);
  }
  // Collect and align, then one more sample so the output is valid.
  void toRunning() {
    estimator.reinitialize();
    atRest(static_cast<int>(backend.needed) + 1);
    TEST_ASSERT_EQUAL(PadAlignedEstimator::Phase::Running, estimator.phase());
  }
};

void assertNothingValid(const NavState& s) {
  TEST_ASSERT_FALSE(s.attitudeValid);
  TEST_ASSERT_FALSE(s.velocityValid);
  TEST_ASSERT_FALSE(s.altitudeValid);
}

// --- Sequencing ---------------------------------------------------------------

void test_nothing_is_valid_before_reinitialize() {
  Rig rig;
  rig.atRest(20);
  assertNothingValid(rig.out);
  TEST_ASSERT_EQUAL(PadAlignedEstimator::Phase::Idle, rig.estimator.phase());
  TEST_ASSERT_EQUAL_UINT32(0, rig.backend.collectCalls);
  TEST_ASSERT_EQUAL_UINT64(rig.timeUs - kPeriodUs, rig.out.timeUs);
}

void test_aligns_after_the_needed_samples_and_is_valid_from_the_next() {
  Rig rig;
  rig.estimator.reinitialize();
  for (size_t i = 0; i < rig.backend.needed; ++i) {
    TEST_ASSERT_EQUAL_UINT32(0, rig.backend.alignCalls);
    rig.atRest();
    assertNothingValid(rig.out);
  }
  TEST_ASSERT_EQUAL_UINT32(1, rig.backend.alignCalls);
  TEST_ASSERT_EQUAL(PadAlignedEstimator::Phase::Running, rig.estimator.phase());
  TEST_ASSERT_EQUAL_UINT32(rig.backend.needed, rig.backend.collectCalls);
  // Aligned with that sample's accelerometer and the latest mag and baro.
  TEST_ASSERT_EQUAL_FLOAT(fc::kGravity, rig.backend.alignedAccel.x);
  TEST_ASSERT_EQUAL_FLOAT(40.0f, rig.backend.alignedMag.z);
  TEST_ASSERT_EQUAL_FLOAT(101000.0f, rig.backend.alignedPressure);

  rig.atRest();
  TEST_ASSERT_TRUE(rig.out.attitudeValid);
  TEST_ASSERT_TRUE(rig.out.velocityValid);
  TEST_ASSERT_TRUE(rig.out.altitudeValid);
}

void test_waits_for_a_magnetometer_and_a_barometer_sample() {
  Rig rig;
  rig.estimator.reinitialize();
  for (size_t i = 0; i < rig.backend.needed + 3; ++i) {
    rig.step(true, Vec3{fc::kGravity, 0, 0}, Vec3{}, /*magFresh=*/false, /*baroFresh=*/false);
  }
  TEST_ASSERT_EQUAL_UINT32(0, rig.backend.alignCalls);
  TEST_ASSERT_EQUAL_UINT32(rig.backend.needed, rig.backend.collectCalls);  // stops at what it needs
  rig.step(true, Vec3{fc::kGravity, 0, 0}, Vec3{}, /*magFresh=*/true, /*baroFresh=*/false);
  TEST_ASSERT_EQUAL_UINT32(0, rig.backend.alignCalls);
  rig.step(true, Vec3{fc::kGravity, 0, 0}, Vec3{}, /*magFresh=*/false, /*baroFresh=*/true);
  TEST_ASSERT_EQUAL_UINT32(1, rig.backend.alignCalls);
  TEST_ASSERT_EQUAL(PadAlignedEstimator::Phase::Running, rig.estimator.phase());
}

void test_non_finite_or_bad_slow_samples_are_not_used() {
  Rig rig;
  rig.estimator.reinitialize();
  for (size_t i = 0; i < rig.backend.needed + 3; ++i) {
    EstimatorInput in;
    in.timeUs = rig.timeUs;
    in.onPad = true;
    in.accelBody = Vec3{fc::kGravity, 0, 0};
    in.magFresh = true;
    in.magBody = Vec3{kNaN, 0, 0};
    in.baroFresh = true;
    in.pressurePa = (i % 2 == 0) ? kNaN : -5.0f;
    rig.estimator.update(in, rig.out);
    rig.timeUs += kPeriodUs;
  }
  TEST_ASSERT_EQUAL_UINT32(0, rig.backend.alignCalls);
}

void test_samples_off_the_pad_are_not_collected_and_never_align() {
  Rig rig;
  rig.estimator.reinitialize();
  for (int i = 0; i < 50; ++i) rig.step(false, Vec3{6.0f * fc::kGravity, 0, 0});
  TEST_ASSERT_EQUAL_UINT32(0, rig.backend.collectCalls);
  TEST_ASSERT_EQUAL_UINT32(0, rig.backend.alignCalls);
  assertNothingValid(rig.out);
}

void test_a_bump_on_the_pad_only_pauses_collection() {
  Rig rig;
  rig.estimator.reinitialize();
  rig.atRest(2);
  rig.step(false, Vec3{6.0f * fc::kGravity, 0, 0});  // a knock: not at rest
  TEST_ASSERT_EQUAL_UINT32(2, rig.backend.collectCalls);
  rig.atRest(static_cast<int>(rig.backend.needed) - 2);
  TEST_ASSERT_EQUAL(PadAlignedEstimator::Phase::Running, rig.estimator.phase());
}

void test_a_refused_alignment_is_retried_on_the_next_sample() {
  Rig rig;
  rig.backend.refuseAlign = true;
  rig.estimator.reinitialize();
  rig.atRest(static_cast<int>(rig.backend.needed) + 2);
  TEST_ASSERT_EQUAL_UINT32(3, rig.backend.alignCalls);
  TEST_ASSERT_EQUAL(PadAlignedEstimator::Phase::Collecting, rig.estimator.phase());
  rig.backend.refuseAlign = false;
  rig.atRest();
  TEST_ASSERT_EQUAL(PadAlignedEstimator::Phase::Running, rig.estimator.phase());
}

void test_refused_gyro_samples_do_not_count() {
  Rig rig;
  rig.backend.refuseCollect = true;
  rig.estimator.reinitialize();
  rig.atRest(20);
  TEST_ASSERT_EQUAL_UINT32(0, rig.estimator.collected());
  TEST_ASSERT_EQUAL_UINT32(0, rig.backend.alignCalls);
}

void test_reinitialize_starts_over() {
  Rig rig;
  rig.toRunning();
  for (int i = 0; i < 20; ++i) rig.step(false, Vec3{3.0f * fc::kGravity, 0, 0});
  TEST_ASSERT_TRUE(norm(rig.out.velocityNed) > 1.0f);

  rig.estimator.reinitialize();
  TEST_ASSERT_EQUAL(PadAlignedEstimator::Phase::Collecting, rig.estimator.phase());
  rig.atRest();
  assertNothingValid(rig.out);
  rig.atRest(static_cast<int>(rig.backend.needed));
  TEST_ASSERT_EQUAL_UINT32(2, rig.backend.alignCalls);
  TEST_ASSERT_TRUE(rig.out.velocityValid);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, norm(rig.out.velocityNed));
}

// --- Attitude and rate ----------------------------------------------------------

void test_attitude_and_rate_come_from_the_backend() {
  Rig rig;
  rig.toRunning();
  const Vec3 gyro{0.3f, -0.1f, 0.2f};
  rig.step(false, Vec3{fc::kGravity, 0, 0}, gyro);
  const Quat q = rig.backend.attitude();
  TEST_ASSERT_EQUAL_FLOAT(q.w, rig.out.attitude.w);
  TEST_ASSERT_EQUAL_FLOAT(q.x, rig.out.attitude.x);
  TEST_ASSERT_EQUAL_FLOAT(q.y, rig.out.attitude.y);
  TEST_ASSERT_EQUAL_FLOAT(q.z, rig.out.attitude.z);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, gyro.x - rig.backend.bias.x, rig.out.bodyRate.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, gyro.y - rig.backend.bias.y, rig.out.bodyRate.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, gyro.z - rig.backend.bias.z, rig.out.bodyRate.z);
  TEST_ASSERT_FLOAT_WITHIN(1e-7f, kDt, rig.backend.lastDt);
}

void test_no_propagation_with_zero_dt() {
  Rig rig;
  rig.backend.needed = 1;
  rig.estimator.reinitialize();
  rig.first = true;  // the aligning sample is the very first: dt = 0
  rig.atRest();
  rig.first = true;  // and so is the next one
  rig.atRest();
  TEST_ASSERT_EQUAL_UINT32(0, rig.backend.propagateCalls);
  TEST_ASSERT_TRUE(rig.out.attitudeValid);
}

void test_a_failed_propagation_invalidates_attitude_until_reinitialize() {
  Rig rig;
  rig.toRunning();
  rig.backend.failPropagate = true;
  rig.atRest();
  TEST_ASSERT_FALSE(rig.out.attitudeValid);
  TEST_ASSERT_FALSE(rig.out.velocityValid);
  rig.backend.failPropagate = false;
  rig.atRest(5);
  TEST_ASSERT_FALSE(rig.out.attitudeValid);  // that rotation is lost for good
  rig.toRunning();
  TEST_ASSERT_TRUE(rig.out.attitudeValid);
}

// --- Velocity ---------------------------------------------------------------------

void test_velocity_is_zero_on_the_pad() {
  Rig rig;
  rig.toRunning();
  // Even a reading that is not 1 g leaves the velocity at zero on the pad.
  for (int i = 0; i < 20; ++i) rig.step(true, Vec3{1.3f * fc::kGravity, 0.5f, 0.0f});
  TEST_ASSERT_TRUE(rig.out.velocityValid);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, norm(rig.out.velocityNed));
}

void test_velocity_integrates_thrust_minus_gravity_nose_up() {
  Rig rig;
  rig.toRunning();
  // 3 g of specific force along the nose, nose up: 2 g of real acceleration up.
  const int n = 200;  // 1 s
  for (int i = 0; i < n; ++i) rig.step(false, Vec3{3.0f * fc::kGravity, 0, 0}, rig.backend.bias);
  const float expectedUp = 2.0f * fc::kGravity * n * kDt;
  TEST_ASSERT_TRUE(rig.out.velocityValid);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f * expectedUp, -expectedUp, rig.out.velocityNed.z);  // NED z is down
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, rig.out.velocityNed.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, rig.out.velocityNed.y);
}

void test_velocity_is_rotated_into_ned_with_the_attitude() {
  Rig rig;
  // Nose pointing north and level: body x = NED north.
  rig.backend.alignedAttitude = kIdentityQuat;
  rig.toRunning();
  const int n = 100;
  // 1 g along the nose plus the 1 g that holds it up (body z is down, so the
  // specific force that cancels gravity is along body -z).
  for (int i = 0; i < n; ++i) rig.step(false, Vec3{fc::kGravity, 0.0f, -fc::kGravity}, rig.backend.bias);
  const float expected = fc::kGravity * n * kDt;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f * expected, expected, rig.out.velocityNed.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, rig.out.velocityNed.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, rig.out.velocityNed.z);
}

void test_free_fall_reads_zero_and_falls() {
  Rig rig;
  rig.toRunning();
  const int n = 100;
  for (int i = 0; i < n; ++i) rig.step(false, Vec3{}, rig.backend.bias);  // 0 g: free fall
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, fc::kGravity * n * kDt, rig.out.velocityNed.z);  // falling: +down
}

// --- Altitude -----------------------------------------------------------------------

void test_altitude_comes_from_the_latest_fresh_barometer_sample() {
  Rig rig;
  rig.toRunning();
  rig.step(false, Vec3{fc::kGravity, 0, 0}, Vec3{}, true, true, 101000.0f - 12.0f * 50.0f);
  TEST_ASSERT_TRUE(rig.out.altitudeValid);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, rig.out.altitudeAgl);
}

void test_altitude_goes_stale_after_the_limit() {
  Rig rig;
  rig.toRunning();
  rig.step(false, Vec3{fc::kGravity, 0, 0}, Vec3{}, true, true, 101000.0f - 120.0f);
  const uint64_t baroUs = rig.timeUs - kPeriodUs;
  // No new barometer samples: valid up to exactly kBaroStaleUs after it.
  while (rig.timeUs - baroUs <= fc::kBaroStaleUs) {
    rig.step(false, Vec3{fc::kGravity, 0, 0}, Vec3{}, true, false);
    TEST_ASSERT_TRUE(rig.out.altitudeValid);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, rig.out.altitudeAgl);
  }
  rig.step(false, Vec3{fc::kGravity, 0, 0}, Vec3{}, true, false);
  TEST_ASSERT_FALSE(rig.out.altitudeValid);
  TEST_ASSERT_TRUE(rig.out.attitudeValid);  // the other fields do not depend on it
}

void test_output_time_is_the_input_time() {
  Rig rig;
  rig.toRunning();
  const uint64_t t = rig.timeUs;
  rig.atRest();
  TEST_ASSERT_EQUAL_UINT64(t, rig.out.timeUs);
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_nothing_is_valid_before_reinitialize);
  RUN_TEST(test_aligns_after_the_needed_samples_and_is_valid_from_the_next);
  RUN_TEST(test_waits_for_a_magnetometer_and_a_barometer_sample);
  RUN_TEST(test_non_finite_or_bad_slow_samples_are_not_used);
  RUN_TEST(test_samples_off_the_pad_are_not_collected_and_never_align);
  RUN_TEST(test_a_bump_on_the_pad_only_pauses_collection);
  RUN_TEST(test_a_refused_alignment_is_retried_on_the_next_sample);
  RUN_TEST(test_refused_gyro_samples_do_not_count);
  RUN_TEST(test_reinitialize_starts_over);
  RUN_TEST(test_attitude_and_rate_come_from_the_backend);
  RUN_TEST(test_no_propagation_with_zero_dt);
  RUN_TEST(test_a_failed_propagation_invalidates_attitude_until_reinitialize);
  RUN_TEST(test_velocity_is_zero_on_the_pad);
  RUN_TEST(test_velocity_integrates_thrust_minus_gravity_nose_up);
  RUN_TEST(test_velocity_is_rotated_into_ned_with_the_attitude);
  RUN_TEST(test_free_fall_reads_zero_and_falls);
  RUN_TEST(test_altitude_comes_from_the_latest_fresh_barometer_sample);
  RUN_TEST(test_altitude_goes_stale_after_the_limit);
  RUN_TEST(test_output_time_is_the_input_time);
  return UNITY_END();
}
