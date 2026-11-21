#pragma once

// Frames, vectors and quaternions for the whole flight program.
// This header is the single statement of the team's conventions
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 4).
//
// FRAMES
//   Inertial: NED (x north, y east, z down). "Up" is (0, 0, -1).
//   Body:     x out of the nose, then y and z, with z pointing down when the
//             rocket lies on its side. Body rates: x roll, y pitch, z yaw.
//
// QUATERNIONS
//   Hamilton convention, stored [w, x, y, z], unit length.
//   A quaternion q rotates body vectors into NED:  v_ned = q v_body q*.
//
// EULER ANGLES are never used in flight logic. Flying nose-up in this frame is
// pitch = +90 deg, exactly where yaw-pitch-roll is singular. Use tilt(),
// attitudeError(), verticalReference() and rollAboutNose() instead. The only
// Euler helper is in FramesDisplay.h, for display and log decoding.
//
// UNITS: SI. Angles in rad, rates in rad/s.
//
// No Arduino, no heap, no globals. Everything here is a pure function.

#include <cmath>

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Quat {
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

// ---------------------------------------------------------------------------
// Vectors
// ---------------------------------------------------------------------------

constexpr Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3 operator-(const Vec3& a) { return {-a.x, -a.y, -a.z}; }
constexpr Vec3 operator*(const Vec3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
constexpr Vec3 operator*(float s, const Vec3& a) { return a * s; }

constexpr float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

float norm(const Vec3& v);

// Unit vector along v. Returns `fallback` if v is too short to normalise or
// is not finite.
Vec3 normalized(const Vec3& v, const Vec3& fallback = Vec3{1.0f, 0.0f, 0.0f});

bool isFinite(const Vec3& v);

// A 3x3 matrix applied to a vector, row-major: out = m * v. Used for the
// sensor mount maps (sensor axes -> body axes).
struct Mat3 {
  float m[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
};
constexpr Vec3 operator*(const Mat3& a, const Vec3& v) {
  return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
          a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
          a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}

// ---------------------------------------------------------------------------
// Fixed directions
// ---------------------------------------------------------------------------

constexpr Vec3 kNoseBody{1.0f, 0.0f, 0.0f};  // body x: out of the nose
constexpr Vec3 kUpNed{0.0f, 0.0f, -1.0f};    // NED z points down

// ---------------------------------------------------------------------------
// Quaternions
// ---------------------------------------------------------------------------

constexpr Quat kIdentityQuat{1.0f, 0.0f, 0.0f, 0.0f};

// Hamilton product a (x) b: the rotation b followed by the rotation a, when
// both rotate vectors as v' = q v q*.
constexpr Quat operator*(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

constexpr Quat conjugate(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }

float norm(const Quat& q);

// Unit quaternion along q. Returns the identity if q is too short to
// normalise or is not finite.
Quat normalized(const Quat& q);

bool isFinite(const Quat& q);

// True if q is finite and its length is within `tolerance` of 1.
bool isUnit(const Quat& q, float tolerance = 1e-3f);

// Rotates a body vector into NED: q v q*.
Vec3 rotate(const Quat& q, const Vec3& vBody);

// Rotates a NED vector into the body frame: q* v q.
Vec3 rotateInverse(const Quat& q, const Vec3& vNed);

// The rotation of `angleRad` about `axis` (right-handed). `axis` does not need
// to be unit length. A zero or non-finite axis gives the identity.
Quat fromAxisAngle(const Vec3& axis, float angleRad);

// Exp map: the rotation whose rotation vector (axis * angle, rad) is `rv`.
Quat fromRotationVector(const Vec3& rv);

// Log map: the rotation vector (axis * angle, rad) of q, taking the shorter
// way round (the result's length is in [0, pi]). q and -q give the same
// result. Only ambiguous at exactly 180 deg.
Vec3 toRotationVector(const Quat& q);

// ---------------------------------------------------------------------------
// Flight helpers. None of these use Euler angles.
// ---------------------------------------------------------------------------

// The nose direction in NED: rotate(q, kNoseBody).
Vec3 noseNed(const Quat& q);

// Angle between the nose and straight up, in [0, pi]. 0 means nose-up.
float tilt(const Quat& q);

// Attitude error as a rotation vector in the body frame:
//   Log(conj(qRef) (x) q)
// It is the rotation that takes the reference attitude to the current one,
// with components [x] roll, [y] pitch, [z] yaw error. Zero when q == qRef.
// q and -q (and qRef and -qRef) give the same error. Singular only at a
// 180 deg error.
Vec3 attitudeError(const Quat& qRef, const Quat& q);

// The reference attitude "nose to true vertical, keep the current roll":
// the smallest rotation that takes the nose to up, applied on the left
// (in NED) to q:
//   normalize([1 + n.u, n x u]) (x) q     with n = noseNed(q), u = kUpNed.
// If the rocket points almost straight down the smallest rotation is not
// unique, and q is returned unchanged.
Quat verticalReference(const Quat& q);

// Roll angle about the nose, from a swing-twist decomposition of q about
// body x (never from Euler roll), in (-pi, pi]. Only meaningful relative to
// another value of itself (e.g. roll now minus roll at an earlier time).
float rollAboutNose(const Quat& q);
