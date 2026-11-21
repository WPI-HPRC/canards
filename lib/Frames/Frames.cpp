#include "Frames.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;

// Below this length a vector or quaternion cannot be normalised reliably.
constexpr float kTinyLength = 1e-12f;

// Components above about 1e19 overflow when squared, although the vector
// itself is finite. Scaling by this power of two first (exact in float) brings
// any finite value back into range without changing its direction.
constexpr float kOverflowRescale = 0x1p-66f;

// Writes the unit vector along v. Returns false, leaving `unit` untouched, if
// v is not finite or is too short to have a reliable direction.
bool toUnit(const Vec3& v, Vec3& unit) {
  if (!isFinite(v)) return false;
  Vec3 scaled = v;
  float n = norm(scaled);
  if (!std::isfinite(n)) {
    scaled = v * kOverflowRescale;
    n = norm(scaled);
  }
  if (n < kTinyLength) return false;
  unit = scaled * (1.0f / n);
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// Vectors
// ---------------------------------------------------------------------------

float norm(const Vec3& v) { return std::sqrt(dot(v, v)); }

Vec3 normalized(const Vec3& v, const Vec3& fallback) {
  Vec3 unit;
  return toUnit(v, unit) ? unit : fallback;
}

bool isFinite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// ---------------------------------------------------------------------------
// Quaternions
// ---------------------------------------------------------------------------

float norm(const Quat& q) { return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z); }

Quat normalized(const Quat& q) {
  if (!isFinite(q)) return kIdentityQuat;
  Quat scaled = q;
  float n = norm(scaled);
  if (!std::isfinite(n)) {
    // Finite components whose squares overflowed: same remedy as toUnit().
    scaled = {q.w * kOverflowRescale, q.x * kOverflowRescale, q.y * kOverflowRescale,
              q.z * kOverflowRescale};
    n = norm(scaled);
  }
  if (n < kTinyLength) return kIdentityQuat;
  const float s = 1.0f / n;
  return {scaled.w * s, scaled.x * s, scaled.y * s, scaled.z * s};
}

bool isFinite(const Quat& q) {
  return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

bool isUnit(const Quat& q, float tolerance) {
  if (!isFinite(q)) return false;
  return std::fabs(norm(q) - 1.0f) <= tolerance;
}

Vec3 rotate(const Quat& q, const Vec3& v) {
  // v' = v + 2 w (u x v) + 2 u x (u x v), with u the vector part of q.
  const Vec3 u{q.x, q.y, q.z};
  const Vec3 t = 2.0f * cross(u, v);
  return v + q.w * t + cross(u, t);
}

Vec3 rotateInverse(const Quat& q, const Vec3& v) { return rotate(conjugate(q), v); }

Quat fromAxisAngle(const Vec3& axis, float angleRad) {
  Vec3 unit;
  if (!toUnit(axis, unit) || !std::isfinite(angleRad)) return kIdentityQuat;
  const float half = 0.5f * angleRad;
  const float s = std::sin(half);
  return {std::cos(half), unit.x * s, unit.y * s, unit.z * s};
}

Quat fromRotationVector(const Vec3& rv) {
  if (!isFinite(rv)) return kIdentityQuat;
  const float angle = norm(rv);
  if (angle < 1e-6f) {
    // First-order: sin(a/2)/a -> 1/2. This also covers vectors so short that
    // their length underflows to zero.
    return normalized(Quat{1.0f, 0.5f * rv.x, 0.5f * rv.y, 0.5f * rv.z});
  }
  // A vector so long that its length overflows has no meaningful angle left
  // in float; fromAxisAngle() then returns the identity.
  return fromAxisAngle(rv, angle);
}

Vec3 toRotationVector(const Quat& qIn) {
  Quat q = normalized(qIn);
  // q and -q are the same rotation. Take the one with w >= 0: the shorter way.
  if (q.w < 0.0f) q = {-q.w, -q.x, -q.y, -q.z};
  const Vec3 v{q.x, q.y, q.z};
  const float s = norm(v);  // sin(angle / 2)
  if (s < 1e-9f) {
    // First-order: angle ~ 2 sin(angle / 2).
    return 2.0f * v;
  }
  // atan2 keeps full precision near 0 and near pi, where acos(w) does not.
  const float angle = 2.0f * std::atan2(s, q.w);
  return v * (angle / s);
}

// ---------------------------------------------------------------------------
// Flight helpers
//
// Each of these normalises its quaternion first, so a slightly non-unit input
// is harmless. A zero or non-finite quaternion is treated as the identity:
// the result is then finite but meaningless, so callers must check validity
// (isUnit()) before trusting an attitude. Navigation does.
// ---------------------------------------------------------------------------

Vec3 noseNed(const Quat& q) { return rotate(q, kNoseBody); }

float tilt(const Quat& q) {
  const Vec3 n = noseNed(normalized(q));
  // Angle between n and up = (0, 0, -1). The atan2 form resolves small tilts;
  // acos(dot) in float cannot tell anything below about 3e-4 rad from zero.
  return std::atan2(std::sqrt(n.x * n.x + n.y * n.y), -n.z);
}

Vec3 attitudeError(const Quat& qRef, const Quat& q) {
  return toRotationVector(conjugate(normalized(qRef)) * normalized(q));
}

Quat verticalReference(const Quat& qIn) {
  const Quat q = normalized(qIn);
  const Vec3 n = noseNed(q);
  const float c = dot(n, kUpNed);      // cos(tilt)
  const Vec3 axis = cross(n, kUpNed);  // length sin(tilt)
  // The smallest rotation taking n to up is [1 + c, n x u], normalised.
  // Below the horizon 1 + c cancels badly in float (at 170 deg of tilt it
  // would cost several microradians, and more the further down the nose
  // points), so use the equal form sin^2(tilt) / (1 - c) there.
  const float onePlusC = (c >= 0.0f) ? 1.0f + c : dot(axis, axis) / (1.0f - c);
  // The swing's length goes to zero as the nose points straight down, where
  // the smallest rotation is not unique. This cut is about 0.08 deg from
  // straight down.
  if (onePlusC < 1e-6f) return q;
  const Quat swing = normalized(Quat{onePlusC, axis.x, axis.y, axis.z});
  return normalized(swing * q);
}

float rollAboutNose(const Quat& qIn) {
  const Quat q = normalized(qIn);
  // Swing-twist about body x: q = swing (x) twist, twist = normalise([w, x, 0, 0]).
  // The twist is applied first, in the body frame, about the nose.
  if (std::fabs(q.w) < 1e-9f && std::fabs(q.x) < 1e-9f) return 0.0f;  // 180 deg swing: twist undefined
  float angle = 2.0f * std::atan2(q.x, q.w);
  if (angle > kPi) angle -= 2.0f * kPi;
  if (angle <= -kPi) angle += 2.0f * kPi;
  return angle;
}
