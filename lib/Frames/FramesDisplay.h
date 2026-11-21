#pragma once

// Euler angles FOR DISPLAY AND LOG DECODING ONLY.
//
// FLIGHT LOGIC MUST NEVER INCLUDE THIS FILE. Nothing under src/<Folder>/ may
// use it: no state, subsystem, controller or safety check. This rocket flies
// nose-up, which in the team frame is pitch = +90 deg, exactly where
// yaw-pitch-roll angles are singular
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 4). On the pad and
// through the whole ascent the yaw and roll returned here are therefore not
// meaningful one by one (only their difference is), and they jump with the
// smallest change in attitude. Flight logic uses tilt(), attitudeError(),
// verticalReference() and rollAboutNose() from Frames.h instead.
//
// What this is good for: a ground-station readout or a decoded log of the
// rocket lying on its side or under the parachute, where people expect to
// read heading, pitch and roll.

#include "Frames.h"

// Yaw-pitch-roll (ZYX) angles of a body -> NED attitude, in rad:
//   q = Rz(yaw) (x) Ry(pitch) (x) Rx(roll)
//   yaw    heading of the nose, clockwise from north seen from above, (-pi, pi]
//   pitch  nose above the horizon, [-pi/2, pi/2]; +pi/2 is nose-up
//   roll   about the nose, (-pi, pi]
struct EulerZyx {
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
};

// Never returns a non-finite angle. A zero or non-finite quaternion reads as
// the identity (all angles zero), like the other helpers in Frames.h.
inline EulerZyx eulerZyxForDisplay(const Quat& qIn) {
  const Quat q = normalized(qIn);
  // Elements of the rotation matrix R(q), body -> NED.
  const float r11 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);  // nose, north part
  const float r21 = 2.0f * (q.x * q.y + q.w * q.z);         // nose, east part
  const float r31 = 2.0f * (q.x * q.z - q.w * q.y);         // nose, down part
  const float r32 = 2.0f * (q.y * q.z + q.w * q.x);
  const float r33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  EulerZyx e;
  e.yaw = std::atan2(r21, r11);
  // atan2 instead of asin(-r31): it needs no clamping and keeps its precision
  // close to +/-90 deg.
  e.pitch = std::atan2(-r31, std::sqrt(r11 * r11 + r21 * r21));
  e.roll = std::atan2(r32, r33);
  return e;
}
