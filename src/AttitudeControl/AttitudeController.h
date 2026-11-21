#pragma once

// The attitude control law, as the Canards subsystem sees it
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, sections 6.4 and 8.1).
//
// The flight implementation is LqrAttitudeController. Tests of the Canards
// subsystem use a fake.
//
// Everything is in the team-standard body frame (lib/Frames/Frames.h):
// [x] roll (nose), [y] pitch, [z] yaw. Canard arrays are indexed by CANARD
// (controller numbering), never by servo channel: canard i sits a quarter
// turn * i from body +y towards body +z, and a positive angle on any canard
// rolls the rocket about +x (the nose).

#include "Frames.h"

struct AttitudeControlInput {
  Vec3 attitudeError;        // rad, rotation vector Log(conj(qRef) (x) q), body frame
  Vec3 bodyRate;             // rad/s, body frame
  float canardAngle[4] = {}; // rad, where the canards are now (measured or modelled)
  float speed = 0.0f;        // m/s, picks the gains
};

class AttitudeController {
 public:
  virtual ~AttitudeController() = default;

  // Computes four canard commands [rad], each within the controller's limit.
  // Returns false if it cannot produce a trustworthy command (for example a
  // non-finite input); `canardCommand` is then all zeros and the caller must
  // treat the cycle as neutral.
  virtual bool compute(const AttitudeControlInput& in, float canardCommand[4]) = 0;
};
