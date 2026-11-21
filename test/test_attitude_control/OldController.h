#pragma once

// Test-only access to the FROZEN old attitude law in reference/ (a verbatim
// copy of Voyager-launch-controller; see reference/README.md).
//
// The old code picks its gain table at compile time, through the macro
// GNC_GAIN_TABLE_HEADER. To compare it with the port on more than one table,
// this suite builds the same old source three times:
//
//   oldAsShipped()       reference/controller.cpp exactly as it stands.
//                        PlatformIO compiles every source file under a test
//                        suite's folder, sub-folders included, so that file
//                        is built on its own with no macro and therefore
//                        with its default table, reference/lqr_gain_table.h
//                        (the old flight table).
//   oldReferenceTable()  the same source, included with the macro set to
//                        reference/reference_table/lqr_gain_table.h. This is
//                        how the old host test was built, and the table its
//                        expected numbers assume.
//   oldTwoRowTable()     the same source, included with the macro set to
//                        two_row_gain_table.h (made-up test data).
//
// The last two are included inside their own namespaces in
// OldControllerPinnedTables.cpp, so the three builds do not clash at link
// time.
//
// Each build hands out the table it was compiled with. Give that table to
// the port, and both sides of a comparison are certain to use the same one.
//
// Everything here is in the OLD controller frame: x = pitch, y = yaw,
// z = roll (out of the nose). Canard arrays are indexed the same way in both
// frames.

struct OldBuild {
  const char* name;
  const float (*table)[7];  // the table this build of the old code reads
  int rows;
  float canardLimitRad;     // the old code's fixed output limit (4 deg)
  float rollErrorLimitRad;  // the old code's fixed roll-error limit (10 deg)

  // gnc::lqrController. Errors and rates are [0] pitch, [1] yaw, [2] roll.
  void (*lqr)(const float attitudeError[3], const float bodyRate[3], const float canardAngle[4],
              float speed, float canardCommand[4]);

  // gnc::servoModelStep: one 5 ms step of the old first-order servo model.
  void (*servoModelStep)(float canardAngle[4], const float canardCommand[4]);
};

const OldBuild& oldAsShipped();
const OldBuild& oldReferenceTable();
const OldBuild& oldTwoRowTable();
