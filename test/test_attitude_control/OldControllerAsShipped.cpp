// The old controller exactly as it stands: reference/controller.cpp is
// compiled as its own source file by PlatformIO (see OldController.h), so the
// functions below are simply linked in.
//
// reference/controller.cpp includes "lqr_gain_table.h" with no folder, which
// the compiler looks up next to that file first: reference/lqr_gain_table.h.
// Including the same header here gives this file its own copy of the same
// numbers to hand to the port.

#include "OldController.h"

#include "reference/controller.h"
#include "reference/lqr_gain_table.h"

const OldBuild& oldAsShipped() {
  static const OldBuild build = {
      "as shipped (old flight table)", gnc::kGainTable,     gnc::kGainRows,
      gnc::kLqrCanardLimit,            gnc::kRollErrorLimit, &gnc::lqrController,
      &gnc::servoModelStep,
  };
  return build;
}
