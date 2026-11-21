// The frozen old controller, built twice more against pinned gain tables
// (see OldController.h for why).
//
// The old source is included, not copied, each time inside a namespace of its
// own. That is the only way to build one unedited file against two tables in
// one program: the old code selects its table with a macro, and its functions
// would otherwise have the same names as the as-shipped build.
//
// Three things make including a .cpp inside a namespace safe here:
//   - <cmath> is included first, outside any namespace. The old source
//     includes it too, and that second include is then a no-op instead of
//     putting the standard library inside our namespace.
//   - The old files include nothing else from outside their own folder.
//   - Their include guards are cleared between the two builds, so the second
//     one sees the old headers again.

#include <cmath>

#include "OldController.h"

namespace oldReferenceTableBuild {
// Looked up next to reference/controller.cpp.
#define GNC_GAIN_TABLE_HEADER "reference_table/lqr_gain_table.h"
#include "reference/controller.cpp"
#undef GNC_GAIN_TABLE_HEADER
}  // namespace oldReferenceTableBuild

#undef GNC_CONTROLLER_H
#undef GNC_LQR_GAIN_TABLE_H

namespace oldTwoRowTableBuild {
#define GNC_GAIN_TABLE_HEADER "../two_row_gain_table.h"
#include "reference/controller.cpp"
#undef GNC_GAIN_TABLE_HEADER
}  // namespace oldTwoRowTableBuild

const OldBuild& oldReferenceTable() {
  namespace old = oldReferenceTableBuild::gnc;
  static const OldBuild build = {
      "reference table",    old::kGainTable,     old::kGainRows,
      old::kLqrCanardLimit, old::kRollErrorLimit, &old::lqrController,
      &old::servoModelStep,
  };
  return build;
}

const OldBuild& oldTwoRowTable() {
  namespace old = oldTwoRowTableBuild::gnc;
  static const OldBuild build = {
      "two-row test table", old::kGainTable,     old::kGainRows,
      old::kLqrCanardLimit, old::kRollErrorLimit, &old::lqrController,
      &old::servoModelStep,
  };
  return build;
}
