# Frozen reference code. Do not edit. Do not use in flight code.

These files are a verbatim copy of the attitude controller in
`~/Documents/HPRC/EnP/GNC/Voyager-launch-controller` (working tree, copied
2026-11-20). They exist for one reason: `test_attitude_control` compares the
ported controller (`src/AttitudeControl/LqrAttitudeController`) against this
old code, input for input (design doc section 8.1).

- Never edit them, not even to fix a bug or a warning. A "fixed" reference
  makes the equivalence test meaningless. If the old law is wrong, change the
  port and its tests, and say so in the design doc.
- Never include them from `src/`, `lib/` or any other test suite.
- That repo has no commits and no remote, so this copy is also the only
  backup of the code the gains were designed against.

| File | Copied from | SHA-1 |
|---|---|---|
| `controller.h` | `controller.h` | `e6d7cfcfffdd1802359f0cec8bd092a10d9a7f56` |
| `controller.cpp` | `controller.cpp` | `f092df5a7b51bbdc14e0bc1f066ce03e3e781696` |
| `lqr_gain_table.h` | `lqr_gain_table.h` (the old flight table) | `4e282841229ebe7e0a083b5361dece51ec653138` |
| `reference_table/lqr_gain_table.h` | `test/reference_table/lqr_gain_table.h` | `d24df040f3cc48acc83f9ea39de04a4b2ebfe258` |

Check that nothing has drifted with `shasum` in this folder.

Old frame, for reading the code: x = pitch, y = yaw, z = roll (out of the
nose). The team frame used everywhere else in this repo is x = roll (nose),
y = pitch, z = yaw. The canard numbering is the same in both.

How the suite builds this code is explained in `../OldController.h`.
