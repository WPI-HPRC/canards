// TEST DATA, NOT A FLIGHT TABLE. The numbers are made up.
//
// A gain table in the text format of lqr_gain_table.h, with the fewest rows
// the format allows (2), an uneven speed range and six gains that all differ
// and move in different directions. The frozen old controller is compiled
// against it (see OldController.h) so the equivalence test also runs on a
// table that looks nothing like the flight one: a port that assumed 7 rows,
// or 30 to 130 m/s, would pass on the real tables and fail here.

#ifndef GNC_LQR_GAIN_TABLE_H
#define GNC_LQR_GAIN_TABLE_H

namespace gnc {

const float kGainTable[][7] = {
    {20.0f, 2.10f, 0.410f, 0.150f, 6.50f,  4.20f, 0.080f},
    {95.0f, 0.90f, 0.120f, 3.300f, 12.75f, 1.90f, 1.250f},
};

const int kGainRows = static_cast<int>(sizeof(kGainTable) / sizeof(kGainTable[0]));

}  // namespace gnc

#endif  // GNC_LQR_GAIN_TABLE_H
