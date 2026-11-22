#pragma once

// The E22 LoRa radio, configured exactly as the bring-up program does it
// (bringup/main.cpp, Nic's radioInit()), for the flight program's telemetry.
// Board only.

#if defined(ARDUINO)

#include "LoRaE22.h"

// The radio module object. Valid after radioLinkBegin().
LoRaE22& radioLink();

// Free bytes in the radio UART's transmit buffer. A write longer than this
// blocks the loop until the UART catches up, so callers check it first.
int radioTxRoom();

// Configures the radio. Returns false if it failed; never blocks for good
// (the module's timeout is 2 s per attempt, 3 attempts).
bool radioLinkBegin();

#endif  // ARDUINO
