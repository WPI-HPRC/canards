#pragma once

// The E22 LoRa radio, configured exactly as the bring-up program does it
// (bringup/main.cpp, Nic's radioInit()), for the flight program's telemetry.
// Board only.

#if defined(ARDUINO)

#include "LoRaE22.h"

// The radio module object. Valid after radioLinkBegin().
LoRaE22& radioLink();

// Configures the radio. Returns false if it failed; never blocks for good
// (the module's timeout is 2 s per attempt, 3 attempts).
bool radioLinkBegin();

#endif  // ARDUINO
