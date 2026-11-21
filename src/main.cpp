// Flight program entry point. Placeholder until the FlightComputer lands
// (docs/HANDOFF-flight-skeleton-2026-11-20.md, section 6.2).
//
// The bring-up sketch that used to live here is now bringup/main.cpp
// (envs marsv21_bringup / marsv21_bringup_debug).

#include <Arduino.h>

void setup() {
  // USB CDC must be running for the 1200-baud DFU touch (src/dfu_bootloader.cpp)
  // to work. Without it, reflashing needs the BOOT0 button.
  SerialUSB.begin();
}

void loop() {}
