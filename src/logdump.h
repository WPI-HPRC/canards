#pragma once

// Serves the SD card's logs over SerialUSB, for scripts/dump_log.py
// (`pio run -t listlogs` / `pio run -t dumplog`).
//
// Commands, one per line:
//   list      lists the log files with their sizes
//   dump      sends the previous boot's log
//   dump <N>  sends /logs/canardsLog<N>.csv
//
// Replies (all markers are on their own line):
//   ---LIST BEGIN---  <name> <bytes>[ live] ...  ---LIST END---
//   ---BEGIN <name> <bytes>---  <exactly <bytes> raw bytes>  ---END---
//   ---ERROR <message>---
//
// A dump blocks loop() until it finishes, so the CSV log has a gap for that
// long. The current boot's log ("live") can't be dumped.

// Reads any pending command characters and runs a complete command. Call
// every loop().
void logDumpPoll();
