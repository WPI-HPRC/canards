#pragma once

#include <SdFat.h>

#define LOG_FOLDER_NAME "/logs"

// Mounts the SD card over soft SPI and opens a new, numbered log file
// (LOG_FOLDER_NAME/canardsLog<N>.bin). Returns false if the card or the file
// could not be opened.
bool initializeSD();

// The log file opened by initializeSD(). Not open if initializeSD() failed.
FsFile &sdLogFile();

// The index used in this boot's log file name.
int sdLogFileIdx();

// Writes a small CSV (SD_TEST_CSV_PATH) with known values, reads it back and
// checks every row. Prints the result over SerialUSB. Needs initializeSD() to
// have mounted the card. The file is left on the card for inspection and is
// overwritten on every run.
#define SD_TEST_CSV_PATH "/sd_test.csv"
bool sdCsvSelfTest();
