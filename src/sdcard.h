#pragma once

#include <SdFat.h>

#define LOG_FOLDER_NAME "/logs"

// Mounts the SD card over soft SPI and opens a new, numbered log file
// (LOG_FOLDER_NAME/canardsLog<N>.csv). Returns false if the card or the file
// could not be opened.
bool initializeSD();

// The log file opened by initializeSD(). Not open if initializeSD() failed.
FsFile &sdLogFile();

// The index used in this boot's log file name.
int sdLogFileIdx();

// The index the previous boot used (wraps like the file numbering does).
int sdPrevLogFileIdx();

// Full path of log file `idx`, e.g. "/logs/canardsLog7.csv".
void sdLogFileName(int idx, char *buf, size_t len);

// The mounted card, for code that needs other files (e.g. logdump.cpp).
SdFs &sdFs();
bool sdMounted();

