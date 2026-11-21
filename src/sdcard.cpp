// SD card access over SdFat's soft SPI driver.
// Ported from rocket-base-2026 (src/logging.cpp, PR #40).

#include "sdcard.h"

#include <Arduino.h>
#include <stdio.h>

#if SPI_DRIVER_SELECT != 2
#error "Build with -DSPI_DRIVER_SELECT=2 to enable SdFat's soft SPI driver"
#endif

// The SD slot is wired for 4-bit SDMMC. In SPI mode DAT3 is CS, CMD is MOSI and
// DAT0 is MISO. No hardware SPI peripheral maps to those pins, so the bus is
// bit-banged.
static SoftSpiDriver<PIN_SD_DAT0, PIN_SD_CMD, PIN_SD_SCK> sdSpi;
static SdFs sd;

static FsFile logFile;
static int logFileIdx = 0;

static constexpr char PREV_LOG_IDX_PATH[] = LOG_FOLDER_NAME "/.prev_log_idx";
static constexpr int MAX_LOG_FILE_IDX = 100;
static constexpr oflag_t CREATE_LOG_FLAGS = O_RDWR | O_CREAT | O_TRUNC;

// Reads the index used by the previous boot and stores the next one.
static int nextLogFileIdx() {
  FsFile idxFile;
  if (!idxFile.open(PREV_LOG_IDX_PATH, O_RDWR | O_CREAT)) {
    SerialUSB.printf("Failed to open `%s`\n", PREV_LOG_IDX_PATH);
    return 0;
  }

  int fileIdx = idxFile.available() ? idxFile.parseInt() + 1 : 0;
  if (fileIdx > MAX_LOG_FILE_IDX) {
    fileIdx = 0;
  }

  idxFile.truncate(0);
  idxFile.print(fileIdx);
  idxFile.close();
  return fileIdx;
}

bool initializeSD() {
  SerialUSB.print("Initializing SD... ");

  // Soft SPI runs at one fixed speed, so the clock argument is ignored.
  if (!sd.begin(SdSpiConfig(PIN_SD_DAT3, DEDICATED_SPI, SD_SCK_MHZ(0), &sdSpi))) {
    SerialUSB.println("FAILED");
    sd.printSdError(&SerialUSB);
    return false;
  }
  SerialUSB.println("OK");

  if (!sd.exists(LOG_FOLDER_NAME)) {
    sd.mkdir(LOG_FOLDER_NAME);
  }

  logFileIdx = nextLogFileIdx();
  char filename[64];
  sdLogFileName(logFileIdx, filename, sizeof(filename));

  if (!logFile.open(filename, CREATE_LOG_FLAGS)) {
    SerialUSB.printf("Failed to open `%s`\n", filename);
    return false;
  }
  SerialUSB.printf("Log file `%s`\n", filename);
  return true;
}

FsFile &sdLogFile() { return logFile; }

int sdLogFileIdx() { return logFileIdx; }

int sdPrevLogFileIdx() {
  return logFileIdx == 0 ? MAX_LOG_FILE_IDX : logFileIdx - 1;
}

void sdLogFileName(int idx, char *buf, size_t len) {
  snprintf(buf, len, LOG_FOLDER_NAME "/canardsLog%d.csv", idx);
}

SdFs &sdFs() { return sd; }

bool sdMounted() { return sd.fatType() != 0; }

