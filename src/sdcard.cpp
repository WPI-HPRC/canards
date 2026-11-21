// SD card access over SdFat's soft SPI driver.
// Ported from rocket-base-2026 (src/logging.cpp, PR #40).

#include "sdcard.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#if SPI_DRIVER_SELECT != 2
#error "Build with -DSPI_DRIVER_SELECT=2 to enable SdFat's soft SPI driver"
#endif

// The SD slot is wired for 4-bit SDMMC. In SPI mode DAT3 is CS, CMD is MOSI and
// DAT0 is MISO. No hardware SPI peripheral maps to those pins, so the bus is
// bit-banged.
static SoftSpiDriver<PIN_SD_DAT0, PIN_SD_CMD, PIN_SD_SCK> sdSpi;
static SdFs sd;

static bool mounted = false;
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
  mounted = true;

  if (!sd.exists(LOG_FOLDER_NAME)) {
    sd.mkdir(LOG_FOLDER_NAME);
  }

  logFileIdx = nextLogFileIdx();
  char filename[64];
  snprintf(filename, sizeof(filename), LOG_FOLDER_NAME "/canardsLog%d.bin", logFileIdx);

  if (!logFile.open(filename, CREATE_LOG_FLAGS)) {
    SerialUSB.printf("Failed to open `%s`\n", filename);
    return false;
  }
  SerialUSB.printf("Log file `%s`\n", filename);
  return true;
}

FsFile &sdLogFile() { return logFile; }

int sdLogFileIdx() { return logFileIdx; }

// Expected contents of row i of the test CSV.
static constexpr int CSV_TEST_ROWS = 20;
#define CSV_TEST_HEADER "row,id,value"
static long csvTestId(int row) { return 1000L * row + 7; }
static long csvTestValue(int row) { return -3L * row * row + 42; }

bool sdCsvSelfTest() {
  SerialUSB.println("SD CSV test: start");
  if (!mounted) {
    SerialUSB.println("SD CSV test: FAILED (card not mounted)");
    return false;
  }

  // Write
  uint32_t t0 = micros();
  FsFile file;
  if (!file.open(SD_TEST_CSV_PATH, O_RDWR | O_CREAT | O_TRUNC)) {
    SerialUSB.println("SD CSV test: FAILED (could not create " SD_TEST_CSV_PATH ")");
    return false;
  }
  file.println(CSV_TEST_HEADER);
  for (int i = 0; i < CSV_TEST_ROWS; i++) {
    file.printf("%d,%ld,%ld\n", i, csvTestId(i), csvTestValue(i));
  }
  uint64_t writtenSize = file.fileSize();
  if (!file.close()) {
    SerialUSB.println("SD CSV test: FAILED (close after write)");
    return false;
  }
  uint32_t writeUs = micros() - t0;

  // Read back and verify
  t0 = micros();
  if (!file.open(SD_TEST_CSV_PATH, O_RDONLY)) {
    SerialUSB.println("SD CSV test: FAILED (could not reopen for reading)");
    return false;
  }

  char line[64] = "";
  bool ok = true;
  int rowsOk = 0;

  if (file.fgets(line, sizeof(line)) <= 0 || strcmp(line, CSV_TEST_HEADER "\n") != 0) {
    SerialUSB.printf("SD CSV test: bad header `%s`\n", line);
    ok = false;
  }

  for (int i = 0; ok && i < CSV_TEST_ROWS; i++) {
    int row;
    long id, value;
    if (file.fgets(line, sizeof(line)) <= 0) {
      SerialUSB.printf("SD CSV test: file ended at row %d\n", i);
      ok = false;
    } else if (sscanf(line, "%d,%ld,%ld", &row, &id, &value) != 3 ||
               row != i || id != csvTestId(i) || value != csvTestValue(i)) {
      SerialUSB.printf("SD CSV test: row %d mismatch, read `%s`\n", i, line);
      ok = false;
    } else {
      rowsOk++;
    }
  }

  if (ok && file.fgets(line, sizeof(line)) > 0) {
    SerialUSB.printf("SD CSV test: unexpected extra line `%s`\n", line);
    ok = false;
  }
  file.close();
  uint32_t readUs = micros() - t0;

  SerialUSB.printf("SD CSV test: %s (%d/%d rows ok, %lu bytes, write %lu us, read %lu us)\n",
                   ok ? "PASSED" : "FAILED", rowsOk, CSV_TEST_ROWS,
                   (unsigned long)writtenSize, (unsigned long)writeUs, (unsigned long)readUs);
  return ok;
}
