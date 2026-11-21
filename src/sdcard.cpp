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

// --- Getting the card back after a reset -------------------------------------
// With DEDICATED_SPI, SdFat leaves the card inside a multi-block write
// (CMD25) between log rows. A reset of the board does not reset the card (its
// power stays on), so after a reset, a watchdog reset or a reflash the card
// can still be waiting for write data, and then it ignores CMD0 and sd.begin()
// fails with SD_CARD_ERROR_CMD0. A power-cycle cured it; this does the same in
// software: finish any half-sent data block, send the "stop transmission"
// token, wait while the card is busy. On a card that is idle it is harmless.

static uint8_t bitBangByte(uint8_t out) {
  uint8_t in = 0;
  for (int bit = 7; bit >= 0; --bit) {
    digitalWrite(PIN_SD_CMD, (out >> bit) & 1);   // MOSI
    delayMicroseconds(2);
    digitalWrite(PIN_SD_SCK, HIGH);
    in = static_cast<uint8_t>(in << 1 | (digitalRead(PIN_SD_DAT0) ? 1 : 0));  // MISO
    delayMicroseconds(2);
    digitalWrite(PIN_SD_SCK, LOW);
  }
  return in;
}

// Clocks 0xFF until the card stops holding MISO low (busy), or `timeoutMs`.
static bool waitNotBusy(uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (bitBangByte(0xFF) != 0xFF) {
    if (millis() - start > timeoutMs) return false;
  }
  return true;
}

static void recoverCardFromWrite() {
  pinMode(PIN_SD_DAT3, OUTPUT);       // CS
  digitalWrite(PIN_SD_DAT3, HIGH);
  pinMode(PIN_SD_CMD, OUTPUT);
  digitalWrite(PIN_SD_CMD, HIGH);
  pinMode(PIN_SD_SCK, OUTPUT);
  digitalWrite(PIN_SD_SCK, LOW);
  pinMode(PIN_SD_DAT0, INPUT_PULLUP);

  digitalWrite(PIN_SD_DAT3, LOW);
  // A data block is a start token, 512 bytes and a 2-byte CRC: this many 0xFF
  // completes one that was cut off (the card rejects its CRC; that is fine).
  for (int i = 0; i < 520; ++i) bitBangByte(0xFF);
  waitNotBusy(500);
  bitBangByte(0xFD);                  // stop transmission token (ends CMD25)
  bitBangByte(0xFF);
  waitNotBusy(500);
  digitalWrite(PIN_SD_DAT3, HIGH);
  for (int i = 0; i < 10; ++i) bitBangByte(0xFF);   // >= 74 clocks with CS high
}

bool initializeSD() {
  SerialUSB.print("Initializing SD... ");

  // Soft SPI runs at one fixed speed, so the clock argument is ignored.
  // Retried, with the recovery above before each try (see there for why).
  bool mounted = false;
  for (int attempt = 0; attempt < 3 && !mounted; ++attempt) {
    recoverCardFromWrite();
    mounted = sd.begin(SdSpiConfig(PIN_SD_DAT3, DEDICATED_SPI, SD_SCK_MHZ(0), &sdSpi));
    if (!mounted) delay(100);
  }
  if (!mounted) {
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

