#include "csvlog.h"

bool CsvLogger::begin(FsFile &file, const CsvColumn *columns, size_t count) {
  file_ = nullptr;
  if (!file.isOpen()) {
    SerialUSB.println("CSV log: not started (log file not open)");
    return false;
  }
  file_ = &file;
  columns_ = columns;
  count_ = count;
  rows_ = 0;

  writeHeader();
  file_->sync();
  lastSync_ = millis();
  if (!checkWrite("header")) {
    return false;
  }
  SerialUSB.printf("CSV log: %u columns\n", (unsigned)count_);
  return true;
}

void CsvLogger::writeHeader() {
  for (size_t i = 0; i < count_; i++) {
    if (i) file_->write(',');
    file_->print(columns_[i].name);
  }
  file_->println();
}

bool CsvLogger::logRow(uint32_t now_ms) {
  if (!file_) {
    return false;
  }
  for (size_t i = 0; i < count_; i++) {
    if (i) file_->write(',');
    columns_[i].write(*file_);
  }
  file_->println();
  rows_++;

  if (now_ms - lastSync_ >= SYNC_PERIOD_MS) {
    file_->sync();
    lastSync_ = now_ms;
  }
  return checkWrite("row");
}

bool CsvLogger::checkWrite(const char *what) {
  if (!file_->getWriteError()) {
    return true;
  }
  SerialUSB.printf("CSV log: write error on %s %lu, logging stopped\n", what,
                   (unsigned long)rows_);
  file_ = nullptr;
  return false;
}
