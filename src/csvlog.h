#pragma once

// Writes one CSV row per call from a table of columns.
//
// Each column is a header name plus a function that prints its value, so
// adding a field to the log is one line in the table:
//
//   static const CsvColumn logColumns[] = {
//     CSV_UINT("time_ms", millis()),
//     CSV_FLOAT("pressure_hpa", readings.pressure, 3),
//   };
//   csvLog.begin(sdLogFile(), logColumns);   // writes the header row
//   csvLog.logRow();                          // in loop()

#include <Arduino.h>
#include <SdFat.h>

struct CsvColumn {
  const char *name;
  void (*write)(Print &out);
};

// Column helpers. `expr` is evaluated each time a row is written, so it can
// name globals directly. CSV_FLOAT's `dp` is the number of decimal places.
#define CSV_INT(name, expr) \
  CsvColumn { name, [](Print &o) { o.print((long)(expr)); } }
#define CSV_UINT(name, expr) \
  CsvColumn { name, [](Print &o) { o.print((unsigned long)(expr)); } }
#define CSV_FLOAT(name, expr, dp) \
  CsvColumn { name, [](Print &o) { o.print((double)(expr), dp); } }

class CsvLogger {
public:
  // How often buffered rows are flushed to the card. Rows written since the
  // last flush are lost if power drops.
  static constexpr uint32_t SYNC_PERIOD_MS = 1000;

  // Starts logging to an open file and writes the header row. Returns false
  // if the file is not open or the header could not be written.
  template <size_t N>
  bool begin(FsFile &file, const CsvColumn (&columns)[N]) {
    return begin(file, columns, N);
  }
  bool begin(FsFile &file, const CsvColumn *columns, size_t count);

  // Writes one row. Does nothing if begin() failed. After a write error it
  // stops logging and returns false from then on.
  bool logRow(uint32_t now_ms = millis());

  bool active() const { return file_ != nullptr; }
  uint32_t rowsWritten() const { return rows_; }

private:
  FsFile *file_ = nullptr;
  const CsvColumn *columns_ = nullptr;
  size_t count_ = 0;
  uint32_t rows_ = 0;
  uint32_t lastSync_ = 0;

  void writeHeader();
  bool checkWrite(const char *what);
};
