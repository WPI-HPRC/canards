#if defined(ARDUINO)

#include "Platform/SdLineReader.h"

#include "sdcard.h"

bool SdLineReader::open(const char* path) {
  file_.close();
  if (!sdMounted()) return false;
  return file_.open(&sdFs(), path, O_RDONLY);
}

bool SdLineReader::readLine(char* buf, size_t n) {
  if (!file_.isOpen() || n < 2) return false;
  const int len = file_.fgets(buf, static_cast<int>(n));
  if (len <= 0) return false;  // 0: end of file, -1: read error
  // Cut short (no newline, buffer full): drop the rest of this line.
  if (buf[len - 1] != '\n' && static_cast<size_t>(len) == n - 1) {
    int c;
    while ((c = file_.read()) >= 0 && c != '\n') {
    }
  }
  return true;
}

#endif  // ARDUINO
