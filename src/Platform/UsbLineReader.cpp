#if defined(ARDUINO)

#include "Platform/UsbLineReader.h"

#include <cstring>

bool UsbLineReader::readLine(char* buf, size_t n) {
  if (n == 0) return false;
  while (in_.available() > 0) {
    const int c = in_.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c != '\n') {
      if (len_ < kMaxLine - 1) line_[len_++] = static_cast<char>(c);  // else: dropped (the line is cut)
      continue;
    }
    // A whole line.
    const size_t len = len_ < n - 1 ? len_ : n - 1;
    std::memcpy(buf, line_, len);
    buf[len] = '\0';
    len_ = 0;
    return true;
  }
  return false;
}

#endif  // ARDUINO
