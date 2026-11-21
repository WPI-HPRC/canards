#pragma once

// International Morse code as an on/off pattern in time units (ITU timing):
// dot 1 unit on, dash 3 on, 1 off between the parts of a letter, 3 between
// letters, 7 between words. Used by the PreFlight greeting
// (FlightConfig::kPreFlightMorseText). Letters A-Z and spaces only; any other
// character is skipped. No heap, no Arduino.

#include <cstddef>

namespace morse {

// Dots and dashes for 'A'..'Z'.
inline const char* code(char c) {
  static const char* const kCodes[26] = {
      ".-",   "-...", "-.-.", "-..",  ".",   "..-.", "--.",  "....", "..",   ".---", "-.-",  ".-..", "--",
      "-.",   "---",  ".--.", "--.-", ".-.", "...",  "-",    "..-",  "...-", ".--",  "-..-", "-.--", "--.."};
  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
  if (c < 'A' || c > 'Z') return nullptr;
  return kCodes[c - 'A'];
}

// Calls `emit(on, units)` for each on or off stretch of `text`, in order. No
// trailing gap.
template <typename Emit>
void forEachStretch(const char* text, Emit emit) {
  bool first = true;        // nothing emitted yet: no gap before it
  bool pendingWord = false; // a space was seen since the last letter
  for (const char* p = text; *p != '\0'; ++p) {
    if (*p == ' ') {
      pendingWord = true;
      continue;
    }
    const char* c = code(*p);
    if (c == nullptr) continue;
    if (!first) emit(false, pendingWord ? 7u : 3u);
    pendingWord = false;
    first = false;
    for (const char* e = c; *e != '\0'; ++e) {
      if (e != c) emit(false, 1u);
      emit(true, *e == '-' ? 3u : 1u);
    }
  }
}

// Total length of `text` in units.
inline unsigned totalUnits(const char* text) {
  unsigned total = 0;
  forEachStretch(text, [&](bool, unsigned units) { total += units; });
  return total;
}

// Whether the signal is on during unit `unit` (0-based). False past the end.
inline bool isOn(const char* text, unsigned unit) {
  unsigned start = 0;
  bool on = false;
  bool found = false;
  forEachStretch(text, [&](bool stretchOn, unsigned units) {
    if (!found && unit < start + units) {
      on = stretchOn;
      found = true;
    }
    start += units;
  });
  return found && on;
}

}  // namespace morse
