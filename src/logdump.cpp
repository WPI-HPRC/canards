#include "logdump.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "sdcard.h"

static void sendError(const char *msg) {
  SerialUSB.printf("\n---ERROR %s---\n", msg);
}

static void listLogs() {
  FsFile dir;
  if (!dir.open(LOG_FOLDER_NAME, O_RDONLY) || !dir.isDir()) {
    sendError("no " LOG_FOLDER_NAME " folder");
    return;
  }

  char live[64];
  sdLogFileName(sdLogFileIdx(), live, sizeof(live));
  const char *liveName = strrchr(live, '/') + 1;

  SerialUSB.println("\n---LIST BEGIN---");
  FsFile entry;
  char name[64];
  while (entry.openNext(&dir, O_RDONLY)) {
    entry.getName(name, sizeof(name));
    if (!entry.isDir() && name[0] != '.') {
      SerialUSB.printf("%s %lu%s\n", name, (unsigned long)entry.fileSize(),
                       strcmp(name, liveName) == 0 ? " live" : "");
    }
    entry.close();
  }
  dir.close();
  SerialUSB.println("---LIST END---");
}

static void dumpLog(int idx) {
  if (idx == sdLogFileIdx()) {
    sendError("that is this boot's live log");
    return;
  }

  char path[64];
  sdLogFileName(idx, path, sizeof(path));
  FsFile file;
  if (!file.open(path, O_RDONLY)) {
    sendError("no such log file");
    return;
  }

  // Send exactly the size announced in the BEGIN line, so the host can read
  // a fixed byte count no matter what the file contains.
  const uint32_t size = file.fileSize();
  SerialUSB.printf("\n---BEGIN %s %lu---\n", strrchr(path, '/') + 1,
                   (unsigned long)size);

  uint8_t buf[512];
  uint32_t sent = 0;
  while (sent < size) {
    uint32_t want = min<uint32_t>(sizeof(buf), size - sent);
    int got = file.read(buf, want);
    if (got <= 0) {
      // Pad so the host still gets the byte count it expects, then flag it.
      memset(buf, 0, sizeof(buf));
      while (sent < size) {
        uint32_t n = min<uint32_t>(sizeof(buf), size - sent);
        SerialUSB.write(buf, n);
        sent += n;
      }
      file.close();
      sendError("SD read failed mid-file, rest of the dump is zero padding");
      return;
    }
    if (!SerialUSB) {
      file.close();  // host went away; nobody to finish for
      return;
    }
    SerialUSB.write(buf, got);
    sent += got;
  }
  file.close();
  SerialUSB.println("\n---END---");
}

static void runCommand(char *cmd) {
  while (*cmd == ' ') cmd++;
  if (*cmd == '\0') {
    return;
  }
  if (!sdMounted()) {
    sendError("SD card not mounted");
    return;
  }

  if (strcmp(cmd, "list") == 0) {
    listLogs();
  } else if (strcmp(cmd, "dump") == 0) {
    dumpLog(sdPrevLogFileIdx());
  } else if (strncmp(cmd, "dump ", 5) == 0) {
    char *end;
    long idx = strtol(cmd + 5, &end, 10);
    if (end == cmd + 5 || idx < 0) {
      sendError("usage: dump [index]");
      return;
    }
    dumpLog((int)idx);
  } else {
    sendError("unknown command (try list, dump, dump <N>)");
  }
}

void logDumpCommand(char *line) {
  runCommand(line);
}

void logDumpPoll() {
  static char line[32];
  static size_t len = 0;
  static bool overflow = false;

  while (SerialUSB.available()) {
    char c = SerialUSB.read();
    if (c == '\r' || c == '\n') {
      line[len] = '\0';
      if (!overflow) {
        runCommand(line);
      }
      len = 0;
      overflow = false;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    } else {
      overflow = true;
    }
  }
}
