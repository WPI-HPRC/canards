#include "telemetry.h"

// DEBUG builds (the *_debug envs) print every command line received over the
// radio, and what was done with it, over SerialUSB.
#ifdef DEBUG
#define TLM_CMD_DEBUG(...) SerialUSB.printf(__VA_ARGS__)
#else
#define TLM_CMD_DEBUG(...) do {} while (0)
#endif

// ------------------------------------------------------------------ helpers --

// XOR of every byte, as in NMEA.
static uint8_t checksum(const uint8_t *data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) {
    cs ^= data[i];
  }
  return cs;
}

static int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

size_t Telemetry::LineBuffer::write(uint8_t c) {
  if (len_ >= sizeof(buf_)) {
    overflow_ = true;
    return 0;
  }
  buf_[len_++] = c;
  return 1;
}

// ------------------------------------------------------------------- set up --

bool Telemetry::addCommand(const char *name, CommandHandler handler) {
  if (commandCount_ >= MAX_COMMANDS) {
    return false;
  }
  commands_[commandCount_++] = {name, handler};
  return true;
}

bool Telemetry::begin(LoRaE22 &radio, const TelemetryColumn *columns, size_t count,
                      bool radioOk) {
  if (!radioOk) {
    SerialUSB.println("Telemetry: radio failed to initialize, transmitting anyway");
  }
  radio_ = &radio;
  columns_ = columns;
  count_ = count;
  headerDue_ = true;
  lastRow_ = lastHeader_ = millis();
  SerialUSB.printf("Telemetry: %u columns, %u commands\n", (unsigned)count_,
                   (unsigned)commandCount_);
  return true;
}

// --------------------------------------------------------------------- loop --

void Telemetry::update(uint32_t now_ms) {
  if (!radio_) {
    return;
  }

  pollCommands();

  if (now_ms - lastHeader_ >= HEADER_PERIOD_MS) {
    headerDue_ = true;
    lastHeader_ = now_ms;
  }

  // At most one transmission per call, in priority order: a command reply, the
  // header, a data row. Only send when the module's buffer is empty (AUX high),
  // so nothing we write gets dropped by the radio.
  bool rowDue = now_ms - lastRow_ >= ROW_PERIOD_MS;
  if (!pendingReplyCmd_ && !headerDue_ && !rowDue) {
    return;
  }
  if (!radio_->moduleReady()) {
    if (rowDue) {
      rowsSkipped_++;
      lastRow_ = now_ms;
    }
    return;
  }

  if (pendingReplyCmd_) {
    buildReply(pendingReplyCmd_, pendingReplyResult_);
    pendingReplyCmd_ = nullptr;
  } else if (headerDue_) {
    buildHeader();
    headerDue_ = false;
  } else {
    buildRow();
    lastRow_ = now_ms;
    rowsSent_++;
  }
  sendLine();
}

// ----------------------------------------------------------- building lines --

void Telemetry::buildHeader() {
  line_.clear();
  line_.print("$H");
  for (size_t i = 0; i < count_; i++) {
    line_.write(',');
    line_.print(columns_[i].name);
  }
  finishLine();
}

void Telemetry::buildRow() {
  line_.clear();
  line_.print("$D");
  for (size_t i = 0; i < count_; i++) {
    line_.write(',');
    columns_[i].write(line_);
  }
  finishLine();
}

void Telemetry::buildReply(const char *cmd, const char *result) {
  line_.clear();
  line_.print("$A,");
  line_.print(cmd);
  line_.write(',');
  line_.print(result);
  finishLine();
}

// Appends "*XX\n". The checksum covers everything after the leading '$'.
void Telemetry::finishLine() {
  uint8_t cs = checksum(line_.data() + 1, line_.length() - 1);
  line_.printf("*%02X\n", cs);
}

// Hands the line to the radio, split into messages the HAL can frame.
bool Telemetry::sendLine() {
  if (line_.overflow()) {
    // Grow MAX_LINE or trim the column table.
    SerialUSB.println("Telemetry: line too long, dropped");
    return false;
  }
  const uint8_t *data = line_.data();
  size_t left = line_.length();
  bool ok = true;
  while (left > 0) {
    size_t n = left < MAX_MESSAGE_SIZE ? left : MAX_MESSAGE_SIZE;
    ok &= radio_->sendMessage(data, n);
    data += n;
    left -= n;
  }
  return ok;
}

// ---------------------------------------------------------------- commands --

// Reads the messages the radio has received and runs any complete command
// lines. The HAL strips the <callsign><length> framing; the payloads are
// joined into one stream, so a line may span messages.
void Telemetry::pollCommands() {
  radio_->update();
  uint8_t msg[MAX_MESSAGE_SIZE];
  uint16_t len = 0;
  while (radio_->getMessage(msg, sizeof(msg), len)) {
    for (uint16_t i = 0; i < len; i++) {
      pollByte((char)msg[i]);
    }
  }
}

void Telemetry::pollByte(char c) {
  if (c == '$') {
    // Start of a line. Also restarts a line that never ended.
    inRxLine_ = true;
    rxLen_ = 0;
  } else if (!inRxLine_) {
    // Noise between lines.
  } else if (c == '\n' || c == '\r') {
    inRxLine_ = false;
    rx_[rxLen_] = '\0';
    handleRxLine();
  } else if (rxLen_ < sizeof(rx_) - 1) {
    rx_[rxLen_++] = c;
  } else {
    inRxLine_ = false;  // too long to be a command
  }
}

// rx_ holds a line without its '$' and '\n', e.g. "C,armFlight*29".
void Telemetry::handleRxLine() {
  TLM_CMD_DEBUG("Telemetry: received \"$%s\"\n", rx_);
  char *star = strrchr(rx_, '*');
  if (!star || strlen(star) != 3) {
    TLM_CMD_DEBUG("Telemetry:   ignored (no checksum)\n");
    return;
  }
  int hi = hexValue(star[1]);
  int lo = hexValue(star[2]);
  if (hi < 0 || lo < 0 ||
      checksum((const uint8_t *)rx_, star - rx_) != (uint8_t)(hi << 4 | lo)) {
    TLM_CMD_DEBUG("Telemetry:   ignored (bad checksum)\n");
    return;
  }
  *star = '\0';

  if (strncmp(rx_, "C,", 2) != 0) {
    TLM_CMD_DEBUG("Telemetry:   ignored (not a command)\n");
    return;
  }
  const char *name = rx_ + 2;
  commandsReceived_++;

  pendingReplyCmd_ = "?";
  pendingReplyResult_ = "UNKNOWN";
  for (size_t i = 0; i < commandCount_; i++) {
    if (strcmp(name, commands_[i].name) == 0) {
      bool accepted = commands_[i].handler();
      pendingReplyCmd_ = commands_[i].name;  // points at a string that lives on
      pendingReplyResult_ = accepted ? "OK" : "REJECTED";
      break;
    }
  }
  TLM_CMD_DEBUG("Telemetry:   command '%s' -> %s\n", name, pendingReplyResult_);
}
