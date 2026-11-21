#include "Subsystems/Logger/StreamProtocol.h"

#include <cstring>

namespace StreamProtocol {

size_t formatError(const char* reason, char* buf, size_t n) {
  const size_t prefix = std::strlen(kErrorPrefix);
  const size_t reasonLen = std::strlen(reason);
  const size_t len = prefix + reasonLen + 1;  // + '\n'
  if (len + 1 > n) return 0;
  std::memcpy(buf, kErrorPrefix, prefix);
  std::memcpy(buf + prefix, reason, reasonLen);
  buf[len - 1] = '\n';
  buf[len] = '\0';
  return len;
}

}  // namespace StreamProtocol

namespace {

bool isDigit(char c) { return c >= '0' && c <= '9'; }
bool isLetter(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; }

}  // namespace

StreamSession::Result StreamSession::fail(const char* reason) {
  error_ = reason;
  ++errors_;
  return Result::Error;
}

StreamSession::Result StreamSession::feedLine(const char* line) {
  const char* p = line;
  while (*p == ' ' || *p == '\t') ++p;
  if (*p == '\0' || *p == '\r' || *p == '\n' || *p == '#') return Result::Ignored;

  // A row starts with its sample time; a header with a column name.
  if (isLetter(*p)) {
    CycleLogFormat::Columns columns;
    if (!CycleLogFormat::parseHeader(p, columns)) return fail("bad header");
    columns_ = columns;
    headerOk_ = true;
    havePending_ = false;
    return Result::Header;
  }
  if (!isDigit(*p)) return fail("unknown line");
  if (!headerOk_) return fail("no header");
  if (havePending_) return fail("busy");
  if (!CycleLogFormat::parseRow(p, columns_, pending_)) return fail("bad row");
  havePending_ = true;
  ++rowsAccepted_;
  return Result::Row;
}

bool StreamSession::fetch(CycleLogFormat::ReplayInput& out) {
  if (!havePending_) return false;
  out = pending_;
  havePending_ = false;
  return true;
}

size_t StreamSession::replyHeader(char* buf, size_t n) { return CycleLogFormat::formatHeader(buf, n); }

size_t StreamSession::replyRow(const CycleRecord& record, char* buf, size_t n) {
  return CycleLogFormat::formatRow(record, buf, n);
}

size_t StreamSession::replyError(char* buf, size_t n) const { return StreamProtocol::formatError(error_, buf, n); }
