#include "Subsystems/Logger/CycleLogFormat.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace CycleLogFormat {

namespace {

// Appends to a fixed buffer; remembers if anything did not fit.
class Out {
 public:
  Out(char* buf, size_t n) : buf_(buf), n_(n) {
    if (n_ > 0) buf_[0] = '\0';
  }
  void raw(const char* s) {
    while (*s) put(*s++);
  }
  void sep() { put(','); }
  void u64(uint64_t v) {
    // Not printf: newlib-nano's printf has no 64-bit integers.
    char tmp[21];
    int i = 0;
    do {
      tmp[i++] = static_cast<char>('0' + v % 10);
      v /= 10;
    } while (v != 0);
    while (i > 0) put(tmp[--i]);
  }
  void i32(long v) { fmt("%ld", v); }
  void exact(float v) { fmt("%.9g", static_cast<double>(v)); }
  void fixed(float v, int decimals) { fmt("%.*f", decimals, static_cast<double>(v)); }
  // Returns the length, or 0 if it did not fit (the newline included).
  size_t finish() {
    put('\n');
    if (overflow_ || len_ >= n_) return 0;
    buf_[len_] = '\0';
    return len_;
  }

 private:
  void fmt(const char* f, ...) __attribute__((format(printf, 2, 3))) {
    char tmp[48];
    va_list args;
    va_start(args, f);
    const int w = std::vsnprintf(tmp, sizeof(tmp), f, args);
    va_end(args);
    if (w < 0 || static_cast<size_t>(w) >= sizeof(tmp)) {
      overflow_ = true;
      return;
    }
    raw(tmp);
  }
  void put(char c) {
    if (len_ + 1 >= n_) {
      overflow_ = true;
      return;
    }
    buf_[len_++] = c;
  }
  char* buf_;
  size_t n_;
  size_t len_ = 0;
  bool overflow_ = false;
};

const char* const kHeader =
    // inputs
    "t_us,imu_ok,ax,ay,az,gx,gy,gz,mag_new,mx,my,mz,baro_new,p_pa,temp_c,"
    "enc_ok,enc0,enc1,enc2,enc3,cmds,"
    // outputs
    "cycle,flight_us,state,transition,refused,safety,safety_sticky,"
    "nav_ok,sf,speed,alt,tilt,qw,qx,qy,qz,"
    "mode,enabled,locked,cutoff,gates,denied,"
    "cmd0,cmd1,cmd2,cmd3,servo_power,pulse0,pulse1,pulse2,pulse3";

// Splits a line into fields in place (commas -> '\0'); returns the count.
size_t split(char* line, char* fields[], size_t max) {
  size_t n = 0;
  char* p = line;
  while (n < max) {
    fields[n++] = p;
    char* comma = std::strchr(p, ',');
    if (comma == nullptr) break;
    *comma = '\0';
    p = comma + 1;
  }
  // Drop a trailing newline from the last field.
  if (n > 0) {
    char* last = fields[n - 1];
    const size_t len = std::strlen(last);
    if (len > 0 && last[len - 1] == '\n') last[len - 1] = '\0';
    if (len > 1 && last[len - 2] == '\r') last[len - 2] = '\0';
  }
  return n;
}

bool toFloat(const char* s, float& out) {
  if (*s == '\0') return false;
  char* end = nullptr;
  out = std::strtof(s, &end);
  return end != s && *end == '\0';
}

bool toU64(const char* s, uint64_t& out) {
  if (*s == '\0') return false;
  uint64_t v = 0;
  for (const char* p = s; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    v = v * 10 + static_cast<uint64_t>(*p - '0');
  }
  out = v;
  return true;
}

bool toLong(const char* s, long& out) {
  if (*s == '\0') return false;
  char* end = nullptr;
  out = std::strtol(s, &end, 10);
  return end != s && *end == '\0';
}

constexpr size_t kMaxFields = 96;

}  // namespace

size_t formatHeader(char* buf, size_t n) {
  Out o(buf, n);
  o.raw(kHeader);
  return o.finish();
}

size_t formatRow(const CycleRecord& r, char* buf, size_t n) {
  Out o(buf, n);
  const SensorFrame& f = r.frame;
  o.u64(f.timeUs);
  o.sep(); o.i32(f.imuOk ? 1 : 0);
  o.sep(); o.exact(f.accel.x); o.sep(); o.exact(f.accel.y); o.sep(); o.exact(f.accel.z);
  o.sep(); o.exact(f.gyro.x); o.sep(); o.exact(f.gyro.y); o.sep(); o.exact(f.gyro.z);
  o.sep(); o.i32(f.magFresh ? 1 : 0);
  o.sep(); o.exact(f.mag.x); o.sep(); o.exact(f.mag.y); o.sep(); o.exact(f.mag.z);
  o.sep(); o.i32(f.baroFresh ? 1 : 0);
  o.sep(); o.exact(f.pressurePa); o.sep(); o.exact(f.temperatureC);
  long encOk = 0;
  for (int i = 0; i < kNumCanards; ++i) encOk |= r.feedback.encoderValid[i] ? (1L << i) : 0;
  o.sep(); o.i32(encOk);
  for (int i = 0; i < kNumCanards; ++i) {
    o.sep();
    o.exact(r.feedback.encoderAngleRad[i]);
  }
  o.sep();
  for (size_t i = 0; i < r.commandCount && i < CommandList::kCapacity; ++i) {
    if (i) o.raw("|");
    const Command& c = r.commands[i];
    o.i32(static_cast<long>(c.type));
    o.raw(":");
    o.i32(c.canard);
    o.raw(":");
    o.exact(c.angleRad);
  }

  // Outputs.
  o.sep(); o.u64(r.cycleCount);
  o.sep(); o.u64(r.flightTimeUs);
  o.sep(); o.i32(static_cast<long>(r.state));
  o.sep(); o.i32(r.transitioned ? 1 : 0);
  o.sep(); o.u64(r.refusedTransitions);
  o.sep(); o.i32(r.safetyActive);
  o.sep(); o.i32(r.safetySticky);
  const long navOk = (r.imuValid ? 1 : 0) | (r.attitudeValid ? 2 : 0) | (r.speedValid ? 4 : 0) |
                     (r.altitudeValid ? 8 : 0);
  o.sep(); o.i32(navOk);
  o.sep(); o.fixed(r.specificForce, 3);
  o.sep(); o.fixed(r.speed, 3);
  o.sep(); o.fixed(r.nav.altitudeAgl, 2);
  o.sep(); o.fixed(r.tilt, 5);
  o.sep(); o.fixed(r.nav.attitude.w, 6); o.sep(); o.fixed(r.nav.attitude.x, 6);
  o.sep(); o.fixed(r.nav.attitude.y, 6); o.sep(); o.fixed(r.nav.attitude.z, 6);
  const CanardsStatus& c = r.canards;
  o.sep(); o.i32(static_cast<long>(c.mode));
  o.sep(); o.i32(c.enabled ? 1 : 0);
  o.sep(); o.i32(c.lockedOut ? 1 : 0);
  o.sep(); o.i32(c.cutoffLatched ? 1 : 0);
  o.sep(); o.i32(c.gates);
  o.sep(); o.u64(c.deniedCount);
  for (int i = 0; i < kNumCanards; ++i) {
    o.sep();
    o.fixed(c.commandRad[i], 6);
  }
  o.sep(); o.i32(r.actuators.servoPower ? 1 : 0);
  for (int i = 0; i < kNumCanards; ++i) {
    o.sep();
    o.fixed(r.actuators.pulseUs[i], 1);
  }
  return o.finish();
}

bool Columns::complete() const {
  const int needed[] = {tUs, imuOk, ax, ay, az, gx, gy, gz, magNew, mx, my, mz,
                        baroNew, pPa, tempC, encOk, enc[0], enc[1], enc[2], enc[3], cmds};
  for (int index : needed) {
    if (index < 0) return false;
  }
  return true;
}

bool parseHeader(const char* line, Columns& out) {
  char copy[kMaxLine];
  const size_t len = std::strlen(line);
  if (len >= sizeof(copy)) return false;
  std::memcpy(copy, line, len + 1);
  char* fields[kMaxFields];
  const size_t n = split(copy, fields, kMaxFields);
  out = Columns{};
  struct Name {
    const char* name;
    int* slot;
  };
  const Name names[] = {
      {"t_us", &out.tUs},       {"imu_ok", &out.imuOk},   {"ax", &out.ax},       {"ay", &out.ay},
      {"az", &out.az},          {"gx", &out.gx},          {"gy", &out.gy},       {"gz", &out.gz},
      {"mag_new", &out.magNew}, {"mx", &out.mx},          {"my", &out.my},       {"mz", &out.mz},
      {"baro_new", &out.baroNew}, {"p_pa", &out.pPa},     {"temp_c", &out.tempC}, {"enc_ok", &out.encOk},
      {"enc0", &out.enc[0]},    {"enc1", &out.enc[1]},    {"enc2", &out.enc[2]}, {"enc3", &out.enc[3]},
      {"cmds", &out.cmds},
  };
  for (size_t i = 0; i < n; ++i) {
    for (const Name& name : names) {
      if (std::strcmp(fields[i], name.name) == 0) *name.slot = static_cast<int>(i);
    }
  }
  return out.complete();
}

bool parseRow(const char* line, const Columns& col, ReplayInput& out) {
  if (!col.complete()) return false;
  char copy[kMaxLine];
  const size_t len = std::strlen(line);
  if (len >= sizeof(copy)) return false;
  std::memcpy(copy, line, len + 1);
  char* fields[kMaxFields];
  const size_t n = split(copy, fields, kMaxFields);
  auto field = [&](int index) -> const char* {
    return index >= 0 && static_cast<size_t>(index) < n ? fields[index] : nullptr;
  };
  auto flag = [&](int index, bool& v) {
    long x = 0;
    const char* s = field(index);
    if (s == nullptr || !toLong(s, x)) return false;
    v = x != 0;
    return true;
  };
  auto number = [&](int index, float& v) {
    const char* s = field(index);
    return s != nullptr && toFloat(s, v);
  };

  ReplayInput in;
  const char* t = field(col.tUs);
  if (t == nullptr || !toU64(t, in.frame.timeUs)) return false;
  bool ok = flag(col.imuOk, in.frame.imuOk) && number(col.ax, in.frame.accel.x) &&
            number(col.ay, in.frame.accel.y) && number(col.az, in.frame.accel.z) &&
            number(col.gx, in.frame.gyro.x) && number(col.gy, in.frame.gyro.y) &&
            number(col.gz, in.frame.gyro.z) && flag(col.magNew, in.frame.magFresh) &&
            number(col.mx, in.frame.mag.x) && number(col.my, in.frame.mag.y) &&
            number(col.mz, in.frame.mag.z) && flag(col.baroNew, in.frame.baroFresh) &&
            number(col.pPa, in.frame.pressurePa) && number(col.tempC, in.frame.temperatureC);
  if (!ok) return false;

  long encOk = 0;
  const char* e = field(col.encOk);
  if (e == nullptr || !toLong(e, encOk)) return false;
  for (int i = 0; i < kNumCanards; ++i) {
    in.feedback.encoderValid[i] = (encOk >> i) & 1;
    if (!number(col.enc[i], in.feedback.encoderAngleRad[i])) return false;
  }

  // "type:canard:angle|..."
  const char* cmds = field(col.cmds);
  if (cmds == nullptr) return false;
  char list[kMaxLine];
  std::strncpy(list, cmds, sizeof(list) - 1);
  list[sizeof(list) - 1] = '\0';
  char* p = list;
  while (*p != '\0') {
    if (in.commandCount >= CommandList::kCapacity) return false;
    char* bar = std::strchr(p, '|');
    if (bar != nullptr) *bar = '\0';
    char* c1 = std::strchr(p, ':');
    char* c2 = c1 ? std::strchr(c1 + 1, ':') : nullptr;
    if (c1 == nullptr || c2 == nullptr) return false;
    *c1 = '\0';
    *c2 = '\0';
    long type = 0, canard = 0;
    float angle = 0.0f;
    if (!toLong(p, type) || !toLong(c1 + 1, canard) || !toFloat(c2 + 1, angle)) return false;
    if (type < 0 || type > static_cast<long>(CommandType::CanardSweep) || canard < -128 || canard > 127) return false;
    Command& c = in.commands[in.commandCount++];
    c.type = static_cast<CommandType>(type);
    c.canard = static_cast<int8_t>(canard);
    c.angleRad = angle;
    if (bar == nullptr) break;
    p = bar + 1;
  }

  out = in;
  return true;
}

}  // namespace CycleLogFormat
