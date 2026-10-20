// Simulated TriSense module for host builds of the library (extras/hosttest).
//
// A deterministic motion profile drives three register-level device models -
// ICM-42688-P on SPI (and on I2C at 0x68 for MODE_I2C), AK09918C at 0x0C and
// BMP580 at 0x46 - so the real driver and fusion code run unmodified. Nothing
// here tries to be physically exact; it only has to be plausible enough to
// exercise every code path and exactly repeatable, so two builds of the
// library can be compared sample by sample.
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include <stdlib.h>

HardwareSerial Serial;
TwoWire Wire;
SPIClass SPI;

static uint64_t g_us = 0;
// Reading the clock costs a microsecond, as the CPU time around it would: a
// driver loop that busy-waits on micros() must see time move.
uint32_t micros() { return (uint32_t)(++g_us); }
uint32_t millis() { return (uint32_t)(++g_us / 1000); }
void sim_advance_us(uint32_t us) { g_us += us; }
void delay(uint32_t ms) { g_us += (uint64_t)ms * 1000; }
void delayMicroseconds(uint32_t us) { g_us += us; }

size_t Print::out(const char *s) {
  static int verbose = -1;
  if (verbose < 0) verbose = getenv("HOSTTEST_VERBOSE") ? 1 : 0;
  if (verbose) fputs(s, stderr);
  return strlen(s);
}

// ---------------------------------------------------------------------------
// Motion truth: body orientation as a quaternion that is a closed-form
// function of time, body rates by differentiating it, gravity and a fixed
// Earth field rotated into the body frame.
// ---------------------------------------------------------------------------
struct Q { double w, x, y, z; };
static Q qmul(Q a, Q b) {
  return { a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
           a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
           a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
           a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w };
}
static Q qaxis(double ax, double ay, double az, double ang) {
  double s = sin(ang / 2);
  return { cos(ang / 2), ax*s, ay*s, az*s };
}
int g_motion = 0;   // 0 = gentle wander, 1 = still, 2 = fast tumble
static Q truth(double t) {
  if (g_motion == 1) return qaxis(0, 0, 1, 0.3);
  double k = (g_motion == 2) ? 4.0 : 1.0;
  Q yaw   = qaxis(0, 0, 1, 0.4 * k * t + 0.5 * sin(0.3 * k * t));
  Q pitch = qaxis(0, 1, 0, 0.35 * sin(0.7 * k * t));
  Q roll  = qaxis(1, 0, 0, 0.30 * sin(0.45 * k * t + 1.0));
  return qmul(qmul(yaw, pitch), roll);
}
static void rotT(Q q, const double v[3], double o[3]) {   // world -> body
  Q c = { q.w, -q.x, -q.y, -q.z };
  Q p = { 0, v[0], v[1], v[2] };
  Q r = qmul(qmul(c, p), q);
  o[0] = r.x; o[1] = r.y; o[2] = r.z;
}
static void body_rates_dps(double t, double w[3]) {
  const double h = 1e-5;
  Q a = truth(t - h), b = truth(t + h), q = truth(t);
  Q d = { (b.w - a.w) / (2*h), (b.x - a.x) / (2*h), (b.y - a.y) / (2*h), (b.z - a.z) / (2*h) };
  Q c = { q.w, -q.x, -q.y, -q.z };
  Q om = qmul(c, d);                       // q* dq/dt = 0.5 * omega_body
  w[0] = 2 * om.x * RAD_TO_DEG; w[1] = 2 * om.y * RAD_TO_DEG; w[2] = 2 * om.z * RAD_TO_DEG;
}
static uint32_t g_rng = 12345;
static double noise(double amp) {          // deterministic, in [-amp, amp]
  g_rng = g_rng * 1664525u + 1013904223u;
  return amp * (((g_rng >> 8) & 0xFFFF) / 32767.5 - 1.0);
}

// ---------------------------------------------------------------------------
// ICM-42688-P
// ---------------------------------------------------------------------------
struct Icm {
  uint8_t r[128];
  uint8_t fifo[4096];             // ring buffer
  uint16_t n = 0, head = 0;
  uint64_t nextSampleUs = 0;
  uint16_t lost = 0;
  bool full = false;
  void reset() {
    memset(r, 0, sizeof r);
    r[0x75] = 0x47; r[0x4C] = 0x30; r[0x4F] = 0x06; r[0x50] = 0x06; r[0x13] = 0x05;
    n = 0; head = 0; lost = 0; full = false; nextSampleUs = g_us;
  }
  double odrHz() {
    switch (r[0x50] & 0x0F) {
      case 1: return 32000; case 2: return 16000; case 3: return 8000; case 4: return 4000;
      case 5: return 2000;  case 6: return 1000;  case 7: return 200;  case 8: return 100;
      case 9: return 50;    case 10: return 25;   case 11: return 12.5; case 15: return 500;
    }
    return 1000;
  }
  bool hires() { return (r[0x5F] & 0x10) != 0; }
  bool streaming() { return (r[0x16] & 0xC0) == 0x40 && (r[0x5F] & 0x03) == 0x03; }
  void sample(double t, int32_t a[3], int32_t g[3], bool hr) {
    double gb[3], mb[3], w[3];
    const double gw[3] = { 0, 0, 1 };
    rotT(truth(t), gw, gb);
    body_rates_dps(t, w);
    const double bias[3] = { 0.8, -0.5, 0.3 };   // dps, for the bias estimators
    (void)mb;
    double afs = 16.0 / (1 << ((r[0x50] >> 5) & 7));
    double gfs = 2000.0 / (1 << ((r[0x4F] >> 5) & 7));
    double alsb = hr ? 32768.0 : 32768.0 / afs, glsb = hr ? 262.144 : 32768.0 / gfs;
    int32_t lim = hr ? 524287 : 32767;
    for (int i = 0; i < 3; i++) {
      double av = (gb[i] + noise(0.002)) * alsb, gv = (w[i] + bias[i] + noise(0.05)) * glsb;
      a[i] = (int32_t)lrint(av); g[i] = (int32_t)lrint(gv);
      if (a[i] > lim) a[i] = lim; if (a[i] < -lim) a[i] = -lim;
      if (g[i] > lim) g[i] = lim; if (g[i] < -lim) g[i] = -lim;
    }
  }
  void generate() {
    if (!streaming()) { nextSampleUs = g_us; return; }
    const double period = 1e6 / odrHz();
    while ((double)nextSampleUs <= (double)g_us) {
      const double t = nextSampleUs * 1e-6;
      nextSampleUs += (uint64_t)period;
      const bool hr = hires();
      const uint16_t sz = hr ? 20 : 16;
      if (n + sz > 2048) { full = true; lost++; continue; }
      int32_t a[3], g[3];
      sample(t, a, g, hr);
      uint8_t p[20];
      memset(p, 0, sz);
      p[0] = hr ? 0x78 : 0x68;
      for (int i = 0; i < 3; i++) {
        int32_t av = hr ? (a[i] >> 4) : a[i], gv = hr ? (g[i] >> 4) : g[i];
        p[1 + 2*i] = (uint8_t)(av >> 8); p[2 + 2*i] = (uint8_t)av;
        p[7 + 2*i] = (uint8_t)(gv >> 8); p[8 + 2*i] = (uint8_t)gv;
        if (hr) p[17 + i] = (uint8_t)(((g[i] & 0xF) << 4) | (a[i] & 0xF));
      }
      for (uint16_t i = 0; i < sz; i++) fifo[(head + n + i) & 4095] = p[i];
      n += sz;
    }
  }
  uint8_t read(uint8_t reg) {
    generate();
    if (reg == 0x30) {                       // FIFO_DATA pops
      if (n == 0) return 0xFF;
      uint8_t b = fifo[head];
      head = (head + 1) & 4095; n--;
      return b;
    }
    if (reg == 0x2E) return (uint8_t)(n >> 8);
    if (reg == 0x2F) return (uint8_t)n;
    if (reg == 0x2D) { uint8_t s = full ? 0x02 : 0; full = false; return s; }   // INT_STATUS, read-to-clear
    if (reg == 0x6C) return (uint8_t)(lost >> 8);   // High byte first (register map table)
    if (reg == 0x6D) return (uint8_t)lost;
    if (reg >= 0x1F && reg <= 0x2A) {        // direct data registers (FIFO_NONE)
      int32_t a[3], g[3];
      sample(g_us * 1e-6, a, g, false);
      int k = reg - 0x1F; int32_t v = (k < 6) ? a[k / 2] : g[(k - 6) / 2];
      return (k & 1) ? (uint8_t)v : (uint8_t)(v >> 8);
    }
    if (reg == 0x1D) return 0x08;
    if (reg == 0x1E) return 0x00;
    return r[reg & 0x7F];
  }
  void write(uint8_t reg, uint8_t v) {
    generate();
    if (reg == 0x11 && (v & 1)) { reset(); return; }
    if (reg == 0x4B && (v & 0x02)) { n = 0; head = 0; }
    r[reg & 0x7F] = v;
  }
} g_icm;

// ---------------------------------------------------------------------------
// AK09918C
// ---------------------------------------------------------------------------
struct Ak {
  uint8_t r[0x40];
  uint64_t lastSampleUs = 0, lastReadUs = 0;
  void reset() { memset(r, 0, sizeof r); r[0x00] = 0x48; r[0x01] = 0x0C; lastReadUs = g_us; }
  double hz() {
    switch (r[0x31]) { case 0x02: return 10; case 0x04: return 20; case 0x06: return 50; case 0x08: return 100; }
    return 0;
  }
  bool drdy() { return hz() > 0 && (g_us - lastReadUs) >= (uint64_t)(1e6 / hz()); }
  uint8_t data[9];
  void latch() {
    const double ew[3] = { 20.0, 2.0, -45.0 };   // uT, world frame
    double b[3];
    rotT(truth(g_us * 1e-6), ew, b);
    const double hard[3] = { 6.0, -4.0, 3.0 };
    for (int i = 0; i < 3; i++) {
      int16_t v = (int16_t)lrint((b[i] + hard[i] + noise(0.2)) / 0.15);
      data[2*i] = (uint8_t)v; data[2*i + 1] = (uint8_t)(v >> 8);
    }
    data[6] = 0; data[7] = 0; data[8] = 0;
  }
  uint8_t read(uint8_t reg) {
    if (reg == 0x10) { if (drdy()) { latch(); return 0x01; } return 0x00; }
    if (reg >= 0x11 && reg <= 0x18) { if (reg == 0x18) lastReadUs = g_us; return data[reg - 0x11]; }
    return r[reg & 0x3F];
  }
  void write(uint8_t reg, uint8_t v) { if (reg == 0x32 && (v & 1)) { uint8_t m = r[0x31]; reset(); r[0x31] = m; } r[reg & 0x3F] = v; }
} g_ak;

// ---------------------------------------------------------------------------
// BMP580
// ---------------------------------------------------------------------------
struct Bmp {
  uint8_t r[0x80];
  void reset() { memset(r, 0, sizeof r); r[0x01] = 0x50; }
  uint8_t read(uint8_t reg) {
    if (reg == 0x38) return 0x80 | (r[0x36] & 0x3F);
    if (reg >= 0x1D && reg <= 0x22) {
      const double t = g_us * 1e-6;
      const double press = 97000.0 + 30.0 * sin(0.2 * t) + noise(1.0);
      const double temp = 24.5 + 0.1 * sin(0.05 * t);
      int32_t tr = (int32_t)lrint(temp * 65536.0); uint32_t pr = (uint32_t)lrint(press * 64.0);
      switch (reg) {
        case 0x1D: return (uint8_t)tr; case 0x1E: return (uint8_t)(tr >> 8); case 0x1F: return (uint8_t)(tr >> 16);
        case 0x20: return (uint8_t)pr; case 0x21: return (uint8_t)(pr >> 8); default: return (uint8_t)(pr >> 16);
      }
    }
    return r[reg & 0x7F];
  }
  void write(uint8_t reg, uint8_t v) { r[reg & 0x7F] = v; }
} g_bmp;

void sim_reset(int motion, uint32_t seed) {
  g_us = 1000000; g_motion = motion; g_rng = seed;
  g_icm.reset(); g_ak.reset(); g_bmp.reset();
}
// Change the motion without touching the devices' registers.
void sim_motion(int motion, uint32_t seed) { g_motion = motion; g_rng = seed; }

// ---------------------------------------------------------------------------
// Buses
// ---------------------------------------------------------------------------
static bool dev_read(uint8_t addr, uint8_t reg, uint8_t &v) {
  switch (addr) {
    case 0x68: v = g_icm.read(reg); return true;
    case 0x0C: v = g_ak.read(reg);  return true;
    case 0x46: v = g_bmp.read(reg); return true;
  }
  return false;
}
static bool dev_write(uint8_t addr, uint8_t reg, uint8_t v) {
  switch (addr) {
    case 0x68: g_icm.write(reg, v); return true;
    case 0x0C: g_ak.write(reg, v);  return true;
    case 0x46: g_bmp.write(reg, v); return true;
  }
  return false;
}
static bool autoinc(uint8_t addr, uint8_t reg) { return !(addr == 0x68 && reg == 0x30); }

// Bus traffic takes time, as on the real thing: 400 kHz I2C is ~23 us a byte,
// 10 MHz SPI under 1 us. Without it a driver loop that polls for data would
// never see the clock move.
uint8_t TwoWire::endTransmission(bool) {
  g_us += 23u * (_n + 1u);
  uint8_t dummy;
  if (!dev_read(_addr, 0x7F, dummy)) return 2;          // NACK: no such device
  if (_n >= 1) _reg = _tx[0];
  for (uint8_t i = 1; i < _n; i++) { dev_write(_addr, _reg, _tx[i]); if (autoinc(_addr, _reg)) _reg++; }
  return 0;
}
uint8_t TwoWire::requestFrom(int a, int n) {
  _rn = _ri = 0;
  g_us += 23u * (uint32_t)(n + 1);
  uint8_t v;
  if (!dev_read((uint8_t)a, 0x7F, v)) return 0;
  for (int i = 0; i < n && i < (int)sizeof _rx; i++) {
    dev_read((uint8_t)a, _reg, _rx[_rn++]);
    if (autoinc((uint8_t)a, _reg)) _reg++;
  }
  return _rn;
}

uint8_t SPIClass::transfer(uint8_t b) {
  g_us += 1;
  if (_first) { _first = false; _read = (b & 0x80) != 0; _reg = b & 0x7F; return 0; }
  uint8_t v = 0;
  if (_read) v = g_icm.read(_reg); else g_icm.write(_reg, b);
  if (_reg != 0x30) _reg++;
  return v;
}

// Ground truth for accuracy checks: the sensor-frame attitude as a quaternion
// (sensor -> world), at the current simulated time.
void sim_truth(double q[4]) {
  Q t = truth(g_us * 1e-6);
  q[0] = t.w; q[1] = t.x; q[2] = t.y; q[3] = t.z;
}
