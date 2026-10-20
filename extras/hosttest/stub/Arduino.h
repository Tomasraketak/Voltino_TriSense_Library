// Minimal Arduino API for building the library on a PC (extras/hosttest).
// Only what the library uses. Time is simulated: sim_advance_us() moves it.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

typedef uint8_t byte;
typedef unsigned int uint;
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LSBFIRST 0
#define MSBFIRST 1
#define DEC 10
#define HEX 16

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
void sim_advance_us(uint32_t us);
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline void yield() {}

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))
#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(p) (*(const uint8_t *)(p))
#define pgm_read_word(p) (*(const uint16_t *)(p))
#define pgm_read_dword(p) (*(const uint32_t *)(p))
#define pgm_read_float(p) (*(const float *)(p))
#define memcpy_P memcpy

// Serial output is swallowed unless HOSTTEST_VERBOSE is set: the traces the
// harness compares are written by the harness itself.
class Print {
public:
  size_t print(const __FlashStringHelper *s) { return out((const char *)s); }
  size_t print(const char *s) { return out(s); }
  size_t print(char c) { char b[2] = {c, 0}; return out(b); }
  size_t print(int v, int = DEC) { return outf("%d", v); }
  size_t print(unsigned v, int = DEC) { return outf("%u", v); }
  size_t print(long v, int = DEC) { return outf("%ld", v); }
  size_t print(unsigned long v, int = DEC) { return outf("%lu", v); }
  size_t print(double v, int d = 2) { return outf("%.*f", d, v); }
  template <typename T> size_t println(T v) { size_t n = print(v); return n + out("\n"); }
  template <typename T> size_t println(T v, int d) { size_t n = print(v, d); return n + out("\n"); }
  size_t println() { return out("\n"); }
  int printf(const char *, ...) { return 0; }
private:
  size_t out(const char *s);
  template <typename T> size_t outf(const char *f, T v) { char b[64]; snprintf(b, sizeof b, f, v); return out(b); }
  size_t outf(const char *f, int d, double v) { char b[64]; snprintf(b, sizeof b, f, d, v); return out(b); }
};
class HardwareSerial : public Print {
public:
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  operator bool() { return true; }
};
extern HardwareSerial Serial;
