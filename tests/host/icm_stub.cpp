#include "icm_stub.h"
#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include <vector>

HardwareSerial Serial;
SPIClass SPI;
TwoWire Wire;

static uint32_t g_us = 0;
uint32_t millis() { return g_us / 1000; }
uint32_t micros() { return g_us; }
void delay(uint32_t ms) { g_us += ms * 1000; }
void delayMicroseconds(uint32_t us) { g_us += us; }
void sim_advance_us(uint32_t us) { g_us += us; }
size_t Print::out(const char *s) {
#ifdef HOSTTEST_VERBOSE
  fputs(s, stdout);
#endif
  return strlen(s);
}

static const uint8_t REG_BANK_SEL = 0x76;
static uint8_t r[5][128];
static bool ro[5][128];
static uint8_t bank = 0;
static std::vector<uint8_t> fifo;
static std::vector<StubAccess> logv;
static uint16_t lost = 0;

static void power_on() {
  memset(r, 0, sizeof r);
  bank = 0;
  // Bank 0
  r[0][0x75] = 0x47;   // WHO_AM_I
  r[0][0x4C] = 0x30;   // INTF_CONFIG0
  r[0][0x4F] = 0x06;   // GYRO_CONFIG0
  r[0][0x50] = 0x06;   // ACCEL_CONFIG0
  r[0][0x51] = 0x16;   // GYRO_CONFIG1
  r[0][0x52] = 0x11;   // GYRO_ACCEL_CONFIG0
  r[0][0x53] = 0x0D;   // ACCEL_CONFIG1
  r[0][0x54] = 0x23;   // TMST_CONFIG
  // Bank 1 - gyro AAF 585 Hz (DELT 13, DELTSQR 170, BITSHIFT 8)
  r[1][0x0B] = 0xA0; r[1][0x0C] = 0x0D; r[1][0x0D] = 0xAA; r[1][0x0E] = 0x80;
  // Bank 2 - accel AAF 1163 Hz (DELT 24, DELTSQR 576, BITSHIFT 6)
  r[2][0x03] = 0x30; r[2][0x04] = 0x40; r[2][0x05] = 0x62;
  fifo.clear();
  lost = 0;
}

void stub_reset() { power_on(); memset(ro, 0, sizeof ro); logv.clear(); g_us = 1000000; }
size_t stub_log_size() { return logv.size(); }
const StubAccess &stub_log(size_t i) { return logv[i]; }
void stub_log_clear() { logv.clear(); }
uint8_t stub_reg(uint8_t b, uint8_t reg) { return r[b][reg & 0x7F]; }
void stub_set_reg(uint8_t b, uint8_t reg, uint8_t v) { r[b][reg & 0x7F] = v; }
void stub_set_readonly(uint8_t b, uint8_t reg, bool v) { ro[b][reg & 0x7F] = v; }
uint8_t stub_bank() { return bank; }
void stub_fifo_push(const uint8_t *bytes, size_t n) { fifo.insert(fifo.end(), bytes, bytes + n); }
void stub_set_lost(uint16_t v) { lost = v; }

static uint8_t icm_read(uint8_t reg) {
  uint8_t v;
  if (reg == REG_BANK_SEL) v = bank;
  else if (bank == 0 && reg == 0x30) {            // FIFO_DATA pops
    if (fifo.empty()) v = 0xFF;
    else { v = fifo.front(); fifo.erase(fifo.begin()); }
  }
  else if (bank == 0 && reg == 0x2E) v = (uint8_t)(fifo.size() >> 8);
  else if (bank == 0 && reg == 0x2F) v = (uint8_t)fifo.size();
  else if (bank == 0 && reg == 0x6C) v = (uint8_t)(lost >> 8);
  else if (bank == 0 && reg == 0x6D) v = (uint8_t)lost;
  else v = r[bank][reg];
  logv.push_back({false, bank, reg, v, r[0][0x4E]});
  return v;
}

static void icm_write(uint8_t reg, uint8_t v) {
  logv.push_back({true, bank, reg, v, r[0][0x4E]});
  if (reg == REG_BANK_SEL) { bank = v & 0x07; if (bank > 4) bank = 0; return; }
  if (bank == 0 && reg == 0x11 && (v & 1)) { power_on(); return; }   // Soft reset
  if (bank == 0 && reg == 0x4B && (v & 0x02)) { fifo.clear(); return; } // FIFO flush
  if (ro[bank][reg]) return;
  r[bank][reg] = v;
}

uint8_t SPIClass::transfer(uint8_t b) {
  g_us += 1;
  if (_first) { _first = false; _read = (b & 0x80) != 0; _reg = b & 0x7F; return 0; }
  uint8_t v = 0;
  if (_read) v = icm_read(_reg); else icm_write(_reg, b);
  if (!(bank == 0 && _reg == 0x30)) _reg++;
  return v;
}
