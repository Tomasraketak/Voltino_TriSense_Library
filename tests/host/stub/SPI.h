#pragma once
#include "Arduino.h"
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3
class SPISettings {
public:
  SPISettings() {}
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};
// SPI bus wired to the register-level ICM-42688-P stub (icm_stub.cpp). One
// beginTransaction() .. endTransaction() is one CS-low frame: the first byte
// is the address (bit 7 = read), the rest auto-increment except FIFO_DATA.
class SPIClass {
public:
  void begin() {}
  void beginTransaction(SPISettings) { _first = true; }
  void endTransaction() {}
  uint8_t transfer(uint8_t b);
  void transfer(void *buf, size_t n) {
    uint8_t *p = (uint8_t *)buf;
    for (size_t i = 0; i < n; i++) p[i] = transfer(p[i]);
  }
private:
  bool _first = true;
  uint8_t _reg = 0;
  bool _read = false;
};
extern SPIClass SPI;
