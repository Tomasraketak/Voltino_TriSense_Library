#pragma once
#include "Arduino.h"
// Not used by the tests (they run the driver over SPI); present so the driver compiles.
class TwoWire {
public:
  void begin() {}
  void setClock(uint32_t) {}
  void setSDA(int) {}
  void setSCL(int) {}
  void beginTransmission(uint8_t) {}
  size_t write(uint8_t) { return 1; }
  uint8_t endTransmission(bool = true) { return 2; }
  uint8_t requestFrom(int, int) { return 0; }
  uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
  int available() { return 0; }
  int read() { return -1; }
};
extern TwoWire Wire;
