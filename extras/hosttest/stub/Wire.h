#pragma once
#include "Arduino.h"
// I2C bus routed to the simulated BMP580 (0x46) and AK09918C (0x0C).
class TwoWire {
public:
  void begin() {}
  void setClock(uint32_t) {}
  void setSDA(int) {}
  void setSCL(int) {}
  void beginTransmission(uint8_t a) { _addr = a; _n = 0; }
  size_t write(uint8_t b) { if (_n < sizeof _tx) _tx[_n++] = b; return 1; }
  uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(uint8_t a, uint8_t n) { return requestFrom((int)a, (int)n); }
  uint8_t requestFrom(int a, int n);
  int available() { return _rn - _ri; }
  int read() { return _ri < _rn ? _rx[_ri++] : -1; }
private:
  uint8_t _addr = 0, _tx[64], _n = 0, _rx[64], _rn = 0, _ri = 0, _reg = 0;
};
extern TwoWire Wire;
