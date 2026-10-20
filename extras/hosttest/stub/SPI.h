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
// SPI bus routed to the simulated ICM-42688-P. CS framing is inferred from
// beginTransaction()/endTransaction(), which the driver issues once per access.
class SPIClass {
public:
  void begin() {}
  void beginTransaction(SPISettings) { _first = true; }
  void endTransaction() {}
  uint8_t transfer(uint8_t b);
private:
  bool _first = true;
  uint8_t _reg = 0;
  bool _read = false;
};
extern SPIClass SPI;
