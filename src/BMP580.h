#ifndef BMP580_H
#define BMP580_H

#include <Wire.h>
#include <Arduino.h>

// BMP580 I2C address
#define BMP580_DEFAULT_I2C_ADDR 0x46

// Registers (from datasheet BST-BMP580-DS004-02)
#define BMP580_CHIP_ID 0x01
#define BMP580_OSR_CONFIG 0x36
#define BMP580_ODR_CONFIG 0x37
#define BMP580_TEMP_DATA_XLSB 0x1D  // Burst read start

// Oversampling rates (OSR)
enum BMP580_OSR {
  BMP580_OSR_x1 = 0x0,
  BMP580_OSR_x2 = 0x1,
  BMP580_OSR_x4 = 0x2,
  BMP580_OSR_x8 = 0x3,
  BMP580_OSR_x16 = 0x4,
  BMP580_OSR_x32 = 0x5,
  BMP580_OSR_x64 = 0x6,
  BMP580_OSR_x128 = 0x7
};

// Output data rates (ODR)
enum BMP580_ODR {
  BMP580_ODR_240Hz = 0x00,
  BMP580_ODR_218p5Hz = 0x01,
  BMP580_ODR_199p1Hz = 0x02,
  BMP580_ODR_179p2Hz = 0x03,
  BMP580_ODR_160Hz = 0x04,
  BMP580_ODR_149p3Hz = 0x05,
  BMP580_ODR_140Hz = 0x06,
  BMP580_ODR_129p9Hz = 0x07,
  BMP580_ODR_120Hz = 0x08,
  BMP580_ODR_110p2Hz = 0x09,
  BMP580_ODR_100p3Hz = 0x0A,
  BMP580_ODR_89p6Hz = 0x0B,
  BMP580_ODR_80Hz = 0x0C,
  BMP580_ODR_70Hz = 0x0D,
  BMP580_ODR_60Hz = 0x0E,
  BMP580_ODR_50p1Hz = 0x0F,
  BMP580_ODR_45Hz = 0x10,
  BMP580_ODR_40Hz = 0x11,
  BMP580_ODR_35Hz = 0x12,
  BMP580_ODR_30Hz = 0x13,
  BMP580_ODR_25Hz = 0x14,
  BMP580_ODR_20Hz = 0x15,
  BMP580_ODR_15Hz = 0x16,
  BMP580_ODR_10Hz = 0x17,
  BMP580_ODR_5Hz = 0x18,
  BMP580_ODR_4Hz = 0x19,
  BMP580_ODR_3Hz = 0x1A,
  BMP580_ODR_2Hz = 0x1B,
  BMP580_ODR_1Hz = 0x1C,
  BMP580_ODR_0p5Hz = 0x1D,
  BMP580_ODR_0p25Hz = 0x1E,
  BMP580_ODR_0p125Hz = 0x1F
};

// Power modes
enum BMP580_Mode {
  BMP580_MODE_STANDBY = 0x00,
  BMP580_MODE_NORMAL = 0x01,
  BMP580_MODE_FORCED = 0x02,
  BMP580_MODE_CONTINUOUS = 0x03
};

class BMP580 {
 public:
  BMP580();
  bool begin(uint8_t addr = BMP580_DEFAULT_I2C_ADDR);
  void setOversampling(BMP580_OSR osr_p, BMP580_OSR osr_t);
  void setODR(BMP580_ODR odr);
  void setPowerMode(BMP580_Mode mode);
  float readTemperature();
  float readPressure();
  float readAltitude(float seaLevelPressure = 101325.0f);

 private:
  uint8_t _i2cAddr;
  void writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);
  void readBurst(uint8_t reg, uint8_t* buffer, uint8_t length);
};

#endif