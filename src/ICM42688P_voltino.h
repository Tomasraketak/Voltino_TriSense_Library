#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define ICM_ADDR 0x68
#define WHO_AM_I_EXPECTED 0x47

enum ICM_BUS {
  BUS_I2C,
  BUS_SPI
};

// --- ODR mapování podle datasheetu ICM42688-P ---
enum ICM_ODR {
  ODR_32KHZ  = 1,
  ODR_16KHZ  = 2,
  ODR_8KHZ   = 3,
  ODR_4KHZ   = 4,
  ODR_2KHZ   = 5,
  ODR_1KHZ   = 6,
  ODR_200HZ  = 7,
  ODR_100HZ  = 8,
  ODR_50HZ   = 9,
  ODR_25HZ   = 10,
  ODR_12_5HZ = 11,
  ODR_500HZ  = 15
};

class ICM42688P {
public:
  ICM42688P();
  bool begin(ICM_BUS busType, uint8_t csPin = 17);
  void setODR(ICM_ODR odr);
  bool readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

  // --- nové funkce ---
  void setGyroOffset(float ox, float oy, float oz);
  void setAccelOffset(float ox, float oy, float oz);
  void getGyroOffset(float &ox, float &oy, float &oz);
  void getAccelOffset(float &ox, float &oy, float &oz);
  void autoCalibrateGyro(uint16_t samples = 1000);

private:
  ICM_BUS _bus;
  uint8_t _csPin;
  SPISettings _spiSettings;
  ICM_ODR _odr;

  float _gOX, _gOY, _gOZ;  // gyro offsety
  float _aOX, _aOY, _aOZ;  // accel offsety

  void writeReg(uint8_t reg, uint8_t val);
  uint8_t readReg(uint8_t reg);
  void readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};
