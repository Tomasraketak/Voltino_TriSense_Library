#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Definice registrů
#define ICM42688_REG_DEVICE_CONFIG 0x11
#define ICM42688_REG_DRIVE_CONFIG  0x13
#define ICM42688_REG_INT_CONFIG    0x14
#define ICM42688_REG_FIFO_CONFIG   0x16
#define ICM42688_REG_TEMP_DATA1    0x1D
#define ICM42688_REG_ACCEL_DATA_X1 0x1F
#define ICM42688_REG_GYRO_DATA_X1  0x25
#define ICM42688_REG_TMST_CONFIG   0x54
#define ICM42688_REG_APEX_CONFIG0  0x56
#define ICM42688_REG_SMD_CONFIG    0x57
#define ICM42688_REG_FIFO_CONFIG1  0x5F
#define ICM42688_REG_FIFO_CONFIG2  0x60
#define ICM42688_REG_FIFO_CONFIG3  0x61
#define ICM42688_REG_INT_SOURCE0   0x65
#define ICM42688_REG_WHO_AM_I      0x75
#define ICM42688_REG_BANK_SEL      0x76
#define ICM42688_REG_PWR_MGMT0     0x4E
#define ICM42688_REG_GYRO_CONFIG0  0x4F
#define ICM42688_REG_ACCEL_CONFIG0 0x50

#define ICM_ADDR 0x68
#define WHO_AM_I_EXPECTED 0x47

enum ICM_BUS {
  BUS_I2C,
  BUS_SPI
};

enum ICM_ACCEL_FS {
  AFS_16G = 0,
  AFS_8G  = 1,
  AFS_4G  = 2,
  AFS_2G  = 3
};

enum ICM_GYRO_FS {
  GFS_2000DPS = 0,
  GFS_1000DPS = 1,
  GFS_500DPS  = 2,
  GFS_250DPS  = 3,
  GFS_125DPS  = 4
};

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
  
  // Defaultní hodnoty: pin 0, freq 4MHz (pro SPI), 400kHz (pro I2C)
  bool begin(ICM_BUS busType, uint8_t pin = 0, uint32_t freq = 4000000);
  
  void setODR(ICM_ODR odr);
  void setAccelFS(ICM_ACCEL_FS fs);
  void setGyroFS(ICM_GYRO_FS fs);
  
  bool readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  float readTemperature();

  // --- Kalibrace ---
  void resetHardwareOffsets();
  
  void setGyroSoftwareOffset(float ox, float oy, float oz);
  void setAccelSoftwareOffset(float ox, float oy, float oz);
  void setAccelSoftwareScale(float sx, float sy, float sz);

  void getGyroSoftwareOffset(float &ox, float &oy, float &oz);
  void getAccelSoftwareOffset(float &ox, float &oy, float &oz);
  void getAccelSoftwareScale(float &sx, float &sy, float &sz);

  // Wrappery pro kompatibilitu
  void setGyroOffset(float ox, float oy, float oz);
  void setAccelOffset(float ox, float oy, float oz);
  void setAccelScale(float sx, float sy, float sz);
  
  void getGyroOffset(float &ox, float &oy, float &oz);
  void getAccelOffset(float &ox, float &oy, float &oz);
  void getAccelScale(float &sx, float &sy, float &sz);

  float getAccelOffsetX();
  float getAccelOffsetY();
  float getAccelOffsetZ();
  float getAccelScaleX();
  float getAccelScaleY();
  float getAccelScaleZ();
  float getGyroOffsetX();
  float getGyroOffsetY();
  float getGyroOffsetZ();

  void autoCalibrateGyro(uint16_t samples = 750);
  void autoCalibrateAccel(); 

private:
  ICM_BUS _bus;
  uint8_t _csPin;
  uint8_t _i2cAddr = ICM_ADDR;
  uint32_t _spiFreq;
  ICM_ODR _odr;

  float _accelScaleFactor;
  float _gyroScaleFactor;

  float gyrOffset[3] = {0,0,0};
  float accOffset[3] = {0,0,0};
  float accScale[3]  = {1,1,1};

  void writeRegister(uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t reg);
  
  // ZDE BLA CHYBA: Změněno z uint8_t len na size_t len
  void readRegisters(uint8_t reg, uint8_t *buf, size_t len);
  
  void setBank(uint8_t bank);
};