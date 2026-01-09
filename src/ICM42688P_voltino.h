#ifndef ICM42688P_VOLTINO_H
#define ICM42688P_VOLTINO_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h> 

// Registers
#define ICM42688_REG_DEVICE_CONFIG  0x11
#define ICM42688_REG_DRIVE_CONFIG   0x13
#define ICM42688_REG_INT_CONFIG     0x14
#define ICM42688_REG_FIFO_CONFIG    0x16
#define ICM42688_REG_TEMP_DATA1     0x1D
#define ICM42688_REG_ACCEL_DATA_X1  0x1F
#define ICM42688_REG_GYRO_DATA_X1   0x25
#define ICM42688_REG_INT_STATUS     0x2D
#define ICM42688_REG_FIFO_COUNTH    0x2E
#define ICM42688_REG_FIFO_DATA      0x30
#define ICM42688_REG_SIGNAL_PATH_RESET 0x4B
#define ICM42688_REG_INTF_CONFIG0   0x4C
#define ICM42688_REG_INTF_CONFIG1   0x4D
#define ICM42688_REG_PWR_MGMT0      0x4E
#define ICM42688_REG_GYRO_CONFIG0   0x4F
#define ICM42688_REG_ACCEL_CONFIG0  0x50
#define ICM42688_REG_TMST_CONFIG    0x54
#define ICM42688_REG_SMD_CONFIG     0x57
#define ICM42688_REG_FIFO_CONFIG1   0x5F
#define ICM42688_REG_FIFO_CONFIG2   0x60
#define ICM42688_REG_FIFO_CONFIG3   0x61
#define ICM42688_REG_INT_CONFIG0    0x63
#define ICM42688_REG_INT_CONFIG1    0x64
#define ICM42688_REG_INT_SOURCE0    0x65
#define ICM42688_REG_WHO_AM_I       0x75
#define ICM42688_REG_BANK_SEL       0x76

// ODR (Output Data Rate)
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

// Full Scale Range
typedef enum {
  AFS_16G = 0,
  AFS_8G  = 1,
  AFS_4G  = 2,
  AFS_2G  = 3
} ICM_ACCEL_FS;

typedef enum {
  GFS_2000DPS = 0,
  GFS_1000DPS = 1,
  GFS_500DPS  = 2,
  GFS_250DPS  = 3,
  GFS_125DPS  = 4
} ICM_GYRO_FS;

// Bus type
enum ICM_BUS {
  BUS_I2C,
  BUS_SPI
};

class ICM42688P {
public:
  ICM42688P();

  // Initialization
  bool begin(ICM_BUS busType, uint8_t pin = 0, uint32_t freq = 10000000);

  // Configuration
  void setODR(ICM_ODR odr);
  void setAccelFS(ICM_ACCEL_FS fs);
  void setGyroFS(ICM_GYRO_FS fs);
  
  // Calibration Setters
  void setAccelOffset(float x, float y, float z);
  void setAccelScale(float x, float y, float z);
  void setGyroOffset(float x, float y, float z);

  // Calibration Getters (To retrieve values after auto-cal)
  float getAccelOffsetX();
  float getAccelOffsetY();
  float getAccelOffsetZ();
  float getAccelScaleX();
  float getAccelScaleY();
  float getAccelScaleZ();
  float getGyroOffsetX();
  float getGyroOffsetY();
  float getGyroOffsetZ();

  void resetHardwareOffsets(); 
  
  // Automatic Calibration Methods
  // Note: These functions use Serial and delay(), so they are blocking!
  void autoCalibrateGyro(uint16_t samples = 1000);
  
  // This now implements the Sphere Fit algorithm (6-point)
  void autoCalibrateAccel(); 

  // Data Reading
  bool readFIFO(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
  
  float readTemperature();

private:
  ICM_BUS _bus;
  uint8_t _csPin;
  uint32_t _spiFreq;
  uint8_t _i2cAddr = 0x68; 

  // Calibration Values
  float accOffset[3] = {0.0f, 0.0f, 0.0f};
  float accScale[3]  = {1.0f, 1.0f, 1.0f};
  float gyrOffset[3] = {0.0f, 0.0f, 0.0f};

  // Helper variables
  float _accelScaleFactor; 
  float _gyroScaleFactor;  

  // Low level
  void writeRegister(uint8_t reg, uint8_t data);
  uint8_t readRegister(uint8_t reg);
  void readRegisters(uint8_t startReg, uint8_t* buffer, size_t len);
  
  void setBank(uint8_t bank);
};

#endif