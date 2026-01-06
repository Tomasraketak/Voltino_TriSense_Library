#ifndef ICM42688P_VOLTINO_H
#define ICM42688P_VOLTINO_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Registry
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
typedef enum {
  ODR_32KHZ = 1, // LN mode only
  ODR_16KHZ = 2,
  ODR_8KHZ  = 3,
  ODR_4KHZ  = 4,
  ODR_2KHZ  = 5,
  ODR_1KHZ  = 6, // Default I2C
  ODR_200HZ = 7,
  ODR_50HZ  = 8
} ICM_ODR;

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

// Sběrnice
enum ICM_BUS {
  BUS_I2C,
  BUS_SPI
};

class ICM42688P {
public:
  ICM42688P();

  // Inicializace
  // Pro I2C: begin(BUS_I2C)
  // Pro SPI: begin(BUS_SPI, csPin, frequency)
  bool begin(ICM_BUS busType, uint8_t pin = 0, uint32_t freq = 10000000);

  // Nastavení senzoru
  void setODR(ICM_ODR odr);
  void setAccelFS(ICM_ACCEL_FS fs);
  void setGyroFS(ICM_GYRO_FS fs);
  
  // Kalibrace
  void setAccelOffset(float x, float y, float z);
  void setAccelScale(float x, float y, float z);
  void resetHardwareOffsets(); // Vynuluje registry offsetů v čipu
  
  void autoCalibrateGyro(uint16_t samples = 1000);
  void autoCalibrateAccel(); 

  // Čtení dat
  // Vrací true, pokud jsou data platná
  bool readFIFO(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
  
  // Raw teploty
  float readTemperature();

private:
  ICM_BUS _bus;
  uint8_t _csPin;
  uint32_t _spiFreq;
  uint8_t _i2cAddr = 0x68; // Default AD0=0

  // Kalibrační hodnoty (softwarové)
  float accOffset[3] = {0.0f, 0.0f, 0.0f};
  float accScale[3]  = {1.0f, 1.0f, 1.0f};
  float gyrOffset[3] = {0.0f, 0.0f, 0.0f};

  // Optimalizace: Předpočítané škálovací faktory (multiplikátory)
  // Nahrazují dělení v každém cyklu
  float _accelScaleFactor; // např. 1.0 / 16384.0
  float _gyroScaleFactor;  // např. 1.0 / 131.0

  // Low level
  void writeRegister(uint8_t reg, uint8_t data);
  uint8_t readRegister(uint8_t reg);
  // Nová optimalizovaná metoda pro burst read
  void readRegisters(uint8_t startReg, uint8_t* buffer, size_t len);
  
  void setBank(uint8_t bank);
};

#endif