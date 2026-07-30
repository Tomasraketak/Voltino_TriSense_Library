#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Register definitions
#define ICM42688_REG_DEVICE_CONFIG 0x11
#define ICM42688_REG_DRIVE_CONFIG  0x13
#define ICM42688_REG_INT_CONFIG    0x14
#define ICM42688_REG_FIFO_CONFIG   0x16
#define ICM42688_REG_TEMP_DATA1    0x1D
#define ICM42688_REG_ACCEL_DATA_X1 0x1F
#define ICM42688_REG_GYRO_DATA_X1  0x25
#define ICM42688_REG_INT_STATUS    0x2D // Read-to-clear; bit 1 = FIFO_FULL
#define ICM42688_REG_FIFO_COUNTH   0x2E
#define ICM42688_REG_FIFO_COUNTL   0x2F
#define ICM42688_REG_FIFO_DATA     0x30
#define ICM42688_REG_SIGNAL_PATH_RESET 0x4B // [VOLTINO FIX] Added for FIFO flushing
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

#define ICM_ADDR_PRIMARY 0x68
#define ICM_ADDR_SECONDARY 0x69
#define WHO_AM_I_EXPECTED 0x47

// Bit 1 in both INT_STATUS (FIFO_FULL_INT) and INT_SOURCE0 (FIFO_FULL_INT1_EN).
#define ICM42688_BIT_FIFO_FULL 0x02

// Hardware FIFO is 2 KB: 128 x 16-byte packets, or 102 x 20-byte packets.
#define ICM42688_FIFO_BYTES 2048

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

enum ICM_FIFO_MODE {
  FIFO_NONE = 0,        // Standard register read (no FIFO)
  FIFO_16BIT = 1,       // 16-bit FIFO (Packet format 3)
  FIFO_20BIT_HIRES = 2  // 20-bit High-Res FIFO (Packet format 4)
};

class ICM42688P {
public:
  ICM42688P();
  
  // --- Initialization ---
  bool beginI2C(uint32_t freq = 0, uint8_t i2cAddr = ICM_ADDR_PRIMARY, int8_t sdaPin = -1, int8_t sclPin = -1);
  bool beginSPI(int8_t csPin, uint32_t freq = 0, int8_t sckPin = -1, int8_t misoPin = -1, int8_t mosiPin = -1);
  bool begin(ICM_BUS busType, int8_t csPin = -1, uint32_t freq = 0, uint8_t i2cAddr = ICM_ADDR_PRIMARY, int8_t sckSclPin = -1, int8_t misoSdaPin = -1, int8_t mosiPin = -1);
  
  void setDebug(bool enable);

  // --- Configuration ---
  void setODR(ICM_ODR odr);
  void setAccelFS(ICM_ACCEL_FS fs);
  void setGyroFS(ICM_GYRO_FS fs);
  void setFIFOMode(ICM_FIFO_MODE mode);
  
  int getODRHz(); 
  
  // [VOLTINO FIX] Helper method to dynamically adapt fusion integration based on buffer state
  ICM_FIFO_MODE getFIFOMode(); 

  // [VOLTINO FIX] New rescue function added to header!
  void flushFIFO();

  // --- FIFO overflow monitoring ---
  // The hardware FIFO holds 2 KB. If the sketch does not drain it fast enough
  // the sensor discards new samples, and dropped packets are rotation that can
  // never be integrated - a permanent attitude error. The driver latches the
  // hardware FIFO_FULL flag on every buffer refill and counts the events.
  // NOTHING is printed from the library: poll these from your sketch instead.
  // Note: this reads INT_STATUS once per FIFO refill, and that register is
  // read-to-clear - so if your sketch also drives the INT pins and inspects
  // INT_STATUS itself, expect the driver to have consumed the flags first.
  bool fifoOverflowed();            // True if an overflow occurred since the last call (self-clearing)
  uint32_t getFIFOOverflowCount();  // Total overflow events since boot / last reset
  void resetFIFOOverflowCount();

  // --- Data reading ---
  bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  float readTemperature();

  // Wrappers and specific methods for direct calls
  bool readSensorData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  bool readHardwareFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  bool readHardwareFIFOHires(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  bool readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

  // --- Calibration ---
  void resetHardwareOffsets();
  void autoCalibrateGyro(uint16_t samples = 750);
  void autoCalibrateAccel(); 
  
  void setGyroSoftwareOffset(float ox, float oy, float oz);
  void setAccelSoftwareOffset(float ox, float oy, float oz);
  void setAccelSoftwareScale(float sx, float sy, float sz);

  void getGyroSoftwareOffset(float &ox, float &oy, float &oz);
  void getAccelSoftwareOffset(float &ox, float &oy, float &oz);
  void getAccelSoftwareScale(float &sx, float &sy, float &sz);

  // Short names for backward compatibility
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

private:
  ICM_BUS _bus;
  int8_t _csPin;
  uint8_t _i2cAddr;
  uint32_t _spiFreq;
  ICM_ODR _odr;
  ICM_FIFO_MODE _fifoMode;
  bool _debug;

  float _accelScaleFactor;
  float _gyroScaleFactor;

  float gyrOffset[3] = {0,0,0};
  float accOffset[3] = {0,0,0};
  float accScale[3]  = {1,1,1};

  // --- FIFO burst buffer ---
  // readFIFO() still hands out one packet per call, but the bytes are fetched
  // from the sensor a whole burst at a time: one FIFO_COUNT read plus one bulk
  // FIFO_DATA read serve up to FIFO_BURST_PACKETS packets, instead of two bus
  // transactions per single packet.
  static const uint8_t FIFO_MAX_PACKET_SIZE = 20;
#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
  static const uint8_t FIFO_BURST_PACKETS = 8;    // 160 B buffer
#else
  static const uint8_t FIFO_BURST_PACKETS = 32;   // 640 B buffer
#endif
  uint8_t _fifoBuf[FIFO_BURST_PACKETS * FIFO_MAX_PACKET_SIZE];
  uint8_t _fifoBufCount = 0;   // Packets currently held in _fifoBuf
  uint8_t _fifoBufIndex = 0;   // Next packet to hand out
  uint8_t _fifoPacketSize = 16;

  bool _fifoOverflowFlag = false;
  uint32_t _fifoOverflowCount = 0;

  uint8_t fillFIFOBuffer();                        // Refill from hardware; returns packets loaded
  void readFIFOBytes(uint8_t *buf, size_t len);    // Bulk FIFO_DATA read (chunked on I2C)
  void invalidateFIFOBuffer();

  void writeRegister(uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t reg);
  void readRegisters(uint8_t reg, uint8_t *buf, size_t len);
  void setBank(uint8_t bank);

  void enforceBandwidthLimit();
  int _getHzFromODR(ICM_ODR odr);
};