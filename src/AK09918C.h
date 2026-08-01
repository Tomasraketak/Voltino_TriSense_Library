#ifndef AK09918C_H
#define AK09918C_H

#include <Arduino.h>
#include <Wire.h>

// I2C address for AK09918C (Fixed)
#define AK09918_I2C_ADDR  0x0C

// Registers
#define AK09918_REG_WIA1    0x00
#define AK09918_REG_WIA2    0x01
#define AK09918_REG_ST1     0x10
#define AK09918_REG_HXL     0x11
#define AK09918_REG_HXH     0x12
#define AK09918_REG_HYL     0x13
#define AK09918_REG_HYH     0x14
#define AK09918_REG_HZL     0x15
#define AK09918_REG_HZH     0x16
#define AK09918_REG_TMPS    0x17
#define AK09918_REG_ST2     0x18
#define AK09918_REG_CNTL2   0x31
#define AK09918_REG_CNTL3   0x32

// Identifiers
#define AK09918_WIA1_VAL    0x48
#define AK09918_WIA2_VAL    0x0C

// Voltino Standard ODR (Output Data Rate) for continuous measurement
enum AK09918_ODR {
  ODR_10HZ  = 0x02,
  ODR_20HZ  = 0x04,
  ODR_50HZ  = 0x06,
  ODR_100HZ = 0x08
};

// System operation modes
enum AK09918_Mode {
  MODE_POWER_DOWN = 0x00,
  MODE_SINGLE     = 0x01,
  MODE_SELF_TEST  = 0x10
};

class AK09918C {
public:
  AK09918C();

  // Standalone initialization matching Voltino conventions. 
  // Automatically handles bus init. Supports custom pins for ESP32/8266.
  bool beginI2C(uint32_t freq = 400000, int8_t sdaPin = -1, int8_t sclPin = -1, AK09918_ODR odr = ODR_100HZ);
  
  // Standard begin method (used by TriSense wrapper to avoid overriding bus configuration)
  bool begin(AK09918_ODR odr = ODR_100HZ, TwoWire &wire = Wire);
  
  // Highly optimized polled reading with early-exit on ST1
  bool readData();
  
  // Mode and ODR configuration functions
  void setODR(AK09918_ODR odr); 
  void setMode(AK09918_Mode mode);
  
  // Hardware reset
  void softReset();

  // Tilt-compensated magnetic heading (0-360 deg, 0 = magnetic/true North).
  // Pass in already hard/soft-iron calibrated field components (uT) plus the
  // current roll/pitch (deg, e.g. from TriSenseFusion::getOrientationDegrees())
  // and an optional magnetic declination (deg) to get true-north heading.
  // Stateless helper - does not depend on any AK09918C instance/reading, so it
  // can also be used to sanity-check a fusion's yaw against the raw magnetometer.
  static float computeHeading(float mx, float my, float mz, float rollDeg, float pitchDeg, float declinationDeg = 0.0f);

  // Public variables for direct access (Units: uT)
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  
  // Raw integer data
  int16_t x_raw = 0;
  int16_t y_raw = 0;
  int16_t z_raw = 0;
  
  // Magnetic saturation flag
  bool overflow = false; 

private:
  TwoWire* _wire;
  uint8_t _currentMode;
  
  // Sensitivity constant for float multiplication (0.15 uT/LSB according to datasheet)
  static constexpr float MAG_SCALE = 0.15f;

  void writeRegister(uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t reg);
};

#endif