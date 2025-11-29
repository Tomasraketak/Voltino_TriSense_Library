#ifndef TRISENSE_H
#define TRISENSE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "BMP580.h"
#include "AK09918C.h"
#include "ICM42688P_voltino.h"

enum TriSenseMode {
  MODE_I2C,     // All sensors on I2C
  MODE_HYBRID   // AK and BMP on I2C, ICM on SPI
};

class TriSense {
public:
  BMP580 bmp;       // Pressure and temperature sensor
  AK09918C mag;     // Magnetometer
  ICM42688P imu;    // IMU (accelerometer and gyroscope)

  TriSense();

  // Initialize all sensors at once
  bool beginAll(TriSenseMode mode, uint8_t spiCsPin = 17);

  // Initialize individual sensors
  bool beginBMP(uint8_t addr = BMP580_DEFAULT_I2C_ADDR);
  bool beginMAG();
  bool beginIMU(ICM_BUS busType = BUS_I2C, uint8_t csPin = 17);

private:
  TriSenseMode _mode;
};

// Base class for sensor fusion
class TriSenseFusion {
protected:
  ICM42688P* _imu;
  AK09918C* _mag;

  // Quaternion (w, x, y, z)
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  // Gyro offsets
  float gyroOffset[3] = {0.0f, 0.0f, 0.0f};

  // Accel offsets
  float accelOffset[3] = {0.0f, 0.0f, 0.0f};

  // Magnetometer hard-iron offsets
  float magHardIron[3] = {0.0f, 0.0f, 0.0f};

  // Magnetometer soft-iron matrix
  float magSoftIron[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
  };

  // Magnetic declination in degrees
  float declination = 0.0f;

  unsigned long lastTime = 0;

  // Helper to convert quaternion to Euler angles (in radians)
  void quaternionToEuler(float& roll, float& pitch, float& yaw);

public:
  TriSenseFusion(ICM42688P* imu, AK09918C* mag);

  // Set custom gyro offsets
  void setGyroOffsets(float ox, float oy, float oz);

  // Set custom accel offsets
  void setAccelOffsets(float ox, float oy, float oz);

  // Set custom mag hard-iron offsets
  void setMagHardIron(float ox, float oy, float oz);

  // Set custom mag soft-iron matrix
  void setMagSoftIron(float matrix[3][3]);

  // Set magnetic declination in degrees
  void setDeclination(float deg);

  // Get current orientation in degrees (roll, pitch, yaw)
  void getOrientationDegrees(float& roll, float& pitch, float& yaw);

  // Initialize initial orientation using accel and mag
  void initOrientation(int samples = 100);

  // Virtual update method to be implemented by subclasses
  virtual bool update() = 0;
};

// Simple fusion: Gyro integration with initial orientation
class SimpleTriFusion : public TriSenseFusion {
private:
  // No additional parameters for simple fusion

public:
  SimpleTriFusion(ICM42688P* imu, AK09918C* mag);

  // Update fusion state
  bool update() override;
};

// Advanced fusion: Complementary filter with dynamic gains
class AdvancedTriFusion : public TriSenseFusion {
private:
  // Default Gaussian parameters
  float accRef = 1.0f;          // Accel reference (1G)
  float accSigma = 0.0175f;     // Accel sigma
  float magRef = 50.88f;        // Mag reference norm (uT)
  float magSigma = 3.5f;        // Mag sigma
  float magTiltSigmaDeg = 15.0f;// Tilt sigma for mag gain (degrees)
  float yawKi = 0.005f;         // Yaw bias KI (1/s)
  float maxAccelGain = 0.5f;    // Max accel gain
  float maxMagGain = 0.5f;      // Max mag gain

  // Internal state
  unsigned long lastCorrectionTime = 0;
  float gyroBiasZ = 0.0f;
  float lastDeltaYawRad = 0.0f;

  const unsigned long correctionIntervalUs = 10000; // 100Hz corrections

  // Gaussian gain function
  float gaussianGain(float x, float mu, float sigma);

  // Get correction angles from accel and mag (in degrees)
  void getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw);

  // Gyro integration step
  void gyroIntegration(float gx, float gy, float gz, float dt);

  // Correction step
  void complementaryCorrection(float ax, float ay, float az, float mx, float my, float mz);

public:
  AdvancedTriFusion(ICM42688P* imu, AK09918C* mag);

  // Set custom Gaussian parameters
  void setAccelGaussian(float ref, float sigma);
  void setMagGaussian(float ref, float sigma);
  void setMagTiltSigma(float sigmaDeg);
  void setYawKi(float ki);
  void setMaxGains(float maxAccel, float maxMag);

  // Update fusion state
  bool update() override;
};

#endif