#ifndef TRISENSE_H
#define TRISENSE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "BMP580.h"
#include "AK09918C.h"
#include "ICM42688P_voltino.h" // Ujisti se, že název sedí s tvým souborem

enum TriSenseMode {
  MODE_I2C,     // Vše na I2C
  MODE_HYBRID   // AK a BMP na I2C, ICM na SPI (Doporučeno pro raketu)
};

class TriSense {
public:
  BMP580 bmp;       // Tlak a teplota
  AK09918C mag;     // Magnetometr
  ICM42688P imu;    // Akcelerometr a gyroskop

  TriSense();

  // Inicializuje všechny senzory najednou
  // spiCsPin se použije pouze pokud je mode == MODE_HYBRID
  bool beginAll(TriSenseMode mode, uint8_t spiCsPin = 17);

  // Inicializace jednotlivých senzorů
  bool beginBMP(uint8_t addr = BMP580_DEFAULT_I2C_ADDR);
  bool beginMAG();
  bool beginIMU(ICM_BUS busType = BUS_I2C, uint8_t csPin = 17);

private:
  TriSenseMode _mode;
};

// Base class for sensor fusion
class TriSenseFusion {
public: // *** ZMĚNA: ZDE BYLO PROTECTED, NYNÍ PUBLIC ***
        // Díky tomu můžeš číst q[4] a měnit sigma parametry přímo ze sketche.
  
  ICM42688P* _imu;
  AK09918C* _mag;

  // Quaternion (w, x, y, z)
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  // Calibration data
  float gyroOffset[3] = {0, 0, 0};
  float accelOffset[3] = {0, 0, 0};
  float magHardIron[3] = {0, 0, 0};
  float magSoftIron[2][3] = {{1, 0, 0}, {0, 1, 0}}; // Zjednodušeno 2D nebo 3x3 dle implementace

  // Default Gaussian parameters (pro adaptivní filtr)
  float accRef = 1.0f;          // Reference 1G
  float accSigma = 0.0175f;     // Tolerance akcelerometru (pro start rakety nutno snížit/upravit)
  float magRef = 50.88f;        // Reference síly mag. pole (uT)
  float magSigma = 3.5f;        // Tolerance magnetometru
  float magTiltSigmaDeg = 15.0f;// Tilt sigma
  float yawKi = 0.005f;         // Integral gain pro yaw
  float maxAccelGain = 0.5f;    // Omezení maximálního vlivu akcelerometru
  float maxMagGain = 0.5f;      // Omezení maximálního vlivu magnetometru

  // Internal state
  unsigned long lastTime = 0;
  float gyroBiasZ = 0.0f;

  // Accumulation for acceleration
  float axSum = 0.0f;
  float aySum = 0.0f;
  float azSum = 0.0f;
  int accSamples = 0;

  // Interval pro kontrolu magnetometru (šetření CPU)
  unsigned long magReadIntervalUs = 2000; // Čteme mag méně často (např. 500Hz)
  unsigned long lastMagReadAttempt = 0;

  // Gaussian gain function
  float gaussianGain(float x, float mu, float sigma);

  // Get correction angles from accel and mag (in degrees)
  void getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw);

  // Gyro integration step
  void gyroIntegration(float gx, float gy, float gz, float dt);

  // Correction step
  void complementaryCorrection(float ax, float ay, float az, float mx, float my, float mz, float dt);

public:
  TriSenseFusion(ICM42688P* imu, AK09918C* mag);
  
  // Hlavní update funkce
  virtual bool update() = 0;

  // Helpery
  void initOrientation(int samples = 100);
  void getOrientationDegrees(float& roll, float& pitch, float& yaw);
  
  // Setters
  void setAccelGaussian(float ref, float sigma);
  void setMagGaussian(float ref, float sigma, float tiltSigma);
  void setGains(float maxAcc, float maxMag, float ki);
  void setDeclination(float deg);

  float magneticDeclination = 0.0f;
};

// Advanced implementation using Adaptive Complementary Filter
class AdvancedTriFusion : public TriSenseFusion {
public:
  AdvancedTriFusion(ICM42688P* imu, AK09918C* mag);
  bool update() override;
  
  // Veřejné metody pro nastavení limitů zisku (užitečné pro ladění za letu)
  void setMaxGains(float accGain, float magGain) {
      maxAccelGain = accGain;
      maxMagGain = magGain;
  }
};

#endif