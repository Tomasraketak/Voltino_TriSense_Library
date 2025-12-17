#ifndef TRISENSE_H
#define TRISENSE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "BMP580.h"
#include "AK09918C.h"
#include "ICM42688P_voltino.h"

enum TriSenseMode {
  MODE_I2C,
  MODE_HYBRID   // AK a BMP na I2C, ICM na SPI
};

class TriSense {
public:
  BMP580 bmp;
  AK09918C mag;
  ICM42688P imu;

  TriSense();
  bool beginAll(TriSenseMode mode, uint8_t spiCsPin = 17);
  
  // Zachované helper funkce
  bool beginBMP(uint8_t addr = BMP580_DEFAULT_I2C_ADDR);
  bool beginMAG();
  bool beginIMU(ICM_BUS busType = BUS_I2C, uint8_t csPin = 17);

private:
  TriSenseMode _mode;
};

// --- FUSION CLASS ---
class TriSenseFusion {
public: 
  ICM42688P* _imu;
  AK09918C* _mag;

  // Orientace (Quaternion)
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  // --- PUBLIC DATA (RAW) ---
  float lastAx = 0, lastAy = 0, lastAz = 0; // G
  float lastGx = 0, lastGy = 0, lastGz = 0; // dps
  float lastMx = 0, lastMy = 0, lastMz = 0; // uT

  // --- KALIBRAČNÍ DATA (Public pro zápis z main) ---
  float accelOffset[3] = {0.0f, 0.0f, 0.0f};      // Offset akcelerometru
  float magHardIron[3] = {0.0f, 0.0f, 0.0f};      // Hard Iron (bias)
  float magSoftIron[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; // Soft Iron (matice)

  // --- PARAMETRY FILTRU (Public pro ladění) ---
  float accRef = 1.0f;          
  float accSigma = 0.05f;       
  float magRef = 50.0f;         
  float magSigma = 5.0f;       
  float magTiltSigmaDeg = 20.0f;
  float magneticDeclination = 0.0f;

  float maxAccelGain = 0.1f;    
  float maxMagGain = 0.1f;      

  unsigned long lastTime = 0;

  // Interní metody
  float gaussianGain(float x, float mu, float sigma);
  void gyroIntegration(float gx, float gy, float gz, float dt);
  void getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw);

public:
  TriSenseFusion(ICM42688P* imu, AK09918C* mag);
  
  virtual bool update() = 0;
  
  // Funkce pro kalibraci akcelerometru na rampě (předpokládá kolmou raketu)
  void calibrateAccelStatic(int samples = 1000);
  
  void initOrientation(int samples = 200);
  void getOrientationDegrees(float& roll, float& pitch, float& yaw);
  
  // Setters (pro pohodlí, ale proměnné jsou public)
  void setAccelGaussian(float ref, float sigma);
  void setMagGaussian(float ref, float sigma, float tiltSigma);
  void setMagCalibration(float hardIron[3], float softIron[3][3]);
  void setDeclination(float deg);
};

// Advanced implementation
class AdvancedTriFusion : public TriSenseFusion {
public:
  AdvancedTriFusion(ICM42688P* imu, AK09918C* mag);
  bool update() override;
};

#endif