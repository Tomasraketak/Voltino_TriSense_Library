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
  
  // UPRAVENO: Defaultní hodnoty. 
  // Pokud použiješ MODE_I2C, parametry spiCsPin a spiFreq se ignorují.
  // Pokud MODE_HYBRID, freq defaultuje na 4MHz.
  bool beginAll(TriSenseMode mode, uint8_t spiCsPin = 17, uint32_t spiFreq = 4000000);
  
  bool beginBMP(uint8_t addr = BMP580_DEFAULT_I2C_ADDR);
  bool beginMAG();
  
  // Také zde defaulty
  bool beginIMU(ICM_BUS busType = BUS_I2C, uint8_t csPin = 17, uint32_t freq = 4000000);

  void resetHardwareOffsets();
  void autoCalibrateGyro(uint16_t samples = 1000);
  void autoCalibrateAccel(); 

private:
  TriSenseMode _mode;
};

// ... TriSenseFusion TŘÍDY zůstávají stejné jako v předchozím kroku ...
class TriSenseFusion {
public: 
  ICM42688P* _imu;
  AK09918C* _mag;
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float lastAx = 0, lastAy = 0, lastAz = 0;
  float lastGx = 0, lastGy = 0, lastGz = 0;
  float lastMx = 0, lastMy = 0, lastMz = 0;
  float accelOffset[3] = {0.0f, 0.0f, 0.0f};      
  float gyroOffset[3] = {0.0f, 0.0f, 0.0f};       
  float magHardIron[3] = {0.0f, 0.0f, 0.0f};      
  float magSoftIron[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; 
  float accRef = 1.0f;          
  float accSigma = 0.05f;       
  float magRef = 50.88f;         
  float magSigma = 3.5f;       
  float magTiltSigmaDeg = 15.0f;
  float magneticDeclination = 0.0f;
  float yawKi = 0.005f;           
  float maxAccelGain = 0.1f;    
  float maxMagGain = 0.1f;      
  unsigned long lastTime = 0;
  unsigned long magCheckIntervalUs = 500; 
  float gaussianGain(float x, float mu, float sigma);
  void gyroIntegration(float gx, float gy, float gz, float dt);
  void getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw);
  void quaternionToEuler(float& roll, float& pitch, float& yaw);

public:
  TriSenseFusion(ICM42688P* imu, AK09918C* mag);
  virtual bool update() = 0;
  void calibrateAccelStatic(int samples = 1000);
  void initOrientation(int samples = 200);
  void getOrientationDegrees(float& roll, float& pitch, float& yaw);
  void getGlobalAcceleration(float& ax_g, float& ay_g, float& az_g);
  void setAccelGaussian(float ref, float sigma);
  void setMagGaussian(float ref, float sigma, float tiltSigma); 
  void setMagGaussian(float ref, float sigma);                  
  void setMagTiltSigma(float sigmaDeg);                         
  void setMagCalibration(float hardIron[3], float softIron[3][3]);
  void setDeclination(float deg);
  void setGyroOffsets(float x, float y, float z);
  void setMagHardIron(float x, float y, float z);
  void setMagSoftIron(float matrix[3][3]);
  void setYawKi(float ki);
  void setMaxGains(float accelGain, float magGain);
  void setMagCheckInterval(float intervalMs);
};

class SimpleTriFusion : public TriSenseFusion {
public:
  SimpleTriFusion(ICM42688P* imu, AK09918C* mag);
  bool update() override;
};

class AdvancedTriFusion : public TriSenseFusion {
private:
  unsigned long lastMagCheckTime = 0; 
  unsigned long lastSuccessfulCorrectionTime = 0; 
  float gyroBiasZ = 0.0f;
  float lastDeltaYawRad = 0.0f;
  void complementaryCorrection(float ax, float ay, float az, float mx, float my, float mz, float correction_dt);
public:
  AdvancedTriFusion(ICM42688P* imu, AK09918C* mag);
  bool update() override;
};

#endif