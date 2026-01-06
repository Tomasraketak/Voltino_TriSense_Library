#ifndef TRISENSE_H
#define TRISENSE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <cmath> 
#include "BMP580.h"
#include "AK09918C.h"
#include "ICM42688P_voltino.h"

// Konstanty pro rychlé převody
constexpr float DEG_TO_RAD_F = 3.1415926535f / 180.0f;
constexpr float RAD_TO_DEG_F = 180.0f / 3.1415926535f;
constexpr float PI_F = 3.1415926535f;

enum TriSenseMode {
  MODE_I2C,
  MODE_HYBRID   // AK and BMP on I2C, ICM on SPI
};

class TriSense {
public:
  BMP580 bmp;
  AK09918C mag;
  ICM42688P imu;

  TriSense();
  
  bool beginAll(TriSenseMode mode, uint8_t spiCsPin = 17, uint32_t spiFreq = 10000000);
  
  bool beginBMP(uint8_t addr = BMP580_DEFAULT_I2C_ADDR);
  bool beginMAG();
  
  bool beginIMU(ICM_BUS busType = BUS_I2C, uint8_t csPin = 17, uint32_t spiFreq = 10000000);

  void resetHardwareOffsets();
  void autoCalibrateGyro(uint16_t samples = 1000);
  void autoCalibrateAccel(); 

private:
  TriSenseMode _mode;
};

// --- FUSION CLASS ---
class TriSenseFusion {
public: 
  ICM42688P* _imu;
  AK09918C* _mag;

  // Orientation (Quaternion)
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  // --- PUBLIC DATA (RAW Internal) ---
  float lastAx = 0.0f, lastAy = 0.0f, lastAz = 0.0f; // G (Body frame)
  float lastGx = 0.0f, lastGy = 0.0f, lastGz = 0.0f; // dps
  float lastMx = 0.0f, lastMy = 0.0f, lastMz = 0.0f; // uT

  // --- PROCESSED OUTPUT DATA (User accessible) ---
  // Tyto hodnoty už respektují nastavení useG, GlobalAccel, Downsampling
  float ax = 0.0f, ay = 0.0f, az = 0.0f;
  float gx = 0.0f, gy = 0.0f, gz = 0.0f;
  float mx = 0.0f, my = 0.0f, mz = 0.0f;

  // --- CALIBRATION DATA ---
  float accelOffset[3] = {0.0f, 0.0f, 0.0f};      
  float gyroOffset[3] = {0.0f, 0.0f, 0.0f};       
  float magHardIron[3] = {0.0f, 0.0f, 0.0f};      
  float magSoftIron[3][3] = {{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}}; 
  
  // NEW: Global Accel Bias (Drift removal for velocity)
  float globalAccelBias[3] = {0.0f, 0.0f, 0.0f};

  // --- FILTER PARAMETERS ---
  float accRef = 1.0f;            
  float accSigma = 0.02f;       
  float magRef = 50.1f;          
  float magSigma = 3.5f;        
  float magTiltSigmaDeg = 15.0f;
  float magneticDeclination = 0.0f;

  float yawKi = 0.0075f;           
  float maxAccelGain = 0.4f;    
  float maxMagGain = 0.4f;       

  unsigned long lastTime = 0;
  
  // Magnetometer check interval
  unsigned long magCheckIntervalUs = 500; 

protected:
  // OPTIMALIZACE: Předpočítané koeficienty
  float _accGaussCoeff = 0.0f;
  float _magGaussCoeff = 0.0f;
  float _tiltGaussCoeff = 0.0f;

  // Configuration Flags/Variables for Output
  bool _useGUnits;
  float _gravityConstant;
  bool _globalAccelEnabled;
  int _downsampleFactor;
  int _sampleCounter;

  // Accumulators for Downsampling
  float _sumAx, _sumAy, _sumAz;
  float _sumGx, _sumGy, _sumGz;
  float _sumMx, _sumMy, _sumMz;

  // Internal methods
  float gaussianGainOptimized(float diffSq, float coeff); 
  float gaussianGain(float x, float mu, float sigma); 
  
  void gyroIntegration(float gx, float gy, float gz, float dt);
  void getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw);
  void quaternionToEuler(float& roll, float& pitch, float& yaw);
  
  // NEW: Processes raw data into final output (Downsampling, Units, Global Transform)
  void processOutput();

public:
  TriSenseFusion(ICM42688P* imu, AK09918C* mag);
  
  virtual bool update() = 0;
  
  void calibrateAccelStatic(int samples = 1000);
  
  // NEW: Global Static Calibration for Velocity Drift
  void calibrateStaticGlobalAccel(int samples = 1000);
  
  void initOrientation(int samples = 250);
  void getOrientationDegrees(float& roll, float& pitch, float& yaw);
  
  // Legacy helper (returns pure global G without user scaling)
  void getGlobalAccelerationInternal(float& ax_g, float& ay_g, float& az_g);

  // --- New Configuration Methods ---
  void useG(bool use);               // true = G, false = m/s^2 (Default: true)
  void setG(float gravityValue);     // Set gravity constant (Default: 9.8065)
  void enableGlobalAccel(bool enable); // Transform accel to Earth frame
  void setDownsampling(int factor);  // 1 = raw, >1 = average over N samples
  void setGlobalAccelBias(float x, float y, float z);

  // --- Getters for Processed Data ---
  float getAccelX() { return ax; }
  float getAccelY() { return ay; }
  float getAccelZ() { return az; }
  float getGyroX() { return gx; }
  float getGyroY() { return gy; }
  float getGyroZ() { return gz; }
  float getMagX() { return mx; }
  float getMagY() { return my; }
  float getMagZ() { return mz; }

  // Setters
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

// Simple implementation
class SimpleTriFusion : public TriSenseFusion {
public:
  SimpleTriFusion(ICM42688P* imu, AK09918C* mag);
  bool update() override;
};

// Advanced implementation
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