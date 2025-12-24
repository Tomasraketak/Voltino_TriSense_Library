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
  
  // Helper funkce pro jednotlivé senzory
  bool beginBMP(uint8_t addr = BMP580_DEFAULT_I2C_ADDR);
  bool beginMAG();
  bool beginIMU(ICM_BUS busType = BUS_I2C, uint8_t csPin = 17);

  // --- NOVÉ KALIBRAČNÍ FUNKCE (Wrappery pro ICM42688P) ---
  // Tyto funkce volají přímo vylepšené metody z aktualizované knihovny ICM
  
  // Smaže HW offsety v čipu (použij, pokud se senzor chová divně)
  void resetHardwareOffsets();
  
  // Automatická kalibrace gyra (senzor musí být v klidu)
  void autoCalibrateGyro(uint16_t samples = 1000);
  
  // Interaktivní 6-bodová kalibrace akcelerometru (přes Serial)
  void autoCalibrateAccel(); 

private:
  TriSenseMode _mode;
};

// --- FUSION CLASS (Beze změn, ale čti poznámku níže*) ---
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

  // --- KALIBRAČNÍ DATA ---
  float accelOffset[3] = {0.0f, 0.0f, 0.0f};      
  float gyroOffset[3] = {0.0f, 0.0f, 0.0f};       
  float magHardIron[3] = {0.0f, 0.0f, 0.0f};      
  float magSoftIron[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; 

  // --- PARAMETRY FILTRU ---
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
  
  // Interval kontroly magnetometru
  unsigned long magCheckIntervalUs = 500; // Default 0.5ms

  // Interní metody
  float gaussianGain(float x, float mu, float sigma);
  void gyroIntegration(float gx, float gy, float gz, float dt);
  void getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw);
  void quaternionToEuler(float& roll, float& pitch, float& yaw);

public:
  TriSenseFusion(ICM42688P* imu, AK09918C* mag);
  
  virtual bool update() = 0;
  
  // Funkce pro kalibraci a init
  // POZOR: Pokud použiješ autoCalibrateAccel() přímo na senzoru, 
  // tato funkce calibrateAccelStatic() už není potřeba (nebo by mohla způsobit dvojí korekci).
  void calibrateAccelStatic(int samples = 1000);
  
  void initOrientation(int samples = 200);
  void getOrientationDegrees(float& roll, float& pitch, float& yaw);
  
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
  // Interní proměnné pro korekci
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