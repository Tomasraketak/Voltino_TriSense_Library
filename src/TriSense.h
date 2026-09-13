#ifndef TRISENSE_H
#define TRISENSE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <string.h>   // memcpy - used by invSqrt() to reinterpret bits without UB
#include "BMP580.h"

// ---------------------------------------------------------
// RESOLVING ENUMERATOR CONFLICTS (NAMESPACE POLLUTION)
// ---------------------------------------------------------
#define ODR_10HZ  AK_ODR_10HZ
#define ODR_20HZ  AK_ODR_20HZ
#define ODR_50HZ  AK_ODR_50HZ
#define ODR_100HZ AK_ODR_100HZ

#include "AK09918C.h"

// Undefine macros so ICM42688P can define its own ODR enums
#undef ODR_10HZ
#undef ODR_20HZ
#undef ODR_50HZ
#undef ODR_100HZ
// ---------------------------------------------------------

#include "ICM42688P_voltino.h"

// ---------------------------------------------------------
// HARDWARE FPU DETECTION
// ---------------------------------------------------------
// Decides how invSqrt() is implemented. Where the MCU can do sqrt in hardware
// a single VSQRT+VDIV is both faster and exact, so the classic bit-trick is
// strictly worse there; it is kept only for FPU-less cores.
//
//   __ARM_FP        - set by GCC for any ARM core with an FPU. Covers RP2350
//                     (Cortex-M33), SAMD51, STM32F4/F7/H7, nRF52840, Teensy.
//                     NOT set for RP2040 (Cortex-M0+), which is correct.
//   __riscv_flen    - set for RISC-V cores that have the F/D extension.
//   ESP32 (Xtensa)  - classic ESP32 and ESP32-S3 have an FPU; the S2 does not,
//                     and the C-series are RISC-V and handled by the line above.
#if defined(__ARM_FP) \
 || (defined(__riscv) && defined(__riscv_flen)) \
 || (defined(ESP32) && !defined(__riscv) && !defined(CONFIG_IDF_TARGET_ESP32S2))
  #define TRISENSE_HAS_HW_FPU 1
#endif

// ---------------------------------------------------------
// DYNAMIC ARCHITECTURE DETECTION & OPTIMIZATION
// ---------------------------------------------------------
#if defined(FORCE_FUSION_DOUBLE)
  #define FUSION_MATH_TYPE double
  #define DEFAULT_IMU_ODR ODR_1KHZ
  #define DEFAULT_IMU_SPI_ODR ODR_8KHZ
  #define DEFAULT_CALIBRATION_SAMPLES 1000
#elif defined(FORCE_FUSION_FLOAT)
  #define FUSION_MATH_TYPE float
  #define DEFAULT_IMU_ODR ODR_1KHZ
  #define DEFAULT_IMU_SPI_ODR ODR_4KHZ
  #define DEFAULT_CALIBRATION_SAMPLES 1000
#elif defined(__AVR__)
  #define FUSION_MATH_TYPE float
  #define DEFAULT_IMU_ODR ODR_100HZ
  #define DEFAULT_IMU_SPI_ODR ODR_500HZ
  #define DEFAULT_CALIBRATION_SAMPLES 200
#elif defined(ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  #define FUSION_MATH_TYPE float // Default to float for HW FPU optimization!
  #define DEFAULT_IMU_ODR ODR_1KHZ
  #define DEFAULT_IMU_SPI_ODR ODR_8KHZ
  #define DEFAULT_CALIBRATION_SAMPLES 1000
#else
  #define FUSION_MATH_TYPE float
  #define DEFAULT_IMU_ODR ODR_200HZ
  #define DEFAULT_IMU_SPI_ODR ODR_1KHZ
  #define DEFAULT_CALIBRATION_SAMPLES 500
#endif

// Upper bound on any blocking calibration routine. These wait for the sensor to
// deliver samples, and a sensor that has stopped delivering must not hang the
// sketch: without a bound the call never returns and nothing is ever printed
// again, which is indistinguishable from a crash.
#define CALIBRATION_TIMEOUT_MS 10000UL

// --- ACCELERATION UNITS ---
enum AccelUnit {
  ACCEL_UNIT_G,      
  ACCEL_UNIT_MS2     
};

// --- SENSOR MOUNT ORIENTATION ---
enum TriSenseOrientation {
  ORIENTATION_Z_UP,       // Standard (Flat)
  ORIENTATION_Z_DOWN,     // Upside down
  ORIENTATION_X_UP,       // Vertical (X points up/forward)
  ORIENTATION_Y_UP        // Vertical (Y points up/forward)
};

// A snapshot is ALWAYS fully populated with the most recent valid reading of
// every quantity - the three sensors run at different rates (IMU up to 8 kHz,
// magnetometer 100 Hz, barometer 240 Hz), so on any given call some of them
// simply have nothing new to give. Rather than reporting failure for that
// perfectly normal case, TriSense keeps the last good value of each block and
// refreshes only what the hardware actually delivered.
//
// The *Fresh flags say what was refreshed by THIS call; the *AgeUs fields say
// how old each block is, so a sketch that cares (e.g. dead reckoning) can tell
// a 200 us old accelerometer sample from a 3 s old one left over after the
// magnetometer was unplugged.
struct TriSenseDataSnapshot {
  float accelX; float accelY; float accelZ;
  float gyroX; float gyroY; float gyroZ;
  float magX; float magY; float magZ;
  float pressure; float temperature;

  bool imuFresh;       // IMU block was updated by this getSnapshot() call
  bool magFresh;       // Magnetometer block was updated by this call
  bool baroFresh;      // Barometer block was updated by this call

  uint32_t imuAgeUs;   // Microseconds since the IMU block was last refreshed
  uint32_t magAgeUs;   // Microseconds since the magnetometer was last refreshed
  uint32_t baroAgeUs;  // Microseconds since the barometer was last refreshed
};

enum TriSenseMode {
  MODE_I2C,
  MODE_HYBRID   
};

class TriSense {
public:
  BMP580 bmp;
  AK09918C mag;
  ICM42688P imu;

  TriSense();
  
  // All three sensors share one I2C bus (the ICM-42688-P leaves it only when
  // put on SPI in MODE_HYBRID), so the bus is selected once here and handed to
  // every driver. Pass Wire1 etc. on boards whose TriSense module is not on the
  // primary bus. On cores that support pin remapping, call wire.setSDA()/setSCL()
  // (or wire.setPins() on ESP32) BEFORE this - beginAll() calls wire.begin()
  // without pin arguments and therefore keeps whatever mapping you configured.
  bool beginAll(TriSenseMode mode, uint8_t spiCsPin = 17, uint32_t spiFreq = 4000000, TwoWire &wire = Wire);
  
  bool beginBMP(uint8_t addr = BMP580_PRIMARY_I2C_ADDR, TwoWire &wire = Wire);
  bool beginMAG(TwoWire &wire = Wire);
  bool beginIMU(ICM_BUS busType = BUS_I2C, uint8_t csPin = 17, uint32_t freq = 4000000, TwoWire &wire = Wire);

  // The I2C bus every sensor on the module is attached to.
  TwoWire& getWire() { return *_wire; }

  void resetHardwareOffsets();
  void autoCalibrateGyro(uint16_t samples = DEFAULT_CALIBRATION_SAMPLES);
  void autoCalibrateAccel(); 

  // Fills `data` with the newest reading of every quantity, refreshing whatever
  // the sensors have ready and reusing the last known good value for the rest.
  // Returns true once every block has produced at least one valid reading, i.e.
  // once the snapshot is fully meaningful. It does NOT return false merely
  // because a sensor had no new sample this time round - check imuFresh /
  // magFresh / baroFresh (or the *AgeUs fields) for that.
  bool getSnapshot(TriSenseDataSnapshot &data);
  float readPressure();
  float readTemperature();
  // Sea-level reference pressure is in PASCALS (matches BMP580::readAltitude).
  float readAltitude(float seaLevelPressure = 101325.0f);

private:
  TriSenseMode _mode;
  TwoWire* _wire = &Wire;

  // Last known good reading of each block, plus when it was taken.
  TriSenseDataSnapshot _last;
  bool _haveIMU = false;
  bool _haveMag = false;
  bool _haveBaro = false;
  uint32_t _lastImuUs = 0;
  uint32_t _lastMagUs = 0;
  uint32_t _lastBaroUs = 0;
};

class TriSenseFusion {
protected:
  TriSenseOrientation _mountOrientation = ORIENTATION_Z_UP;
  void remapAxes(float& x, float& y, float& z);
  void unremapAxes(float& x, float& y, float& z);

  // Applies hard-iron then soft-iron in the magnetometer's OWN axes, and only
  // then remaps into the mount frame. The order is not interchangeable: a
  // MotionCal fit is derived from raw sensor axes, so applying it after a remap
  // feeds each offset to the wrong axis.
  void applyMagCalibration(float rawX, float rawY, float rawZ,
                           FUSION_MATH_TYPE& mx, FUSION_MATH_TYPE& my, FUSION_MATH_TYPE& mz);

  // VOLTINO UPDATE: Variables for tracking actual Fusion Hz
  unsigned long _lastHzCheckTime = 0;
  uint32_t _updateCount = 0;
  float _actualFusionHz = 0.0f;
  void trackUpdateRate();

public: 
  ICM42688P* _imu;
  AK09918C* _mag;
  
  FUSION_MATH_TYPE q[4] = {1.0, 0.0, 0.0, 0.0};
  FUSION_MATH_TYPE lastAx = 0, lastAy = 0, lastAz = 0;
  FUSION_MATH_TYPE lastGx = 0, lastGy = 0, lastGz = 0;
  FUSION_MATH_TYPE lastMx = 0, lastMy = 0, lastMz = 0;
  
  // NOTE: the fusion layer no longer stores an accel/gyro bias of its own. It
  // used to, applied AFTER remapAxes() (i.e. in the mount frame) while the
  // driver applied its own in the sensor's axes - so populating both subtracted
  // the bias twice, and a value read from a driver getter landed on the wrong
  // axis for every orientation but ORIENTATION_Z_UP. Calibration now lives in
  // one place only: ICM42688P's accOffset / accScale / gyrOffset, all in SENSOR
  // axes, applied before the mount remap. Use setGyroOffsets() (forwards to the
  // driver) or the driver's own setters, and read back with its getters.
  
  // Dynamic Gyro Bias (In-flight drift correction). Units: dps, matching lastGx/y/z.
  FUSION_MATH_TYPE gyroBias[3] = {0.0, 0.0, 0.0};
  bool _dynamicBiasEnabled = false;
  float _biasKi = 0.0001f;
  float _maxGyroBiasDps = 5.0f;   // Anti-windup bound on the learned bias

  float magHardIron[3] = {0.0f, 0.0f, 0.0f};      
  float magSoftIron[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; 
  
  float _localGravity = 9.80665f; 

  float accRef = 1.0f;          
  float accSigma = 0.05f;       
  float magRef = 50.88f;         
  float magSigma = 3.5f;       
  float magTiltSigmaDeg = 15.0f;
  float magneticDeclination = 0.0f;
  float yawKi = 0.005f;           
  float maxAccelGain = 0.1f;    
  float maxMagGain = 0.1f;      
  
  unsigned long magCheckIntervalUs = 5000; 

  // NOTE: _realDt, _sampleCount and _lastOdrCheckTime used to live here as the
  // remains of an unfinished "RC oscillator ODR drift tracking" feature. Two
  // were written once and never read, the third was never touched at all, and
  // library.properties advertised the feature to the Library Manager. The claim
  // and the fields are both gone.
  //
  // The drift they were meant to cancel is real - the sensor's ODR comes from
  // an internal RC oscillator, so a nominal 8 kHz may run at 8087 Hz and move
  // with temperature - but it is already handled: the FIFO path divides the
  // MCU's measured elapsed time by the packet count, so the dt fed to the
  // integrator tracks the true rate whatever the oscillator does. Reading the
  // sensor's own ODR timestamp out of the FIFO (FIFO_TMST_FSYNC_EN plus
  // TMST_CONFIG) would be better still - it would give true per-packet timing
  // rather than assuming a batch arrived evenly - but that is a feature to
  // build deliberately, not a field to leave lying around.
  unsigned long _lastIntegrationTime = 0; 

  FUSION_MATH_TYPE invSqrt(FUSION_MATH_TYPE x);
  void clampSampleDt(FUSION_MATH_TYPE& dt, FUSION_MATH_TYPE ideal_dt);
  FUSION_MATH_TYPE gaussianGain(FUSION_MATH_TYPE x, FUSION_MATH_TYPE mu, FUSION_MATH_TYPE sigma);
  void gyroIntegration(FUSION_MATH_TYPE gx, FUSION_MATH_TYPE gy, FUSION_MATH_TYPE gz, FUSION_MATH_TYPE dt);
  void getCorrectionAngles(FUSION_MATH_TYPE ax, FUSION_MATH_TYPE ay, FUSION_MATH_TYPE az, 
                           FUSION_MATH_TYPE mx, FUSION_MATH_TYPE my, FUSION_MATH_TYPE mz, 
                           FUSION_MATH_TYPE& roll, FUSION_MATH_TYPE& pitch, FUSION_MATH_TYPE& yaw);
  void quaternionToEuler(FUSION_MATH_TYPE& roll, FUSION_MATH_TYPE& pitch, FUSION_MATH_TYPE& yaw);

public:
  TriSenseFusion(ICM42688P* imu, AK09918C* mag);
  virtual bool update() = 0;
  
  void setMountOrientation(TriSenseOrientation orientation);
  float getActualFusionHz(); 
  
  // One-point gravity calibration: hold the board still, any face up. Refines
  // the driver's accelerometer offset (in sensor axes) - it does NOT touch the
  // scale factors, so it is the quick alternative to the 6-point
  // ICM42688P::autoCalibrateAccel(), not a replacement for it. Both write the
  // same storage, so the later call refines the earlier one instead of
  // silently stacking on top of it.
  void calibrateAccelStatic(int samples = DEFAULT_CALIBRATION_SAMPLES);
  void initOrientation(int samples = DEFAULT_CALIBRATION_SAMPLES);
  
  void getOrientationDegrees(float& roll, float& pitch, float& yaw);
  float getMagHeadingDegrees();

  void setLocalGravity(float g);
  void getGlobalAcceleration(float& ax, float& ay, float& az, AccelUnit unit = ACCEL_UNIT_G);
  void getLinearAcceleration(float& ax, float& ay, float& az, AccelUnit unit = ACCEL_UNIT_G);
  void getGlobalLinearAcceleration(float& ax, float& ay, float& az, AccelUnit unit = ACCEL_UNIT_G);
  
  void setDynamicGyroBias(bool enable, float ki = 0.0001f);
  void setMaxGyroBias(float maxDps);
  void setAccelGaussian(float ref, float sigma);
  void setMagGaussian(float ref, float sigma, float tiltSigma); 
  void setMagGaussian(float ref, float sigma);                  
  void setMagTiltSigma(float sigmaDeg);                         
  void setMagCalibration(float hardIron[3], float softIron[3][3]);
  void setDeclination(float deg);
  // Gyro bias in the SENSOR's own axes (dps), stored in the driver. Matches
  // what ICM42688P::autoCalibrateGyro() computes and getGyroOffset() reports,
  // so a value saved to EEPROM can be restored through either, at any mount
  // orientation.
  void setGyroOffsets(float x, float y, float z);
  void setMagHardIron(float x, float y, float z);
  void setMagSoftIron(float matrix[3][3]);
  void setYawKi(float ki);
  void setMaxGains(float accelGain, float magGain);
  void setMagCheckInterval(float intervalMs);
};

class SimpleTriFusion : public TriSenseFusion {
private:
  bool _lightweightGravityEnabled = false;
  float _lightweightKp = 0.02f;
public:
  SimpleTriFusion(ICM42688P* imu, AK09918C* mag);
  bool update() override;
  void enableLightweightGravity(bool enable, float kp = 0.02f); 
};

class AdvancedTriFusion : public TriSenseFusion {
private:
  unsigned long lastMagCheckTime = 0;
  unsigned long lastSuccessfulCorrectionTime = 0;

  void complementaryCorrection(FUSION_MATH_TYPE ax, FUSION_MATH_TYPE ay, FUSION_MATH_TYPE az, 
                               FUSION_MATH_TYPE mx, FUSION_MATH_TYPE my, FUSION_MATH_TYPE mz, 
                               FUSION_MATH_TYPE correction_dt);
public:
  AdvancedTriFusion(ICM42688P* imu, AK09918C* mag);
  bool update() override;
};

#endif