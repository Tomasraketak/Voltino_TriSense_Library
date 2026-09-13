#include "TriSense.h"

TriSense::TriSense() {
  memset(&_last, 0, sizeof(_last));
}

bool TriSense::beginAll(TriSenseMode mode, uint8_t spiCsPin, uint32_t spiFreq, TwoWire &wire) {
  _mode = mode;
  _wire = &wire;

  // One shared bus for the whole module. Started once here, so the individual
  // drivers never have to guess which TwoWire instance they are on.
  _wire->begin();

  if (!bmp.begin(BMP580_PRIMARY_I2C_ADDR, *_wire)) return false;
  if (!mag.begin(AK_ODR_100HZ, *_wire)) return false;

  if (_mode == MODE_I2C) {
    imu.setWire(*_wire);
    if (!imu.begin(BUS_I2C)) return false;
    imu.setODR(DEFAULT_IMU_ODR); 
  } else {
    if (!imu.begin(BUS_SPI, spiCsPin, spiFreq)) return false;
    imu.setODR(DEFAULT_IMU_SPI_ODR); 
  }

  imu.setFIFOMode(FIFO_16BIT);

  bmp.setOversampling(BMP580_OSR_x2, BMP580_OSR_x2);
  bmp.setODR(BMP580_ODR_240Hz);
  bmp.setPowerMode(BMP580_MODE_NORMAL);
  mag.setODR(AK_ODR_100HZ); 
  
  return true;
}

bool TriSense::beginBMP(uint8_t addr, TwoWire &wire) { _wire = &wire; return bmp.begin(addr, wire); }
bool TriSense::beginMAG(TwoWire &wire) { _wire = &wire; return mag.begin(AK_ODR_100HZ, wire); }
bool TriSense::beginIMU(ICM_BUS busType, uint8_t csPin, uint32_t freq, TwoWire &wire) {
  if (busType == BUS_I2C) { _wire = &wire; imu.setWire(wire); }
  return imu.begin(busType, csPin, freq);
}

void TriSense::resetHardwareOffsets() { imu.resetHardwareOffsets(); }
void TriSense::autoCalibrateGyro(uint16_t samples) { imu.autoCalibrateGyro(samples); }
void TriSense::autoCalibrateAccel() { imu.autoCalibrateAccel(); }

bool TriSense::getSnapshot(TriSenseDataSnapshot &data) {
  const uint32_t now = micros();

  // --- IMU ------------------------------------------------------------------
  // In FIFO mode the oldest queued packet is not what a "snapshot" means, so
  // drain what is buffered and keep the newest. The loop is bounded twice over:
  // by the FIFO emptying, and by a hard cap so a misbehaving sensor that always
  // reports data-ready can never stall the caller.
  {
    float ax, ay, az, gx, gy, gz;
    const bool streaming = (imu.getFIFOMode() != FIFO_NONE);
    uint16_t guard = streaming ? 256 : 1;   // Direct register reads never "empty"

    bool got = false;
    while (guard-- && imu.readFIFO(ax, ay, az, gx, gy, gz)) {
      _last.accelX = ax; _last.accelY = ay; _last.accelZ = az;
      _last.gyroX  = gx; _last.gyroY  = gy; _last.gyroZ  = gz;
      got = true;
      if (!streaming) break;
    }

    if (got) { _haveIMU = true; _lastImuUs = now; }
    data.imuFresh = got;
  }

  // --- Magnetometer ---------------------------------------------------------
  // readData() returns false whenever DRDY is clear, which at 100 Hz ODR is the
  // common case in a fast loop. That is not an error, it just means "nothing new".
  {
    const bool got = mag.readData();
    if (got) {
      _last.magX = mag.x; _last.magY = mag.y; _last.magZ = mag.z;
      _haveMag = true; _lastMagUs = now;
    }
    data.magFresh = got;
  }

  // --- Barometer ------------------------------------------------------------
  // BMP580 already caches internally against its own ODR, so these calls only
  // touch the bus when a new conversion is actually ready. Compare against the
  // previous values to report whether this call saw a new conversion.
  {
    const float p = bmp.readPressure();
    const float t = bmp.readTemperature();
    const bool got = (!_haveBaro) || (p != _last.pressure) || (t != _last.temperature);
    if (got) { _haveBaro = true; _lastBaroUs = now; }
    _last.pressure = p;
    _last.temperature = t;
    data.baroFresh = got;
  }

  // --- Publish --------------------------------------------------------------
  data.accelX = _last.accelX; data.accelY = _last.accelY; data.accelZ = _last.accelZ;
  data.gyroX  = _last.gyroX;  data.gyroY  = _last.gyroY;  data.gyroZ  = _last.gyroZ;
  data.magX   = _last.magX;   data.magY   = _last.magY;   data.magZ   = _last.magZ;
  data.pressure = _last.pressure;
  data.temperature = _last.temperature;

  // Unsigned subtraction, so these stay correct across the ~71 min micros() wrap.
  data.imuAgeUs  = _haveIMU  ? (now - _lastImuUs)  : 0xFFFFFFFFUL;
  data.magAgeUs  = _haveMag  ? (now - _lastMagUs)  : 0xFFFFFFFFUL;
  data.baroAgeUs = _haveBaro ? (now - _lastBaroUs) : 0xFFFFFFFFUL;

  return _haveIMU && _haveMag && _haveBaro;
}

float TriSense::readPressure() { return bmp.readPressure(); }
float TriSense::readTemperature() { return bmp.readTemperature(); }
float TriSense::readAltitude(float seaLevelPressure) { return bmp.readAltitude(seaLevelPressure); }

TriSenseFusion::TriSenseFusion(ICM42688P* imu, AK09918C* mag) : _imu(imu), _mag(mag) {}

void TriSenseFusion::trackUpdateRate() {
  _updateCount++;
  unsigned long now = millis();
  if (now - _lastHzCheckTime >= 1000) {
    if (_lastHzCheckTime > 0) {
      _actualFusionHz = (float)_updateCount / ((now - _lastHzCheckTime) / 1000.0f);
    }
    _updateCount = 0;
    _lastHzCheckTime = now;
  }
}

float TriSenseFusion::getActualFusionHz() {
  return _actualFusionHz;
}

// Reciprocal square root - called on every quaternion normalisation, so it sits
// in the hottest path of the whole fusion loop.
//
// Implementation is chosen per platform (see TRISENSE_HAS_HW_FPU in TriSense.h):
//
//   * MCU with a hardware FPU (RP2350, ESP32/S3, SAMD51, STM32F4+, nRF52840,
//     Teensy): sqrtf() is a SINGLE instruction (VSQRT.F32 on ARM), followed by
//     one divide. On the RP2350 that is ~20 cycles total and exact to the last
//     bit. The bit-trick below needs ~4 multiplies plus a shift and is both
//     SLOWER and less accurate there, so it is not used.
//
//   * MCU without an FPU (Arduino Uno/Nano/Mega, RP2040, ESP32-C3, SAMD21):
//     software sqrtf() costs hundreds of cycles, so the classic Quake bit-trick
//     wins. Two Newton-Raphson steps are used instead of one: the seed alone is
//     good to ~3.4%, one step to ~1.8e-3, two steps to ~5e-6. The second step
//     costs 2 multiplies and a subtract, which is cheap insurance given this
//     result scales the quaternion on EVERY integration step - a systematic
//     0.18% norm error would otherwise accumulate into the attitude.
//
// The bit reinterpretation goes through memcpy rather than a pointer cast.
// Casting a float* to uint32_t* violates C++ strict aliasing and GCC is free to
// miscompile it at -O2; memcpy is the well-defined spelling and every compiler
// lowers it to the same single register move (zero cost).
FUSION_MATH_TYPE TriSenseFusion::invSqrt(FUSION_MATH_TYPE x) {
#if defined(FORCE_FUSION_DOUBLE)
  return 1.0 / sqrt(x);
#elif defined(TRISENSE_HAS_HW_FPU)
  return (FUSION_MATH_TYPE)(1.0f / sqrtf((float)x));
#else
  const float f = (float)x;
  const float xhalf = 0.5f * f;

  uint32_t i;
  memcpy(&i, &f, sizeof(i));
  i = 0x5f3759df - (i >> 1);

  float y;
  memcpy(&y, &i, sizeof(y));
  y = y * (1.5f - xhalf * y * y);   // Newton step 1 -> ~1.8e-3 worst case
  y = y * (1.5f - xhalf * y * y);   // Newton step 2 -> ~5e-6 worst case
  return (FUSION_MATH_TYPE)y;
#endif
}

// Sanity bounds on the measured per-sample dt.
//
// The measured value (elapsed wall time / packets drained) is the RIGHT number
// to integrate with, and it is deliberately allowed to exceed the nominal
// sample period. When the FIFO overflows - a stalled loop, a long SD write -
// packets are genuinely lost, and spreading the surviving samples across the
// real elapsed time is the best available estimate of how far the device turned
// while we were not looking. It keeps the integrated time consistent with the
// clock.
//
// The old code clamped anything above 2x nominal back DOWN to exactly nominal,
// which threw that time away: 128 packets drained after a 300 ms stall at 1 kHz
// integrated 128 ms and silently discarded 172 ms of rotation - about 15 deg at
// 90 deg/s, unrecoverable, and triggered precisely when the MCU was busiest.
//
// So the upper bound here is NOT a nominal-rate check; it is purely a numerical
// guard. gyroIntegration() is a first-order quaternion update, only valid while
// the rotation per step stays small, so an absurd dt (a multi-second gap after
// flushFIFO(), a stale _lastIntegrationTime) must not be fed into it. 50 ms
// keeps the first-order error negligible at any realistic rate, and at slow
// ODRs the floor makes sure the cap never drops below one real sample period.
//
// The lower bound stays: a dt well under nominal means more packets arrived than
// elapsed time allows, which is physically impossible - the sensor's own sample
// period is the truth there.
void TriSenseFusion::clampSampleDt(FUSION_MATH_TYPE& dt, FUSION_MATH_TYPE ideal_dt) {
  FUSION_MATH_TYPE dt_cap = (FUSION_MATH_TYPE)0.05;
  if (dt_cap < ideal_dt) dt_cap = ideal_dt;
  if (dt > dt_cap) dt = dt_cap;
  if (dt < ideal_dt * (FUSION_MATH_TYPE)0.5) dt = ideal_dt;
}

FUSION_MATH_TYPE TriSenseFusion::gaussianGain(FUSION_MATH_TYPE x, FUSION_MATH_TYPE mu, FUSION_MATH_TYPE sigma) {
  if (sigma == 0.0) return 0.0;
  FUSION_MATH_TYPE diff = x - mu;
  return exp(-(diff * diff) / ((FUSION_MATH_TYPE)2.0 * sigma * sigma));
}

void TriSenseFusion::setMountOrientation(TriSenseOrientation orientation) { _mountOrientation = orientation; }

void TriSenseFusion::remapAxes(float& x, float& y, float& z) {
  float tx = x, ty = y, tz = z;
  switch (_mountOrientation) {
    case ORIENTATION_X_UP:
      x = -tz; y = ty; z = tx;  
      break;
    case ORIENTATION_Y_UP:
      x = tx; y = -tz; z = ty;  
      break;
    case ORIENTATION_Z_DOWN:
      x = -tx; y = ty; z = -tz;
      break;
    case ORIENTATION_Z_UP:
    default:
      break; 
  }
}

// Exact inverse of remapAxes(). Every orientation above is a signed permutation
// of the axes, so the inverse is another signed permutation - no numerics, no
// error. Used to express a correction that was worked out in the mount frame
// back in the sensor's own axes, which is where all calibration is stored.
void TriSenseFusion::unremapAxes(float& x, float& y, float& z) {
  float tx = x, ty = y, tz = z;
  switch (_mountOrientation) {
    case ORIENTATION_X_UP:
      x = tz; y = ty; z = -tx;
      break;
    case ORIENTATION_Y_UP:
      x = tx; y = tz; z = -ty;
      break;
    case ORIENTATION_Z_DOWN:
      x = -tx; y = ty; z = -tz;
      break;
    case ORIENTATION_Z_UP:
    default:
      break;
  }
}

// Hard iron is a fixed offset in the sensor's physical axes and soft iron a
// linear distortion in those same axes - both are properties of the sensor and
// whatever magnetic junk is mounted next to it. So they must be removed while
// the reading is still in sensor axes; only the corrected, physically-real
// field vector may then be rotated into the mount frame.
//
// Doing it the other way round (remap first) silently applies each hard-iron
// offset to the wrong axis for every orientation except ORIENTATION_Z_UP, where
// remapAxes() happens to be the identity. The result is an off-centre locus in
// the horizontal plane, which makes heading sensitivity vary with direction.
void TriSenseFusion::applyMagCalibration(float rawX, float rawY, float rawZ,
                                         FUSION_MATH_TYPE& mx, FUSION_MATH_TYPE& my, FUSION_MATH_TYPE& mz) {
  FUSION_MATH_TYPE hx = (FUSION_MATH_TYPE)rawX - magHardIron[0];
  FUSION_MATH_TYPE hy = (FUSION_MATH_TYPE)rawY - magHardIron[1];
  FUSION_MATH_TYPE hz = (FUSION_MATH_TYPE)rawZ - magHardIron[2];

  float cx = (float)(magSoftIron[0][0]*hx + magSoftIron[0][1]*hy + magSoftIron[0][2]*hz);
  float cy = (float)(magSoftIron[1][0]*hx + magSoftIron[1][1]*hy + magSoftIron[1][2]*hz);
  float cz = (float)(magSoftIron[2][0]*hx + magSoftIron[2][1]*hy + magSoftIron[2][2]*hz);

  remapAxes(cx, cy, cz); // Rotate the CALIBRATED vector into the mount frame

  mx = (FUSION_MATH_TYPE)cx; my = (FUSION_MATH_TYPE)cy; mz = (FUSION_MATH_TYPE)cz;
}

void TriSenseFusion::setDynamicGyroBias(bool enable, float ki) { _dynamicBiasEnabled = enable; _biasKi = ki; }
void TriSenseFusion::setMaxGyroBias(float maxDps) { _maxGyroBiasDps = (maxDps < 0.0f) ? -maxDps : maxDps; }
void TriSenseFusion::setAccelGaussian(float ref, float sigma) { accRef = ref; accSigma = sigma; }
void TriSenseFusion::setMagGaussian(float ref, float sigma, float tiltSigma) { magRef = ref; magSigma = sigma; magTiltSigmaDeg = tiltSigma; }
void TriSenseFusion::setMagGaussian(float ref, float sigma) { magRef = ref; magSigma = sigma; }
void TriSenseFusion::setMagTiltSigma(float sigmaDeg) { magTiltSigmaDeg = sigmaDeg; }
void TriSenseFusion::setMagCalibration(float hard[3], float soft[3][3]) {
  for(int i=0; i<3; i++) magHardIron[i] = hard[i];
  for(int i=0; i<3; i++) for(int j=0; j<3; j++) magSoftIron[i][j] = soft[i][j];
}
void TriSenseFusion::setDeclination(float deg) { magneticDeclination = deg; }
// Forwards to the driver, so there is exactly one place a gyro bias is stored.
// The values are in the SENSOR's own axes - the same axes the driver's
// autoCalibrateGyro() and getGyroOffset() use - so a bias read back from the
// driver (or restored from EEPROM) can be handed straight back here regardless
// of the mount orientation. The mount remap happens afterwards, on the already
// bias-corrected sample.
void TriSenseFusion::setGyroOffsets(float x, float y, float z) { if (_imu) _imu->setGyroOffset(x, y, z); }
void TriSenseFusion::setMagHardIron(float x, float y, float z) { magHardIron[0] = x; magHardIron[1] = y; magHardIron[2] = z; }
void TriSenseFusion::setMagSoftIron(float matrix[3][3]) { for(int i=0; i<3; i++) for(int j=0; j<3; j++) magSoftIron[i][j] = matrix[i][j]; }
void TriSenseFusion::setYawKi(float ki) { yawKi = ki; }
void TriSenseFusion::setMaxGains(float accelGain, float magGain) { maxAccelGain = accelGain; maxMagGain = magGain; }
void TriSenseFusion::setMagCheckInterval(float intervalMs) { magCheckIntervalUs = (unsigned long)(intervalMs * 1000.0f); }
void TriSenseFusion::setLocalGravity(float g) { _localGravity = g; }

void TriSenseFusion::calibrateAccelStatic(int samples) {
  double sumX=0, sumY=0, sumZ=0; 
  int count = 0;
  
  // Discard whatever the FIFO already holds, so the average below is taken from
  // samples captured AFTER the user was told to hold still.
  //
  // This used to be `while (readFIFO(...));` - drain until it comes up empty.
  // That loop has no fixed point when the sensor refills faster than the loop
  // can consume: at a high ODR readFIFO() simply never returns false and the
  // sketch hangs here forever with no output at all. The hardware has a
  // single-register flush for exactly this, and it is O(1).
  _imu->flushFIFO();

  // Bounded: a sensor that stops delivering must not hang the sketch with no
  // output. autoCalibrateGyro() has had this guard all along; these two had not.
  const unsigned long deadline = millis() + CALIBRATION_TIMEOUT_MS;
  while(count < samples) { 
    if ((long)(millis() - deadline) >= 0) break;
    float ax, ay, az, gx, gy, gz;
    if (_imu->readFIFO(ax, ay, az, gx, gy, gz)) {
      remapAxes(ax, ay, az);
      sumX += ax; sumY += ay; sumZ += az; 
      count++;
    } else {
      delay(1); 
    }
  }

  if (count == 0) return;          // Nothing measured - leave calibration alone
  samples = count;                 // Average over what actually arrived
  
  float avgX = (float)(sumX / samples);
  float avgY = (float)(sumY / samples);
  float avgZ = (float)(sumZ / samples);
  
  // Which axis is pointing up can only be decided in the MOUNT frame, so the
  // residual is worked out here, exactly as before: the up axis should read
  // 1 g and every other axis 0 g.
  float dx, dy, dz;
  if (fabs(avgZ) > 0.7f) {
    dx = avgX; dy = avgY;
    dz = (avgZ > 0) ? (avgZ - 1.0f) : (avgZ + 1.0f);
  } else if (fabs(avgX) > 0.7f) {
    dx = (avgX > 0) ? (avgX - 1.0f) : (avgX + 1.0f);
    dy = avgY; dz = avgZ;
  } else if (fabs(avgY) > 0.7f) {
    dx = avgX;
    dy = (avgY > 0) ? (avgY - 1.0f) : (avgY + 1.0f);
    dz = avgZ;
  } else {
    dx = avgX; dy = avgY; dz = avgZ - 1.0f; 
  }

  // ...and then rotated back into the sensor's own axes, because that is the
  // one place an accelerometer bias is stored (the driver's accOffset). Storing
  // it in the mount frame instead is what used to make this calibration and the
  // driver's autoCalibrateAccel() subtract on top of each other.
  unremapAxes(dx, dy, dz);

  if (!_imu) return;

  // Fold the residual into the offset the driver already holds. The driver
  // computes (raw - offset) * scale, so a correction of `d` on its OUTPUT costs
  // d/scale on the offset. Reusing the existing offset (rather than replacing
  // it) is what makes this callable a second time to refine a previous run.
  float ox, oy, oz, sx, sy, sz;
  _imu->getAccelOffset(ox, oy, oz);
  _imu->getAccelScale(sx, sy, sz);

  // A zero scale would mean the axis is dead anyway; leave its offset alone
  // rather than dividing by zero and poisoning it with an infinity.
  if (sx != 0.0f) ox += dx / sx;
  if (sy != 0.0f) oy += dy / sy;
  if (sz != 0.0f) oz += dz / sz;

  _imu->setAccelOffset(ox, oy, oz);
}

void TriSenseFusion::initOrientation(int samples) {
  FUSION_MATH_TYPE axSum=0, aySum=0, azSum=0, mxSum=0, mySum=0, mzSum=0; 
  int count = 0;
  
  // Same as in calibrateAccelStatic(): flush in one register write rather than
  // looping until the FIFO reads empty, which never happens at a high ODR.
  _imu->flushFIFO();

  // Bounded, for the same reason as calibrateAccelStatic(): this is the call
  // that hung a sketch at 32 kHz with nothing printed after "keep still".
  const unsigned long deadline = millis() + CALIBRATION_TIMEOUT_MS;
  while(count < samples) {
     if ((long)(millis() - deadline) >= 0) break;
     float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
     
     bool imuReady = _imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
     _mag->readData(); 

     if(imuReady) {
         remapAxes(ax_raw, ay_raw, az_raw);
         FUSION_MATH_TYPE ax = ax_raw; 
         FUSION_MATH_TYPE ay = ay_raw; 
         FUSION_MATH_TYPE az = az_raw; 
         axSum+=ax; aySum+=ay; azSum+=az;
         
         FUSION_MATH_TYPE mx, my, mz;
         applyMagCalibration(_mag->x, _mag->y, _mag->z, mx, my, mz);

         mxSum+=mx; mySum+=my; mzSum+=mz;
         count++; 
     } else {
         delay(1); 
     }
  }
  
  // Average over what actually arrived, not over what was asked for: a run cut
  // short by the deadline would otherwise divide by too large a number and
  // report a gravity vector shorter than 1 g, tilting the seeded attitude.
  if (count == 0) return;          // Nothing measured - keep the identity quaternion
  samples = count;

  FUSION_MATH_TYPE r, p, y; 
  getCorrectionAngles(axSum/samples, aySum/samples, azSum/samples, mxSum/samples, mySum/samples, mzSum/samples, r, p, y);
  
  FUSION_MATH_TYPE c1 = cos(y * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE s1 = sin(y * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE c2 = cos(p * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE s2 = sin(p * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE c3 = cos(r * (FUSION_MATH_TYPE)PI / 360.0); 
  FUSION_MATH_TYPE s3 = sin(r * (FUSION_MATH_TYPE)PI / 360.0);
  
  q[0] = c1*c2*c3 + s1*s2*s3; 
  q[1] = c1*c2*s3 - s1*s2*c3; 
  q[2] = c1*s2*c3 + s1*c2*s3; 
  q[3] = s1*c2*c3 - c1*s2*s3;

  _lastIntegrationTime = micros(); 
}

void TriSenseFusion::quaternionToEuler(FUSION_MATH_TYPE& roll, FUSION_MATH_TYPE& pitch, FUSION_MATH_TYPE& yaw) {
  FUSION_MATH_TYPE sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]); 
  FUSION_MATH_TYPE cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]); 
  roll = atan2(sinr_cosp, cosr_cosp);
  
  // fabs(), not abs(): Arduino.h defines abs() as a macro, but when <stdlib.h>'s
  // integer abs(int) wins the lookup instead - which happens on some cores and
  // in any translation unit that includes <cstdlib> after Arduino.h - the
  // argument is truncated to an int and every value below 1.0 becomes 0.
  FUSION_MATH_TYPE sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]); 
  if (fabs(sinp) >= 1.0) pitch = copysign((FUSION_MATH_TYPE)PI / 2.0, sinp); else pitch = asin(sinp);
  
  FUSION_MATH_TYPE siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]); 
  FUSION_MATH_TYPE cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]); 
  yaw = atan2(siny_cosp, cosy_cosp);
}

void TriSenseFusion::getOrientationDegrees(float& roll, float& pitch, float& yaw) {
  FUSION_MATH_TYPE r, p, y; quaternionToEuler(r, p, y); 
  roll = (float)(r * 180.0 / PI); pitch = (float)(p * 180.0 / PI); yaw = (float)(y * 180.0 / PI); 
  if (yaw < 0) yaw += 360.0f; if (yaw >= 360.0f) yaw -= 360.0f;
}

// Magnetometer-only heading (tilt-compensated using the fusion's current roll/
// pitch), independent of the gyro-integrated yaw. Useful for comparing against
// getOrientationDegrees()'s yaw to spot magnetic interference or bad calibration.
// Requires the magnetometer to have been read at least once (AdvancedTriFusion
// does this automatically; SimpleTriFusion never reads it, so lastMx/My/Mz stay 0).
float TriSenseFusion::getMagHeadingDegrees() {
  float roll, pitch, yaw;
  getOrientationDegrees(roll, pitch, yaw);
  return AK09918C::computeHeading((float)lastMx, (float)lastMy, (float)lastMz, roll, pitch, magneticDeclination);
}

// Reference roll/pitch/yaw straight from the accelerometer and magnetometer.
//
// These are Euler angles and therefore degenerate at pitch = +/-90 degrees,
// where roll and yaw stop being separable. Callers must weight the yaw this
// returns by how far the board is from that attitude - complementaryCorrection()
// does so with both an estimate-based and a measurement-based gain. The roll and
// pitch corrections do not come through here at all: they use the accelerometer
// cross product in complementaryCorrection(), which has no such singularity.
void TriSenseFusion::getCorrectionAngles(FUSION_MATH_TYPE ax, FUSION_MATH_TYPE ay, FUSION_MATH_TYPE az,
                                         FUSION_MATH_TYPE mx, FUSION_MATH_TYPE my, FUSION_MATH_TYPE mz, 
                                         FUSION_MATH_TYPE& roll, FUSION_MATH_TYPE& pitch, FUSION_MATH_TYPE& yaw) {
  roll  = atan2(ay, az) * 180.0 / PI; pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  FUSION_MATH_TYPE phi = roll * (FUSION_MATH_TYPE)PI / 180.0; 
  FUSION_MATH_TYPE theta = pitch * (FUSION_MATH_TYPE)PI / 180.0;
  FUSION_MATH_TYPE by = my * cos(phi) - mz * sin(phi); 
  FUSION_MATH_TYPE bx = mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi);
  yaw = atan2(-by, bx) * 180.0 / PI + (FUSION_MATH_TYPE)magneticDeclination; 
  if (yaw < 0) yaw += 360.0; if (yaw >= 360.0) yaw -= 360.0;
}

void TriSenseFusion::getGlobalAcceleration(float& ax, float& ay, float& az, AccelUnit unit) {
  FUSION_MATH_TYPE qw = q[0], qx = q[1], qy = q[2], qz = q[3]; 
  FUSION_MATH_TYPE xx = qx * qx, yy = qy * qy, zz = qz * qz;
  FUSION_MATH_TYPE xy = qx * qy, xz = qx * qz, yz = qy * qz;
  FUSION_MATH_TYPE wx = qw * qx, wy = qw * qy, wz = qw * qz;
  
  FUSION_MATH_TYPE ax_g = (1.0 - 2.0*(yy + zz)) * lastAx + 2.0*(xy - wz) * lastAy + 2.0*(xz + wy) * lastAz;
  FUSION_MATH_TYPE ay_g = 2.0*(xy + wz) * lastAx + (1.0 - 2.0*(xx + zz)) * lastAy + 2.0*(yz - wx) * lastAz;
  FUSION_MATH_TYPE az_g = 2.0*(xz - wy) * lastAx + 2.0*(yz + wx) * lastAy + (1.0 - 2.0*(xx + yy)) * lastAz;

  if (unit == ACCEL_UNIT_MS2) {
     ax = (float)(ax_g * _localGravity); ay = (float)(ay_g * _localGravity); az = (float)(az_g * _localGravity);
  } else {
     ax = (float)ax_g; ay = (float)ay_g; az = (float)az_g;
  }
}

// World-frame acceleration with gravity removed (i.e. getGlobalAcceleration() minus the
// world-frame gravity vector) — this is the quantity to double-integrate for dead reckoning.
void TriSenseFusion::getGlobalLinearAcceleration(float& ax, float& ay, float& az, AccelUnit unit) {
  getGlobalAcceleration(ax, ay, az, unit);
  az -= (unit == ACCEL_UNIT_MS2) ? _localGravity : 1.0f;
}

void TriSenseFusion::getLinearAcceleration(float& ax, float& ay, float& az, AccelUnit unit) {
  FUSION_MATH_TYPE qw = q[0], qx = q[1], qy = q[2], qz = q[3]; 
  FUSION_MATH_TYPE grav_x = 2.0 * (qx * qz - qw * qy);
  FUSION_MATH_TYPE grav_y = 2.0 * (qw * qx + qy * qz);
  FUSION_MATH_TYPE grav_z = 1.0 - 2.0 * (qx * qx + qy * qy);

  FUSION_MATH_TYPE lin_x = lastAx - grav_x;
  FUSION_MATH_TYPE lin_y = lastAy - grav_y;
  FUSION_MATH_TYPE lin_z = lastAz - grav_z;

  if (unit == ACCEL_UNIT_MS2) {
     ax = (float)(lin_x * _localGravity); ay = (float)(lin_y * _localGravity); az = (float)(lin_z * _localGravity);
  } else {
     ax = (float)lin_x; ay = (float)lin_y; az = (float)lin_z;
  }
}

void TriSenseFusion::gyroIntegration(FUSION_MATH_TYPE gx, FUSION_MATH_TYPE gy, FUSION_MATH_TYPE gz, FUSION_MATH_TYPE dt) {
  FUSION_MATH_TYPE qDot1 = 0.5 * (-q[1] * gx - q[2] * gy - q[3] * gz); 
  FUSION_MATH_TYPE qDot2 = 0.5 * (q[0] * gx + q[2] * gz - q[3] * gy);
  FUSION_MATH_TYPE qDot3 = 0.5 * (q[0] * gy - q[1] * gz + q[3] * gx); 
  FUSION_MATH_TYPE qDot4 = 0.5 * (q[0] * gz + q[1] * gy - q[2] * gx);
  
  q[0] += qDot1 * dt; q[1] += qDot2 * dt; q[2] += qDot3 * dt; q[3] += qDot4 * dt;
  FUSION_MATH_TYPE recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]); 
  q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

SimpleTriFusion::SimpleTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

void SimpleTriFusion::enableLightweightGravity(bool enable, float kp) {
  _lightweightGravityEnabled = enable;
  _lightweightKp = kp;
}

bool SimpleTriFusion::update() {
  bool dataProcessed = false;
  
  if (_imu->getFIFOMode() == FIFO_NONE) {
    float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    if (_imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) {
      dataProcessed = true;
      remapAxes(ax_raw, ay_raw, az_raw);
      remapAxes(gx_raw, gy_raw, gz_raw);
      
      lastAx = ax_raw; lastAy = ay_raw; lastAz = az_raw; 
      lastGx = gx_raw;  lastGy = gy_raw;  lastGz = gz_raw;
      
      unsigned long nowMicros = micros();
      if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
      FUSION_MATH_TYPE dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
      _lastIntegrationTime = nowMicros;
      if (dt <= 0.0) dt = 0.00001;
      // Numerical guard only - same reasoning as clampSampleDt(), which this
      // path cannot use wholesale: without a FIFO the loop may poll faster than
      // the ODR and re-read the same sample, so a dt BELOW nominal is normal
      // here and must not be rounded up, or that sample gets integrated twice.
      if (dt > 0.05) dt = 0.05;
      
      FUSION_MATH_TYPE gx_rad = lastGx * (FUSION_MATH_TYPE)PI/180.0;
      FUSION_MATH_TYPE gy_rad = lastGy * (FUSION_MATH_TYPE)PI/180.0;
      FUSION_MATH_TYPE gz_rad = lastGz * (FUSION_MATH_TYPE)PI/180.0;

      if (_lightweightGravityEnabled) {
          FUSION_MATH_TYPE norm = invSqrt(lastAx*lastAx + lastAy*lastAy + lastAz*lastAz);
          FUSION_MATH_TYPE ax_n = lastAx * norm, ay_n = lastAy * norm, az_n = lastAz * norm;
          FUSION_MATH_TYPE grav_x = 2.0 * (q[1] * q[3] - q[0] * q[2]);
          FUSION_MATH_TYPE grav_y = 2.0 * (q[0] * q[1] + q[2] * q[3]);
          FUSION_MATH_TYPE grav_z = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
          gx_rad += (FUSION_MATH_TYPE)_lightweightKp * (ay_n * grav_z - az_n * grav_y);
          gy_rad += (FUSION_MATH_TYPE)_lightweightKp * (az_n * grav_x - ax_n * grav_z);
          gz_rad += (FUSION_MATH_TYPE)_lightweightKp * (ax_n * grav_y - ay_n * grav_x);
      }
      gyroIntegration(gx_rad, gy_rad, gz_rad, dt);
      trackUpdateRate();
    }
  } else {
    // Ask the driver how many packets are waiting BEFORE draining them. That is
    // the only thing the old two-pass version needed its 3 KB stack buffer for:
    // perfect_dt is total_dt / packetCount, so the count had to be known up
    // front. One FIFO_COUNT read supplies it, and the packets can then be
    // integrated as they stream in - nothing has to be held in RAM.
    uint16_t pending = _imu->availablePackets();
    if (pending == 0) return false;

    unsigned long nowMicros = micros();
    if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
    FUSION_MATH_TYPE total_dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
    if (total_dt <= 0.0) total_dt = 0.00001;

    FUSION_MATH_TYPE perfect_dt = total_dt / (FUSION_MATH_TYPE)pending;

    int hz = _imu->getODRHz();
    FUSION_MATH_TYPE ideal_dt = (hz > 0) ? (1.0 / (FUSION_MATH_TYPE)hz) : 0.001;
    clampSampleDt(perfect_dt, ideal_dt);

    FUSION_MATH_TYPE sumAx = 0, sumAy = 0, sumAz = 0;
    FUSION_MATH_TYPE sumGx = 0, sumGy = 0, sumGz = 0;
    uint16_t processed = 0;

    for (uint16_t i = 0; i < pending; i++) {
        float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        if (!_imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) break;

        remapAxes(ax_raw, ay_raw, az_raw);
        remapAxes(gx_raw, gy_raw, gz_raw);

        FUSION_MATH_TYPE ax = ax_raw;
        FUSION_MATH_TYPE ay = ay_raw;
        FUSION_MATH_TYPE az = az_raw;
        FUSION_MATH_TYPE gx = gx_raw;
        FUSION_MATH_TYPE gy = gy_raw;
        FUSION_MATH_TYPE gz = gz_raw;

        sumAx += ax; sumAy += ay; sumAz += az;
        sumGx += gx; sumGy += gy; sumGz += gz;

        FUSION_MATH_TYPE gx_rad = gx * (FUSION_MATH_TYPE)PI/180.0;
        FUSION_MATH_TYPE gy_rad = gy * (FUSION_MATH_TYPE)PI/180.0;
        FUSION_MATH_TYPE gz_rad = gz * (FUSION_MATH_TYPE)PI/180.0;

        if (_lightweightGravityEnabled) {
            FUSION_MATH_TYPE norm = invSqrt(ax*ax + ay*ay + az*az);
            FUSION_MATH_TYPE ax_n = ax * norm, ay_n = ay * norm, az_n = az * norm;
            FUSION_MATH_TYPE grav_x = 2.0 * (q[1] * q[3] - q[0] * q[2]);
            FUSION_MATH_TYPE grav_y = 2.0 * (q[0] * q[1] + q[2] * q[3]);
            FUSION_MATH_TYPE grav_z = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
            gx_rad += (FUSION_MATH_TYPE)_lightweightKp * (ay_n * grav_z - az_n * grav_y);
            gy_rad += (FUSION_MATH_TYPE)_lightweightKp * (az_n * grav_x - ax_n * grav_z);
            gz_rad += (FUSION_MATH_TYPE)_lightweightKp * (ax_n * grav_y - ay_n * grav_x);
        }

        gyroIntegration(gx_rad, gy_rad, gz_rad, perfect_dt);
        trackUpdateRate();
        processed++;
    }

    // Only consume the elapsed time if something was actually integrated. If the
    // drain came up empty the interval belongs to the next call, not to nothing.
    if (processed == 0) return false;
    _lastIntegrationTime = nowMicros;
    dataProcessed = true;

    // Publish the mean of the whole batch rather than just the final packet, so
    // getGlobalAcceleration() sees every accelerometer sample the FIFO delivered.
    // The per-sample values above still drive the integration.
    FUSION_MATH_TYPE invCount = (FUSION_MATH_TYPE)1.0 / (FUSION_MATH_TYPE)processed;
    lastAx = sumAx * invCount; lastAy = sumAy * invCount; lastAz = sumAz * invCount;
    lastGx = sumGx * invCount; lastGy = sumGy * invCount; lastGz = sumGz * invCount;
  }
  return dataProcessed;
}

AdvancedTriFusion::AdvancedTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

void AdvancedTriFusion::complementaryCorrection(FUSION_MATH_TYPE ax, FUSION_MATH_TYPE ay, FUSION_MATH_TYPE az, 
                                                FUSION_MATH_TYPE mx, FUSION_MATH_TYPE my, FUSION_MATH_TYPE mz, 
                                                FUSION_MATH_TYPE correction_dt) {
  // Keep the raw vector: the tilt-compensated heading below needs the real
  // magnitudes, and re-multiplying the normalised copy by totalAccelG just to
  // recover them costs three multiplies and loses a little precision.
  const FUSION_MATH_TYPE rawAx = ax, rawAy = ay, rawAz = az;

  FUSION_MATH_TYPE accelSq = ax * ax + ay * ay + az * az;

  // An all-zero accelerometer vector means the sensor gave us nothing (bus
  // fault, sensor asleep) - never a real reading, since gravity alone is 1 g in
  // free fall only. Normalising it divides by zero: invSqrt(0) is +inf on the
  // hardware-FPU path, so ax/ay/az become inf, ex/ey become NaN, and the NaN
  // lands in the quaternion, where it is permanent - every later normalisation
  // reproduces it and the attitude output never recovers. Skipping the
  // correction leaves the gyro integration to carry on alone, which is exactly
  // the right behaviour for one missing accelerometer sample.
  if (!(accelSq > (FUSION_MATH_TYPE)1e-12)) return;   // also catches NaN

  FUSION_MATH_TYPE recipNorm = invSqrt(accelSq);
  FUSION_MATH_TYPE totalAccelG = accelSq * recipNorm;   // sqrt(s) == s / sqrt(s)
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
  
  FUSION_MATH_TYPE magStrength = sqrt(mx * mx + my * my + mz * mz);
  
  FUSION_MATH_TYPE vx = 2.0 * (q[1] * q[3] - q[0] * q[2]); 
  FUSION_MATH_TYPE vy = 2.0 * (q[0] * q[1] + q[2] * q[3]); 
  FUSION_MATH_TYPE vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  FUSION_MATH_TYPE ex = (ay * vz - az * vy); 
  FUSION_MATH_TYPE ey = (az * vx - ax * vz);
  
  FUSION_MATH_TYPE g_gain_accel_raw = gaussianGain(totalAccelG, (FUSION_MATH_TYPE)accRef, (FUSION_MATH_TYPE)accSigma); 
  FUSION_MATH_TYPE g_gain_mag_raw = gaussianGain(magStrength, (FUSION_MATH_TYPE)magRef, (FUSION_MATH_TYPE)magSigma);
  FUSION_MATH_TYPE final_accel_gain = g_gain_accel_raw * (FUSION_MATH_TYPE)maxAccelGain; 
  FUSION_MATH_TYPE final_mag_gain = g_gain_mag_raw * (FUSION_MATH_TYPE)maxMagGain;
  
  FUSION_MATH_TYPE tilt_rad_sq = 2.0 * (1.0 - vz);
  if (tilt_rad_sq < 0.0) tilt_rad_sq = 0.0;
  FUSION_MATH_TYPE tilt_deg_sq = tilt_rad_sq * 3282.806; 
  
  FUSION_MATH_TYPE tilt_gain = exp(-tilt_deg_sq / (2.0 * (FUSION_MATH_TYPE)magTiltSigmaDeg * (FUSION_MATH_TYPE)magTiltSigmaDeg));
  final_mag_gain *= tilt_gain;

  // --- Gimbal-lock guard on the magnetometer heading ---
  //
  // The quaternion integration itself cannot gimbal-lock; that is the whole
  // point of using one. The tilt-compensated heading below can, because it goes
  // through Euler angles: roll = atan2(ay, az) is indeterminate when ay and az
  // both vanish, i.e. when the board points straight up or straight down. There
  // the measured roll is pure noise, and it feeds sin(phi)/cos(phi) in the
  // heading projection, so yaw_corr becomes meaningless rather than merely
  // imprecise.
  //
  // tilt_gain above is close to zero in that attitude and mostly hides this,
  // but it is derived from the ESTIMATED gravity direction, not the measured
  // one - so it stays open when the estimate is wrong, which is exactly when a
  // correction is most likely to be applied and most likely to be wrong. It is
  // also user-tunable: magTiltSigmaDeg = 60 still lets ~40% of a garbage
  // heading through at 90 degrees of tilt.
  //
  // So gate on the measurement as well. cos(pitch) = sqrt(ay^2 + az^2) for a
  // unit gravity vector; it is 1 when level and 0 at the singularity. Ramping
  // instead of cutting keeps the filter continuous - a step in the gain would
  // show up as a visible jump in the fused yaw.
  {
    const FUSION_MATH_TYPE cos_pitch = sqrt(ay * ay + az * az);
    const FUSION_MATH_TYPE ramp_from = (FUSION_MATH_TYPE)0.17365;  // cos(80 deg)
    if (cos_pitch < ramp_from) final_mag_gain *= (cos_pitch / ramp_from);
  }
  
  FUSION_MATH_TYPE roll_corr, pitch_corr, yaw_corr; 
  getCorrectionAngles(rawAx, rawAy, rawAz, mx, my, mz, roll_corr, pitch_corr, yaw_corr);
  
  FUSION_MATH_TYPE siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]); 
  FUSION_MATH_TYPE cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]); 
  FUSION_MATH_TYPE yaw_rad = atan2(siny_cosp, cosy_cosp);
  
  FUSION_MATH_TYPE yaw_deg = yaw_rad * 180.0 / (FUSION_MATH_TYPE)PI; 
  if (yaw_deg < 0) yaw_deg += 360.0; if (yaw_deg >= 360.0) yaw_deg -= 360.0;
  
  FUSION_MATH_TYPE delta_yaw_deg = yaw_corr - yaw_deg; 
  if (delta_yaw_deg > 180.0) delta_yaw_deg -= 360.0; if (delta_yaw_deg < -180.0) delta_yaw_deg += 360.0;
  FUSION_MATH_TYPE delta_yaw_rad = delta_yaw_deg * (FUSION_MATH_TYPE)PI / 180.0;
  
  if (_dynamicBiasEnabled) {
      gyroBias[0] -= (FUSION_MATH_TYPE)_biasKi * ex * final_accel_gain * correction_dt * 57.29578;
      gyroBias[1] -= (FUSION_MATH_TYPE)_biasKi * ey * final_accel_gain * correction_dt * 57.29578;
  }
  // gyroBias[] is subtracted from lastGx/y/z, which are in dps, so the radian
  // yaw error has to be converted to degrees here - the same 57.29578 factor
  // the X/Y terms above already apply.
  gyroBias[2] -= (FUSION_MATH_TYPE)yawKi * delta_yaw_rad * final_mag_gain * correction_dt * 57.29578;

  // Anti-windup: bound the learned bias. The previous implementation zeroed
  // gyroBias[2] whenever the yaw error changed sign, but near convergence the
  // error oscillates about zero, so the accumulator was wiped on almost every
  // correction and could never settle on the steady-state bias it exists to find.
  for (int i = 0; i < 3; i++) {
    if (gyroBias[i] >  (FUSION_MATH_TYPE)_maxGyroBiasDps) gyroBias[i] =  (FUSION_MATH_TYPE)_maxGyroBiasDps;
    if (gyroBias[i] < -(FUSION_MATH_TYPE)_maxGyroBiasDps) gyroBias[i] = -(FUSION_MATH_TYPE)_maxGyroBiasDps;
  }

  FUSION_MATH_TYPE w_x = final_accel_gain * ex * correction_dt; 
  FUSION_MATH_TYPE w_y = final_accel_gain * ey * correction_dt; 
  FUSION_MATH_TYPE w_z = final_mag_gain * delta_yaw_rad * correction_dt;
  
  FUSION_MATH_TYPE q0_old = q[0], q1_old = q[1], q2_old = q[2], q3_old = q[3];
  q[0] += 0.5 * (-q1_old * w_x - q2_old * w_y - q3_old * w_z); 
  q[1] += 0.5 * (q0_old * w_x + q2_old * w_z - q3_old * w_y);
  q[2] += 0.5 * (q0_old * w_y - q1_old * w_z + q3_old * w_x); 
  q[3] += 0.5 * (q0_old * w_z + q1_old * w_y - q2_old * w_x);
  
  recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]); 
  q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

bool AdvancedTriFusion::update() {
  bool dataProcessed = false;
  
  if (_imu->getFIFOMode() == FIFO_NONE) {
    float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    if (_imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) {
      dataProcessed = true;
      remapAxes(ax_raw, ay_raw, az_raw);
      remapAxes(gx_raw, gy_raw, gz_raw);
      
      lastAx = ax_raw; lastAy = ay_raw; lastAz = az_raw;
      lastGx = gx_raw;  lastGy = gy_raw;  lastGz = gz_raw;
      
      unsigned long nowMicros = micros();
      if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
      FUSION_MATH_TYPE dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
      _lastIntegrationTime = nowMicros;
      if (dt <= 0.0) dt = 0.00001;
      // Numerical guard only - same reasoning as clampSampleDt(), which this
      // path cannot use wholesale: without a FIFO the loop may poll faster than
      // the ODR and re-read the same sample, so a dt BELOW nominal is normal
      // here and must not be rounded up, or that sample gets integrated twice.
      if (dt > 0.05) dt = 0.05;
      
      gyroIntegration((lastGx - gyroBias[0]) * (FUSION_MATH_TYPE)PI/180.0, 
                      (lastGy - gyroBias[1]) * (FUSION_MATH_TYPE)PI/180.0, 
                      (lastGz - gyroBias[2]) * (FUSION_MATH_TYPE)PI/180.0, dt);
      trackUpdateRate(); 
      
      unsigned long now = micros();
      if (now - lastMagCheckTime >= magCheckIntervalUs) {
        lastMagCheckTime = now;
        if (_mag->readData()) {
          applyMagCalibration(_mag->x, _mag->y, _mag->z, lastMx, lastMy, lastMz);
          
          FUSION_MATH_TYPE correction_dt = (now - lastSuccessfulCorrectionTime) / 1000000.0f;
          if (correction_dt > 0.1f || lastSuccessfulCorrectionTime == 0) correction_dt = 0.01f;
          complementaryCorrection(lastAx, lastAy, lastAz, lastMx, lastMy, lastMz, correction_dt);
          lastSuccessfulCorrectionTime = now;
        }
      }
    }
  } else {
    // Ask the driver how many packets are waiting BEFORE draining them. That is
    // the only thing the old two-pass version needed its 3 KB stack buffer for:
    // perfect_dt is total_dt / packetCount, so the count had to be known up
    // front. One FIFO_COUNT read supplies it, and the packets can then be
    // integrated as they stream in - nothing has to be held in RAM.
    uint16_t pending = _imu->availablePackets();
    if (pending == 0) return false;

    unsigned long nowMicros = micros();
    if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
    FUSION_MATH_TYPE total_dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
    if (total_dt <= 0.0) total_dt = 0.00001;

    FUSION_MATH_TYPE perfect_dt = total_dt / (FUSION_MATH_TYPE)pending;

    int hz = _imu->getODRHz();
    FUSION_MATH_TYPE ideal_dt = (hz > 0) ? (1.0 / (FUSION_MATH_TYPE)hz) : 0.05;
    clampSampleDt(perfect_dt, ideal_dt);

    FUSION_MATH_TYPE sumAx = 0, sumAy = 0, sumAz = 0;
    FUSION_MATH_TYPE sumGx = 0, sumGy = 0, sumGz = 0;
    uint16_t processed = 0;

    for (uint16_t i = 0; i < pending; i++) {
        float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        if (!_imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) break;

        remapAxes(ax_raw, ay_raw, az_raw);
        remapAxes(gx_raw, gy_raw, gz_raw);

        FUSION_MATH_TYPE ax = ax_raw;
        FUSION_MATH_TYPE ay = ay_raw;
        FUSION_MATH_TYPE az = az_raw;
        FUSION_MATH_TYPE gx = gx_raw;
        FUSION_MATH_TYPE gy = gy_raw;
        FUSION_MATH_TYPE gz = gz_raw;

        sumAx += ax; sumAy += ay; sumAz += az;
        sumGx += gx; sumGy += gy; sumGz += gz;

        gyroIntegration((gx - gyroBias[0]) * (FUSION_MATH_TYPE)PI/180.0,
                        (gy - gyroBias[1]) * (FUSION_MATH_TYPE)PI/180.0,
                        (gz - gyroBias[2]) * (FUSION_MATH_TYPE)PI/180.0, perfect_dt);
        trackUpdateRate();
        processed++;
    }

    // Only consume the elapsed time if something was actually integrated. If the
    // drain came up empty the interval belongs to the next call, not to nothing.
    if (processed == 0) return false;
    _lastIntegrationTime = nowMicros;
    dataProcessed = true;

    // Publish the mean of the whole batch. Previously lastA* was overwritten
    // every iteration, so getGlobalAcceleration() only ever saw the final
    // packet - at 8kHz ODR read at 50Hz that discarded ~99% of the samples.
    // Averaging keeps every sample and low-passes vibration, which also makes
    // the gravity estimate fed to complementaryCorrection() below steadier.
    {
      FUSION_MATH_TYPE invCount = (FUSION_MATH_TYPE)1.0 / (FUSION_MATH_TYPE)processed;
      lastAx = sumAx * invCount; lastAy = sumAy * invCount; lastAz = sumAz * invCount;
      lastGx = sumGx * invCount; lastGy = sumGy * invCount; lastGz = sumGz * invCount;
    }

    // Aplikace pomalé komplementární korekce (jen jednou za batch pro úsporu výkonu)
    {
      unsigned long now = micros();
      if (now - lastMagCheckTime >= magCheckIntervalUs) {
        lastMagCheckTime = now;
        if (_mag->readData()) {
          applyMagCalibration(_mag->x, _mag->y, _mag->z, lastMx, lastMy, lastMz);
          
          FUSION_MATH_TYPE correction_dt = (now - lastSuccessfulCorrectionTime) / 1000000.0f;
          if (correction_dt > 0.1f || lastSuccessfulCorrectionTime == 0) correction_dt = 0.01f;
          complementaryCorrection(lastAx, lastAy, lastAz, lastMx, lastMy, lastMz, correction_dt);
          lastSuccessfulCorrectionTime = now;
        }
      }
    }
  }
  return dataProcessed;
}