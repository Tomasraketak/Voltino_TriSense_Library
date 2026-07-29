#include "TriSense.h"

TriSense::TriSense() {}

bool TriSense::beginAll(TriSenseMode mode, uint8_t spiCsPin, uint32_t spiFreq) {
  _mode = mode;
  Wire.begin();
  if (!bmp.begin()) return false;
  if (!mag.begin()) return false;

  if (_mode == MODE_I2C) {
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

bool TriSense::beginBMP(uint8_t addr) { return bmp.begin(addr); }
bool TriSense::beginMAG() { return mag.begin(); }
bool TriSense::beginIMU(ICM_BUS busType, uint8_t csPin, uint32_t freq) { return imu.begin(busType, csPin, freq); }

void TriSense::resetHardwareOffsets() { imu.resetHardwareOffsets(); }
void TriSense::autoCalibrateGyro(uint16_t samples) { imu.autoCalibrateGyro(samples); }
void TriSense::autoCalibrateAccel() { imu.autoCalibrateAccel(); }

bool TriSense::getSnapshot(TriSenseDataSnapshot &data) {
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  bool imuOk = imu.readFIFO(ax, ay, az, gx, gy, gz);
  data.accelX = ax; data.accelY = ay; data.accelZ = az;
  data.gyroX = gx;  data.gyroY = gy;  data.gyroZ = gz;

  bool magOk = mag.readData();
  data.magX = mag.x; data.magY = mag.y; data.magZ = mag.z;

  data.pressure = bmp.readPressure();
  data.temperature = bmp.readTemperature();

  return imuOk && magOk;
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

FUSION_MATH_TYPE TriSenseFusion::invSqrt(FUSION_MATH_TYPE x) {
#if defined(FORCE_FUSION_DOUBLE)
  return 1.0 / sqrt(x);
#else
  float xhalf = 0.5f * (float)x;
  uint32_t i = *(uint32_t*)&x;
  i = 0x5f3759df - (i >> 1);
  float y = *(float*)&i;
  y = y * (1.5f - xhalf * y * y);
  return (FUSION_MATH_TYPE)y;
#endif
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
void TriSenseFusion::setGyroOffsets(float x, float y, float z) { gyroOffset[0] = x; gyroOffset[1] = y; gyroOffset[2] = z; }
void TriSenseFusion::setMagHardIron(float x, float y, float z) { magHardIron[0] = x; magHardIron[1] = y; magHardIron[2] = z; }
void TriSenseFusion::setMagSoftIron(float matrix[3][3]) { for(int i=0; i<3; i++) for(int j=0; j<3; j++) magSoftIron[i][j] = matrix[i][j]; }
void TriSenseFusion::setYawKi(float ki) { yawKi = ki; }
void TriSenseFusion::setMaxGains(float accelGain, float magGain) { maxAccelGain = accelGain; maxMagGain = magGain; }
void TriSenseFusion::setMagCheckInterval(float intervalMs) { magCheckIntervalUs = (unsigned long)(intervalMs * 1000.0f); }
void TriSenseFusion::setLocalGravity(float g) { _localGravity = g; }

void TriSenseFusion::calibrateAccelStatic(int samples) {
  double sumX=0, sumY=0, sumZ=0; 
  int count = 0;
  
  if (_imu->getFIFOMode() != FIFO_NONE) {
      float ax, ay, az, gx, gy, gz;
      while(_imu->readFIFO(ax, ay, az, gx, gy, gz));
  }

  while(count < samples) { 
    float ax, ay, az, gx, gy, gz;
    if (_imu->readFIFO(ax, ay, az, gx, gy, gz)) {
      remapAxes(ax, ay, az);
      sumX += ax; sumY += ay; sumZ += az; 
      count++;
    } else {
      delay(1); 
    }
  }
  
  float avgX = (float)(sumX / samples);
  float avgY = (float)(sumY / samples);
  float avgZ = (float)(sumZ / samples);
  
  if (abs(avgZ) > 0.7f) {
    accelOffset[0] = avgX; accelOffset[1] = avgY;
    accelOffset[2] = (avgZ > 0) ? (avgZ - 1.0f) : (avgZ + 1.0f);
  } else if (abs(avgX) > 0.7f) {
    accelOffset[0] = (avgX > 0) ? (avgX - 1.0f) : (avgX + 1.0f);
    accelOffset[1] = avgY; accelOffset[2] = avgZ;
  } else if (abs(avgY) > 0.7f) {
    accelOffset[0] = avgX;
    accelOffset[1] = (avgY > 0) ? (avgY - 1.0f) : (avgY + 1.0f);
    accelOffset[2] = avgZ;
  } else {
    accelOffset[0] = avgX; accelOffset[1] = avgY; accelOffset[2] = avgZ - 1.0f; 
  }
}

void TriSenseFusion::initOrientation(int samples) {
  FUSION_MATH_TYPE axSum=0, aySum=0, azSum=0, mxSum=0, mySum=0, mzSum=0; 
  int count = 0;
  
  if (_imu->getFIFOMode() != FIFO_NONE) {
      float ax, ay, az, gx, gy, gz;
      while(_imu->readFIFO(ax, ay, az, gx, gy, gz));
  }

  while(count < samples) {
     float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
     
     bool imuReady = _imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
     _mag->readData(); 

     if(imuReady) {
         remapAxes(ax_raw, ay_raw, az_raw);
         FUSION_MATH_TYPE ax = ax_raw - accelOffset[0]; 
         FUSION_MATH_TYPE ay = ay_raw - accelOffset[1]; 
         FUSION_MATH_TYPE az = az_raw - accelOffset[2]; 
         axSum+=ax; aySum+=ay; azSum+=az;
         
         float mxr = _mag->x, myr = _mag->y, mzr = _mag->z;
         remapAxes(mxr, myr, mzr);
         
         FUSION_MATH_TYPE mx_raw = mxr - magHardIron[0]; 
         FUSION_MATH_TYPE my_raw = myr - magHardIron[1]; 
         FUSION_MATH_TYPE mz_raw = mzr - magHardIron[2];
         
         FUSION_MATH_TYPE mx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
         FUSION_MATH_TYPE my = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
         FUSION_MATH_TYPE mz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;
         
         mxSum+=mx; mySum+=my; mzSum+=mz; 
         count++; 
     } else {
         delay(1); 
     }
  }
  
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

  int nominalHz = _imu->getODRHz();
  _realDt = (nominalHz > 0) ? (1.0 / (FUSION_MATH_TYPE)nominalHz) : 0.001;
  _lastIntegrationTime = micros(); 
  _sampleCount = 0;
}

void TriSenseFusion::quaternionToEuler(FUSION_MATH_TYPE& roll, FUSION_MATH_TYPE& pitch, FUSION_MATH_TYPE& yaw) {
  FUSION_MATH_TYPE sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]); 
  FUSION_MATH_TYPE cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]); 
  roll = atan2(sinr_cosp, cosr_cosp);
  
  FUSION_MATH_TYPE sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]); 
  if (abs(sinp) >= 1.0) pitch = copysign((FUSION_MATH_TYPE)PI / 2.0, sinp); else pitch = asin(sinp);
  
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
      
      lastAx = ax_raw - accelOffset[0]; lastAy = ay_raw - accelOffset[1]; lastAz = az_raw - accelOffset[2]; 
      lastGx = gx_raw - gyroOffset[0];  lastGy = gy_raw - gyroOffset[1];  lastGz = gz_raw - gyroOffset[2];
      
      unsigned long nowMicros = micros();
      if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
      FUSION_MATH_TYPE dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
      _lastIntegrationTime = nowMicros;
      if (dt <= 0.0) dt = 0.00001;
      if (dt > 0.1) dt = 1.0 / (FUSION_MATH_TYPE)(_imu->getODRHz() > 0 ? _imu->getODRHz() : 1000);
      
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
    // === OPRAVA: POUZE VYČTU BUFFER ===
    #if defined(__AVR__)
      const int max_packets = 16;  
    #else
      const int max_packets = 128; 
    #endif

    struct FIFOPacket { float ax, ay, az, gx, gy, gz; };
    FIFOPacket buffer[max_packets];
    int packetCount = 0;

    while (packetCount < max_packets) {
        float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        if (!_imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) break; 
        buffer[packetCount].ax = ax_raw; buffer[packetCount].ay = ay_raw; buffer[packetCount].az = az_raw;
        buffer[packetCount].gx = gx_raw; buffer[packetCount].gy = gy_raw; buffer[packetCount].gz = gz_raw;
        packetCount++;
    }

    if (packetCount > 0) {
        dataProcessed = true;
        
        // === OPRAVA: MĚŘENÍ ČASU POUZE KDYŽ MÁME DATA ===
        unsigned long nowMicros = micros();
        if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
        FUSION_MATH_TYPE total_dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
        _lastIntegrationTime = nowMicros; // Fix here!
        
        if (total_dt <= 0.0) total_dt = 0.00001;
        FUSION_MATH_TYPE perfect_dt = total_dt / (FUSION_MATH_TYPE)packetCount;

        // Bezpečnostní mantinely pro DT v případě záseku mikrokontroléru
        int hz = _imu->getODRHz();
        FUSION_MATH_TYPE ideal_dt = (hz > 0) ? (1.0 / (FUSION_MATH_TYPE)hz) : 0.001;
        if (perfect_dt > ideal_dt * 2.0) perfect_dt = ideal_dt;
        if (perfect_dt < ideal_dt * 0.5) perfect_dt = ideal_dt;

        // Accumulate every packet so lastA*/lastG* can publish the batch mean
        // instead of only the final sample (see note after the loop).
        FUSION_MATH_TYPE sumAx = 0, sumAy = 0, sumAz = 0;
        FUSION_MATH_TYPE sumGx = 0, sumGy = 0, sumGz = 0;

        for(int i = 0; i < packetCount; i++) {
            float ax_raw = buffer[i].ax, ay_raw = buffer[i].ay, az_raw = buffer[i].az;
            float gx_raw = buffer[i].gx, gy_raw = buffer[i].gy, gz_raw = buffer[i].gz;

            remapAxes(ax_raw, ay_raw, az_raw);
            remapAxes(gx_raw, gy_raw, gz_raw);

            FUSION_MATH_TYPE ax = ax_raw - accelOffset[0];
            FUSION_MATH_TYPE ay = ay_raw - accelOffset[1];
            FUSION_MATH_TYPE az = az_raw - accelOffset[2];
            FUSION_MATH_TYPE gx = gx_raw - gyroOffset[0];
            FUSION_MATH_TYPE gy = gy_raw - gyroOffset[1];
            FUSION_MATH_TYPE gz = gz_raw - gyroOffset[2];

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
        }

        // Publish the mean of the whole batch rather than just the final packet,
        // so getGlobalAcceleration() sees every accelerometer sample the FIFO
        // delivered. The per-sample values above still drive the integration.
        FUSION_MATH_TYPE invCount = (FUSION_MATH_TYPE)1.0 / (FUSION_MATH_TYPE)packetCount;
        lastAx = sumAx * invCount; lastAy = sumAy * invCount; lastAz = sumAz * invCount;
        lastGx = sumGx * invCount; lastGy = sumGy * invCount; lastGz = sumGz * invCount;
    }
  }
  return dataProcessed;
}

AdvancedTriFusion::AdvancedTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

void AdvancedTriFusion::complementaryCorrection(FUSION_MATH_TYPE ax, FUSION_MATH_TYPE ay, FUSION_MATH_TYPE az, 
                                                FUSION_MATH_TYPE mx, FUSION_MATH_TYPE my, FUSION_MATH_TYPE mz, 
                                                FUSION_MATH_TYPE correction_dt) {
  FUSION_MATH_TYPE totalAccelG = sqrt(ax * ax + ay * ay + az * az); 
  FUSION_MATH_TYPE recipNorm = invSqrt(ax * ax + ay * ay + az * az); 
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
  
  FUSION_MATH_TYPE roll_corr, pitch_corr, yaw_corr; 
  getCorrectionAngles(ax * totalAccelG, ay * totalAccelG, az * totalAccelG, mx, my, mz, roll_corr, pitch_corr, yaw_corr);
  
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
      
      lastAx = ax_raw - accelOffset[0]; lastAy = ay_raw - accelOffset[1]; lastAz = az_raw - accelOffset[2];
      lastGx = gx_raw - gyroOffset[0];  lastGy = gy_raw - gyroOffset[1];  lastGz = gz_raw - gyroOffset[2];
      
      unsigned long nowMicros = micros();
      if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
      FUSION_MATH_TYPE dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
      _lastIntegrationTime = nowMicros;
      if (dt <= 0.0) dt = 0.00001;
      if (dt > 0.1) dt = 1.0 / (FUSION_MATH_TYPE)(_imu->getODRHz() > 0 ? _imu->getODRHz() : 1000);
      
      gyroIntegration((lastGx - gyroBias[0]) * (FUSION_MATH_TYPE)PI/180.0, 
                      (lastGy - gyroBias[1]) * (FUSION_MATH_TYPE)PI/180.0, 
                      (lastGz - gyroBias[2]) * (FUSION_MATH_TYPE)PI/180.0, dt);
      trackUpdateRate(); 
      
      unsigned long now = micros();
      if (now - lastMagCheckTime >= magCheckIntervalUs) {
        lastMagCheckTime = now;
        if (_mag->readData()) {
          float mxr = _mag->x, myr = _mag->y, mzr = _mag->z;
          remapAxes(mxr, myr, mzr);
          FUSION_MATH_TYPE mx_raw = mxr - magHardIron[0]; 
          FUSION_MATH_TYPE my_raw = myr - magHardIron[1]; 
          FUSION_MATH_TYPE mz_raw = mzr - magHardIron[2];
          lastMx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
          lastMy = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
          lastMz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;
          
          FUSION_MATH_TYPE correction_dt = (now - lastSuccessfulCorrectionTime) / 1000000.0f;
          if (correction_dt > 0.1f || lastSuccessfulCorrectionTime == 0) correction_dt = 0.01f;
          complementaryCorrection(lastAx, lastAy, lastAz, lastMx, lastMy, lastMz, correction_dt);
          lastSuccessfulCorrectionTime = now;
        }
      }
    }
  } else {
    // === OPRAVA: POUZE VYČTU BUFFER ===
    #if defined(__AVR__)
      const int max_packets = 16;
    #else
      const int max_packets = 128;
    #endif

    struct FIFOPacket { float ax, ay, az, gx, gy, gz; };
    FIFOPacket buffer[max_packets];
    int packetCount = 0;

    while (packetCount < max_packets) {
        float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        if (!_imu->readFIFO(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) break; 
        buffer[packetCount].ax = ax_raw; buffer[packetCount].ay = ay_raw; buffer[packetCount].az = az_raw;
        buffer[packetCount].gx = gx_raw; buffer[packetCount].gy = gy_raw; buffer[packetCount].gz = gz_raw;
        packetCount++;
    }
    
    if (packetCount > 0) {
      dataProcessed = true;
      
      // === OPRAVA: MĚŘENÍ ČASU POUZE KDYŽ MÁME DATA ===
      unsigned long nowMicros = micros();
      if (_lastIntegrationTime == 0) _lastIntegrationTime = nowMicros;
      FUSION_MATH_TYPE total_dt = (nowMicros - _lastIntegrationTime) / 1000000.0;
      _lastIntegrationTime = nowMicros; // Fix here!
      
      if (total_dt <= 0.0) total_dt = 0.00001;
      FUSION_MATH_TYPE perfect_dt = total_dt / (FUSION_MATH_TYPE)packetCount;

      // Bezpečnostní mantinely
      int hz = _imu->getODRHz();
      FUSION_MATH_TYPE ideal_dt = (hz > 0) ? (1.0 / (FUSION_MATH_TYPE)hz) : 0.05;
      if (perfect_dt > ideal_dt * 2.0) perfect_dt = ideal_dt;
      if (perfect_dt < ideal_dt * 0.5) perfect_dt = ideal_dt;

      // Accumulate every packet so lastA*/lastG* can publish the batch mean
      // instead of only the final sample (see note after the loop).
      FUSION_MATH_TYPE sumAx = 0, sumAy = 0, sumAz = 0;
      FUSION_MATH_TYPE sumGx = 0, sumGy = 0, sumGz = 0;

      for(int i = 0; i < packetCount; i++) {
          float ax_raw = buffer[i].ax, ay_raw = buffer[i].ay, az_raw = buffer[i].az;
          float gx_raw = buffer[i].gx, gy_raw = buffer[i].gy, gz_raw = buffer[i].gz;

          remapAxes(ax_raw, ay_raw, az_raw);
          remapAxes(gx_raw, gy_raw, gz_raw);

          FUSION_MATH_TYPE ax = ax_raw - accelOffset[0];
          FUSION_MATH_TYPE ay = ay_raw - accelOffset[1];
          FUSION_MATH_TYPE az = az_raw - accelOffset[2];
          FUSION_MATH_TYPE gx = gx_raw - gyroOffset[0];
          FUSION_MATH_TYPE gy = gy_raw - gyroOffset[1];
          FUSION_MATH_TYPE gz = gz_raw - gyroOffset[2];

          sumAx += ax; sumAy += ay; sumAz += az;
          sumGx += gx; sumGy += gy; sumGz += gz;

          gyroIntegration((gx - gyroBias[0]) * (FUSION_MATH_TYPE)PI/180.0,
                          (gy - gyroBias[1]) * (FUSION_MATH_TYPE)PI/180.0,
                          (gz - gyroBias[2]) * (FUSION_MATH_TYPE)PI/180.0, perfect_dt);
          trackUpdateRate();
      }

      // Publish the mean of the whole batch. Previously lastA* was overwritten
      // every iteration, so getGlobalAcceleration() only ever saw the final
      // packet - at 8kHz ODR read at 50Hz that discarded ~99% of the samples.
      // Averaging keeps every sample and low-passes vibration, which also makes
      // the gravity estimate fed to complementaryCorrection() below steadier.
      FUSION_MATH_TYPE invCount = (FUSION_MATH_TYPE)1.0 / (FUSION_MATH_TYPE)packetCount;
      lastAx = sumAx * invCount; lastAy = sumAy * invCount; lastAz = sumAz * invCount;
      lastGx = sumGx * invCount; lastGy = sumGy * invCount; lastGz = sumGz * invCount;

      // Aplikace pomalé komplementární korekce (jen jednou za batch pro úsporu výkonu)
      unsigned long now = micros();
      if (now - lastMagCheckTime >= magCheckIntervalUs) {
        lastMagCheckTime = now;
        if (_mag->readData()) {
          float mxr = _mag->x, myr = _mag->y, mzr = _mag->z;
          remapAxes(mxr, myr, mzr);
          FUSION_MATH_TYPE mx_raw = mxr - magHardIron[0]; 
          FUSION_MATH_TYPE my_raw = myr - magHardIron[1]; 
          FUSION_MATH_TYPE mz_raw = mzr - magHardIron[2];
          lastMx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
          lastMy = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
          lastMz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;
          
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