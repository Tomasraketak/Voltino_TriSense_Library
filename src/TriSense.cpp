#include "TriSense.h"

// --- TriSense Implementation ---
TriSense::TriSense() : mag(Wire) {}

bool TriSense::beginAll(TriSenseMode mode, uint8_t spiCsPin, uint32_t spiFreq) {
  _mode = mode;
  Wire.begin();
  if (!bmp.begin()) return false;
  if (!mag.begin()) return false;

  if (_mode == MODE_I2C) {
    if (!imu.begin(BUS_I2C)) return false;
    imu.setODR(ODR_1KHZ); 
  } else {
    if (!imu.begin(BUS_SPI, spiCsPin, spiFreq)) return false;
    imu.setODR(ODR_8KHZ); 
  }

  bmp.setOversampling(BMP580_OSR_x2, BMP580_OSR_x2);
  bmp.setODR(BMP580_ODR_240Hz);
  bmp.setPowerMode(BMP580_MODE_NORMAL);
  mag.setODR(100); 
  
  return true;
}

bool TriSense::beginBMP(uint8_t addr) { return bmp.begin(addr); }
bool TriSense::beginMAG() { return mag.begin(); }

bool TriSense::beginIMU(ICM_BUS busType, uint8_t csPin, uint32_t spiFreq) { 
  return imu.begin(busType, csPin, spiFreq); 
}

void TriSense::resetHardwareOffsets() {
  imu.resetHardwareOffsets();
}

void TriSense::autoCalibrateGyro(uint16_t samples) {
  imu.autoCalibrateGyro(samples);
}

void TriSense::autoCalibrateAccel() {
  imu.autoCalibrateAccel();
}

// --- TriSenseFusion Implementation ---

TriSenseFusion::TriSenseFusion(ICM42688P* imu, AK09918C* mag) : _imu(imu), _mag(mag) {
  // Inicializace výchozích Gaussian koeficientů
  setAccelGaussian(accRef, accSigma);
  setMagGaussian(magRef, magSigma, magTiltSigmaDeg);
  
  // Default Settings requested by user
  _useGUnits = true;            
  _gravityConstant = 9.8065f;   
  _globalAccelEnabled = false;  
  _downsampleFactor = 1;        
  _sampleCounter = 0;
  
  _sumAx = 0; _sumAy = 0; _sumAz = 0;
  _sumGx = 0; _sumGy = 0; _sumGz = 0;
  _sumMx = 0; _sumMy = 0; _sumMz = 0;
}

// --- New Configuration Methods ---

void TriSenseFusion::useG(bool use) {
    _useGUnits = use;
}

void TriSenseFusion::setG(float gravityValue) {
    _gravityConstant = gravityValue;
}

void TriSenseFusion::enableGlobalAccel(bool enable) {
    _globalAccelEnabled = enable;
}

void TriSenseFusion::setDownsampling(int factor) {
    if (factor < 1) factor = 1;
    _downsampleFactor = factor;
    _sampleCounter = 0; 
    
    // Reset accumulators
    _sumAx = 0; _sumAy = 0; _sumAz = 0;
    _sumGx = 0; _sumGy = 0; _sumGz = 0;
    _sumMx = 0; _sumMy = 0; _sumMz = 0;
}

void TriSenseFusion::setGlobalAccelBias(float x, float y, float z) {
    globalAccelBias[0] = x;
    globalAccelBias[1] = y;
    globalAccelBias[2] = z;
}

// --- Calibration Logic ---

void TriSenseFusion::calibrateStaticGlobalAccel(int samples) {
    if (samples <= 0) samples = 1000;

    // Reset bias
    globalAccelBias[0] = 0;
    globalAccelBias[1] = 0;
    globalAccelBias[2] = 0;

    // Force Global Mode and No Downsampling specifically for calibration calculation
    bool oldState = _globalAccelEnabled;
    _globalAccelEnabled = true;

    int oldDownsample = _downsampleFactor;
    _downsampleFactor = 1;

    double sumX = 0;
    double sumY = 0;
    double sumZ = 0;

    for (int i = 0; i < samples; i++) {
        // Must call the virtual update to get data processed
        update(); 
        
        // Accumulate processed data (ax is now Global Accel in G or m/s^2 depending on useG)
        // Note: Bias calculation depends on current unit setting!
        sumX += ax;
        sumY += ay;
        sumZ += az;
        
        delay(3);
    }

    // Restore settings
    _globalAccelEnabled = oldState;
    _downsampleFactor = oldDownsample;
    _sampleCounter = 0;

    globalAccelBias[0] = (float)(sumX / samples);
    globalAccelBias[1] = (float)(sumY / samples);
    globalAccelBias[2] = (float)(sumZ / samples);
}

// --- Processing Logic ---

void TriSenseFusion::processOutput() {
    float finalAx, finalAy, finalAz;

    if (_globalAccelEnabled) {
        // Transform to Global Frame
        getGlobalAccelerationInternal(finalAx, finalAy, finalAz);
        
        // Remove Gravity (Assuming Z is UP in global frame, remove 1G)
        // Gravity is removed in G units here because getGlobalAccelerationInternal returns G based on lastAx
        finalAz -= 1.0f; 

        // Apply Static Bias Correction (Drift removal)
        finalAx -= globalAccelBias[0];
        finalAy -= globalAccelBias[1];
        finalAz -= globalAccelBias[2];
    } else {
        // Body Frame
        finalAx = lastAx;
        finalAy = lastAy;
        finalAz = lastAz;
    }

    // Accumulate for downsampling
    _sumAx += finalAx;
    _sumAy += finalAy;
    _sumAz += finalAz;
    
    _sumGx += lastGx;
    _sumGy += lastGy;
    _sumGz += lastGz;
    
    _sumMx += lastMx;
    _sumMy += lastMy;
    _sumMz += lastMz;

    _sampleCounter++;

    if (_sampleCounter >= _downsampleFactor) {
        float scale = 1.0f / _downsampleFactor;
        
        ax = _sumAx * scale;
        ay = _sumAy * scale;
        az = _sumAz * scale;
        
        gx = _sumGx * scale;
        gy = _sumGy * scale;
        gz = _sumGz * scale;
        
        mx = _sumMx * scale;
        my = _sumMy * scale;
        mz = _sumMz * scale;
        
        // Unit Conversion
        if (!_useGUnits) {
            ax *= _gravityConstant;
            ay *= _gravityConstant;
            az *= _gravityConstant;
        }

        // Reset
        _sumAx = 0; _sumAy = 0; _sumAz = 0;
        _sumGx = 0; _sumGy = 0; _sumGz = 0;
        _sumMx = 0; _sumMy = 0; _sumMz = 0;
        _sampleCounter = 0;
    }
}


// OPTIMALIZACE: Rychlá verze s předpočítaným koeficientem
inline float TriSenseFusion::gaussianGainOptimized(float diffSq, float coeff) {
    if (coeff == 0.0f) return 0.0f; 
    if (diffSq * coeff > 10.0f) return 0.0f; 
    return expf(-diffSq * coeff);
}

float TriSenseFusion::gaussianGain(float x, float mu, float sigma) {
  if (sigma == 0.0f) return 0.0f;
  float diff = x - mu;
  return expf(-(diff * diff) / (2.0f * sigma * sigma));
}

void TriSenseFusion::setAccelGaussian(float ref, float sigma) {
  accRef = ref; accSigma = sigma;
  if (sigma > 0.0001f) {
      _accGaussCoeff = 1.0f / (2.0f * sigma * sigma);
  } else {
      _accGaussCoeff = 0.0f;
  }
}

void TriSenseFusion::setMagGaussian(float ref, float sigma, float tiltSigma) {
  magRef = ref; magSigma = sigma; magTiltSigmaDeg = tiltSigma;
  if (sigma > 0.0001f) _magGaussCoeff = 1.0f / (2.0f * sigma * sigma);
  else _magGaussCoeff = 0.0f;
  if (tiltSigma > 0.0001f) _tiltGaussCoeff = 1.0f / (2.0f * tiltSigma * tiltSigma);
  else _tiltGaussCoeff = 0.0f;
}

void TriSenseFusion::setMagGaussian(float ref, float sigma) {
  setMagGaussian(ref, sigma, magTiltSigmaDeg);
}

void TriSenseFusion::setMagTiltSigma(float sigmaDeg) {
  magTiltSigmaDeg = sigmaDeg;
  if (sigmaDeg > 0.0001f) _tiltGaussCoeff = 1.0f / (2.0f * sigmaDeg * sigmaDeg);
  else _tiltGaussCoeff = 0.0f;
}

void TriSenseFusion::setMagCalibration(float hard[3], float soft[3][3]) {
  for(int i=0; i<3; i++) magHardIron[i] = hard[i];
  for(int i=0; i<3; i++) for(int j=0; j<3; j++) magSoftIron[i][j] = soft[i][j];
}

void TriSenseFusion::setDeclination(float deg) {
  magneticDeclination = deg;
}

void TriSenseFusion::setGyroOffsets(float x, float y, float z) {
  gyroOffset[0] = x; gyroOffset[1] = y; gyroOffset[2] = z;
}

void TriSenseFusion::setMagHardIron(float x, float y, float z) {
  magHardIron[0] = x; magHardIron[1] = y; magHardIron[2] = z;
}

void TriSenseFusion::setMagSoftIron(float matrix[3][3]) {
  for(int i=0; i<3; i++) for(int j=0; j<3; j++) magSoftIron[i][j] = matrix[i][j];
}

void TriSenseFusion::setYawKi(float ki) {
  yawKi = ki;
}

void TriSenseFusion::setMaxGains(float accelGain, float magGain) {
  maxAccelGain = accelGain;
  maxMagGain = magGain;
}

void TriSenseFusion::setMagCheckInterval(float intervalMs) {
  magCheckIntervalUs = (unsigned long)(intervalMs * 1000.0f);
}

void TriSenseFusion::calibrateAccelStatic(int samples) {
  double sumX=0, sumY=0, sumZ=0;
  float ax, ay, az, gx, gy, gz;
  
  for(int i=0; i<samples; i++) {
    _imu->readFIFO(ax, ay, az, gx, gy, gz);
    sumX += ax; sumY += ay; sumZ += az;
    delay(1);
  }
  
  float invSamples = 1.0f / (float)samples;
  accelOffset[0] = (float)(sumX * invSamples) - 0.0f;
  accelOffset[1] = (float)(sumY * invSamples) - 0.0f;
  accelOffset[2] = (float)(sumZ * invSamples) - 1.0f; 
}

void TriSenseFusion::initOrientation(int samples) {
  float axSum=0, aySum=0, azSum=0, mxSum=0, mySum=0, mzSum=0;
  int count = 0;
  
  while(count < samples) {
     float ax, ay, az, gx, gy, gz;
     if(_imu->readFIFO(ax, ay, az, gx, gy, gz) && _mag->readData()) {
         ax -= accelOffset[0]; ay -= accelOffset[1]; az -= accelOffset[2];
         axSum+=ax; aySum+=ay; azSum+=az;
         
         float mx_raw = _mag->x - magHardIron[0];
         float my_raw = _mag->y - magHardIron[1];
         float mz_raw = _mag->z - magHardIron[2];
         float mx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
         float my = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
         float mz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;

         mxSum+=mx; mySum+=my; mzSum+=mz;
         count++;
         delay(2);
     }
  }
  
  float invSamples = 1.0f / (float)samples;
  float axAvg = axSum * invSamples;
  float ayAvg = aySum * invSamples;
  float azAvg = azSum * invSamples;
  float mxAvg = mxSum * invSamples;
  float myAvg = mySum * invSamples;
  float mzAvg = mzSum * invSamples;

  float r, p, y;
  getCorrectionAngles(axAvg, ayAvg, azAvg, mxAvg, myAvg, mzAvg, r, p, y);

  float y_rad_half = y * (PI_F / 360.0f);
  float p_rad_half = p * (PI_F / 360.0f);
  float r_rad_half = r * (PI_F / 360.0f);

  float c1 = cosf(y_rad_half), s1 = sinf(y_rad_half);
  float c2 = cosf(p_rad_half), s2 = sinf(p_rad_half);
  float c3 = cosf(r_rad_half), s3 = sinf(r_rad_half);

  q[0] = c1*c2*c3 + s1*s2*s3; 
  q[1] = c1*c2*s3 - s1*s2*c3;
  q[2] = c1*s2*c3 + s1*c2*s3; 
  q[3] = s1*c2*c3 - c1*s2*s3;
}

void TriSenseFusion::quaternionToEuler(float& roll, float& pitch, float& yaw) {
  float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
  roll = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
  if (fabsf(sinp) >= 1.0f)
    pitch = copysignf(PI_F / 2.0f, sinp);
  else
    pitch = asinf(sinp);

  float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
  float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
  yaw = atan2f(siny_cosp, cosy_cosp);
}

void TriSenseFusion::getOrientationDegrees(float& roll, float& pitch, float& yaw) {
  float r, p, y;
  quaternionToEuler(r, p, y);
  roll = r * RAD_TO_DEG_F;
  pitch = p * RAD_TO_DEG_F;
  yaw = y * RAD_TO_DEG_F;
  
  if (yaw < 0.0f) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;
}

void TriSenseFusion::getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw) {
  roll  = atan2f(ay, az) * RAD_TO_DEG_F;
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;
  
  float phi = roll * DEG_TO_RAD_F;
  float theta = pitch * DEG_TO_RAD_F;
  
  float c_phi = cosf(phi);
  float s_phi = sinf(phi);
  float c_theta = cosf(theta);
  float s_theta = sinf(theta);
  
  float by = my * c_phi - mz * s_phi;
  float bx = mx * c_theta + my * s_theta * s_phi + mz * s_theta * c_phi;
  
  yaw = atan2f(-by, bx) * RAD_TO_DEG_F + magneticDeclination;
  
  if (yaw < 0.0f) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;
}

// Helper function used by processOutput
void TriSenseFusion::getGlobalAccelerationInternal(float& ax_g, float& ay_g, float& az_g) {
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

  float x2 = qx + qx; 
  float y2 = qy + qy; 
  float z2 = qz + qz;
  
  float xx = qx * x2; 
  float xy = qx * y2; 
  float xz = qx * z2;
  
  float yy = qy * y2; 
  float yz = qy * z2; 
  float zz = qz * z2;
  
  float wx = qw * x2; 
  float wy = qw * y2; 
  float wz = qw * z2;

  ax_g = (1.0f - (yy + zz)) * lastAx + (xy - wz) * lastAy + (xz + wy) * lastAz;
  ay_g = (xy + wz) * lastAx + (1.0f - (xx + zz)) * lastAy + (yz - wx) * lastAz;
  az_g = (xz - wy) * lastAx + (yz + wx) * lastAy + (1.0f - (xx + yy)) * lastAz;
}

void TriSenseFusion::gyroIntegration(float gx, float gy, float gz, float dt) {
  float halfDt = 0.5f * dt; 
  
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

  float qDot1 = (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = ( q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = ( q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = ( q0 * gz + q1 * gy - q2 * gx);

  q[0] += qDot1 * halfDt;
  q[1] += qDot2 * halfDt;
  q[2] += qDot3 * halfDt;
  q[3] += qDot4 * halfDt;

  float normSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  
  if (normSq > 0.0f) {
      float recipNorm = 1.0f / sqrtf(normSq);
      q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
  }
}

SimpleTriFusion::SimpleTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

bool SimpleTriFusion::update() {
  float ax, ay, az, gx, gy, gz;
  if (!_imu->readFIFO(ax, ay, az, gx, gy, gz)) return false;

  ax -= accelOffset[0]; ay -= accelOffset[1]; az -= accelOffset[2];
  gx -= gyroOffset[0];  gy -= gyroOffset[1];  gz -= gyroOffset[2];

  unsigned long now = micros();
  float dt = (float)(now - lastTime) * 1.0e-6f; 
  if (dt <= 0.0f) dt = 0.000125f; 
  lastTime = now;

  float gx_rad = gx * DEG_TO_RAD_F;
  float gy_rad = gy * DEG_TO_RAD_F;
  float gz_rad = gz * DEG_TO_RAD_F;
  
  gyroIntegration(gx_rad, gy_rad, gz_rad, dt);
  
  lastAx = ax; lastAy = ay; lastAz = az;
  lastGx = gx; lastGy = gy; lastGz = gz;

  // Final step: Process for Output (Downsample, Global, Units)
  processOutput();

  return true;
}

AdvancedTriFusion::AdvancedTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

void AdvancedTriFusion::complementaryCorrection(float ax, float ay, float az, float mx, float my, float mz, float correction_dt) {
  float totalAccelGSq = ax * ax + ay * ay + az * az;
  float totalAccelG = sqrtf(totalAccelGSq);
  
  if (totalAccelG > 0.0f) {
      float recipNorm = 1.0f / totalAccelG;
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
  }

  float magStrengthSq = mx * mx + my * my + mz * mz;
  float magStrength = sqrtf(magStrengthSq);

  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);

  float accDiff = totalAccelG - accRef;
  float g_gain_accel_raw = gaussianGainOptimized(accDiff * accDiff, _accGaussCoeff);

  float magDiff = magStrength - magRef;
  float g_gain_mag_raw = gaussianGainOptimized(magDiff * magDiff, _magGaussCoeff);

  float final_accel_gain = g_gain_accel_raw * maxAccelGain;
  float final_mag_gain = g_gain_mag_raw * maxMagGain;

  float roll_rad, pitch_rad, yaw_rad;
  quaternionToEuler(roll_rad, pitch_rad, yaw_rad);
  
  float roll_deg = roll_rad * RAD_TO_DEG_F;
  float pitch_deg = pitch_rad * RAD_TO_DEG_F;
  
  float tilt_gain_roll = gaussianGainOptimized(roll_deg * roll_deg, _tiltGaussCoeff);
  float tilt_gain_pitch = gaussianGainOptimized(pitch_deg * pitch_deg, _tiltGaussCoeff);
  
  final_mag_gain *= tilt_gain_roll * tilt_gain_pitch;

  float roll_corr, pitch_corr, yaw_corr;
  getCorrectionAngles(ax * totalAccelG, ay * totalAccelG, az * totalAccelG, mx, my, mz, roll_corr, pitch_corr, yaw_corr);

  float yaw_deg = yaw_rad * RAD_TO_DEG_F;
  if (yaw_deg < 0.0f) yaw_deg += 360.0f;
  if (yaw_deg >= 360.0f) yaw_deg -= 360.0f;
  
  float delta_yaw_deg = yaw_corr - yaw_deg;
  if (delta_yaw_deg > 180.0f) delta_yaw_deg -= 360.0f;
  if (delta_yaw_deg < -180.0f) delta_yaw_deg += 360.0f;
  
  float delta_yaw_rad = delta_yaw_deg * DEG_TO_RAD_F;

  if (lastDeltaYawRad * delta_yaw_rad < 0.0f) {
    gyroBiasZ = 0.0f;
  }
  lastDeltaYawRad = delta_yaw_rad;
  
  gyroBiasZ -= yawKi * delta_yaw_rad * final_mag_gain * correction_dt;

  float w_x = final_accel_gain * ex * correction_dt;
  float w_y = final_accel_gain * ey * correction_dt;
  float w_z = final_mag_gain * delta_yaw_rad * correction_dt;

  float half_wx = 0.5f * w_x;
  float half_wy = 0.5f * w_y;
  float half_wz = 0.5f * w_z;

  float q0_new = q0 + (-q1 * half_wx - q2 * half_wy - q3 * half_wz);
  float q1_new = q1 + ( q0 * half_wx + q2 * half_wz - q3 * half_wy);
  float q2_new = q2 + ( q0 * half_wy - q1 * half_wz + q3 * half_wx);
  float q3_new = q3 + ( q0 * half_wz + q1 * half_wy - q2 * half_wx);

  q[0] = q0_new; q[1] = q1_new; q[2] = q2_new; q[3] = q3_new;

  float normSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  if (normSq > 0.0f) {
     float recipNorm = 1.0f / sqrtf(normSq);
     q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
  }
}

bool AdvancedTriFusion::update() {
  float ax, ay, az, gx, gy, gz;
  if (!_imu->readFIFO(ax, ay, az, gx, gy, gz)) return false;

  lastAx = ax - accelOffset[0];
  lastAy = ay - accelOffset[1];
  lastAz = az - accelOffset[2];
  
  lastGx = gx - gyroOffset[0];
  lastGy = gy - gyroOffset[1];
  lastGz = gz - gyroOffset[2];

  unsigned long now = micros();
  float dt = (float)(now - lastTime) * 1.0e-6f;
  if (dt <= 0.0f) dt = 0.0000625f; 
  lastTime = now;

  float gx_rad = lastGx * DEG_TO_RAD_F;
  float gy_rad = lastGy * DEG_TO_RAD_F;
  float gz_rad = (lastGz - gyroBiasZ) * DEG_TO_RAD_F; 
  
  gyroIntegration(gx_rad, gy_rad, gz_rad, dt);

  if (now - lastMagCheckTime >= magCheckIntervalUs) {
     lastMagCheckTime = now;
     if (_mag->readData()) {
        float mx_raw = _mag->x - magHardIron[0];
        float my_raw = _mag->y - magHardIron[1];
        float mz_raw = _mag->z - magHardIron[2];
        
        lastMx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
        lastMy = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
        lastMz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;
        
        float correction_dt = (float)(now - lastSuccessfulCorrectionTime) * 1.0e-6f;
        if (correction_dt > 0.1f || lastSuccessfulCorrectionTime == 0) correction_dt = 0.01f;

        complementaryCorrection(lastAx, lastAy, lastAz, lastMx, lastMy, lastMz, correction_dt);
        lastSuccessfulCorrectionTime = now;
     }
  }

  // Final step: Process for Output (Downsample, Global, Units)
  processOutput();

  return true;
}