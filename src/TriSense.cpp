// TriSense.cpp
#include "TriSense.h"

// TriSense constructor
TriSense::TriSense() : mag(Wire) {}

// Initialize all sensors at once
bool TriSense::beginAll(TriSenseMode mode, uint8_t spiCsPin) {
  _mode = mode;

  if (_mode == MODE_I2C) {
    Wire.begin();
    if (!bmp.begin()) return false;
    if (!mag.begin()) return false;
    if (!imu.begin(BUS_I2C)) return false;
  } else { // MODE_HYBRID
    Wire.begin();
    if (!bmp.begin()) return false;
    if (!mag.begin()) return false;
    if (!imu.begin(BUS_SPI, spiCsPin)) return false;
  }

  // Default configurations
  bmp.setOversampling(BMP580_OSR_x4, BMP580_OSR_x4);
  bmp.setODR(BMP580_ODR_240Hz);
  bmp.setPowerMode(BMP580_MODE_NORMAL);

  mag.setODR(100);

  imu.setODR(ODR_16KHZ);

  return true;
}

// Initialize BMP individually
bool TriSense::beginBMP(uint8_t addr) {
  return bmp.begin(addr);
}

// Initialize MAG individually
bool TriSense::beginMAG() {
  return mag.begin();
}

// Initialize IMU individually
bool TriSense::beginIMU(ICM_BUS busType, uint8_t csPin) {
  return imu.begin(busType, csPin);
}

// TriSenseFusion base class
TriSenseFusion::TriSenseFusion(ICM42688P* imu, AK09918C* mag) : _imu(imu), _mag(mag) {}

void TriSenseFusion::setGyroOffsets(float ox, float oy, float oz) {
  gyroOffset[0] = ox;
  gyroOffset[1] = oy;
  gyroOffset[2] = oz;
}

void TriSenseFusion::setAccelOffsets(float ox, float oy, float oz) {
  accelOffset[0] = ox;
  accelOffset[1] = oy;
  accelOffset[2] = oz;
}

void TriSenseFusion::setMagHardIron(float ox, float oy, float oz) {
  magHardIron[0] = ox;
  magHardIron[1] = oy;
  magHardIron[2] = oz;
}

void TriSenseFusion::setMagSoftIron(float matrix[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      magSoftIron[i][j] = matrix[i][j];
    }
  }
}

void TriSenseFusion::setDeclination(float deg) {
  declination = deg;
}

void TriSenseFusion::quaternionToEuler(float& roll, float& pitch, float& yaw) {
  float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
  roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
  if (abs(sinp) >= 1.0f)
    pitch = copysign(PI / 2.0f, sinp);
  else
    pitch = asin(sinp);

  float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
  float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
  yaw = atan2(siny_cosp, cosy_cosp);
}

void TriSenseFusion::getOrientationDegrees(float& roll, float& pitch, float& yaw) {
  quaternionToEuler(roll, pitch, yaw);
  roll *= 180.0f / PI;
  pitch *= 180.0f / PI;
  yaw *= 180.0f / PI;
  if (yaw < 0) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;
}

void TriSenseFusion::initOrientation(int samples) {
  // Average accel and mag for initial orientation
  float axSum = 0, aySum = 0, azSum = 0;
  float mxSum = 0, mySum = 0, mzSum = 0;
  int validSamples = 0;

  while (validSamples < samples) {
    float ax, ay, az, gx, gy, gz;
    if (_imu->readFIFO(ax, ay, az, gx, gy, gz) && _mag->readData()) {
      ax -= accelOffset[0];
      ay -= accelOffset[1];
      az -= accelOffset[2];

      axSum += ax;
      aySum += ay;
      azSum += az;

      float mx = _mag->x - magHardIron[0];
      float my = _mag->y - magHardIron[1];
      float mz = _mag->z - magHardIron[2];

      float mxCorr = magSoftIron[0][0] * mx + magSoftIron[0][1] * my + magSoftIron[0][2] * mz;
      float myCorr = magSoftIron[1][0] * mx + magSoftIron[1][1] * my + magSoftIron[1][2] * mz;
      float mzCorr = magSoftIron[2][0] * mx + magSoftIron[2][1] * my + magSoftIron[2][2] * mz;

      mxSum += mxCorr;
      mySum += myCorr;
      mzSum += mzCorr;

      validSamples++;
      delay(5);
    }
  }

  float axAvg = axSum / samples;
  float ayAvg = aySum / samples;
  float azAvg = azSum / samples;
  float mxAvg = mxSum / samples;
  float myAvg = mySum / samples;
  float mzAvg = mzSum / samples;

  float roll0 = atan2(ayAvg, azAvg);
  float pitch0 = atan2(-axAvg, sqrt(ayAvg * ayAvg + azAvg * azAvg));

  float magX = mxAvg * cos(pitch0) + mzAvg * sin(pitch0);
  float magY = mxAvg * sin(roll0) * sin(pitch0) + myAvg * cos(roll0) - mzAvg * sin(roll0) * cos(pitch0);
  float yaw0 = atan2(-magY, magX) + declination * PI / 180.0f;

  float cy = cos(yaw0 * 0.5f);
  float sy = sin(yaw0 * 0.5f);
  float cp = cos(pitch0 * 0.5f);
  float sp = sin(pitch0 * 0.5f);
  float cr = cos(roll0 * 0.5f);
  float sr = sin(roll0 * 0.5f);

  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
}

// SimpleTriFusion
SimpleTriFusion::SimpleTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

bool SimpleTriFusion::update() {
  float ax, ay, az, gx, gy, gz;
  if (!_imu->readFIFO(ax, ay, az, gx, gy, gz)) return false;

  ax -= accelOffset[0];
  ay -= accelOffset[1];
  az -= accelOffset[2];
  gx -= gyroOffset[0];
  gy -= gyroOffset[1];
  gz -= gyroOffset[2];

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0) dt = 0.000125f;
  lastTime = now;

  gx *= PI / 180.0f;
  gy *= PI / 180.0f;
  gz *= PI / 180.0f;

  float half_dt = dt * 0.5f;
  float dq[4];
  dq[0] = -q[1] * gx * half_dt - q[2] * gy * half_dt - q[3] * gz * half_dt;
  dq[1] = q[0] * gx * half_dt + q[2] * gz * half_dt - q[3] * gy * half_dt;
  dq[2] = q[0] * gy * half_dt - q[1] * gz * half_dt + q[3] * gx * half_dt;
  dq[3] = q[0] * gz * half_dt + q[1] * gy * half_dt - q[2] * gx * half_dt;

  q[0] += dq[0];
  q[1] += dq[1];
  q[2] += dq[2];
  q[3] += dq[3];

  float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;

  return true;
}

// AdvancedTriFusion
AdvancedTriFusion::AdvancedTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

void AdvancedTriFusion::setAccelGaussian(float ref, float sigma) {
  accRef = ref;
  accSigma = sigma;
}

void AdvancedTriFusion::setMagGaussian(float ref, float sigma) {
  magRef = ref;
  magSigma = sigma;
}

void AdvancedTriFusion::setMagTiltSigma(float sigmaDeg) {
  magTiltSigmaDeg = sigmaDeg;
}

void AdvancedTriFusion::setYawKi(float ki) {
  yawKi = ki;
}

void AdvancedTriFusion::setMaxGains(float maxAccel, float maxMag) {
  maxAccelGain = maxAccel;
  maxMagGain = maxMag;
}

float AdvancedTriFusion::gaussianGain(float x, float mu, float sigma) {
  if (sigma == 0.0f) return 0.0f;
  float exponent = -0.5f * pow((x - mu) / sigma, 2);
  return exp(exponent);
}

void AdvancedTriFusion::getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw) {
  roll = atan2(ay, az) * 180.0f / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

  float pitchRad = pitch * PI / 180.0f;
  float rollRad = roll * PI / 180.0f;

  float magX = mx * cos(pitchRad) + mz * sin(pitchRad);
  float magY = mx * sin(rollRad) * sin(pitchRad) + my * cos(rollRad) - mz * sin(rollRad) * cos(pitchRad);
  yaw = atan2(-magY, magX) * 180.0f / PI + declination;
  if (yaw < 0) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;
}

void AdvancedTriFusion::gyroIntegration(float gx, float gy, float gz, float dt) {
  float qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
  float qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
  float qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
  float qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

  q[0] += qDot1 * dt;
  q[1] += qDot2 * dt;
  q[2] += qDot3 * dt;
  q[3] += qDot4 * dt;

  float recipNorm = 1.0f / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}

void AdvancedTriFusion::complementaryCorrection(float ax, float ay, float az, float mx, float my, float mz) {
  float totalAccelG = sqrt(ax * ax + ay * ay + az * az);
  float recipNorm = 1.0f / totalAccelG;
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  float magStrength = sqrt(mx * mx + my * my + mz * mz);
  recipNorm = 1.0f / magStrength;
  float mxNorm = mx * recipNorm;
  float myNorm = my * recipNorm;
  float mzNorm = mz * recipNorm;

  float vx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
  float vy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);

  float g_gain_accel_raw = gaussianGain(totalAccelG, accRef, accSigma);
  float g_gain_mag_raw = gaussianGain(magStrength, magRef, magSigma);

  float final_accel_gain = g_gain_accel_raw * maxAccelGain;
  float final_mag_gain = g_gain_mag_raw * maxMagGain;

  float roll_rad, pitch_rad, yaw_rad;
  quaternionToEuler(roll_rad, pitch_rad, yaw_rad);
  float roll_deg = roll_rad * 180.0f / PI;
  float pitch_deg = pitch_rad * 180.0f / PI;
  float tilt_gain_roll = gaussianGain(fabs(roll_deg), 0.0f, magTiltSigmaDeg);
  float tilt_gain_pitch = gaussianGain(fabs(pitch_deg), 0.0f, magTiltSigmaDeg);
  final_mag_gain *= tilt_gain_roll * tilt_gain_pitch;

  float roll_corr, pitch_corr, yaw_corr;
  getCorrectionAngles(ax * totalAccelG, ay * totalAccelG, az * totalAccelG, mx, my, mz, roll_corr, pitch_corr, yaw_corr); // Denormalized for atan2

  float yaw_deg = yaw_rad * 180.0f / PI;
  if (yaw_deg < 0) yaw_deg += 360.0f;
  if (yaw_deg >= 360.0f) yaw_deg -= 360.0f;
  float delta_yaw_deg = yaw_corr - yaw_deg;
  if (delta_yaw_deg > 180.0f) delta_yaw_deg -= 360.0f;
  if (delta_yaw_deg < -180.0f) delta_yaw_deg += 360.0f;
  float delta_yaw_rad = delta_yaw_deg * PI / 180.0f;

  if (lastDeltaYawRad * delta_yaw_rad < 0.0f) {
    gyroBiasZ = 0.0f;
  }
  lastDeltaYawRad = delta_yaw_rad;
  float correction_dt = correctionIntervalUs / 1000000.0f;
  gyroBiasZ -= yawKi * delta_yaw_rad * final_mag_gain * correction_dt;

  float w_x = final_accel_gain * ex * correction_dt;
  float w_y = final_accel_gain * ey * correction_dt;
  float w_z = final_mag_gain * delta_yaw_rad * correction_dt;

  float q0_old = q[0];
  float q1_old = q[1];
  float q2_old = q[2];
  float q3_old = q[3];

  q[0] += 0.5f * (-q1_old * w_x - q2_old * w_y - q3_old * w_z);
  q[1] += 0.5f * (q0_old * w_x + q2_old * w_z - q3_old * w_y);
  q[2] += 0.5f * (q0_old * w_y - q1_old * w_z + q3_old * w_x);
  q[3] += 0.5f * (q0_old * w_z + q1_old * w_y - q2_old * w_x);

  recipNorm = 1.0f / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}

bool AdvancedTriFusion::update() {
  float ax, ay, az, gx, gy, gz;
  if (!_imu->readFIFO(ax, ay, az, gx, gy, gz) || !_mag->readData()) return false;

  ax -= accelOffset[0];
  ay -= accelOffset[1];
  az -= accelOffset[2];
  gx -= gyroOffset[0];
  gy -= gyroOffset[1];
  gz -= gyroOffset[2];

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0) dt = 0.0000625f;
  lastTime = now;

  gx *= PI / 180.0f;
  gy *= PI / 180.0f;
  gz *= PI / 180.0f;
  gz -= gyroBiasZ;

  gyroIntegration(gx, gy, gz, dt);

  float temp_x = _mag->x - magHardIron[0];
  float temp_y = _mag->y - magHardIron[1];
  float temp_z = _mag->z - magHardIron[2];
  float mx = magSoftIron[0][0] * temp_x + magSoftIron[0][1] * temp_y + magSoftIron[0][2] * temp_z;
  float my = magSoftIron[1][0] * temp_x + magSoftIron[1][1] * temp_y + magSoftIron[1][2] * temp_z;
  float mz = magSoftIron[2][0] * temp_x + magSoftIron[2][1] * temp_y + magSoftIron[2][2] * temp_z;

  if (now - lastCorrectionTime >= correctionIntervalUs) {
    complementaryCorrection(ax, ay, az, mx, my, mz);
    lastCorrectionTime = now;
  }

  return true;
}