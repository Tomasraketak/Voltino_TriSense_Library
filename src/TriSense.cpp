#include "TriSense.h"

// --- TriSense Implementation ---
TriSense::TriSense() : mag(Wire) {}

bool TriSense::beginAll(TriSenseMode mode, uint8_t spiCsPin) {
  _mode = mode;
  Wire.begin();
  if (!bmp.begin()) return false;
  if (!mag.begin()) return false;

  if (_mode == MODE_I2C) {
    if (!imu.begin(BUS_I2C)) return false;
    imu.setODR(ODR_1KHZ); // I2C limit
  } else {
    if (!imu.begin(BUS_SPI, spiCsPin)) return false;
    imu.setODR(ODR_8KHZ); // SPI Turbo mode
  }

  // Config
  bmp.setOversampling(BMP580_OSR_x2, BMP580_OSR_x2);
  bmp.setODR(BMP580_ODR_240Hz);
  bmp.setPowerMode(BMP580_MODE_NORMAL);
  mag.setODR(100); 
  
  return true;
}

bool TriSense::beginBMP(uint8_t addr) { return bmp.begin(addr); }
bool TriSense::beginMAG() { return mag.begin(); }
bool TriSense::beginIMU(ICM_BUS busType, uint8_t csPin) { return imu.begin(busType, csPin); }

// --- TriSenseFusion Implementation ---
TriSenseFusion::TriSenseFusion(ICM42688P* imu, AK09918C* mag) : _imu(imu), _mag(mag) {}

float TriSenseFusion::gaussianGain(float x, float mu, float sigma) {
  if (sigma == 0.0f) return 0.0f;
  return exp(-pow(x - mu, 2) / (2 * sigma * sigma));
}

void TriSenseFusion::setAccelGaussian(float ref, float sigma) {
  accRef = ref; accSigma = sigma;
}

void TriSenseFusion::setMagGaussian(float ref, float sigma, float tiltSigma) {
  magRef = ref; magSigma = sigma; magTiltSigmaDeg = tiltSigma;
}

void TriSenseFusion::setMagGaussian(float ref, float sigma) {
  magRef = ref; magSigma = sigma;
}

void TriSenseFusion::setMagTiltSigma(float sigmaDeg) {
  magTiltSigmaDeg = sigmaDeg;
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

// NOVÉ: Nastavení intervalu kontroly Mag
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
  
  accelOffset[0] = (float)(sumX / samples) - 0.0f;
  accelOffset[1] = (float)(sumY / samples) - 0.0f;
  accelOffset[2] = (float)(sumZ / samples) - 1.0f; 
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
  float r, p, y;
  getCorrectionAngles(axSum/samples, aySum/samples, azSum/samples, 
                      mxSum/samples, mySum/samples, mzSum/samples, r, p, y);

  float c1 = cos(y*PI/360), s1 = sin(y*PI/360);
  float c2 = cos(p*PI/360), s2 = sin(p*PI/360);
  float c3 = cos(r*PI/360), s3 = sin(r*PI/360);
  q[0] = c1*c2*c3 + s1*s2*s3; q[1] = c1*c2*s3 - s1*s2*c3;
  q[2] = c1*s2*c3 + s1*c2*s3; q[3] = s1*c2*c3 - c1*s2*s3;
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
  float r, p, y;
  quaternionToEuler(r, p, y);
  roll = r * 180.0f/PI;
  pitch = p * 180.0f/PI;
  yaw = y * 180.0f/PI;
  if (yaw < 0) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;
}

void TriSenseFusion::getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw) {
  roll  = atan2(ay, az) * 180.0f / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  float phi = roll * PI/180.0f;
  float theta = pitch * PI/180.0f;
  float by = my * cos(phi) - mz * sin(phi);
  float bx = mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi);
  yaw = atan2(-by, bx) * 180.0f / PI + magneticDeclination;
  if (yaw < 0) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;
}

void TriSenseFusion::gyroIntegration(float gx, float gy, float gz, float dt) {
  float qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
  float qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
  float qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
  float qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

  q[0] += qDot1 * dt;
  q[1] += qDot2 * dt;
  q[2] += qDot3 * dt;
  q[3] += qDot4 * dt;

  float recipNorm = 1.0f / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

// --- Simple Implementation ---
SimpleTriFusion::SimpleTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

bool SimpleTriFusion::update() {
  float ax, ay, az, gx, gy, gz;
  if (!_imu->readFIFO(ax, ay, az, gx, gy, gz)) return false;

  // Aplikace offsetů
  ax -= accelOffset[0]; ay -= accelOffset[1]; az -= accelOffset[2];
  gx -= gyroOffset[0];  gy -= gyroOffset[1];  gz -= gyroOffset[2];

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0) dt = 0.000125f; 
  lastTime = now;

  // Jen integrace gyra
  gyroIntegration(gx * PI/180.0f, gy * PI/180.0f, gz * PI/180.0f, dt);
  return true;
}

// --- Advanced Implementation ---
AdvancedTriFusion::AdvancedTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

void AdvancedTriFusion::complementaryCorrection(float ax, float ay, float az, float mx, float my, float mz, float correction_dt) {
  float totalAccelG = sqrt(ax * ax + ay * ay + az * az);
  // Normalizace vektorů
  float recipNorm = 1.0f / totalAccelG;
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

  float magStrength = sqrt(mx * mx + my * my + mz * mz);
  recipNorm = 1.0f / magStrength;
  float mxNorm = mx * recipNorm; // používáme lokální proměnné pro normalizované hodnoty
  
  // Vypočet error vektoru (cross product estimated vs measured)
  float vx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
  float vy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);

  // Výpočet zisku
  float g_gain_accel_raw = gaussianGain(totalAccelG, accRef, accSigma);
  float g_gain_mag_raw = gaussianGain(magStrength, magRef, magSigma);

  float final_accel_gain = g_gain_accel_raw * maxAccelGain;
  float final_mag_gain = g_gain_mag_raw * maxMagGain;

  // Tilt compensation gain
  float roll_rad, pitch_rad, yaw_rad;
  quaternionToEuler(roll_rad, pitch_rad, yaw_rad);
  float roll_deg = roll_rad * 180.0f / PI;
  float pitch_deg = pitch_rad * 180.0f / PI;
  
  float tilt_gain_roll = gaussianGain(fabs(roll_deg), 0.0f, magTiltSigmaDeg);
  float tilt_gain_pitch = gaussianGain(fabs(pitch_deg), 0.0f, magTiltSigmaDeg);
  final_mag_gain *= tilt_gain_roll * tilt_gain_pitch;

  // Yaw correction
  float roll_corr, pitch_corr, yaw_corr;
  getCorrectionAngles(ax * totalAccelG, ay * totalAccelG, az * totalAccelG, mx, my, mz, roll_corr, pitch_corr, yaw_corr);

  float yaw_deg = yaw_rad * 180.0f / PI;
  if (yaw_deg < 0) yaw_deg += 360.0f;
  if (yaw_deg >= 360.0f) yaw_deg -= 360.0f;
  
  float delta_yaw_deg = yaw_corr - yaw_deg;
  if (delta_yaw_deg > 180.0f) delta_yaw_deg -= 360.0f;
  if (delta_yaw_deg < -180.0f) delta_yaw_deg += 360.0f;
  float delta_yaw_rad = delta_yaw_deg * PI / 180.0f;

  // Bias tracking
  if (lastDeltaYawRad * delta_yaw_rad < 0.0f) {
    gyroBiasZ = 0.0f; // Reset bias on crossing
  }
  lastDeltaYawRad = delta_yaw_rad;
  
  // Aplikujeme vypočítané DT místo fixního
  gyroBiasZ -= yawKi * delta_yaw_rad * final_mag_gain * correction_dt;

  // Apply correction to quaternion
  float w_x = final_accel_gain * ex * correction_dt;
  float w_y = final_accel_gain * ey * correction_dt;
  float w_z = final_mag_gain * delta_yaw_rad * correction_dt;

  float q0_old = q[0], q1_old = q[1], q2_old = q[2], q3_old = q[3];
  q[0] += 0.5f * (-q1_old * w_x - q2_old * w_y - q3_old * w_z);
  q[1] += 0.5f * (q0_old * w_x + q2_old * w_z - q3_old * w_y);
  q[2] += 0.5f * (q0_old * w_y - q1_old * w_z + q3_old * w_x);
  q[3] += 0.5f * (q0_old * w_z + q1_old * w_y - q2_old * w_x);

  recipNorm = 1.0f / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] *= recipNorm; q[1] *= recipNorm; q[2] *= recipNorm; q[3] *= recipNorm;
}

bool AdvancedTriFusion::update() {
  float ax, ay, az, gx, gy, gz;
  if (!_imu->readFIFO(ax, ay, az, gx, gy, gz)) return false;

  // 1. Aplikace offsetů (Raw data uložení + korekce pro výpočet)
  lastAx = ax - accelOffset[0];
  lastAy = ay - accelOffset[1];
  lastAz = az - accelOffset[2];
  
  lastGx = gx - gyroOffset[0];
  lastGy = gy - gyroOffset[1];
  lastGz = gz - gyroOffset[2];

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0) dt = 0.0000625f; // approx 16kHz default safe
  lastTime = now;

  // 2. Integrace Gyra (vždy běží, ODR 8KHz nebo 1KHz)
  float gx_rad = lastGx * PI / 180.0f;
  float gy_rad = lastGy * PI / 180.0f;
  float gz_rad = (lastGz - gyroBiasZ) * PI / 180.0f; 
  
  gyroIntegration(gx_rad, gy_rad, gz_rad, dt);

  // 3. Magnetometr Check Timer (default 0.5ms)
  if (now - lastMagCheckTime >= magCheckIntervalUs) {
     lastMagCheckTime = now; // Reset timeru pro kontrolu

     // Dotaz na senzor: Jsou data?
     if (_mag->readData()) {
        float mx_raw = _mag->x - magHardIron[0];
        float my_raw = _mag->y - magHardIron[1];
        float mz_raw = _mag->z - magHardIron[2];
        
        lastMx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
        lastMy = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
        lastMz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;
        
        // Vypočteme skutečný čas od poslední korekce pro bias integraci
        float correction_dt = (now - lastSuccessfulCorrectionTime) / 1000000.0f;
        if (correction_dt > 0.1f || lastSuccessfulCorrectionTime == 0) correction_dt = 0.01f; // Safety clamp

        complementaryCorrection(lastAx, lastAy, lastAz, lastMx, lastMy, lastMz, correction_dt);
        lastSuccessfulCorrectionTime = now;
     }
  }

  return true;
}