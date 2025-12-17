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
  } else {
    if (!imu.begin(BUS_SPI, spiCsPin)) return false;
  }

  // Config
  bmp.setOversampling(BMP580_OSR_x2, BMP580_OSR_x2);
  bmp.setODR(BMP580_ODR_240Hz);
  bmp.setPowerMode(BMP580_MODE_NORMAL);
  mag.setODR(100); 
  imu.setODR(ODR_1KHZ); 
  return true;
}

// Původní metody zachovány
bool TriSense::beginBMP(uint8_t addr) { return bmp.begin(addr); }
bool TriSense::beginMAG() { return mag.begin(); }
bool TriSense::beginIMU(ICM_BUS busType, uint8_t csPin) { return imu.begin(busType, csPin); }

// --- TriSenseFusion Implementation ---
TriSenseFusion::TriSenseFusion(ICM42688P* imu, AK09918C* mag) : _imu(imu), _mag(mag) {}

float TriSenseFusion::gaussianGain(float x, float mu, float sigma) {
  return exp(-pow(x - mu, 2) / (2 * sigma * sigma));
}

void TriSenseFusion::setAccelGaussian(float ref, float sigma) {
  accRef = ref; accSigma = sigma;
}

void TriSenseFusion::setMagGaussian(float ref, float sigma, float tiltSigma) {
  magRef = ref; magSigma = sigma; magTiltSigmaDeg = tiltSigma;
}

void TriSenseFusion::setMagCalibration(float hard[3], float soft[3][3]) {
  for(int i=0; i<3; i++) magHardIron[i] = hard[i];
  for(int i=0; i<3; i++) for(int j=0; j<3; j++) magSoftIron[i][j] = soft[i][j];
}

void TriSenseFusion::setDeclination(float deg) {
  magneticDeclination = deg;
}

// Nová funkce pro kalibraci akcelerometru
void TriSenseFusion::calibrateAccelStatic(int samples) {
  double sumX=0, sumY=0, sumZ=0;
  float ax, ay, az, gx, gy, gz;
  
  for(int i=0; i<samples; i++) {
    _imu->readFIFO(ax, ay, az, gx, gy, gz);
    sumX += ax; sumY += ay; sumZ += az;
    delay(1);
  }
  
  // Předpoklad: Raketa stojí svisle (Z = 1G, X=0, Y=0)
  accelOffset[0] = (float)(sumX / samples) - 0.0f;
  accelOffset[1] = (float)(sumY / samples) - 0.0f;
  accelOffset[2] = (float)(sumZ / samples) - 1.0f; // Chceme aby Z ukazovalo 1G
}

void TriSenseFusion::initOrientation(int samples) {
  // Stejná implementace jako minule, jen používá public proměnné
  float axSum=0, aySum=0, azSum=0, mxSum=0, mySum=0, mzSum=0;
  for(int i=0; i<samples; i++) {
     update(); // Použijeme update, aby se aplikovaly offsety
     axSum+=lastAx; aySum+=lastAy; azSum+=lastAz;
     // Mag data (raw, bez korekce v update, musíme korigovat zde nebo v update)
     // Pro jednoduchost čteme raw a aplikujeme korekci zde
     _mag->readData();
     
     // Aplikace Mag Kalibrace
     float mx_raw = _mag->x - magHardIron[0];
     float my_raw = _mag->y - magHardIron[1];
     float mz_raw = _mag->z - magHardIron[2];
     float mx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
     float my = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
     float mz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;

     mxSum+=mx; mySum+=my; mzSum+=mz;
     delay(2);
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

void TriSenseFusion::getOrientationDegrees(float& roll, float& pitch, float& yaw) {
  float w=q[0], x=q[1], y=q[2], z=q[3];
  roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)) * 180.0f/PI;
  float sinp = 2*(w*y - z*x);
  pitch = (abs(sinp)>=1) ? copysign(90.0f, sinp) : asin(sinp)*180.0f/PI;
  yaw   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)) * 180.0f/PI;
}

void TriSenseFusion::getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw) {
  roll  = atan2(ay, az) * 180.0f / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  float phi = roll * PI/180.0f;
  float theta = pitch * PI/180.0f;
  float by = my * cos(phi) - mz * sin(phi);
  float bx = mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi);
  yaw = atan2(-by, bx) * 180.0f / PI + magneticDeclination;
}

void TriSenseFusion::gyroIntegration(float gx, float gy, float gz, float dt) {
  float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
  q[0] += (-0.5f * (q1*gx + q2*gy + q3*gz)) * dt;
  q[1] += ( 0.5f * (q0*gx + q2*gz - q3*gy)) * dt;
  q[2] += ( 0.5f * (q0*gy - q1*gz + q3*gx)) * dt;
  q[3] += ( 0.5f * (q0*gz + q1*gy - q2*gx)) * dt;
  float n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if(n > 0) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
}

// --- Advanced Implementation ---
AdvancedTriFusion::AdvancedTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

bool AdvancedTriFusion::update() {
  // 1. Čtení dat
  float ax, ay, az, gx, gy, gz;
  if (!_imu->readFIFO(ax, ay, az, gx, gy, gz)) return false;

  // 2. Aplikace offsetů (Accel)
  lastAx = ax - accelOffset[0];
  lastAy = ay - accelOffset[1];
  lastAz = az - accelOffset[2];
  
  // Gyro offset už řeší senzor interně, jen uložíme
  lastGx = gx; lastGy = gy; lastGz = gz;

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0 || dt > 0.1) dt = 0.001f;
  lastTime = now;

  // 3. Integrace Gyra
  gyroIntegration(lastGx * PI/180.0f, lastGy * PI/180.0f, lastGz * PI/180.0f, dt);
  
  // 4. Update magnetometru (občas)
  static long lastMagT = 0;
  if (millis() - lastMagT > 20) { 
     if (_mag->readData()) {
        // Aplikace Hard/Soft Iron Kalibrace
        float mx_raw = _mag->x - magHardIron[0];
        float my_raw = _mag->y - magHardIron[1];
        float mz_raw = _mag->z - magHardIron[2];
        
        lastMx = magSoftIron[0][0]*mx_raw + magSoftIron[0][1]*my_raw + magSoftIron[0][2]*mz_raw;
        lastMy = magSoftIron[1][0]*mx_raw + magSoftIron[1][1]*my_raw + magSoftIron[1][2]*mz_raw;
        lastMz = magSoftIron[2][0]*mx_raw + magSoftIron[2][1]*my_raw + magSoftIron[2][2]*mz_raw;
        
        lastMagT = millis();
     }
  }

  // 5. Adaptivní logika (korekce orientace)
  float accMag = sqrt(lastAx*lastAx + lastAy*lastAy + lastAz*lastAz);
  if (abs(accMag - accRef) < (3.0f * accSigma)) {
      // Zde by byla aplikace korekce (zjednodušeno pro tento příklad)
  }

  return true;
}