#include "TriSense.h"

// --- TriSense Implementation ---

TriSense::TriSense() : mag(Wire) {}

bool TriSense::beginAll(TriSenseMode mode, uint8_t spiCsPin) {
  _mode = mode;

  // 1. Spustíme I2C pro barometr a magnetometr (ty jsou vždy I2C)
  Wire.begin();
  
  // Barometr
  if (!bmp.begin()) {
      Serial.println("BMP580 init failed!");
      return false;
  }
  // Magnetometr
  if (!mag.begin()) {
      Serial.println("AK09918C init failed!");
      return false;
  }

  // 2. Inicializace IMU podle módu
  if (_mode == MODE_I2C) {
    if (!imu.begin(BUS_I2C)) {
        Serial.println("IMU I2C init failed!");
        return false;
    }
  } else { // MODE_HYBRID (SPI)
    // Zde předáváme CS pin a typ sběrnice BUS_SPI
    if (!imu.begin(BUS_SPI, spiCsPin)) {
        Serial.println("IMU SPI init failed!");
        return false;
    }
  }

  // Defaultní konfigurace pro raketu (rychlá data)
  bmp.setOversampling(BMP580_OSR_x4, BMP580_OSR_x4);
  bmp.setODR(BMP580_ODR_240Hz);
  bmp.setPowerMode(BMP580_MODE_NORMAL);

  mag.setODR(100); // Max pro AK09918

  // IMU na max rychlost, interní filtry si pak pořešíme
  imu.setODR(ODR_1KHZ); 

  return true;
}

bool TriSense::beginBMP(uint8_t addr) { return bmp.begin(addr); }
bool TriSense::beginMAG() { return mag.begin(); }
bool TriSense::beginIMU(ICM_BUS busType, uint8_t csPin) { return imu.begin(busType, csPin); }


// --- TriSenseFusion Implementation ---

TriSenseFusion::TriSenseFusion(ICM42688P* imu, AK09918C* mag) : _imu(imu), _mag(mag) {}

float TriSenseFusion::gaussianGain(float x, float mu, float sigma) {
  return exp(-pow(x - mu, 2) / (2 * sigma * sigma));
}

void TriSenseFusion::setAccelGaussian(float ref, float sigma) {
  accRef = ref;
  accSigma = sigma;
}

void TriSenseFusion::setMagGaussian(float ref, float sigma, float tiltSigma) {
  magRef = ref;
  magSigma = sigma;
  magTiltSigmaDeg = tiltSigma;
}

void TriSenseFusion::setGains(float maxAcc, float maxMag, float ki) {
  maxAccelGain = maxAcc;
  maxMagGain = maxMag;
  yawKi = ki;
}

void TriSenseFusion::setDeclination(float deg) {
  magneticDeclination = deg;
}

void TriSenseFusion::initOrientation(int samples) {
  float axSum = 0, aySum = 0, azSum = 0;
  float mxSum = 0, mySum = 0, mzSum = 0;

  for (int i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    _imu->readFIFO(ax, ay, az, gx, gy, gz);
    _mag->readData();
    
    axSum += ax; aySum += ay; azSum += az;
    mxSum += _mag->x; mySum += _mag->y; mzSum += _mag->z;
    delay(2);
  }

  float ax = axSum / samples;
  float ay = aySum / samples;
  float az = azSum / samples;
  float mx = mxSum / samples;
  float my = mySum / samples;
  float mz = mzSum / samples;

  float r, p, y;
  getCorrectionAngles(ax, ay, az, mx, my, mz, r, p, y);

  // Inicializace kvaternionu z Eulerových úhlů
  float c1 = cos(y * PI / 360.0f); // /2 a deg2rad -> /360
  float s1 = sin(y * PI / 360.0f);
  float c2 = cos(p * PI / 360.0f);
  float s2 = sin(p * PI / 360.0f);
  float c3 = cos(r * PI / 360.0f);
  float s3 = sin(r * PI / 360.0f);

  q[0] = c1 * c2 * c3 + s1 * s2 * s3;
  q[1] = c1 * c2 * s3 - s1 * s2 * c3;
  q[2] = c1 * s2 * c3 + s1 * c2 * s3;
  q[3] = s1 * c2 * c3 - c1 * s2 * s3;
}

void TriSenseFusion::getOrientationDegrees(float& roll, float& pitch, float& yaw) {
  // Quaternion to Euler (ZYX convention)
  float w = q[0], x = q[1], y = q[2], z = q[3];
  
  float sinr_cosp = 2 * (w * x + y * z);
  float cosr_cosp = 1 - 2 * (x * x + y * y);
  roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;

  float sinp = 2 * (w * y - z * x);
  if (abs(sinp) >= 1)
    pitch = copysign(90.0f, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp) * 180.0f / PI;

  float siny_cosp = 2 * (w * z + x * y);
  float cosy_cosp = 1 - 2 * (y * y + z * z);
  yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
}

void TriSenseFusion::getCorrectionAngles(float ax, float ay, float az, float mx, float my, float mz, float& roll, float& pitch, float& yaw) {
  // Pitch & Roll from Accel
  roll  = atan2(ay, az) * 180.0f / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

  // Yaw from Mag (tilt compensated)
  float phi = roll * PI / 180.0f;
  float theta = pitch * PI / 180.0f;
  
  // Rotate mag to horizontal plane
  float by = my * cos(phi) - mz * sin(phi);
  float bx = mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi);
  
  yaw = atan2(-by, bx) * 180.0f / PI;
  yaw += magneticDeclination;
}

void TriSenseFusion::gyroIntegration(float gx, float gy, float gz, float dt) {
  // Quaternion derivative: q_dot = 0.5 * q * w
  float w_x = gx;
  float w_y = gy;
  float w_z = gz;

  float q0_dot = -0.5f * (q[1] * w_x + q[2] * w_y + q[3] * w_z);
  float q1_dot =  0.5f * (q[0] * w_x + q[2] * w_z - q[3] * w_y);
  float q2_dot =  0.5f * (q[0] * w_y - q[1] * w_z + q[3] * w_x);
  float q3_dot =  0.5f * (q[0] * w_z + q[1] * w_y - q[2] * w_x);

  q[0] += q0_dot * dt;
  q[1] += q1_dot * dt;
  q[2] += q2_dot * dt;
  q[3] += q3_dot * dt;

  // Normalize
  float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm > 0) {
      q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
  }
}

void TriSenseFusion::complementaryCorrection(float ax, float ay, float az, float mx, float my, float mz, float dt) {
  // 1. Vypočítáme aktuální odhad orientace z Gyra (už máme v q)
  float r_gyro, p_gyro, y_gyro;
  getOrientationDegrees(r_gyro, p_gyro, y_gyro);

  // 2. Vypočítáme "ideální" orientaci z Accel+Mag
  float r_acc, p_acc, y_mag;
  getCorrectionAngles(ax, ay, az, mx, my, mz, r_acc, p_acc, y_mag);

  // 3. Vypočítáme váhy (Gains) podle důvěryhodnosti senzorů
  // Accel Gain: Pokud je celkové zrychlení blízko 1G, věříme mu. Pokud je to start rakety (5G), nevěříme.
  float accelNorm = sqrt(ax*ax + ay*ay + az*az);
  float alphaAcc = gaussianGain(accelNorm, accRef, accSigma) * maxAccelGain;

  // Mag Gain: Pokud se magnetometr moc neliší od očekávané síly
  float magNorm = sqrt(mx*mx + my*my + mz*mz);
  // Pokud je náklon velký, mag je méně přesný pro yaw
  float tiltErr = max(abs(r_acc), abs(p_acc)); 
  float alphaMag = gaussianGain(magNorm, magRef, magSigma) * gaussianGain(tiltErr, 0, magTiltSigmaDeg) * maxMagGain;

  // 4. Korekce (Lineární interpolace Eulerových úhlů - zjednodušeno, ale funkční)
  // Poznámka: Pro přesnější TVC by se měla dělat korekce přímo v kvaternionu (SLERP),
  // ale pro udržení jednoduchosti a rychlosti na Pico 2 toto stačí.
  
  // Roll & Pitch correction
  float deltaRoll = r_acc - r_gyro;
  float deltaPitch = p_acc - p_gyro;
  
  // Ošetření přechodu přes 180 stupňů
  while (deltaRoll > 180) deltaRoll -= 360; while (deltaRoll < -180) deltaRoll += 360;
  while (deltaPitch > 180) deltaPitch -= 360; while (deltaPitch < -180) deltaPitch += 360;

  // Aplikujeme korekci do kvaternionu (aproximace malých úhlů)
  // Vytvoříme korekční rotaci
  // q_new = q_old * q_correction
  // Pro jednoduchost zde modifikujeme q nepřímo přes integraci chyby (Mahony style error accumulation)
  // Ale jelikož máme AdvancedTriFusion, uděláme jednoduchý complementary blend na úrovni úhlů a rekonstrukci q?
  // NE, lepší je nechat gyro integrovat a jen ho "postrkovat".
  
  // Zde používám velmi jednoduchou metodu "Nudging":
  // Pootočíme q směrem k acc/mag orientaci o malé procento (alpha * dt)
  // To se dělá složitě, proto v update() raději použijeme Mahony/Madgwick logiku,
  // pokud bychom to psali od nuly. 
  // Zde implementace spoléhá na to, že update() volá tuto funkci. 
  // Abychom zachovali tvůj styl, uděláme soft update.
  
  // Pro účely tohoto kódu, a protože v update() už máme q z gyra:
  // Spočítáme error vektor a přičteme ho k gyru pro příští krok (PI regulátor)
  // Ale TriSenseFusion je stavěný spíše pro přímou korekci.
  
  // Zjednodušení: Přepíšeme q interpolací (LERP) mezi GyroQ a AccMagQ
  // Pozor: Toto není matematicky puristické, ale pro drony/rakety často používané pro stabilitu.
  
  // Převedeme r_acc, p_acc, y_mag na quaternion q_target
  float c1 = cos(y_mag * PI / 360.0f);
  float s1 = sin(y_mag * PI / 360.0f);
  float c2 = cos(p_acc * PI / 360.0f);
  float s2 = sin(p_acc * PI / 360.0f);
  float c3 = cos(r_acc * PI / 360.0f);
  float s3 = sin(r_acc * PI / 360.0f);
  float q_target[4];
  q_target[0] = c1 * c2 * c3 + s1 * s2 * s3;
  q_target[1] = c1 * c2 * s3 - s1 * s2 * c3;
  q_target[2] = c1 * s2 * c3 + s1 * c2 * s3;
  q_target[3] = s1 * c2 * c3 - c1 * s2 * s3;

  // Interpolace (SLERP/LERP)
  // q = (1-alpha)*q + alpha*q_target
  // alpha kombinujeme z alphaAcc a alphaMag
  float alpha = (alphaAcc + alphaMag) / 2.0f * dt; // Velmi malé číslo
  if (alpha > 1.0f) alpha = 1.0f;

  // Dot product pro zjištění, zda jdeme nejkratší cestou
  float dot = q[0]*q_target[0] + q[1]*q_target[1] + q[2]*q_target[2] + q[3]*q_target[3];
  if (dot < 0) {
      // Invert target
      q_target[0] = -q_target[0]; q_target[1] = -q_target[1];
      q_target[2] = -q_target[2]; q_target[3] = -q_target[3];
  }

  for(int i=0; i<4; i++) {
      q[i] = q[i] * (1.0f - alpha) + q_target[i] * alpha;
  }
  
  // Renormalize
  float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
}

// --- AdvancedTriFusion Implementation ---

AdvancedTriFusion::AdvancedTriFusion(ICM42688P* imu, AK09918C* mag) : TriSenseFusion(imu, mag) {}

bool AdvancedTriFusion::update() {
  float ax, ay, az, gx, gy, gz;
  
  // 1. Čtení IMU (Rychlé)
  if (!_imu->readFIFO(ax, ay, az, gx, gy, gz)) return false;

  // Apply offsets
  ax -= accelOffset[0]; ay -= accelOffset[1]; az -= accelOffset[2];
  gx -= gyroOffset[0]; gy -= gyroOffset[1]; gz -= gyroOffset[2];

  // Time delta
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0 || dt > 0.1) dt = 0.001f; // Safety fix
  lastTime = now;

  // Convert gyro to rad
  gx *= PI / 180.0f;
  gy *= PI / 180.0f;
  gz *= PI / 180.0f;
  
  // 2. Gyro Integration (Prediction step)
  gyroIntegration(gx, gy, gz, dt);

  // 3. Magnetometer reading (Slow - check interval)
  // Čteme jen pokud je čas, abychom nezdržovali TVC smyčku
  bool newMagData = false;
  if (now - lastMagReadAttempt >= magReadIntervalUs) {
      if (_mag->readData()) {
          newMagData = true;
          lastMagReadAttempt = now;
      }
  }

  // 4. Correction step (pokud máme nová data z magu nebo každou smyčku z akcelerometru?)
  // Pro raketu: Accel data máme každou smyčku. Mag data občas.
  // Použijeme poslední známá mag data.
  
  // Calibration MAG
  float mx = (_mag->x - magHardIron[0]);
  float my = (_mag->y - magHardIron[1]);
  float mz = (_mag->z - magHardIron[2]);
  // Soft iron by se tu aplikoval maticovým násobením...

  complementaryCorrection(ax, ay, az, mx, my, mz, dt);

  return true;
}