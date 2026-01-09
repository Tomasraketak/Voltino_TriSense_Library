#include "ICM42688P_voltino.h"

ICM42688P::ICM42688P() {
  _accelScaleFactor = 1.0f / 2048.0f; 
  _gyroScaleFactor = 1.0f / 16.4f;
}

bool ICM42688P::begin(ICM_BUS busType, uint8_t pin, uint32_t freq) {
  _bus = busType;
  _csPin = pin;
  _spiFreq = freq;

  if (_bus == BUS_SPI) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin();
  } else {
    // I2C Mód
    Wire.begin();
    Wire.setClock(400000); // 400 kHz Fast Mode
  }
  
  delay(10);
  writeRegister(ICM42688_REG_DEVICE_CONFIG, 0x01); // Soft Reset
  delay(10); 

  uint8_t who = readRegister(ICM42688_REG_WHO_AM_I);
  if (who != WHO_AM_I_EXPECTED) {
    Serial.print("ICM WHO_AM_I Error. Expected 0x47, got 0x");
    Serial.println(who, HEX);
    return false;
  }

  writeRegister(ICM42688_REG_PWR_MGMT0, 0x0F); // Sensors ON
  delay(5);

  setAccelFS(AFS_16G);
  setGyroFS(GFS_2000DPS);
  setODR(ODR_1KHZ);

  return true;
}

// Opravená definice s size_t
void ICM42688P::readRegisters(uint8_t startReg, uint8_t* buffer, size_t len) {
  if (_bus == BUS_SPI) {
    SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    SPI.transfer(startReg | 0x80); 
    for(size_t i=0; i<len; i++) buffer[i] = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  } else {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(startReg);
    Wire.endTransmission(false);
    // Přetypování size_t na int pro requestFrom
    Wire.requestFrom((int)_i2cAddr, (int)len);
    for(size_t i=0; i<len; i++) buffer[i] = (Wire.available()) ? Wire.read() : 0;
  }
}

void ICM42688P::writeRegister(uint8_t reg, uint8_t data) {
  if (_bus == BUS_SPI) {
    SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg & 0x7F); 
    SPI.transfer(data);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  } else {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg); 
    Wire.write(data);
    Wire.endTransmission();
  }
}

uint8_t ICM42688P::readRegister(uint8_t reg) {
  uint8_t data = 0;
  if (_bus == BUS_SPI) {
    SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80); 
    data = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  } else {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg); 
    Wire.endTransmission(false);
    Wire.requestFrom((int)_i2cAddr, 1);
    if (Wire.available()) data = Wire.read();
  }
  return data;
}

void ICM42688P::setODR(ICM_ODR odr) {
  _odr = odr;
  uint8_t odd = (uint8_t)odr;
  // ODR se nastavuje v registrech GYRO_CONFIG0 a ACCEL_CONFIG0 (spodní 4 bity)
  // Musíme načíst aktuální hodnotu, vymaskovat staré ODR a nastavit nové.
  uint8_t gConf = readRegister(ICM42688_REG_GYRO_CONFIG0) & 0xF0;
  uint8_t aConf = readRegister(ICM42688_REG_ACCEL_CONFIG0) & 0xF0;
  writeRegister(ICM42688_REG_GYRO_CONFIG0, gConf | odd);
  writeRegister(ICM42688_REG_ACCEL_CONFIG0, aConf | odd);
}

void ICM42688P::setAccelFS(ICM_ACCEL_FS fs) {
  uint8_t conf = readRegister(ICM42688_REG_ACCEL_CONFIG0) & 0x1F;
  // FS je v horních 3 bitech (bity 7:5), registr má formát: [7:5 FS] [4 rsvd] [3:0 ODR]
  // V datasheetu je to často: UI_FS_SEL je bity 6:5 pro Gyro, 6:5 pro Accel v separátních registrech?
  // Zkontrolujeme datasheet pro ICM-42688-P:
  // ACCEL_CONFIG0 (0x50): Bity 7:5 = ACCEL_UI_FS_SEL, Bity 3:0 = ACCEL_ODR
  writeRegister(ICM42688_REG_ACCEL_CONFIG0, (conf & 0x1F) | (fs << 5));
  
  switch(fs) {
    case AFS_16G: _accelScaleFactor = 16.0f / 32768.0f; break; // 1/2048
    case AFS_8G:  _accelScaleFactor = 8.0f  / 32768.0f; break; // 1/4096
    case AFS_4G:  _accelScaleFactor = 4.0f  / 32768.0f; break; // 1/8192
    case AFS_2G:  _accelScaleFactor = 2.0f  / 32768.0f; break; // 1/16384
  }
}

void ICM42688P::setGyroFS(ICM_GYRO_FS fs) {
  uint8_t conf = readRegister(ICM42688_REG_GYRO_CONFIG0) & 0x1F;
  // GYRO_CONFIG0 (0x4F): Bity 7:5 = GYRO_UI_FS_SEL, Bity 3:0 = GYRO_ODR
  writeRegister(ICM42688_REG_GYRO_CONFIG0, (conf & 0x1F) | (fs << 5));
  
  switch(fs) {
    case GFS_2000DPS: _gyroScaleFactor = 2000.0f / 32768.0f; break; // 1/16.4
    case GFS_1000DPS: _gyroScaleFactor = 1000.0f / 32768.0f; break; // 1/32.8
    case GFS_500DPS:  _gyroScaleFactor = 500.0f  / 32768.0f; break; // 1/65.5
    case GFS_250DPS:  _gyroScaleFactor = 250.0f  / 32768.0f; break; // 1/131
    case GFS_125DPS:  _gyroScaleFactor = 125.0f  / 32768.0f; break; // 1/262
  }
}

void ICM42688P::setAccelOffset(float x, float y, float z) { accOffset[0] = x; accOffset[1] = y; accOffset[2] = z; }
void ICM42688P::setAccelScale(float x, float y, float z) { accScale[0] = x; accScale[1] = y; accScale[2] = z; }
void ICM42688P::setGyroOffset(float x, float y, float z) { gyrOffset[0] = x; gyrOffset[1] = y; gyrOffset[2] = z; }

void ICM42688P::setGyroSoftwareOffset(float ox, float oy, float oz) { setGyroOffset(ox, oy, oz); }
void ICM42688P::setAccelSoftwareOffset(float ox, float oy, float oz) { setAccelOffset(ox, oy, oz); }
void ICM42688P::setAccelSoftwareScale(float sx, float sy, float sz) { setAccelScale(sx, sy, sz); }

void ICM42688P::getGyroSoftwareOffset(float &ox, float &oy, float &oz) { ox = gyrOffset[0]; oy = gyrOffset[1]; oz = gyrOffset[2]; }
void ICM42688P::getAccelSoftwareOffset(float &ox, float &oy, float &oz) { ox = accOffset[0]; oy = accOffset[1]; oz = accOffset[2]; }
void ICM42688P::getAccelSoftwareScale(float &sx, float &sy, float &sz) { sx = accScale[0]; sy = accScale[1]; sz = accScale[2]; }

void ICM42688P::getGyroOffset(float &ox, float &oy, float &oz) { ox = gyrOffset[0]; oy = gyrOffset[1]; oz = gyrOffset[2]; }
void ICM42688P::getAccelOffset(float &ox, float &oy, float &oz) { ox = accOffset[0]; oy = accOffset[1]; oz = accOffset[2]; }
void ICM42688P::getAccelScale(float &sx, float &sy, float &sz) { sx = accScale[0]; sy = accScale[1]; sz = accScale[2]; }

float ICM42688P::getAccelOffsetX() { return accOffset[0]; }
float ICM42688P::getAccelOffsetY() { return accOffset[1]; }
float ICM42688P::getAccelOffsetZ() { return accOffset[2]; }
float ICM42688P::getAccelScaleX() { return accScale[0]; }
float ICM42688P::getAccelScaleY() { return accScale[1]; }
float ICM42688P::getAccelScaleZ() { return accScale[2]; }
float ICM42688P::getGyroOffsetX() { return gyrOffset[0]; }
float ICM42688P::getGyroOffsetY() { return gyrOffset[1]; }
float ICM42688P::getGyroOffsetZ() { return gyrOffset[2]; }

void ICM42688P::resetHardwareOffsets() { setBank(0); }
void ICM42688P::setBank(uint8_t bank) { writeRegister(ICM42688_REG_BANK_SEL, bank); }

bool ICM42688P::readFIFO(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint8_t buffer[12];
  readRegisters(ICM42688_REG_ACCEL_DATA_X1, buffer, 12);
  
  int16_t rawAx = (int16_t)((buffer[0] << 8) | buffer[1]);
  int16_t rawAy = (int16_t)((buffer[2] << 8) | buffer[3]);
  int16_t rawAz = (int16_t)((buffer[4] << 8) | buffer[5]);
  int16_t rawGx = (int16_t)((buffer[6] << 8) | buffer[7]);
  int16_t rawGy = (int16_t)((buffer[8] << 8) | buffer[9]);
  int16_t rawGz = (int16_t)((buffer[10] << 8) | buffer[11]);
  
  // Detekce neplatných dat (vše -1 nebo vše 0 je podezřelé, ale 0 může být validní. -1 je u SPI často chyba)
  if (rawAx == -1 && rawAy == -1 && rawAz == -1 && rawGx == -1) return false;
  
  // Aplikace škálování a kalibrace
  ax = ((float)rawAx * _accelScaleFactor - accOffset[0]) * accScale[0];
  ay = ((float)rawAy * _accelScaleFactor - accOffset[1]) * accScale[1];
  az = ((float)rawAz * _accelScaleFactor - accOffset[2]) * accScale[2];
  
  gx = (float)rawGx * _gyroScaleFactor - gyrOffset[0];
  gy = (float)rawGy * _gyroScaleFactor - gyrOffset[1];
  gz = (float)rawGz * _gyroScaleFactor - gyrOffset[2];
  
  return true;
}

float ICM42688P::readTemperature() {
  uint8_t buffer[2];
  readRegisters(ICM42688_REG_TEMP_DATA1, buffer, 2);
  int16_t rawTemp = (int16_t)((buffer[0] << 8) | buffer[1]);
  return ((float)rawTemp / 132.48f) + 25.0f;
}

void ICM42688P::autoCalibrateGyro(uint16_t samples) {
  Serial.println("GYRO CALIBRATION (SW)... Keep still.");
  gyrOffset[0] = 0; gyrOffset[1] = 0; gyrOffset[2] = 0;
  double gxSum = 0, gySum = 0, gzSum = 0;
  float ax, ay, az, gx, gy, gz;
  int count = 0;
  unsigned long startT = millis();
  
  while(count < samples) {
    if (millis() - startT > 10000) { // Timeout 10s
      Serial.println("Error: Sensor read timeout during calibration.");
      break;
    }
    if(readFIFO(ax, ay, az, gx, gy, gz)) {
      gxSum += gx; gySum += gy; gzSum += gz;
      count++;
      delay(2); 
    }
  }
  
  if (count > 0) {
      gyrOffset[0] = (float)(gxSum / count);
      gyrOffset[1] = (float)(gySum / count);
      gyrOffset[2] = (float)(gzSum / count);
  }
  
  Serial.println("DONE. Results:");
  Serial.print("Gyro Bias: ");
  Serial.print(gyrOffset[0], 4); Serial.print(", ");
  Serial.print(gyrOffset[1], 4); Serial.print(", ");
  Serial.println(gyrOffset[2], 4);
}

void ICM42688P::autoCalibrateAccel() {
  Serial.println(F("\n=== 6-POINT ACCEL CALIBRATION (SW) ==="));
  Serial.println(F("Place sensor in 6 orientations (Z+, Z-, Y+, Y-, X+, X-)"));
  
  accOffset[0] = 0; accOffset[1] = 0; accOffset[2] = 0;
  accScale[0] = 1; accScale[1] = 1; accScale[2] = 1;
  
  struct Vector { float x, y, z; };
  Vector points[6];
  
  for (int i = 0; i < 6; i++) {
    Serial.print(F("\nPosition ")); Serial.print(i + 1); Serial.println(F("/6 -> Send 'y' to measure"));
    
    // Čekání na vstup
    while (Serial.available()) Serial.read(); 
    while (!Serial.available()); 
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') { while(!Serial.available()); Serial.read(); }
    
    Serial.println(F("Measuring..."));
    double sumX = 0, sumY = 0, sumZ = 0;
    int count = 0;
    unsigned long start = millis();
    
    while (millis() - start < 1500) {
      float ax, ay, az, gx, gy, gz;
      if (readFIFO(ax, ay, az, gx, gy, gz)) {
        sumX += ax; sumY += ay; sumZ += az; count++;
      }
      delay(2);
    }
    
    if (count == 0) { Serial.println("Error: No data from sensor!"); return; }
    
    points[i].x = sumX / count; 
    points[i].y = sumY / count; 
    points[i].z = sumZ / count;
    
    Serial.print(F("Raw G: ")); Serial.print(points[i].x); Serial.print(", ");
    Serial.print(points[i].y); Serial.print(", "); Serial.println(points[i].z);
  }
  
  Serial.println(F("\nCalculating Sphere Fit..."));
  // Jednoduchý gradient descent pro nalezení středu a škálování elipsoidu
  float bx = 0, by = 0, bz = 0;
  float sx = 1, sy = 1, sz = 1;
  float learningRate = 0.05;
  
  for (int iter = 0; iter < 2000; iter++) {
    float dbx = 0, dby = 0, dbz = 0;
    float dsx = 0, dsy = 0, dsz = 0;
    
    for (int i = 0; i < 6; i++) {
      float adjX = (points[i].x - bx) * sx;
      float adjY = (points[i].y - by) * sy;
      float adjZ = (points[i].z - bz) * sz;
      
      float radius = sqrt(adjX*adjX + adjY*adjY + adjZ*adjZ);
      float error = radius - 1.0f; // Cílový poloměr je 1G
      float common = error / radius;
      
      dbx += -2.0f * common * adjX * sx;
      dby += -2.0f * common * adjY * sy;
      dbz += -2.0f * common * adjZ * sz;
      
      dsx += 2.0f * common * adjX * (points[i].x - bx);
      dsy += 2.0f * common * adjY * (points[i].y - by);
      dsz += 2.0f * common * adjZ * (points[i].z - bz);
    }
    
    bx -= learningRate * (dbx / 6.0f);
    by -= learningRate * (dby / 6.0f);
    bz -= learningRate * (dbz / 6.0f);
    sx -= learningRate * (dsx / 6.0f);
    sy -= learningRate * (dsy / 6.0f);
    sz -= learningRate * (dsz / 6.0f);
    
    if (iter % 200 == 0) learningRate *= 0.8;
  }
  
  accOffset[0] = bx; accOffset[1] = by; accOffset[2] = bz;
  accScale[0] = sx; accScale[1] = sy; accScale[2] = sz;
  
  Serial.println(F("\n--- COPY TO SETUP() ---"));
  Serial.print(F("IMU.setAccelOffset(")); 
  Serial.print(bx, 5); Serial.print(", "); Serial.print(by, 5); Serial.print(", "); Serial.print(bz, 5); Serial.println(");");
  Serial.print(F("IMU.setAccelScale(")); 
  Serial.print(sx, 5); Serial.print(", "); Serial.print(sy, 5); Serial.print(", "); Serial.print(sz, 5); Serial.println(");");
  Serial.println(F("-----------------------"));
}