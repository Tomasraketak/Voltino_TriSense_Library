#include "ICM42688P_voltino.h"

// Konstruktor
ICM42688P::ICM42688P() {
  // Defaultní hodnoty
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
    Wire.begin();
    Wire.setClock(400000); 
  }
  
  delay(10);

  // Soft Reset
  writeRegister(ICM42688_REG_DEVICE_CONFIG, 0x01);
  delay(10); 

  // Check WHO_AM_I
  uint8_t who = readRegister(ICM42688_REG_WHO_AM_I);
  if (who != 0x47) { 
    return false;
  }

  // Zapnutí senzorů
  writeRegister(ICM42688_REG_PWR_MGMT0, 0x0F); 
  delay(1);

  // Defaultní konfigurace
  setAccelFS(AFS_16G);
  setGyroFS(GFS_2000DPS);
  setODR(ODR_1KHZ);

  return true;
}

// --- OPTIMALIZACE: Blokové čtení ---
void ICM42688P::readRegisters(uint8_t startReg, uint8_t* buffer, size_t len) {
  if (_bus == BUS_SPI) {
    SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    SPI.transfer(startReg | 0x80); 
    for(size_t i=0; i<len; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  } else {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(startReg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)_i2cAddr, (int)len);
    for(size_t i=0; i<len; i++) {
        if(Wire.available()) buffer[i] = Wire.read();
        else buffer[i] = 0;
    }
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
  uint8_t odd = (uint8_t)odr;
  uint8_t gConf = readRegister(ICM42688_REG_GYRO_CONFIG0);
  uint8_t aConf = readRegister(ICM42688_REG_ACCEL_CONFIG0);
  
  gConf &= 0xF0; 
  gConf |= odd;
  
  aConf &= 0xF0; 
  aConf |= odd;
  
  writeRegister(ICM42688_REG_GYRO_CONFIG0, gConf);
  writeRegister(ICM42688_REG_ACCEL_CONFIG0, aConf);
}

void ICM42688P::setAccelFS(ICM_ACCEL_FS fs) {
  uint8_t conf = readRegister(ICM42688_REG_ACCEL_CONFIG0);
  conf &= 0x1F; 
  conf |= (fs << 5);
  writeRegister(ICM42688_REG_ACCEL_CONFIG0, conf);
  
  switch(fs) {
    case AFS_16G: _accelScaleFactor = 1.0f / 2048.0f; break;
    case AFS_8G:  _accelScaleFactor = 1.0f / 4096.0f; break;
    case AFS_4G:  _accelScaleFactor = 1.0f / 8192.0f; break;
    case AFS_2G:  _accelScaleFactor = 1.0f / 16384.0f; break;
  }
}

void ICM42688P::setGyroFS(ICM_GYRO_FS fs) {
  uint8_t conf = readRegister(ICM42688_REG_GYRO_CONFIG0);
  conf &= 0x1F; 
  conf |= (fs << 5);
  writeRegister(ICM42688_REG_GYRO_CONFIG0, conf);
  
  switch(fs) {
    case GFS_2000DPS: _gyroScaleFactor = 1.0f / 16.4f; break;
    case GFS_1000DPS: _gyroScaleFactor = 1.0f / 32.8f; break;
    case GFS_500DPS:  _gyroScaleFactor = 1.0f / 65.5f; break;
    case GFS_250DPS:  _gyroScaleFactor = 1.0f / 131.0f; break;
    case GFS_125DPS:  _gyroScaleFactor = 1.0f / 262.0f; break;
  }
}

void ICM42688P::setAccelOffset(float x, float y, float z) {
  accOffset[0] = x; accOffset[1] = y; accOffset[2] = z;
}

void ICM42688P::setAccelScale(float x, float y, float z) {
  accScale[0] = x; accScale[1] = y; accScale[2] = z;
}

// OPRAVA: Implementace funkce pro nastavení offsetu gyra
void ICM42688P::setGyroOffset(float x, float y, float z) {
  gyrOffset[0] = x;
  gyrOffset[1] = y;
  gyrOffset[2] = z;
}

void ICM42688P::resetHardwareOffsets() {
  setBank(0); 
}

void ICM42688P::setBank(uint8_t bank) {
  writeRegister(ICM42688_REG_BANK_SEL, bank);
}

// OPTIMALIZOVANÉ ČTENÍ DAT
bool ICM42688P::readFIFO(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint8_t buffer[12];
  readRegisters(ICM42688_REG_ACCEL_DATA_X1, buffer, 12);

  int16_t rawAx = (int16_t)((buffer[0] << 8) | buffer[1]);
  int16_t rawAy = (int16_t)((buffer[2] << 8) | buffer[3]);
  int16_t rawAz = (int16_t)((buffer[4] << 8) | buffer[5]);
  
  int16_t rawGx = (int16_t)((buffer[6] << 8) | buffer[7]);
  int16_t rawGy = (int16_t)((buffer[8] << 8) | buffer[9]);
  int16_t rawGz = (int16_t)((buffer[10] << 8) | buffer[11]);
  
  if (rawAx == -32768) return false;

  // Aplikace kalibrace
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
  double gxSum = 0, gySum = 0, gzSum = 0;
  float ax, ay, az, gx, gy, gz;
  
  float oldOff[3] = {gyrOffset[0], gyrOffset[1], gyrOffset[2]};
  gyrOffset[0] = 0; gyrOffset[1] = 0; gyrOffset[2] = 0;
  
  int count = 0;
  while(count < samples) {
    if(readFIFO(ax, ay, az, gx, gy, gz)) {
      gxSum += gx;
      gySum += gy;
      gzSum += gz;
      count++;
      delay(2); 
    }
  }
  
  gyrOffset[0] = (float)(gxSum / samples);
  gyrOffset[1] = (float)(gySum / samples);
  gyrOffset[2] = (float)(gzSum / samples);
}

void ICM42688P::autoCalibrateAccel() {
  double axSum = 0, aySum = 0, azSum = 0;
  float ax, ay, az, gx, gy, gz;
  int samples = 500;
  
  accOffset[0] = 0; accOffset[1] = 0; accOffset[2] = 0;
  accScale[0] = 1; accScale[1] = 1; accScale[2] = 1;

  int count = 0;
  while(count < samples) {
    if(readFIFO(ax, ay, az, gx, gy, gz)) {
      axSum += ax;
      aySum += ay;
      azSum += az;
      count++;
      delay(2);
    }
  }
  
  float axAvg = (float)(axSum / samples);
  float ayAvg = (float)(aySum / samples);
  float azAvg = (float)(azSum / samples);
  
  accOffset[0] = axAvg;
  accOffset[1] = ayAvg;
  accOffset[2] = azAvg - 1.0f; 
}