#include "ICM42688P_voltino.h"

ICM42688P::ICM42688P()
: _bus(BUS_I2C), _csPin(17), 
  _spiSettings(10000000, MSBFIRST, SPI_MODE3), // 10 MHz SPI
  _odr(ODR_500HZ), // Default (přepíše se v begin)
  _gOX(0), _gOY(0), _gOZ(0),
  _aOX(0), _aOY(0), _aOZ(0),
  _aSx(1.0f), _aSy(1.0f), _aSz(1.0f) {}

bool ICM42688P::begin(ICM_BUS busType, uint8_t csPin) {
  _bus = busType;
  _csPin = csPin;

  // Automatické nastavení ODR podle typu sběrnice
  if (_bus == BUS_SPI) {
    _odr = ODR_4KHZ; // SPI = 4 kHz
  } else {
    _odr = ODR_500HZ; // I2C = 500 Hz
  }

  if (_bus == BUS_I2C) {
    Wire.begin();
  } else {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin();
    for (int i = 0; i < 3; i++) {
      digitalWrite(_csPin, LOW); delay(1); digitalWrite(_csPin, HIGH); delay(1);
    }
  }

  setBank(0);
  uint8_t who = readReg(0x75);
  if (who != 0x47 && who != 0x98) return false;

  writeReg(0x00, 0x01); // Reset
  delay(10); 

  uint8_t intConfig1 = readReg(0x64);
  writeReg(0x64, intConfig1 & ~(1 << 4));

  writeReg(0x4E, 0x0F); // LN mode
  
  // Zápis ODR
  writeReg(0x4F, (0 << 5) | (uint8_t)_odr); 
  writeReg(0x50, (0 << 5) | (uint8_t)_odr); 

  writeReg(0x5F, (1 << 4) | (1 << 3) | (1 << 1) | (1 << 0)); 
  writeReg(0x16, 0x40); // Stream mode

  return true;
}

void ICM42688P::setODR(ICM_ODR odr) {
  _odr = odr;
  setBank(0);
  writeReg(0x4F, (0 << 5) | (uint8_t)_odr);
  writeReg(0x50, (0 << 5) | (uint8_t)_odr);
}

void ICM42688P::setBank(uint8_t bank) {
  writeReg(REG_BANK_SEL, bank);
}

// --- ČTENÍ FIFO ---
bool ICM42688P::readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint16_t fifoCount = ((uint16_t)readReg(0x2E) << 8) | readReg(0x2F);
  if (fifoCount < 20) return false;

  uint8_t packet[20];
  readRegs(0x30, packet, 20);

  uint8_t header = packet[0];
  if ((header & 0x80) || !(header & 0x10)) return false; 

  int32_t ax_r = (int32_t)((int8_t)packet[1] << 12 | packet[2] << 4 | (packet[17] >> 4));
  ax_r = (ax_r << 12) >> 12; 
  ax_r >>= 2; 

  int32_t ay_r = (int32_t)((int8_t)packet[3] << 12 | packet[4] << 4 | (packet[18] >> 4));
  ay_r = (ay_r << 12) >> 12; 
  ay_r >>= 2;

  int32_t az_r = (int32_t)((int8_t)packet[5] << 12 | packet[6] << 4 | (packet[19] >> 4));
  az_r = (az_r << 12) >> 12; 
  az_r >>= 2;

  int32_t gx_r = (int32_t)((int8_t)packet[7] << 12 | packet[8] << 4 | (packet[17] & 0x0F));
  gx_r = (gx_r << 12) >> 12; 
  gx_r >>= 1; 

  int32_t gy_r = (int32_t)((int8_t)packet[9] << 12 | packet[10] << 4 | (packet[18] & 0x0F));
  gy_r = (gy_r << 12) >> 12; 
  gy_r >>= 1;

  int32_t gz_r = (int32_t)((int8_t)packet[11] << 12 | packet[12] << 4 | (packet[19] & 0x0F));
  gz_r = (gz_r << 12) >> 12; 
  gz_r >>= 1;

  // SW Kalibrace
  float raw_ax = (ax_r / 8192.0f) - _aOX;
  float raw_ay = (ay_r / 8192.0f) - _aOY;
  float raw_az = (az_r / 8192.0f) - _aOZ;

  ax = raw_ax * _aSx;
  ay = raw_ay * _aSy;
  az = raw_az * _aSz;

  gx = (gx_r / 131.0f) - _gOX;
  gy = (gy_r / 131.0f) - _gOY;
  gz = (gz_r / 131.0f) - _gOZ;

  return true;
}

// --- RESET HW OFFSETS ---
void ICM42688P::resetHardwareOffsets() {
  setBank(4);
  for (uint8_t r = 0x77; r <= 0x7F; r++) {
    writeReg(r, 0x00);
  }
  setBank(0);
}

// --- SW Setters ---
void ICM42688P::setGyroSoftwareOffset(float ox, float oy, float oz) { _gOX = ox; _gOY = oy; _gOZ = oz; }
void ICM42688P::setAccelSoftwareOffset(float ox, float oy, float oz) { _aOX = ox; _aOY = oy; _aOZ = oz; }
void ICM42688P::setAccelSoftwareScale(float sx, float sy, float sz) { _aSx = sx; _aSy = sy; _aSz = sz; }

// --- Getters ---
void ICM42688P::getGyroSoftwareOffset(float &ox, float &oy, float &oz) { ox = _gOX; oy = _gOY; oz = _gOZ; }
void ICM42688P::getAccelSoftwareOffset(float &ox, float &oy, float &oz) { ox = _aOX; oy = _aOY; oz = _aOZ; }
void ICM42688P::getAccelSoftwareScale(float &sx, float &sy, float &sz) { sx = _aSx; sy = _aSy; sz = _aSz; }

// --- Wrappers (Kompatibilita) ---
void ICM42688P::setGyroOffset(float ox, float oy, float oz) { setGyroSoftwareOffset(ox, oy, oz); }
void ICM42688P::setAccelOffset(float ox, float oy, float oz) { setAccelSoftwareOffset(ox, oy, oz); }
void ICM42688P::setAccelScale(float sx, float sy, float sz) { setAccelSoftwareScale(sx, sy, sz); }

void ICM42688P::getGyroOffset(float &ox, float &oy, float &oz) { getGyroSoftwareOffset(ox, oy, oz); }
void ICM42688P::getAccelOffset(float &ox, float &oy, float &oz) { getAccelSoftwareOffset(ox, oy, oz); }
void ICM42688P::getAccelScale(float &sx, float &sy, float &sz) { getAccelSoftwareScale(sx, sy, sz); }

// --- KALIBRACE ---
void ICM42688P::autoCalibrateGyro(uint16_t samples) {
  Serial.println("GYRO CALIBRATION (SW)... Keep still.");
  _gOX = 0; _gOY = 0; _gOZ = 0; 
  delay(200);
  
  double sumX = 0, sumY = 0, sumZ = 0;
  uint16_t count = 0;
  for (uint16_t i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readFIFO(ax, ay, az, gx, gy, gz)) {
      sumX += gx; sumY += gy; sumZ += gz; count++;
    }
    delayMicroseconds(1000);
  }
  
  if (count > 0) {
    _gOX = (float)(sumX / count);
    _gOY = (float)(sumY / count);
    _gOZ = (float)(sumZ / count);
    
    Serial.println("DONE. Results:");
    Serial.print("Gyro Bias: ");
    Serial.print(_gOX, 4); Serial.print(", ");
    Serial.print(_gOY, 4); Serial.print(", ");
    Serial.println(_gOZ, 4);
  }
}

void ICM42688P::autoCalibrateAccel() {
  Serial.println(F("\n=== 6-POINT ACCEL CALIBRATION (SW) ==="));
  Serial.println(F("Place sensor in 6 orientations (Z+, Z-, Y+, Y-, X+, X-)"));
  
  _aOX = 0; _aOY = 0; _aOZ = 0;
  _aSx = 1.0; _aSy = 1.0; _aSz = 1.0;
  
  struct Vector { float x, y, z; };
  Vector points[6];

  for (int i = 0; i < 6; i++) {
    Serial.print(F("\nPosition ")); Serial.print(i + 1); Serial.println(F("/6 -> Send 'y'"));
    while (Serial.available()) Serial.read(); 
    while (!Serial.available()); 
    Serial.read();

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
    points[i].x = sumX / count; points[i].y = sumY / count; points[i].z = sumZ / count;
    Serial.print(F("Raw: ")); Serial.print(points[i].x); Serial.print(", ");
    Serial.print(points[i].y); Serial.print(", "); Serial.println(points[i].z);
  }

  Serial.println(F("\nCalculating Sphere Fit..."));
  
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
      float error = radius - 1.0f;
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

  _aOX = bx; _aOY = by; _aOZ = bz;
  _aSx = sx; _aSy = sy; _aSz = sz;

  Serial.println(F("\n--- COPY TO SETUP() ---"));
  Serial.print(F("IMU.setAccelOffset(")); 
  Serial.print(bx, 5); Serial.print(", "); 
  Serial.print(by, 5); Serial.print(", "); 
  Serial.print(bz, 5); Serial.println(");");
  
  Serial.print(F("IMU.setAccelScale(")); 
  Serial.print(sx, 5); Serial.print(", "); 
  Serial.print(sy, 5); Serial.print(", "); 
  Serial.print(sz, 5); Serial.println(");");
  Serial.println(F("-----------------------"));
}

// Low level IO
uint8_t ICM42688P::readReg(uint8_t reg) {
  if (_bus == BUS_I2C) {
    Wire.beginTransmission(ICM_ADDR); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom(ICM_ADDR, (uint8_t)1); return Wire.available() ? Wire.read() : 0;
  } else {
    uint8_t val; SPI.beginTransaction(_spiSettings); digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80); val = SPI.transfer(0x00); digitalWrite(_csPin, HIGH); SPI.endTransaction(); return val;
  }
}
void ICM42688P::writeReg(uint8_t reg, uint8_t val) {
  if (_bus == BUS_I2C) {
    Wire.beginTransmission(ICM_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
  } else {
    SPI.beginTransaction(_spiSettings); digitalWrite(_csPin, LOW); SPI.transfer(reg); SPI.transfer(val); digitalWrite(_csPin, HIGH); SPI.endTransaction();
  }
}
void ICM42688P::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  if (_bus == BUS_I2C) {
    Wire.beginTransmission(ICM_ADDR); Wire.write(reg); Wire.endTransmission(false);
    Wire.requestFrom(ICM_ADDR, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
  } else {
    SPI.beginTransaction(_spiSettings); digitalWrite(_csPin, LOW); SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00); digitalWrite(_csPin, HIGH); SPI.endTransaction();
  }
}