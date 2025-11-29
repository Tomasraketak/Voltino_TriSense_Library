#include "ICM42688P_voltino.h"

ICM42688P::ICM42688P()
: _bus(BUS_I2C), _csPin(17), _spiSettings(10000000, MSBFIRST, SPI_MODE3),
  _odr(ODR_500HZ),
  _gOX(0), _gOY(0), _gOZ(0),
  _aOX(0), _aOY(0), _aOZ(0) {}

bool ICM42688P::begin(ICM_BUS busType, uint8_t csPin) {
  _bus = busType;
  _csPin = csPin;

  if (_bus == BUS_I2C) {
    Wire.begin();
  } else {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin();
    for (int i = 0; i < 3; i++) {
      digitalWrite(_csPin, LOW);
      delay(1);
      digitalWrite(_csPin, HIGH);
      delay(1);
    }
  }

  uint8_t who = readReg(0x75);
  if (who != 0x47 && who != 0x98) return false;

  writeReg(0x00, 0x01); // reset
  delay(2);

  uint8_t intConfig1 = readReg(0x64);
  writeReg(0x64, intConfig1 & ~(1 << 4));

  writeReg(0x4E, 0x0F); // LN mode

  writeReg(0x4F, (0 << 5) | (uint8_t)_odr);
  writeReg(0x50, (0 << 5) | (uint8_t)_odr);

  writeReg(0x5F, (1 << 4) | (1 << 3) | (1 << 1) | (1 << 0)); // FIFO config
  writeReg(0x16, 0x40); // stream mode

  return true;
}

void ICM42688P::setODR(ICM_ODR odr) {
  _odr = odr;
  writeReg(0x4F, (0 << 5) | (uint8_t)_odr);
  writeReg(0x50, (0 << 5) | (uint8_t)_odr);
}

bool ICM42688P::readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint16_t fifoCount = ((uint16_t)readReg(0x2E) << 8) | readReg(0x2F);
  if (fifoCount < 20) return false;

  uint8_t packet[20];
  readRegs(0x30, packet, 20);

  uint8_t header = packet[0];
  if ((header & 0x80) || !(header & 0x10)) return false;

  int32_t ax_r = (int32_t)((int8_t)packet[1] << 12 | packet[2] << 4 | (packet[17] >> 4));
  ax_r = (ax_r << 12) >> 12; ax_r >>= 2;
  int32_t ay_r = (int32_t)((int8_t)packet[3] << 12 | packet[4] << 4 | (packet[18] >> 4));
  ay_r = (ay_r << 12) >> 12; ay_r >>= 2;
  int32_t az_r = (int32_t)((int8_t)packet[5] << 12 | packet[6] << 4 | (packet[19] >> 4));
  az_r = (az_r << 12) >> 12; az_r >>= 2;

  int32_t gx_r = (int32_t)((int8_t)packet[7] << 12 | packet[8] << 4 | (packet[17] & 0x0F));
  gx_r = (gx_r << 12) >> 12; gx_r >>= 1;
  int32_t gy_r = (int32_t)((int8_t)packet[9] << 12 | packet[10] << 4 | (packet[18] & 0x0F));
  gy_r = (gy_r << 12) >> 12; gy_r >>= 1;
  int32_t gz_r = (int32_t)((int8_t)packet[11] << 12 | packet[12] << 4 | (packet[19] & 0x0F));
  gz_r = (gz_r << 12) >> 12; gz_r >>= 1;

  ax = (ax_r / 8192.0f) - _aOX;
  ay = (ay_r / 8192.0f) - _aOY;
  az = (az_r / 8192.0f) - _aOZ;
  gx = (gx_r / 131.0f) - _gOX;
  gy = (gy_r / 131.0f) - _gOY;
  gz = (gz_r / 131.0f) - _gOZ;

  return true;
}

// --- OFFSET API ---
void ICM42688P::setGyroOffset(float ox, float oy, float oz) {
  _gOX = ox; _gOY = oy; _gOZ = oz;
}

void ICM42688P::setAccelOffset(float ox, float oy, float oz) {
  _aOX = ox; _aOY = oy; _aOZ = oz;
}

void ICM42688P::getGyroOffset(float &ox, float &oy, float &oz) {
  ox = _gOX; oy = _gOY; oz = _gOZ;
}

void ICM42688P::getAccelOffset(float &ox, float &oy, float &oz) {
  ox = _aOX; oy = _aOY; oz = _aOZ;
}

// --- Automatická kalibrace gyra v klidu ---
void ICM42688P::autoCalibrateGyro(uint16_t samples) {
  Serial.println("Starting gyro calibration... Keep IMU still.");
  delay(500);
  float sumX = 0, sumY = 0, sumZ = 0;
  uint16_t count = 0;

  for (uint16_t i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readFIFO(ax, ay, az, gx, gy, gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      count++;
    }
    delayMicroseconds(2000); // ~500Hz vzorkování
  }

  if (count > 0) {
    _gOX = sumX / count;
    _gOY = sumY / count;
    _gOZ = sumZ / count;
  }

  Serial.print("Gyro offsets set to: ");
  Serial.print(_gOX, 5); Serial.print(", ");
  Serial.print(_gOY, 5); Serial.print(", ");
  Serial.println(_gOZ, 5);
}

// --- Low-level I/O ---
uint8_t ICM42688P::readReg(uint8_t reg) {
  if (_bus == BUS_I2C) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(ICM_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
  } else {
    uint8_t val;
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80);
    val = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    return val;
  }
}

void ICM42688P::writeReg(uint8_t reg, uint8_t val) {
  if (_bus == BUS_I2C) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  } else {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg);
    SPI.transfer(val);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  }
}

void ICM42688P::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  if (_bus == BUS_I2C) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(ICM_ADDR, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
  } else {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  }
}
