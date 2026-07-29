#include "ICM42688P_voltino.h"

ICM42688P::ICM42688P() {
  _accelScaleFactor = 1.0f / 2048.0f; 
  _gyroScaleFactor = 1.0f / 16.4f;
  _i2cAddr = ICM_ADDR_PRIMARY;
  _fifoMode = FIFO_NONE;
  _fifoPacketSize = 16;
  _debug = false;
  _csPin = -1;
}

void ICM42688P::setDebug(bool enable) { _debug = enable; }

bool ICM42688P::beginI2C(uint32_t freq, uint8_t i2cAddr, int8_t sdaPin, int8_t sclPin) {
  return begin(BUS_I2C, -1, freq, i2cAddr, sclPin, sdaPin, -1);
}

bool ICM42688P::beginSPI(int8_t csPin, uint32_t freq, int8_t sckPin, int8_t misoPin, int8_t mosiPin) {
  return begin(BUS_SPI, csPin, freq, ICM_ADDR_PRIMARY, sckPin, misoPin, mosiPin);
}

bool ICM42688P::begin(ICM_BUS busType, int8_t csPin, uint32_t freq, uint8_t i2cAddr, int8_t sckSclPin, int8_t misoSdaPin, int8_t mosiPin) {
  _bus = busType;
  _csPin = csPin;
  _i2cAddr = i2cAddr;

  #if defined(ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
    uint32_t defaultSpiFreq = 12000000; 
    uint32_t defaultI2cFreq = 400000;   
    ICM_ODR defaultOdr = ODR_8KHZ;
  #else
    uint32_t defaultSpiFreq = 4000000;  
    uint32_t defaultI2cFreq = 100000;   
    ICM_ODR defaultOdr = ODR_500HZ;
  #endif

  _spiFreq = (freq == 0) ? ((_bus == BUS_SPI) ? defaultSpiFreq : defaultI2cFreq) : freq;

  if (_bus == BUS_SPI) {
    if (_csPin != -1) {
      pinMode(_csPin, OUTPUT);
      digitalWrite(_csPin, HIGH);
    }
    
    #if defined(ESP32)
      if (sckSclPin != -1 && misoSdaPin != -1 && mosiPin != -1) {
        SPI.begin(sckSclPin, misoSdaPin, mosiPin, _csPin);
      } else {
        SPI.begin();
      }
    #elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
      if (sckSclPin != -1 && misoSdaPin != -1 && mosiPin != -1) {
        SPI.setSCK(sckSclPin);
        SPI.setRX(misoSdaPin);
        SPI.setTX(mosiPin);
      }
      SPI.begin();
    #else
      SPI.begin(); 
    #endif

  } else {
    #if defined(ESP32) 
      if (sckSclPin != -1 && misoSdaPin != -1) {
        Wire.setPins(misoSdaPin, sckSclPin);
      }
    #elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
      if (sckSclPin != -1 && misoSdaPin != -1) {
        Wire.setSDA(misoSdaPin);
        Wire.setSCL(sckSclPin);
      }
    #endif
    
    Wire.begin();
    Wire.setClock(_spiFreq);

    Wire.beginTransmission(_i2cAddr);
    if (Wire.endTransmission() != 0) {
      _i2cAddr = (_i2cAddr == ICM_ADDR_PRIMARY) ? ICM_ADDR_SECONDARY : ICM_ADDR_PRIMARY;
    }
  }
  
  delay(10);
  writeRegister(ICM42688_REG_DEVICE_CONFIG, 0x01); // Soft Reset
  delay(10); 

  uint8_t who = readRegister(ICM42688_REG_WHO_AM_I);
  if (who != WHO_AM_I_EXPECTED) {
    if (_debug) {
      Serial.print(F("ICM WHO_AM_I Error. Expected 0x47, got 0x"));
      Serial.println(who, HEX);
    }
    return false;
  }

  writeRegister(ICM42688_REG_PWR_MGMT0, 0x0F); // Sensors ON
  delay(5);

  setAccelFS(AFS_16G);
  setGyroFS(GFS_2000DPS);
  setODR(defaultOdr);
  setFIFOMode(FIFO_NONE);

  return true;
}

void ICM42688P::readRegisters(uint8_t startReg, uint8_t* buffer, size_t len) {
  if (_bus == BUS_SPI) {
    SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    if (_csPin != -1) digitalWrite(_csPin, LOW);
    SPI.transfer(startReg | 0x80); 
    for(size_t i=0; i<len; i++) buffer[i] = SPI.transfer(0x00);
    if (_csPin != -1) digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  } else {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(startReg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)_i2cAddr, (int)len);
    for(size_t i=0; i<len; i++) buffer[i] = (Wire.available()) ? Wire.read() : 0;
  }
}

void ICM42688P::writeRegister(uint8_t reg, uint8_t data) {
  if (_bus == BUS_SPI) {
    SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, SPI_MODE0));
    if (_csPin != -1) digitalWrite(_csPin, LOW);
    SPI.transfer(reg & 0x7F); 
    SPI.transfer(data);
    if (_csPin != -1) digitalWrite(_csPin, HIGH);
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
    if (_csPin != -1) digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80); 
    data = SPI.transfer(0x00);
    if (_csPin != -1) digitalWrite(_csPin, HIGH);
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

int ICM42688P::getODRHz() {
  return _getHzFromODR(_odr);
}

// [VOLTINO FIX] FIFO Helper Method
ICM_FIFO_MODE ICM42688P::getFIFOMode() {
  return _fifoMode;
}

int ICM42688P::_getHzFromODR(ICM_ODR odr) {
  switch(odr) {
    case ODR_32KHZ: return 32000;
    case ODR_16KHZ: return 16000;
    case ODR_8KHZ:  return 8000;
    case ODR_4KHZ:  return 4000;
    case ODR_2KHZ:  return 2000;
    case ODR_1KHZ:  return 1000;
    case ODR_500HZ: return 500;
    case ODR_200HZ: return 200;
    case ODR_100HZ: return 100;
    case ODR_50HZ:  return 50;
    case ODR_25HZ:  return 25;
    case ODR_12_5HZ:return 12; 
    default: return 0;
  }
}

void ICM42688P::enforceBandwidthLimit() {
  uint32_t bytesPerRead = 0;
  
  if (_fifoMode == FIFO_NONE) bytesPerRead = 13; 
  else if (_fifoMode == FIFO_16BIT) bytesPerRead = 16; 
  else if (_fifoMode == FIFO_20BIT_HIRES) bytesPerRead = 20; 

  uint32_t bitsPerByte = (_bus == BUS_I2C) ? 10 : 8; 
  uint32_t bitsPerRead = bytesPerRead * bitsPerByte;
  uint32_t maxAllowedBps = (uint32_t)(_spiFreq * 0.8f);

  ICM_ODR odrList[] = {ODR_32KHZ, ODR_16KHZ, ODR_8KHZ, ODR_4KHZ, ODR_2KHZ, ODR_1KHZ, ODR_500HZ, ODR_200HZ, ODR_100HZ, ODR_50HZ, ODR_25HZ, ODR_12_5HZ};
  ICM_ODR safeODR = _odr;
  int requestedHz = _getHzFromODR(_odr); 

  for (int i = 0; i < 12; i++) {
    int targetHz = _getHzFromODR(odrList[i]);
    if (targetHz > requestedHz) continue; 
    
    uint32_t requiredBps = bitsPerRead * targetHz;
    if (requiredBps <= maxAllowedBps) {
      safeODR = odrList[i];
      break;
    }
  }

  if (safeODR != _odr) {
    if (_debug) {
      Serial.print(F("Voltino TriSense WARNING: Bus bandwidth capacity exceeded. Downgrading ODR to "));
      Serial.print(_getHzFromODR(safeODR));
      Serial.println(F(" Hz."));
    }
    _odr = safeODR; 
  }

  uint8_t odd = (uint8_t)_odr;
  uint8_t gConf = readRegister(ICM42688_REG_GYRO_CONFIG0) & 0xF0;
  uint8_t aConf = readRegister(ICM42688_REG_ACCEL_CONFIG0) & 0xF0;
  writeRegister(ICM42688_REG_GYRO_CONFIG0, gConf | odd);
  writeRegister(ICM42688_REG_ACCEL_CONFIG0, aConf | odd);
}

void ICM42688P::setODR(ICM_ODR odr) {
  _odr = odr;
  enforceBandwidthLimit();
}

void ICM42688P::setAccelFS(ICM_ACCEL_FS fs) {
  if (_fifoMode == FIFO_20BIT_HIRES) fs = AFS_16G;

  uint8_t conf = readRegister(ICM42688_REG_ACCEL_CONFIG0) & 0x1F;
  writeRegister(ICM42688_REG_ACCEL_CONFIG0, (conf & 0x1F) | (fs << 5));
  switch(fs) {
    case AFS_16G: _accelScaleFactor = 16.0f / 32768.0f; break; 
    case AFS_8G:  _accelScaleFactor = 8.0f  / 32768.0f; break; 
    case AFS_4G:  _accelScaleFactor = 4.0f  / 32768.0f; break; 
    case AFS_2G:  _accelScaleFactor = 2.0f  / 32768.0f; break; 
  }
}

void ICM42688P::setGyroFS(ICM_GYRO_FS fs) {
  if (_fifoMode == FIFO_20BIT_HIRES) fs = GFS_2000DPS;

  uint8_t conf = readRegister(ICM42688_REG_GYRO_CONFIG0) & 0x1F;
  writeRegister(ICM42688_REG_GYRO_CONFIG0, (conf & 0x1F) | (fs << 5));
  switch(fs) {
    case GFS_2000DPS: _gyroScaleFactor = 2000.0f / 32768.0f; break; 
    case GFS_1000DPS: _gyroScaleFactor = 1000.0f / 32768.0f; break; 
    case GFS_500DPS:  _gyroScaleFactor = 500.0f  / 32768.0f; break; 
    case GFS_250DPS:  _gyroScaleFactor = 250.0f  / 32768.0f; break; 
    case GFS_125DPS:  _gyroScaleFactor = 125.0f  / 32768.0f; break; 
  }
}

void ICM42688P::setFIFOMode(ICM_FIFO_MODE mode) {
  #if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
    if (mode == FIFO_20BIT_HIRES) {
      mode = FIFO_16BIT;
    }
  #endif

  _fifoMode = mode;
  _fifoPacketSize = (mode == FIFO_20BIT_HIRES) ? 20 : 16;
  invalidateFIFOBuffer(); // Packets already buffered use the old packet layout

  // [VOLTINO FIX] First reset FIFO to bypass with empty configuration
  writeRegister(ICM42688_REG_FIFO_CONFIG, 0x00);
  writeRegister(ICM42688_REG_FIFO_CONFIG1, 0x00);

  // [VOLTINO FIX] Flush FIFO before changing mode (prevents leftover data)
  writeRegister(ICM42688_REG_SIGNAL_PATH_RESET, 0x02); // FIFO_FLUSH

  if (mode == FIFO_16BIT) {
    // Stream-to-FIFO + ACCEL_EN + GYRO_EN
    // Without ACCEL_EN and GYRO_EN FIFO receives no data!
    writeRegister(ICM42688_REG_FIFO_CONFIG1, 0x03);  
    writeRegister(ICM42688_REG_FIFO_CONFIG, 0x40);    
  } 
  else if (mode == FIFO_20BIT_HIRES) {
    setAccelFS(AFS_16G);
    setGyroFS(GFS_2000DPS);
    // HIRES_EN + ACCEL_EN + GYRO_EN (all 3 bits required!)
    writeRegister(ICM42688_REG_FIFO_CONFIG1, 0x13);  
    writeRegister(ICM42688_REG_FIFO_CONFIG, 0x40);    
  }
  
  enforceBandwidthLimit(); 
}

void ICM42688P::flushFIFO() {
  writeRegister(ICM42688_REG_SIGNAL_PATH_RESET, 0x02);
  invalidateFIFOBuffer(); // Drop software-buffered packets too - they are now stale
}

// ---------------------------------------------------------------------------
// FIFO burst buffering + overflow monitoring
// ---------------------------------------------------------------------------

void ICM42688P::invalidateFIFOBuffer() {
  _fifoBufCount = 0;
  _fifoBufIndex = 0;
}

bool ICM42688P::fifoOverflowed() {
  bool flagged = _fifoOverflowFlag;
  _fifoOverflowFlag = false;
  return flagged;
}

uint32_t ICM42688P::getFIFOOverflowCount() { return _fifoOverflowCount; }

void ICM42688P::resetFIFOOverflowCount() {
  _fifoOverflowCount = 0;
  _fifoOverflowFlag = false;
}

// Bulk read of FIFO_DATA. The FIFO read pointer advances per byte, so on I2C the
// transfer can safely be split into Wire-buffer-sized chunks and still stream
// consecutive packets.
void ICM42688P::readFIFOBytes(uint8_t *buf, size_t len) {
  if (_bus == BUS_SPI) {
    readRegisters(ICM42688_REG_FIFO_DATA, buf, len);
  } else {
    const size_t chunk = 30; // Conservative: fits AVR's 32-byte Wire buffer
    size_t done = 0;
    while (done < len) {
      size_t n = ((len - done) < chunk) ? (len - done) : chunk;
      readRegisters(ICM42688_REG_FIFO_DATA, buf + done, n);
      done += n;
    }
  }
}

uint8_t ICM42688P::fillFIFOBuffer() {
  invalidateFIFOBuffer();

  if (_fifoPacketSize == 0) return 0; // Guard against a divide-by-zero

  // Latch the hardware overflow flag. INT_STATUS is read-to-clear, so this is
  // checked once per burst rather than once per packet.
  uint8_t intStatus = readRegister(ICM42688_REG_INT_STATUS);
  if (intStatus & ICM42688_BIT_FIFO_FULL) {
    _fifoOverflowFlag = true;
    if (_fifoOverflowCount < 0xFFFFFFFFUL) _fifoOverflowCount++;
  }

  uint8_t countBuf[2];
  readRegisters(ICM42688_REG_FIFO_COUNTH, countBuf, 2);
  uint16_t fifoBytes = ((uint16_t)countBuf[0] << 8) | countBuf[1];

  // A count above the physical FIFO size means a garbled read - discard it.
  if (fifoBytes > ICM42688_FIFO_BYTES) return 0;

  uint16_t packets = fifoBytes / _fifoPacketSize;
  if (packets == 0) return 0;
  if (packets > FIFO_BURST_PACKETS) packets = FIFO_BURST_PACKETS;

  readFIFOBytes(_fifoBuf, (size_t)packets * _fifoPacketSize);
  _fifoBufCount = (uint8_t)packets;
  return _fifoBufCount;
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

bool ICM42688P::readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  switch (_fifoMode) {
    case FIFO_NONE: return readSensorData(ax, ay, az, gx, gy, gz);
    case FIFO_16BIT: return readHardwareFIFO(ax, ay, az, gx, gy, gz);
    case FIFO_20BIT_HIRES: return readHardwareFIFOHires(ax, ay, az, gx, gy, gz);
    default: return false;
  }
}

bool ICM42688P::readFIFO(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  return readIMU(ax, ay, az, gx, gy, gz);
}

bool ICM42688P::readSensorData(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint8_t buffer[12];
  readRegisters(ICM42688_REG_ACCEL_DATA_X1, buffer, 12);
  
  int16_t rawAx = (int16_t)((buffer[0] << 8) | buffer[1]);
  int16_t rawAy = (int16_t)((buffer[2] << 8) | buffer[3]);
  int16_t rawAz = (int16_t)((buffer[4] << 8) | buffer[5]);
  int16_t rawGx = (int16_t)((buffer[6] << 8) | buffer[7]);
  int16_t rawGy = (int16_t)((buffer[8] << 8) | buffer[9]);
  int16_t rawGz = (int16_t)((buffer[10] << 8) | buffer[11]);
  
  if (rawAx == -1 && rawAy == -1 && rawAz == -1 && rawGx == -1) return false;
  
  ax = ((float)rawAx * _accelScaleFactor - accOffset[0]) * accScale[0];
  ay = ((float)rawAy * _accelScaleFactor - accOffset[1]) * accScale[1];
  az = ((float)rawAz * _accelScaleFactor - accOffset[2]) * accScale[2];
  
  gx = (float)rawGx * _gyroScaleFactor - gyrOffset[0];
  gy = (float)rawGy * _gyroScaleFactor - gyrOffset[1];
  gz = (float)rawGz * _gyroScaleFactor - gyrOffset[2];
  
  return true;
}

bool ICM42688P::readHardwareFIFO(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  if (_fifoBufIndex >= _fifoBufCount) {
    if (fillFIFOBuffer() == 0) return false;
  }

  const uint8_t* buffer = _fifoBuf + (size_t)_fifoBufIndex * _fifoPacketSize;
  _fifoBufIndex++;

  // Header bit 7 set marks an empty/message packet: the rest of the burst is
  // not trustworthy, so drop it and resynchronise on the next call.
  if ((buffer[0] & 0x80) != 0) {
    invalidateFIFOBuffer();
    return false;
  }

  int16_t rawAx = (int16_t)((buffer[1] << 8) | buffer[2]);
  int16_t rawAy = (int16_t)((buffer[3] << 8) | buffer[4]);
  int16_t rawAz = (int16_t)((buffer[5] << 8) | buffer[6]);
  int16_t rawGx = (int16_t)((buffer[7] << 8) | buffer[8]);
  int16_t rawGy = (int16_t)((buffer[9] << 8) | buffer[10]);
  int16_t rawGz = (int16_t)((buffer[11] << 8) | buffer[12]);

  ax = ((float)rawAx * _accelScaleFactor - accOffset[0]) * accScale[0];
  ay = ((float)rawAy * _accelScaleFactor - accOffset[1]) * accScale[1];
  az = ((float)rawAz * _accelScaleFactor - accOffset[2]) * accScale[2];
  
  gx = (float)rawGx * _gyroScaleFactor - gyrOffset[0];
  gy = (float)rawGy * _gyroScaleFactor - gyrOffset[1];
  gz = (float)rawGz * _gyroScaleFactor - gyrOffset[2];

  return true;
}

// [VOLTINO FIX] PERFEKTNÍ 20-BIT PARSOVÁNÍ PODLE TDK DATASHEETU
bool ICM42688P::readHardwareFIFOHires(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  if (_fifoBufIndex >= _fifoBufCount) {
    if (fillFIFOBuffer() == 0) return false;
  }

  const uint8_t* buffer = _fifoBuf + (size_t)_fifoBufIndex * _fifoPacketSize;
  _fifoBufIndex++;

  if ((buffer[0] & 0x80) != 0) {
    invalidateFIFOBuffer();
    return false;
  }

  // Extrakce 20-bit hodnot podle TDK Packet 4 (Byte 17, 18, 19 sdílejí bity)
  // Byte 17 (0x11): Bity 7:4 = Gyro X [3:0], Bity 3:0 = Accel X [3:0]
  // Byte 18 (0x12): Bity 7:4 = Gyro Y [3:0], Bity 3:0 = Accel Y [3:0]
  // Byte 19 (0x13): Bity 7:4 = Gyro Z [3:0], Bity 3:0 = Accel Z [3:0]

  int32_t rawAx = (int32_t)((buffer[1] << 12) | (buffer[2] << 4) | (buffer[17] & 0x0F));
  if (rawAx & 0x80000) rawAx |= 0xFFF00000; // Sign-extend 20-bit to 32-bit

  int32_t rawAy = (int32_t)((buffer[3] << 12) | (buffer[4] << 4) | (buffer[18] & 0x0F));
  if (rawAy & 0x80000) rawAy |= 0xFFF00000;

  int32_t rawAz = (int32_t)((buffer[5] << 12) | (buffer[6] << 4) | (buffer[19] & 0x0F));
  if (rawAz & 0x80000) rawAz |= 0xFFF00000;
  
  int32_t rawGx = (int32_t)((buffer[7] << 12) | (buffer[8] << 4) | (buffer[17] >> 4));
  if (rawGx & 0x80000) rawGx |= 0xFFF00000;

  int32_t rawGy = (int32_t)((buffer[9] << 12) | (buffer[10] << 4) | (buffer[18] >> 4));
  if (rawGy & 0x80000) rawGy |= 0xFFF00000;

  int32_t rawGz = (int32_t)((buffer[11] << 12) | (buffer[12] << 4) | (buffer[19] >> 4));
  if (rawGz & 0x80000) rawGz |= 0xFFF00000;

  // V 20-bit módu je senzor natvrdo v rozsazích 16G a 2000DPS.
  // Maximum obou podepsaných (signed) 20-bit čísel je +/- 524288
  float scaleAccel20 = 16.0f / 524288.0f;
  float scaleGyro20  = 2000.0f / 524288.0f;

  ax = ((float)rawAx * scaleAccel20 - accOffset[0]) * accScale[0];
  ay = ((float)rawAy * scaleAccel20 - accOffset[1]) * accScale[1];
  az = ((float)rawAz * scaleAccel20 - accOffset[2]) * accScale[2];
  
  gx = (float)rawGx * scaleGyro20 - gyrOffset[0];
  gy = (float)rawGy * scaleGyro20 - gyrOffset[1];
  gz = (float)rawGz * scaleGyro20 - gyrOffset[2];

  return true;
}

float ICM42688P::readTemperature() {
  uint8_t buffer[2];
  readRegisters(ICM42688_REG_TEMP_DATA1, buffer, 2);
  int16_t rawTemp = (int16_t)((buffer[0] << 8) | buffer[1]);
  return ((float)rawTemp / 132.48f) + 25.0f;
}

void ICM42688P::autoCalibrateGyro(uint16_t samples) {
  Serial.println(F("GYRO CALIBRATION (SW)... Keep still."));
  gyrOffset[0] = 0; gyrOffset[1] = 0; gyrOffset[2] = 0;
  double gxSum = 0, gySum = 0, gzSum = 0;
  float ax, ay, az, gx, gy, gz;
  int count = 0;
  unsigned long startT = millis();
  unsigned long lastMicros = micros();
  
  while(count < samples) {
    if (millis() - startT > 10000) { 
      Serial.println(F("Error: Sensor read timeout during calibration."));
      break;
    }
    
    if (micros() - lastMicros >= 1000) {
      lastMicros = micros();
      if(readIMU(ax, ay, az, gx, gy, gz)) {
        gxSum += gx; gySum += gy; gzSum += gz;
        count++;
      }
    }
  }
  
  if (count > 0) {
      gyrOffset[0] = (float)(gxSum / count);
      gyrOffset[1] = (float)(gySum / count);
      gyrOffset[2] = (float)(gzSum / count);
  }
  
  Serial.println(F("DONE. Results:"));
  Serial.print(F("Gyro Bias: "));
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
    
    while (Serial.available()) Serial.read(); 
    
    unsigned long waitStart = millis();
    while (!Serial.available()) {
      if (millis() - waitStart > 60000) {
         Serial.println(F("Timeout waiting for user input."));
         return;
      }
      yield();
    }
    
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') { 
        while(!Serial.available()) { yield(); } 
        Serial.read(); 
    }
    
    Serial.println(F("Measuring..."));
    double sumX = 0, sumY = 0, sumZ = 0;
    int count = 0;
    unsigned long start = millis();
    unsigned long lastMicros = micros();
    
    while (millis() - start < 1500) {
      if (micros() - lastMicros >= 1000) { 
        lastMicros = micros();
        float ax, ay, az, gx, gy, gz;
        if (readIMU(ax, ay, az, gx, gy, gz)) {
          sumX += ax; sumY += ay; sumZ += az; count++;
        }
      }
    }
    
    if (count == 0) { Serial.println(F("Error: No data from sensor!")); return; }
    
    points[i].x = sumX / count; 
    points[i].y = sumY / count; 
    points[i].z = sumZ / count;
    
    Serial.print(F("Raw G: ")); Serial.print(points[i].x); Serial.print(F(", "));
    Serial.print(points[i].y); Serial.print(F(", ")); Serial.println(points[i].z);
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
  
  accOffset[0] = bx; accOffset[1] = by; accOffset[2] = bz;
  accScale[0] = sx; accScale[1] = sy; accScale[2] = sz;
  
  Serial.println(F("\n--- COPY TO SETUP() ---"));
  Serial.print(F("IMU.setAccelOffset(")); 
  Serial.print(bx, 5); Serial.print(F(", ")); Serial.print(by, 5); Serial.print(F(", ")); Serial.print(bz, 5); Serial.println(F(");"));
  Serial.print(F("IMU.setAccelScale(")); 
  Serial.print(sx, 5); Serial.print(F(", ")); Serial.print(sy, 5); Serial.print(F(", ")); Serial.print(sz, 5); Serial.println(F(");"));
  Serial.println(F("-----------------------"));
}