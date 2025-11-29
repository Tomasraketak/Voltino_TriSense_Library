#include "BMP580.h"

// Default values
#define DEFAULT_OSR_P BMP580_OSR_x4
#define DEFAULT_OSR_T BMP580_OSR_x4
#define DEFAULT_ODR BMP580_ODR_240Hz
#define DEFAULT_MODE BMP580_MODE_NORMAL

BMP580::BMP580() {
  _i2cAddr = BMP580_DEFAULT_I2C_ADDR;
}

bool BMP580::begin(uint8_t addr) {
  _i2cAddr = addr;
  Wire.begin();

  // Check CHIP_ID
  uint8_t chipId = readRegister(BMP580_CHIP_ID);
  if (chipId != 0x50) {  // Expected CHIP_ID for BMP580
    return false;
  }

  // Defaults: NORMAL mode, ODR=240Hz, OSR=4x pro P&T
  writeRegister(BMP580_OSR_CONFIG, (1 << 6) | (DEFAULT_OSR_P << 3) | DEFAULT_OSR_T);  // press_en=1
  writeRegister(BMP580_ODR_CONFIG, DEFAULT_MODE | (DEFAULT_ODR << 2));  // pwr_mode a odr
  delay(100);  // Stabilization delay
  return true;
}

void BMP580::setOversampling(BMP580_OSR osr_p, BMP580_OSR osr_t) {
  uint8_t osrConfig = (1 << 6) | (osr_p << 3) | osr_t;  // press_en=1
  writeRegister(BMP580_OSR_CONFIG, osrConfig);
}

void BMP580::setODR(BMP580_ODR odr) {
  uint8_t odrConfig = (readRegister(BMP580_ODR_CONFIG) & 0x03) | ((odr & 0x3F) << 2);  
  writeRegister(BMP580_ODR_CONFIG, odrConfig);
}

void BMP580::setPowerMode(BMP580_Mode mode) {
  uint8_t odr = (readRegister(BMP580_ODR_CONFIG) >> 2) & 0x3F;
  writeRegister(BMP580_ODR_CONFIG, mode | (odr << 2));
}

float BMP580::readTemperature() {
  uint8_t data[6];
  readBurst(BMP580_TEMP_DATA_XLSB, data, 6);  // Read temp + press

  long temp_raw = ((long)data[0] | ((long)data[1] << 8) | ((long)data[2] << 16));
  if (temp_raw & 0x800000) temp_raw |= 0xFF000000;  // Sign extension
  return temp_raw / 65536.0f;
}

float BMP580::readPressure() {
  uint8_t data[6];
  readBurst(BMP580_TEMP_DATA_XLSB, data, 6);  // Read temp + press

  long press_raw = ((long)data[3] | ((long)data[4] << 8) | ((long)data[5] << 16));
  if (press_raw & 0x800000) press_raw |= 0xFF000000;  // Sign extension
  return press_raw / 64.0f;
}

float BMP580::readAltitude(float seaLevelPressure) {
  float pressure = readPressure();
  return 44308.0f * (1.0f - pow(pressure / seaLevelPressure, 1.0f / 5.25588f));
}

// Private helper functions
void BMP580::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_i2cAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t BMP580::readRegister(uint8_t reg) {
  Wire.beginTransmission(_i2cAddr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(_i2cAddr, 1);
  return Wire.read();
}

void BMP580::readBurst(uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(_i2cAddr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(_i2cAddr, length);
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
}