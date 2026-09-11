#include "BMP580.h"

// Default values
#define DEFAULT_OSR_P BMP580_OSR_x4
#define DEFAULT_OSR_T BMP580_OSR_x4
#define DEFAULT_ODR BMP580_ODR_240Hz
#define DEFAULT_MODE BMP580_MODE_NORMAL
#define DEFAULT_IIR BMP580_IIR_OFF

BMP580::BMP580() {
  _wire = &Wire;
  _i2cAddr = BMP580_PRIMARY_I2C_ADDR;
}

bool BMP580::begin(uint8_t addr, TwoWire &wire) {
  _wire = &wire;
  _wire->begin();
  _i2cAddr = addr;

  // Check CHIP_ID on primary or requested address
  uint8_t chipId = readRegister(BMP580_CHIP_ID);
  
  // Smart failover: if default primary fails, try secondary 0x47
  if (chipId != 0x50 && addr == BMP580_PRIMARY_I2C_ADDR) {
    _i2cAddr = BMP580_SECONDARY_I2C_ADDR;
    chipId = readRegister(BMP580_CHIP_ID);
  }

  if (chipId != 0x50) {  // Expected CHIP_ID for BMP580
    return false;
  }

  // Defaults: NORMAL mode, ODR=240Hz, OSR=4x pro P&T, IIR OFF
  writeRegister(BMP580_OSR_CONFIG, (1 << 6) | (DEFAULT_OSR_P << 3) | DEFAULT_OSR_T);  // press_en=1
  setIIRFilter(DEFAULT_IIR, DEFAULT_IIR); // Initialize IIR to OFF by default
  writeRegister(BMP580_ODR_CONFIG, DEFAULT_MODE | (DEFAULT_ODR << 2));  // pwr_mode a odr
  
  // Calculate and set the initial cache timeout based on default ODR
  updateTimeoutUs(DEFAULT_ODR);

  delay(100);  // Stabilization delay
  updateCache(); // Pre-load cache with first reading
  return true;
}

void BMP580::setI2CSpeed(uint32_t speed) {
  // Can be standard (100000), Fast (400000) or Fast Mode Plus (1000000)
  _wire->setClock(speed); 
}

void BMP580::setOversampling(BMP580_OSR osr_p, BMP580_OSR osr_t) {
  uint8_t osrConfig = (1 << 6) | (osr_p << 3) | osr_t;  // press_en=1
  writeRegister(BMP580_OSR_CONFIG, osrConfig);
}

void BMP580::setODR(BMP580_ODR odr) {
  uint8_t odrConfig = (readRegister(BMP580_ODR_CONFIG) & 0x03) | ((odr & 0x3F) << 2);  
  writeRegister(BMP580_ODR_CONFIG, odrConfig);
  
  // Dynamically update the timeout whenever ODR changes
  updateTimeoutUs(odr);
}

void BMP580::setPowerMode(BMP580_Mode mode) {
  uint8_t odr = (readRegister(BMP580_ODR_CONFIG) >> 2) & 0x3F;
  writeRegister(BMP580_ODR_CONFIG, mode | (odr << 2));
}

void BMP580::setIIRFilter(BMP580_IIR iir_p, BMP580_IIR iir_t) {
  // We use read-modify-write to safely preserve other config bits (like shnd_ext) in DSP_CONFIG
  uint8_t dspConfig = readRegister(BMP580_DSP_CONFIG);
  dspConfig &= 0xC0; // Clear bits 0-5 (iir_p and iir_t)
  dspConfig |= (iir_p << 3) | iir_t;
  writeRegister(BMP580_DSP_CONFIG, dspConfig);
}

// ----------------------
// Cache Logic & Reading
// ----------------------

void BMP580::updateTimeoutUs(BMP580_ODR odr) {
  float frequency = 240.0f; // Default frequency
  
  // Convert ODR enum to float frequency. Using switch avoids storing large arrays 
  // in RAM, saving precious memory on 8-bit platforms like Arduino Uno.
  switch (odr) {
    case BMP580_ODR_240Hz: frequency = 240.0f; break;
    case BMP580_ODR_218p5Hz: frequency = 218.5f; break;
    case BMP580_ODR_199p1Hz: frequency = 199.1f; break;
    case BMP580_ODR_179p2Hz: frequency = 179.2f; break;
    case BMP580_ODR_160Hz: frequency = 160.0f; break;
    case BMP580_ODR_149p3Hz: frequency = 149.3f; break;
    case BMP580_ODR_140Hz: frequency = 140.0f; break;
    case BMP580_ODR_129p9Hz: frequency = 129.9f; break;
    case BMP580_ODR_120Hz: frequency = 120.0f; break;
    case BMP580_ODR_110p2Hz: frequency = 110.2f; break;
    case BMP580_ODR_100p3Hz: frequency = 100.3f; break;
    case BMP580_ODR_89p6Hz: frequency = 89.6f; break;
    case BMP580_ODR_80Hz: frequency = 80.0f; break;
    case BMP580_ODR_70Hz: frequency = 70.0f; break;
    case BMP580_ODR_60Hz: frequency = 60.0f; break;
    case BMP580_ODR_50p1Hz: frequency = 50.1f; break;
    case BMP580_ODR_45Hz: frequency = 45.0f; break;
    case BMP580_ODR_40Hz: frequency = 40.0f; break;
    case BMP580_ODR_35Hz: frequency = 35.0f; break;
    case BMP580_ODR_30Hz: frequency = 30.0f; break;
    case BMP580_ODR_25Hz: frequency = 25.0f; break;
    case BMP580_ODR_20Hz: frequency = 20.0f; break;
    case BMP580_ODR_15Hz: frequency = 15.0f; break;
    case BMP580_ODR_10Hz: frequency = 10.0f; break;
    case BMP580_ODR_5Hz: frequency = 5.0f; break;
    case BMP580_ODR_4Hz: frequency = 4.0f; break;
    case BMP580_ODR_3Hz: frequency = 3.0f; break;
    case BMP580_ODR_2Hz: frequency = 2.0f; break;
    case BMP580_ODR_1Hz: frequency = 1.0f; break;
    case BMP580_ODR_0p5Hz: frequency = 0.5f; break;
    case BMP580_ODR_0p25Hz: frequency = 0.25f; break;
    case BMP580_ODR_0p125Hz: frequency = 0.125f; break;
  }
  
  // Calculate timeout in microseconds. Safe 1000000UL to prevent 16-bit int overflow!
  _cacheTimeoutUs = (unsigned long)(1000000UL / frequency);
}

void BMP580::updateCache() {
  uint8_t data[6];
  readBurst(BMP580_TEMP_DATA_XLSB, data, 6);  // Read temp + press simultaneously

  // Calculate Temperature
  // Temperature IS signed 24-bit, so this one genuinely needs sign extension.
  int32_t temp_raw = (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16));
  if (temp_raw & 0x800000) temp_raw |= (int32_t)0xFF000000;
  _cachedTemp = temp_raw / 65536.0f;

  // Calculate Pressure. Unlike temperature this field is UNSIGNED 24-bit, so it
  // must NOT be sign-extended: bit 23 sets at 8388608 LSB = 131072 Pa, and the
  // old sign extension turned readings above ~131 kPa into large negative
  // pressures. The BMP580 tops out at 125 kPa so it never quite triggered in
  // normal use, but it left only ~6 kPa of headroom before the output flipped.
  uint32_t press_raw = ((uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16));
  _cachedPress = press_raw / 64.0f;

  // Save the time of caching
  _lastReadTimeUs = micros();
}

float BMP580::readTemperature() {
  // Safe subtraction method specifically prevents the 71-minute micros() overflow glitch
  if ((micros() - _lastReadTimeUs) >= _cacheTimeoutUs) {
    updateCache();
  }
  return _cachedTemp;
}

float BMP580::readPressure() {
  if ((micros() - _lastReadTimeUs) >= _cacheTimeoutUs) {
    updateCache();
  }
  return _cachedPress;
}

float BMP580::readAltitude(float seaLevelPressure) {
  float pressure = readPressure(); // Automatically uses smart cache
  // The exponent is roughly 0.19026 (a fractional power), so it cannot be replaced by simple integer multiplication.
  return 44308.0f * (1.0f - pow(pressure / seaLevelPressure, 1.0f / 5.25588f)); 
}

// ----------------------
// Private helper functions
// ----------------------

void BMP580::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_i2cAddr);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

uint8_t BMP580::readRegister(uint8_t reg) {
  _wire->beginTransmission(_i2cAddr);
  _wire->write(reg);
  _wire->endTransmission(false); // Repeated start added for bus stability
  if (_wire->requestFrom(_i2cAddr, (uint8_t)1) != 1) return 0;
  return _wire->read();
}

void BMP580::readBurst(uint8_t reg, uint8_t* buffer, uint8_t length) {
  _wire->beginTransmission(_i2cAddr);
  _wire->write(reg);
  _wire->endTransmission(false); // Repeated start
  _wire->requestFrom(_i2cAddr, length);
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = _wire->available() ? _wire->read() : 0;
  }
}