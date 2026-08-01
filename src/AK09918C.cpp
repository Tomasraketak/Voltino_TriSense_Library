#include "AK09918C.h"

AK09918C::AK09918C() {
  _wire = &Wire;
  _currentMode = ODR_100HZ;
}

// Standalone initialization matching Voltino conventions
bool AK09918C::beginI2C(uint32_t freq, int8_t sdaPin, int8_t sclPin, AK09918_ODR odr) {
  _wire = &Wire;
  _currentMode = odr;

  // Handle custom I2C pins primarily for ESP32 and ESP8266 architectures
#if defined(ESP32) || defined(ESP8266)
  if (sdaPin >= 0 && sclPin >= 0) {
    _wire->begin(sdaPin, sclPin);
  } else {
    _wire->begin();
  }
#else
  // Standard Arduino platforms (AVR, etc.) do not accept pins in Wire.begin()
  _wire->begin();
  (void)sdaPin; // Suppress unused variable warnings
  (void)sclPin;
#endif

  _wire->setClock(freq);
  
  // Call the core initialization sequence
  return begin(odr, *_wire);
}

// Core initialization (does not force bus start - safe for TriSense master class)
bool AK09918C::begin(AK09918_ODR odr, TwoWire &wire) {
  _wire = &wire;
  _currentMode = odr;

  delay(10);
  
  // Check Device ID
  uint8_t wia1 = readRegister(AK09918_REG_WIA1);
  uint8_t wia2 = readRegister(AK09918_REG_WIA2);
  
  if (wia1 != AK09918_WIA1_VAL || wia2 != AK09918_WIA2_VAL) {
    return false; // Connection failed or wrong sensor attached
  }

  softReset();
  
  // Set Output Data Rate mode
  setODR(odr);
  
  return true;
}

void AK09918C::softReset() {
  writeRegister(AK09918_REG_CNTL3, 0x01); // Reset trigger
  delay(2); // Wait for the reset sequence to complete
}

void AK09918C::setODR(AK09918_ODR odr) {
  // The sensor must be placed into Power-down mode before changing the operation mode
  writeRegister(AK09918_REG_CNTL2, MODE_POWER_DOWN);
  delay(1);
  writeRegister(AK09918_REG_CNTL2, odr);
  _currentMode = odr;
  delay(1);
}

void AK09918C::setMode(AK09918_Mode mode) {
  // Used for switching to system modes like Single Measurement or Power-Down
  writeRegister(AK09918_REG_CNTL2, MODE_POWER_DOWN);
  delay(1);
  writeRegister(AK09918_REG_CNTL2, mode);
  _currentMode = mode;
  delay(1);
}

// Highly optimized early-exit read implementation
bool AK09918C::readData() {
  // Step 1: Read only ST1 (Status 1) to check if data is ready.
  // This avoids hammering the I2C bus with 9-byte requests during high-frequency sensor fusion loops.
  _wire->beginTransmission(AK09918_I2C_ADDR);
  _wire->write(AK09918_REG_ST1);
  if (_wire->endTransmission(false) != 0) {
    return false; // Bus communication error
  }

  // Use explicit casts to avoid ambiguous overloading on some Arduino cores
  if (_wire->requestFrom((uint8_t)AK09918_I2C_ADDR, (uint8_t)1) != 1) {
    return false; // Failed to retrieve status byte
  }

  uint8_t st1 = _wire->read();

  // Check DRDY (Data Ready) bit 0
  if ((st1 & 0x01) == 0) {
    return false; // Early exit, saves significant I2C bandwidth
  }

  // Step 2: Data is ready, perform 8-byte burst read (HXL to ST2)
  _wire->beginTransmission(AK09918_I2C_ADDR);
  _wire->write(AK09918_REG_HXL);
  if (_wire->endTransmission(false) != 0) {
    return false; 
  }

  if (_wire->requestFrom((uint8_t)AK09918_I2C_ADDR, (uint8_t)8) != 8) {
    return false; 
  }

  uint8_t buffer[8];
  for (int i = 0; i < 8; i++) {
    buffer[i] = _wire->read();
  }

  // Parse Raw Measurement Data (Little Endian format)
  int16_t rx = (int16_t)((buffer[1] << 8) | buffer[0]);
  int16_t ry = (int16_t)((buffer[3] << 8) | buffer[2]);
  int16_t rz = (int16_t)((buffer[5] << 8) | buffer[4]);

  // Index 7 is ST2. Must be read to unlock the data registers for the next cycle.
  uint8_t st2 = buffer[7];
  
  // Check HOFL (Magnetic Sensor Overflow) bit 3
  if (st2 & 0x08) {
    overflow = true;
    return false; // Discard invalid values during magnetic saturation to protect fusion integrity
  } else {
    overflow = false;
  }

  // Store valid raw integers
  x_raw = rx;
  y_raw = ry;
  z_raw = rz;

  // Convert raw values to microTeslas (uT) using pre-calculated scale factor
  x = (float)rx * MAG_SCALE;
  y = (float)ry * MAG_SCALE;
  z = (float)rz * MAG_SCALE;

  return true;
}

// Tilt-compensated heading: rotates the horizontal-plane magnetic field
// components into the level frame using the current roll/pitch, then takes
// the arctangent of the result (see AK09918C.h for parameter meaning).
float AK09918C::computeHeading(float mx, float my, float mz, float rollDeg, float pitchDeg, float declinationDeg) {
  float phi = rollDeg * PI / 180.0f;
  float theta = pitchDeg * PI / 180.0f;

  float by = my * cos(phi) - mz * sin(phi);
  float bx = mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi);

  float heading = atan2(-by, bx) * 180.0f / PI + declinationDeg;
  if (heading < 0) heading += 360.0f;
  if (heading >= 360.0f) heading -= 360.0f;
  return heading;
}

void AK09918C::writeRegister(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(AK09918_I2C_ADDR);
  _wire->write(reg);
  _wire->write(val);
  _wire->endTransmission();
}

uint8_t AK09918C::readRegister(uint8_t reg) {
  _wire->beginTransmission(AK09918_I2C_ADDR);
  _wire->write(reg);
  _wire->endTransmission(false);
  
  _wire->requestFrom((uint8_t)AK09918_I2C_ADDR, (uint8_t)1);
  if (_wire->available()) {
    return _wire->read();
  }
  return 0;
}