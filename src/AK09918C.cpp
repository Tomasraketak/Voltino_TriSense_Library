#include "AK09918C.h"

AK09918C::AK09918C(TwoWire &w) {
    _wire = &w;
}

bool AK09918C::begin() {
    _wire->begin();
    delay(100);

    // Soft reset
    writeRegister(REG_CNTL3, 0x01);
    delay(100);

    // Read device ID (WIA2)
    uint8_t id = readRegister(REG_WIA2);

    // Accept both known IDs (0x09 = AK09918, 0x0C = AK09918C)
    if (id == 0x09 || id == 0x0C) {
        Serial.print("AK09918C detected, ID=0x");
        Serial.println(id, HEX);
    } else {
        Serial.print("Unexpected ID: 0x");
        Serial.println(id, HEX);
        return false;
    }

    // Default ODR = 100 Hz
    setODR(100);
    return true;
}

void AK09918C::setODR(uint8_t odr) {
    uint8_t mode = 0x00;

    // Enter power-down first (required by datasheet)
    writeRegister(REG_CNTL2, 0x00);
    delayMicroseconds(200);

    switch (odr) {
        case 10:  mode = 0x02; break;  // Continuous 10Hz
        case 20:  mode = 0x04; break;  // Continuous 20Hz
        case 50:  mode = 0x06; break;  // Continuous 50Hz
        case 100: mode = 0x08; break;  // Continuous 100Hz
        default:  mode = 0x08; break;  // Default to 100Hz
    }

    writeRegister(REG_CNTL2, mode);
    delay(10);
}

bool AK09918C::dataReady() {
    uint8_t st1 = readRegister(REG_ST1);
    return (st1 & 0x01);
}

bool AK09918C::readData() {
    if (!dataReady()) return false;

    _wire->beginTransmission(AK09918C_ADDR);
    _wire->write(REG_HXL);
    _wire->endTransmission(false);
    _wire->requestFrom(AK09918C_ADDR, (uint8_t)6);

    if (_wire->available() < 6) return false;

    int16_t rawX = _wire->read() | (_wire->read() << 8);
    int16_t rawY = _wire->read() | (_wire->read() << 8);
    int16_t rawZ = _wire->read() | (_wire->read() << 8);

    x = rawX * AK09918C_SENS;
    y = rawY * AK09918C_SENS;
    z = rawZ * AK09918C_SENS;

    // Read ST2 to release data hold
    uint8_t st2 = readRegister(REG_ST2);

    // Optional: detect magnetic overflow
    if (st2 & 0x08) {
        Serial.println("Warning: Magnetic sensor overflow!");
    }

    return true;
}

float AK09918C::getHeading() {
    float heading = atan2(y, x) * 180.0f / PI;
    if (heading < 0) heading += 360.0f;
    return heading;
}

// ======================
//   I2C low-level
// ======================
void AK09918C::writeRegister(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(AK09918C_ADDR);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}

uint8_t AK09918C::readRegister(uint8_t reg) {
    _wire->beginTransmission(AK09918C_ADDR);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(AK09918C_ADDR, (uint8_t)1);
    if (_wire->available()) return _wire->read();
    return 0;
}
