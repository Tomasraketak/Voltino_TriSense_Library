#ifndef AK09918C_H
#define AK09918C_H

#include <Arduino.h>
#include <Wire.h>

#define AK09918C_ADDR 0x0C

// Registry
#define REG_WIA2   0x01
#define REG_ST1    0x10
#define REG_HXL    0x11
#define REG_ST2    0x18
#define REG_CNTL2  0x31
#define REG_CNTL3  0x32

// ID čipu
#define AK09918C_ID 0x09

// Přepočet dat
#define AK09918C_SENS 0.15f  // µT/LSB

class AK09918C {
public:
    AK09918C(TwoWire &w = Wire);
    bool begin();
    bool dataReady();
    bool readData();
    void setODR(uint8_t odr);
    float getHeading();

    float x, y, z;  // naměřené hodnoty v µT

private:
    TwoWire *_wire;
    void writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);
};

#endif
