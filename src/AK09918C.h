#ifndef AK09918C_H
#define AK09918C_H

#include <Arduino.h>
#include <Wire.h>

// I2C adresa AK09918C
#define AK09918_I2C_ADDR  0x0C

// Registry
#define AK09918_REG_WIA1    0x00
#define AK09918_REG_WIA2    0x01
#define AK09918_REG_ST1     0x10
#define AK09918_REG_HXL     0x11
#define AK09918_REG_HXH     0x12
#define AK09918_REG_HYL     0x13
#define AK09918_REG_HYH     0x14
#define AK09918_REG_HZL     0x15
#define AK09918_REG_HZH     0x16
#define AK09918_REG_TMPS    0x17
#define AK09918_REG_ST2     0x18
#define AK09918_REG_CNTL2   0x31
#define AK09918_REG_CNTL3   0x32

// Módy
#define AK09918_MODE_POWER_DOWN 0x00
#define AK09918_MODE_SINGLE     0x01
#define AK09918_MODE_CONT_10HZ  0x02
#define AK09918_MODE_CONT_20HZ  0x04
#define AK09918_MODE_CONT_50HZ  0x06
#define AK09918_MODE_CONT_100HZ 0x08
#define AK09918_MODE_SELF_TEST  0x10

// Identifikátory
#define AK09918_WIA1_VAL    0x48
#define AK09918_WIA2_VAL    0x0C

class AK09918C {
public:
  AK09918C(TwoWire& wire = Wire);

  bool begin(uint8_t mode = AK09918_MODE_CONT_100HZ);
  
  // Hlavní funkce pro update
  // Vrací true, pokud jsou nová data přečtena a uložena do public proměnných
  bool readData();
  
  void setODR(uint8_t mode); // Nastavení rychlosti (10, 20, 50, 100 Hz)
  void softReset();

  // Veřejné proměnné pro přímý přístup (jako v TriSense)
  // Jednotky: uT (micro Tesla)
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  
  // Raw data, pokud by byla potřeba
  int16_t x_raw = 0;
  int16_t y_raw = 0;
  int16_t z_raw = 0;
  
  bool overflow = false; // Indikuje magnetické zahlcení

private:
  TwoWire* _wire;
  
  // OPTIMALIZACE: Fixní konstanta citlivosti pro float násobení
  // 0.15 uT per LSB podle datasheetu
  static constexpr float MAG_SCALE = 0.15f;

  void writeRegister(uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t reg);
};

#endif