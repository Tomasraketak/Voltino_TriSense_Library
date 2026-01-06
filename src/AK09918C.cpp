#include "AK09918C.h"

AK09918C::AK09918C(TwoWire& wire) {
  _wire = &wire;
}

bool AK09918C::begin(uint8_t mode) {
  _wire->begin();
  // Standard I2C 400kHz, AK09918 to zvládá
  _wire->setClock(400000); 

  delay(10);
  
  // Check ID
  uint8_t wia1 = readRegister(AK09918_REG_WIA1);
  uint8_t wia2 = readRegister(AK09918_REG_WIA2);
  
  if (wia1 != AK09918_WIA1_VAL || wia2 != AK09918_WIA2_VAL) {
    return false;
  }

  softReset();
  
  // Nastavení módu (defaultně Continuous 100Hz)
  setODR(mode);
  
  return true;
}

void AK09918C::softReset() {
  writeRegister(AK09918_REG_CNTL3, 0x01); // Reset
  delay(2); // Počkat na dokončení resetu
}

void AK09918C::setODR(uint8_t mode) {
  // Nejprve musíme přepnout do Power-down, abychom mohli změnit mód
  writeRegister(AK09918_REG_CNTL2, AK09918_MODE_POWER_DOWN);
  delay(1);
  writeRegister(AK09918_REG_CNTL2, mode);
  delay(1);
}

// OPTIMALIZOVANÉ ČTENÍ
bool AK09918C::readData() {
  // Burst read od ST1 (0x10) až po ST2 (0x18)
  // Registry: ST1, HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2
  // Celkem 9 bytů.
  
  // 1. Nastavit pointer na ST1
  _wire->beginTransmission(AK09918_I2C_ADDR);
  _wire->write(AK09918_REG_ST1);
  if (_wire->endTransmission(false) != 0) {
    return false; // Chyba sběrnice
  }

  // 2. Přečíst 9 bytů najednou
  if (_wire->requestFrom(AK09918_I2C_ADDR, 9) != 9) {
    return false; // Nedostali jsme všechna data
  }

  uint8_t buffer[9];
  for(int i=0; i<9; i++) {
    buffer[i] = _wire->read();
  }

  // 3. Analýza dat
  // Byte 0: ST1 - Status 1
  // Bit 0 (DRDY) musí být 1, jinak data nejsou připravena
  // Bit 1 (DOR) indikuje Data Overrun (přeskočená data), ale to pro fúzi tolik nevadí, bereme nejnovější.
  if ((buffer[0] & 0x01) == 0) {
    return false; // Data nejsou připravena
  }

  // Raw data (Little Endian)
  // Buffer: 1=XL, 2=XH, 3=YL, 4=YH, 5=ZL, 6=ZH
  int16_t rx = (int16_t)((buffer[2] << 8) | buffer[1]);
  int16_t ry = (int16_t)((buffer[4] << 8) | buffer[3]);
  int16_t rz = (int16_t)((buffer[6] << 8) | buffer[5]);

  // Byte 8: ST2 - Status 2
  // Bit 3 (HOFL) indikuje magnetic sensor overflow
  uint8_t st2 = buffer[8];
  
  // Senzor vyžaduje přečtení ST2 k uvolnění dat (to jsme právě udělali v burst readu),
  // takže není třeba další čtení.
  
  if (st2 & 0x08) {
    overflow = true;
    // Při overflow jsou data nesmyslná, ale pro fusion je lepší nevracet nic
    // nebo vrátit limitní hodnotu. Zde vrátíme false, aby fúze nepočítala s chybou.
    return false; 
  } else {
    overflow = false;
  }

  // Uložení raw hodnot
  x_raw = rx;
  y_raw = ry;
  z_raw = rz;

  // Převod na uT (float optimalizace)
  x = (float)rx * MAG_SCALE;
  y = (float)ry * MAG_SCALE;
  z = (float)rz * MAG_SCALE;

  return true;
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
  
  _wire->requestFrom(AK09918_I2C_ADDR, 1);
  if (_wire->available()) {
    return _wire->read();
  }
  return 0;
}