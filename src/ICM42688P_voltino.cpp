#include "ICM42688P_voltino.h"

// SPI pin release needs direct control of the GPIO function select, which only
// the arduino-pico core (earlephilhower) exposes through the Pico SDK. On every
// other core the release calls compile to nothing.
//
// ESP32 could do the same through the GPIO matrix (pinMatrixOutDetach() and a
// re-attach to the SPI peripheral's output signal), but the signal index and
// the peripheral-manager rules differ between core 2.x and 3.x, so it is left
// as a no-op there rather than shipped untested. See README, "SPI pin release".
#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)) && !defined(ARDUINO_ARCH_MBED)
  #define ICM_SPI_PIN_RELEASE 1
  #include <hardware/gpio.h>
#endif

// ArduinoCore-API based cores (arduino-pico, SAMD, mbed, Renesas) take an
// SPIMode enum in SPISettings; the classic cores (AVR, ESP32) take a uint8_t.
#if defined(ARDUINO_API_VERSION)
  #define ICM_SPI_MODE_T SPIMode
#else
  #define ICM_SPI_MODE_T uint8_t
#endif

ICM42688P::ICM42688P() {
  _accelScaleFactor = 1.0f / 2048.0f; 
  _gyroScaleFactor = 1.0f / 16.4f;
  _i2cAddr = ICM_ADDR_PRIMARY;
  _requestedOdr = ODR_1KHZ;
  _odr = ODR_1KHZ;
  _fifoMode = FIFO_NONE;
  _fifoPacketSize = 16;
  _debug = false;
  _csPin = -1;
  _spiMode = (uint8_t)SPI_MODE0;
  _accelFs = AFS_16G;
  _gyroFs = GFS_2000DPS;
  _pwrMgmt0 = ICM42688_PWR_SENSORS_LN;
}

void ICM42688P::setDebug(bool enable) { _debug = enable; }

void ICM42688P::setWire(TwoWire &wire) { _wire = &wire; }

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

    // SPI.begin() below hands the pins back to the SPI peripheral, so any
    // release from an earlier begin() no longer holds.
    _relActive = false;
    
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
    _spiStarted = true;

    // SPI.begin() leaves SCK and MOSI as push-pull outputs. Release them right
    // away, before the first transaction, so the level shifter on the TriSense
    // Pro is not fought even for the few milliseconds of the reset below.
    if (!_relConfigured) {
      int8_t sck = sckSclPin;
      int8_t mosi = mosiPin;
      if (sck < 0 || mosi < 0) detectSpiPins(sck, mosi);
      _relSck = sck;
      _relMosi = mosi;
    }
    activateSpiPinRelease();

  } else {
    #if defined(ESP32) 
      if (sckSclPin != -1 && misoSdaPin != -1) {
        _wire->setPins(misoSdaPin, sckSclPin);
      }
    #elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
      if (sckSclPin != -1 && misoSdaPin != -1) {
        _wire->setSDA(misoSdaPin);
        _wire->setSCL(sckSclPin);
      }
    #endif
    
    _wire->begin();
    _wire->setClock(_spiFreq);

    _wire->beginTransmission(_i2cAddr);
    if (_wire->endTransmission() != 0) {
      _i2cAddr = (_i2cAddr == ICM_ADDR_PRIMARY) ? ICM_ADDR_SECONDARY : ICM_ADDR_PRIMARY;
    }
  }
  
  delay(10);
  if (!resetDevice()) return false;

  _pwrMgmt0 = ICM42688_PWR_SENSORS_LN;
  _sleeping = false;
  writeRegister(ICM42688_REG_PWR_MGMT0, _pwrMgmt0); // Sensors ON
  _powerOnMs = millis();
  delay(5);

  setAccelFS(AFS_16G);
  setGyroFS(GFS_2000DPS);
  setODR(defaultOdr);
  setFIFOMode(FIFO_NONE);

  return true;
}

// ---------------------------------------------------------------------------
// Reset and interface setup
// ---------------------------------------------------------------------------

bool ICM42688P::resetDevice() {
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

  // The soft reset put INTF_CONFIG0 back to its reset value (0x30), which has
  // both serial interfaces enabled, so this must follow every reset.
  applyInterfaceConfig();
  return true;
}

// In BUS_SPI, switch the chip's I2C slave off: INTF_CONFIG0.UI_SIFS_CFG = 0b11
// ("disable I2C", DS-000347, INTF_CONFIG0 bits 1:0). On the TriSense Pro the
// TXS0102 joins the IMU's SCLK/SDI pins to the module's I2C bus whenever CS is
// HIGH, so with I2C left on the IMU also answers there at 0x68 and decodes all
// of the magnetometer / barometer traffic as I2C. Read-modify-write so the FIFO
// count format and endianness bits are left alone - setFIFOMode() relies on
// them and updates them the same way.
//
// Never in BUS_I2C: that would cut off the very interface being used.
void ICM42688P::applyInterfaceConfig() {
  if (_bus != BUS_SPI) return;
  uint8_t intf = readRegister(ICM42688_REG_INTF_CONFIG0);
  intf = (uint8_t)((intf & ~ICM42688_MASK_UI_SIFS_CFG) | ICM42688_UI_SIFS_DISABLE_I2C);
  writeRegister(ICM42688_REG_INTF_CONFIG0, intf);
}

// ---------------------------------------------------------------------------
// SPI transactions and pin release
// ---------------------------------------------------------------------------
//
// The order matters on the TriSense Pro, where the SN74LVC2T45 in the SCK/MOSI
// path takes its direction from CS:
//   1. beginTransaction()
//   2. CS LOW         - the shifter turns, the connector side becomes an input
//   3. SCK/MOSI -> SPI function, now driving into an input
//   4. transfer
//   5. SCK/MOSI -> released (GPIO_FUNC_NULL, high impedance)
//   6. CS HIGH        - the shifter turns back and drives the pins again,
//                       which is harmless now that nothing drives against it
//   7. endTransaction()
// A FIFO burst is a single transaction, so it pays for one attach/release pair
// no matter how many bytes it moves.

void ICM42688P::spiSelect() {
  SPI.beginTransaction(SPISettings(_spiFreq, MSBFIRST, (ICM_SPI_MODE_T)_spiMode));
  if (_csPin != -1) digitalWrite(_csPin, LOW);
#if defined(ICM_SPI_PIN_RELEASE)
  if (_relActive) {
    gpio_set_function((uint)_relSck, GPIO_FUNC_SPI);
    gpio_set_function((uint)_relMosi, GPIO_FUNC_SPI);
  }
#endif
}

void ICM42688P::spiDeselect() {
#if defined(ICM_SPI_PIN_RELEASE)
  if (_relActive) {
    gpio_set_function((uint)_relSck, GPIO_FUNC_NULL);
    gpio_set_function((uint)_relMosi, GPIO_FUNC_NULL);
  }
#endif
  if (_csPin != -1) digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

// On RP2040/RP2350 GPIO function 1 (SPI) is fixed per pin: GPIO n belongs to
// SPI((n >> 3) & 1), and n % 4 selects the signal - 0 RX, 1 CSn, 2 SCK, 3 TX.
// Checking against that table means a wrong pin number can never route an SPI
// signal onto a pin that is wired to something else.
bool ICM42688P::spiPinsValid(int8_t sck, int8_t mosi) {
#if defined(ICM_SPI_PIN_RELEASE)
  if (sck < 0 || mosi < 0) return false;
  if (sck >= (int8_t)NUM_BANK0_GPIOS || mosi >= (int8_t)NUM_BANK0_GPIOS) return false;
  if ((sck & 3) != 2 || (mosi & 3) != 3) return false;
  return ((sck >> 3) & 1) == ((mosi >> 3) & 1);   // Same SPI instance
#else
  (void)sck; (void)mosi;
  return false;
#endif
}

// For begin() without explicit pins: after SPI.begin() the pins the core chose
// are the ones in SPI function. Accepted only if exactly one SCK and one TX pin
// are found - a second SPI bus in use makes it ambiguous, and then release
// stays off rather than guess. No private SPIClass state is touched.
bool ICM42688P::detectSpiPins(int8_t &sck, int8_t &mosi) {
#if defined(ICM_SPI_PIN_RELEASE)
  int8_t foundSck = -1, foundMosi = -1;
  uint8_t nSck = 0, nMosi = 0;
  for (uint pin = 0; pin < NUM_BANK0_GPIOS; pin++) {
    if (gpio_get_function(pin) != GPIO_FUNC_SPI) continue;
    if ((pin & 3) == 2) { foundSck = (int8_t)pin; nSck++; }
    if ((pin & 3) == 3) { foundMosi = (int8_t)pin; nMosi++; }
  }
  if (nSck == 1 && nMosi == 1 && spiPinsValid(foundSck, foundMosi)) {
    sck = foundSck;
    mosi = foundMosi;
    return true;
  }
  if (_debug) Serial.println(F("ICM: SPI pins not detected - pin release off. Use setSpiPinRelease()."));
#endif
  sck = -1;
  mosi = -1;
  return false;
}

// While released, SCK is held at its idle level by a pull resistor. Without
// it, the pin floats between CS falling and the SPI function taking over (and
// again between release and CS rising), and a float to the non-idle level
// followed by the snap back would be a clock edge the IMU counts as a bit.
// MOSI is only sampled on SCK edges, so its pulls are simply switched off.
void ICM42688P::applySckIdlePull() {
#if defined(ICM_SPI_PIN_RELEASE)
  if (_spiMode == (uint8_t)SPI_MODE3) gpio_pull_up((uint)_relSck);
  else                                gpio_pull_down((uint)_relSck);
#endif
}

void ICM42688P::activateSpiPinRelease() {
#if defined(ICM_SPI_PIN_RELEASE)
  if (_relActive) return;
  if (!_spiStarted || _bus != BUS_SPI || _csPin < 0) return;  // CS must be ours to order it
  if (!spiPinsValid(_relSck, _relMosi)) return;
  applySckIdlePull();
  gpio_disable_pulls((uint)_relMosi);
  gpio_set_function((uint)_relSck, GPIO_FUNC_NULL);
  gpio_set_function((uint)_relMosi, GPIO_FUNC_NULL);
  _relActive = true;
#endif
}

void ICM42688P::deactivateSpiPinRelease() {
#if defined(ICM_SPI_PIN_RELEASE)
  if (!_relActive) return;
  gpio_set_function((uint)_relSck, GPIO_FUNC_SPI);
  gpio_set_function((uint)_relMosi, GPIO_FUNC_SPI);
  _relActive = false;
#endif
}

bool ICM42688P::setSpiPinRelease(int8_t sckPin, int8_t mosiPin) {
  _relConfigured = true;
  deactivateSpiPinRelease();   // Hand the old pins back before switching

  if (!spiPinsValid(sckPin, mosiPin)) {
    if (_debug && (sckPin >= 0 || mosiPin >= 0)) {
      Serial.println(F("ICM: setSpiPinRelease() - unsupported core or not an SCK/TX pin pair."));
    }
    _relSck = -1;
    _relMosi = -1;
    return false;
  }

  _relSck = sckPin;
  _relMosi = mosiPin;
  activateSpiPinRelease();     // No-op until begin() has started SPI
  return true;
}

bool ICM42688P::isSpiPinReleaseActive() { return _relActive; }

bool ICM42688P::setSpiMode(uint8_t mode) {
  if (mode != (uint8_t)SPI_MODE0 && mode != (uint8_t)SPI_MODE3) return false;
  _spiMode = mode;
  if (_relActive) applySckIdlePull();
  return true;
}

uint8_t ICM42688P::getSpiMode() { return _spiMode; }

// ---------------------------------------------------------------------------
// Sleep / wake
// ---------------------------------------------------------------------------

void ICM42688P::sleep() {
  if (_sleeping) return;
  // The gyro must have been on for 45 ms before it may be switched off again.
  uint32_t onFor = millis() - _powerOnMs;
  if (onFor < ICM42688_GYRO_MIN_ON_MS) delay(ICM42688_GYRO_MIN_ON_MS - onFor);

  writeRegister(ICM42688_REG_PWR_MGMT0, ICM42688_PWR_SENSORS_OFF);
  invalidateFIFOBuffer();
  _sleeping = true;
}

bool ICM42688P::wake() {
  if (!_sleeping) return true;

  writeRegister(ICM42688_REG_PWR_MGMT0, _pwrMgmt0);
  _powerOnMs = millis();
  delayMicroseconds(ICM42688_PWR_WRITE_HOLDOFF_US + 100);  // No writes for 200 us
  _sleeping = false;

  if (readRegister(ICM42688_REG_WHO_AM_I) != WHO_AM_I_EXPECTED) return false;

  // The registers survive sleep, but re-applying them costs a handful of writes
  // and guarantees the state the driver's scale factors assume - including
  // after a sketch has poked the chip directly in the meantime.
  applyInterfaceConfig();
  setAccelFS(_accelFs);
  setGyroFS(_gyroFs);
  setFIFOMode(_fifoMode);       // Also re-applies the ODR (enforceBandwidthLimit)

  delay(ICM42688_GYRO_STARTUP_MS);
  flushFIFO();                  // Drop samples taken while the gyro was settling
  return true;
}

bool ICM42688P::isSleeping() { return _sleeping; }

void ICM42688P::readRegisters(uint8_t startReg, uint8_t* buffer, size_t len) {
  if (_bus == BUS_SPI) {
    spiSelect();
    SPI.transfer(startReg | 0x80); 
    for(size_t i=0; i<len; i++) buffer[i] = SPI.transfer(0x00);
    spiDeselect();
  } else {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(startReg);
    _wire->endTransmission(false);
    _wire->requestFrom((int)_i2cAddr, (int)len);
    for(size_t i=0; i<len; i++) buffer[i] = (_wire->available()) ? _wire->read() : 0;
  }
}

void ICM42688P::writeRegister(uint8_t reg, uint8_t data) {
  if (_bus == BUS_SPI) {
    spiSelect();
    SPI.transfer(reg & 0x7F); 
    SPI.transfer(data);
    spiDeselect();
  } else {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg); 
    _wire->write(data);
    _wire->endTransmission();
  }
}

uint8_t ICM42688P::readRegister(uint8_t reg) {
  uint8_t data = 0;
  if (_bus == BUS_SPI) {
    spiSelect();
    SPI.transfer(reg | 0x80); 
    data = SPI.transfer(0x00);
    spiDeselect();
  } else {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg); 
    _wire->endTransmission(false);
    _wire->requestFrom((int)_i2cAddr, 1);
    if (_wire->available()) data = _wire->read();
  }
  return data;
}

int ICM42688P::getODRHz() {
  return _getHzFromODR(_odr);
}

int ICM42688P::getRequestedODRHz() {
  return _getHzFromODR(_requestedOdr);
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

  // Always re-derive from the REQUESTED rate, never from whatever _odr was last
  // clamped to. Otherwise a downgrade is permanent: switching to a cheaper FIFO
  // mode later frees bandwidth, but the original request has been forgotten and
  // the sensor stays stuck at the reduced rate.
  ICM_ODR odrList[] = {ODR_32KHZ, ODR_16KHZ, ODR_8KHZ, ODR_4KHZ, ODR_2KHZ, ODR_1KHZ, ODR_500HZ, ODR_200HZ, ODR_100HZ, ODR_50HZ, ODR_25HZ, ODR_12_5HZ};
  ICM_ODR safeODR = _requestedOdr;
  int requestedHz = _getHzFromODR(_requestedOdr);

  for (int i = 0; i < 12; i++) {
    int targetHz = _getHzFromODR(odrList[i]);
    if (targetHz > requestedHz) continue; 
    
    uint32_t requiredBps = bitsPerRead * targetHz;
    if (requiredBps <= maxAllowedBps) {
      safeODR = odrList[i];
      break;
    }
  }

  if (_debug && safeODR != _requestedOdr) {
    Serial.print(F("Voltino TriSense WARNING: Bus bandwidth capacity exceeded. Downgrading ODR to "));
    Serial.print(_getHzFromODR(safeODR));
    Serial.println(F(" Hz."));
  }
  if (_debug && safeODR != _odr && safeODR == _requestedOdr) {
    Serial.print(F("Voltino TriSense: bandwidth freed, restoring requested ODR of "));
    Serial.print(_getHzFromODR(safeODR));
    Serial.println(F(" Hz."));
  }
  _odr = safeODR;

  // Interrupt timing must follow the ODR. The datasheet requires
  // INT_TPULSE_DURATION and INT_TDEASSERT_DISABLE to be set at 4 kHz and above,
  // because the defaults - a 100 us pulse plus a mandatory 100 us de-assert
  // window - are longer than the sample period itself (250 us at 4 kHz, 31 us
  // at 32 kHz). The interrupt machinery then cannot complete one cycle per
  // sample, and INT_STATUS is part of that machinery, which this driver reads
  // on every FIFO refill. The bits are cleared again below 4 kHz so the longer,
  // easier-to-catch pulse comes back for sketches that wire up INT1.
  {
    const uint8_t fastBits = ICM42688_BIT_INT_TPULSE_DURATION
                           | ICM42688_BIT_INT_TDEASSERT_DISABLE;
    uint8_t intCfg1 = readRegister(ICM42688_REG_INT_CONFIG1);
    if (_getHzFromODR(_odr) >= 4000) intCfg1 |= fastBits;
    else                             intCfg1 &= (uint8_t)~fastBits;
    writeRegister(ICM42688_REG_INT_CONFIG1, intCfg1);
  }

  uint8_t odd = (uint8_t)_odr;
  uint8_t gConf = readRegister(ICM42688_REG_GYRO_CONFIG0) & 0xF0;
  uint8_t aConf = readRegister(ICM42688_REG_ACCEL_CONFIG0) & 0xF0;
  writeRegister(ICM42688_REG_GYRO_CONFIG0, gConf | odd);
  writeRegister(ICM42688_REG_ACCEL_CONFIG0, aConf | odd);
}

void ICM42688P::setODR(ICM_ODR odr) {
  _requestedOdr = odr;
  enforceBandwidthLimit();
}

void ICM42688P::setAccelFS(ICM_ACCEL_FS fs) {
  if (_fifoMode == FIFO_20BIT_HIRES) fs = AFS_16G;

  _accelFs = fs;
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

  _gyroFs = fs;
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
    // Bit 6 = FIFO_RESUME_PARTIAL_RD. This driver reads the FIFO in bursts and
    // routinely stops before the buffer is empty, which is exactly the
    // "interrupted read" the bit governs: left clear, the read pointer rewinds
    // to the start of the FIFO and the next burst re-reads bytes already
    // consumed instead of advancing, so the FIFO never actually drains.
    writeRegister(ICM42688_REG_FIFO_CONFIG1,
                  ICM42688_BIT_FIFO_RESUME_PARTIAL_RD | 0x03);
    writeRegister(ICM42688_REG_FIFO_CONFIG, 0x40);    
  } 
  else if (mode == FIFO_20BIT_HIRES) {
    setAccelFS(AFS_16G);
    setGyroFS(GFS_2000DPS);
    // HIRES_EN + ACCEL_EN + GYRO_EN (all 3 bits required!)
    writeRegister(ICM42688_REG_FIFO_CONFIG1,
                  ICM42688_BIT_FIFO_RESUME_PARTIAL_RD | 0x13);
    writeRegister(ICM42688_REG_FIFO_CONFIG, 0x40);
  }

  if (mode != FIFO_NONE) {
    // UI_SIFS_CFG (bits 1:0) rides along untouched: in BUS_SPI it holds the
    // "disable I2C" setting from applyInterfaceConfig().
    //
    // FIFO_COUNT must report BYTES, big-endian, because that is what
    // availablePackets() and fillFIFOBuffer() decode. Both are the power-on
    // defaults, but a sketch (or a previous library) may have changed them, and
    // a count in RECORDS silently reads 16-20x too small: the drain then leaves
    // most of the FIFO behind on every pass and it fills up regardless of how
    // fast the loop runs.
    uint8_t intf = readRegister(ICM42688_REG_INTF_CONFIG0);
    intf &= (uint8_t)~ICM42688_BIT_FIFO_COUNT_REC;   // 0 = count in bytes
    intf |= ICM42688_BIT_FIFO_COUNT_ENDIAN;          // 1 = big endian
    writeRegister(ICM42688_REG_INTF_CONFIG0, intf);

    // Enable the FIFO_FULL condition so it latches in INT_STATUS and overflow
    // becomes detectable. Read-modify-write leaves the data-ready routing alone.
    // This also makes the INT1 pin pulse on FIFO full, which is harmless when
    // the pin is unused.
    //
    // Bit 1 in both registers: FIFO_FULL_INT1_EN here lines up with
    // FIFO_FULL_INT in INT_STATUS. FIFO_THS is bit 2 in both.
    uint8_t intSource = readRegister(ICM42688_REG_INT_SOURCE0);
    writeRegister(ICM42688_REG_INT_SOURCE0, intSource | ICM42688_INT_SOURCE0_FIFO_FULL);
  }

  enforceBandwidthLimit();
}

bool ICM42688P::stepDownODR() {
  // Descending ladder. The enum is not ordered by frequency - ODR_500HZ is 0xF,
  // below ODR_12_5HZ's 0xB - so the order has to be spelled out.
  static const ICM_ODR ladder[] = {
    ODR_32KHZ, ODR_16KHZ, ODR_8KHZ, ODR_4KHZ, ODR_2KHZ, ODR_1KHZ,
    ODR_500HZ, ODR_200HZ, ODR_100HZ, ODR_50HZ, ODR_25HZ, ODR_12_5HZ
  };
  const uint8_t n = sizeof(ladder) / sizeof(ladder[0]);

  uint8_t cur = n;
  for (uint8_t i = 0; i < n; i++) {
    if (ladder[i] == _odr) { cur = i; break; }
  }
  if (cur >= n - 1) return false;   // Unknown, or already at the bottom

  const ICM_ODR next = ladder[cur + 1];
  Serial.print(F("Voltino TriSense: "));
  Serial.print(_getHzFromODR(_odr));
  Serial.print(F(" Hz could not be serviced - falling back to "));
  Serial.print(_getHzFromODR(next));
  Serial.println(F(" Hz."));

  setODR(next);    // Also becomes the new request, so it will not be raised back
  flushFIFO();     // Anything still buffered was sampled at the old rate
  return true;
}

uint16_t ICM42688P::availablePackets() {
  if (_fifoMode == FIFO_NONE) return 1;

  uint16_t buffered = (_fifoBufIndex < _fifoBufCount) ? (uint16_t)(_fifoBufCount - _fifoBufIndex) : 0;
  if (_fifoPacketSize == 0) return buffered;

  uint8_t countBuf[2];
  readRegisters(ICM42688_REG_FIFO_COUNTH, countBuf, 2);
  uint16_t fifoBytes = ((uint16_t)countBuf[0] << 8) | countBuf[1];

  // Same sanity check fillFIFOBuffer() applies: a count beyond what the part can
  // physically report means the read was garbled, so trust only what is already
  // buffered. The bound includes the read cache - see ICM42688_FIFO_COUNT_MAX.
  if (fifoBytes > ICM42688_FIFO_COUNT_MAX) return buffered;

  return buffered + (fifoBytes / _fifoPacketSize);
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

uint32_t ICM42688P::getLostPacketCount() { return _lostPacketTotal; }

void ICM42688P::resetLostPacketCount() {
  _lostPacketTotal = 0;
  _lostPacketPrimed = false;   // Re-baseline against the chip's counter
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

  bool overflow = false;

  // Detector 1: the hardware's latched FIFO_FULL flag. INT_STATUS is
  // read-to-clear, so it is checked once per burst rather than once per packet.
  // This only reports anything because setFIFOMode() enables FIFO_FULL in
  // INT_SOURCE0 - the status bit does not latch for a disabled source.
  uint8_t intStatus = readRegister(ICM42688_REG_INT_STATUS);
  if (intStatus & ICM42688_INT_STATUS_FIFO_FULL) overflow = true;

  uint8_t countBuf[2];
  readRegisters(ICM42688_REG_FIFO_COUNTH, countBuf, 2);
  uint16_t fifoBytes = ((uint16_t)countBuf[0] << 8) | countBuf[1];

  // A count beyond what the part can physically report means a garbled read.
  if (fifoBytes > ICM42688_FIFO_COUNT_MAX) return 0;

  // Detector 2: config-free backstop. A count within one packet of the 2 KB
  // capacity means the FIFO is saturated, so samples are being dropped (or are
  // about to be). This needs no interrupt configuration at all, and therefore
  // still works if INT_SOURCE0 is overwritten by the sketch.
  if ((uint32_t)fifoBytes + _fifoPacketSize > ICM42688_FIFO_BYTES) overflow = true;

  if (overflow) {
    _fifoOverflowFlag = true;
    if (_fifoOverflowCount < 0xFFFFFFFFUL) _fifoOverflowCount++;

    // Both detectors above answer "is the FIFO full", which is not the same
    // question as "did we lose anything" - a FIFO sitting full loses nothing so
    // long as the drain keeps pace, and a single stalled loop loses hundreds of
    // packets while raising one flag. The part keeps the real figure itself, so
    // read it rather than inferring one. Polled only on an overflow event to
    // keep two register reads out of the normal path.
    uint8_t lost[2];
    readRegisters(ICM42688_REG_FIFO_LOST_PKT0, lost, 2);
    const uint16_t raw = (uint16_t)lost[0] | ((uint16_t)lost[1] << 8);

    if (!_lostPacketPrimed) {
      _lostPacketPrimed = true;
    } else {
      // Unsigned subtraction, so a wrap of the 16-bit register still yields the
      // right increment as long as fewer than 65536 packets were lost between
      // two overflow events.
      _lostPacketTotal += (uint32_t)(uint16_t)(raw - _lostPacketLastRaw);
    }
    _lostPacketLastRaw = raw;
  }

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

// Hands back the next usable packet in the burst buffer, refilling from the
// hardware when it runs dry, and returns nullptr when nothing is available.
//
// Header bit 7 marks an empty/message packet. Such a packet still occupies a
// full packet slot, so the ones behind it are perfectly good and stay aligned -
// it is skipped rather than treated as a fault. The previous code dropped the
// ENTIRE burst on the first one and made the caller abort its drain, which at a
// high ODR meant throwing away up to a hundred valid samples and then meeting
// the same packet again on the next pass: once one appeared, the driver could
// fall permanently behind the sensor.
const uint8_t* ICM42688P::nextFIFOPacket() {
  // Bounded so a FIFO returning nothing but 0xFF (read past the end, or a
  // sensor that has stopped streaming) cannot spin here forever.
  for (uint16_t guard = 0; guard < 2 * (uint16_t)FIFO_BURST_PACKETS + 4; guard++) {
    if (_fifoBufIndex >= _fifoBufCount) {
      if (fillFIFOBuffer() == 0) return nullptr;
    }
    const uint8_t* packet = _fifoBuf + (size_t)_fifoBufIndex * _fifoPacketSize;
    _fifoBufIndex++;
    if ((packet[0] & 0x80) == 0) return packet;
  }
  return nullptr;
}

bool ICM42688P::readHardwareFIFO(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  const uint8_t* buffer = nextFIFOPacket();
  if (buffer == nullptr) return false;

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
  const uint8_t* buffer = nextFIFOPacket();
  if (buffer == nullptr) return false;

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
  
  // On a timeout, drop to the next ODR down and try again rather than returning
  // an offset averaged from a handful of samples - or none at all.
  for (uint8_t attempt = 0; ; attempt++) {
    gxSum = 0; gySum = 0; gzSum = 0; count = 0;
    startT = millis();
    lastMicros = micros();
    bool timedOut = false;

    while(count < samples) {
      if (millis() - startT > CALIBRATION_TIMEOUT_MS) { 
        Serial.println(F("Error: Sensor read timeout during calibration."));
        timedOut = true;
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

    if (!timedOut) break;
    if (attempt + 1 >= CALIBRATION_ODR_FALLBACK_ATTEMPTS) break;
    if (!stepDownODR()) break;   // Already as low as it goes
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

      // A zero radius means this point sits exactly at the current centre
      // estimate - which in practice means the sensor returned all zeros (bus
      // fault, sensor asleep). Dividing by it produces NaN, and NaN then
      // propagates into bx/by/bz and sx/sy/sz and silently poisons the whole
      // calibration with no error message. Skip the point instead.
      if (radius < 1e-6f) continue;

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