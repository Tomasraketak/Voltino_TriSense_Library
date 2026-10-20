#include "ICM42688P_voltino.h"

// Constant lookup tables live in flash on AVR, where plain const data is
// copied to the (scarce) RAM at start-up. Elsewhere const data is already in
// flash and the accessors are plain reads.
#if defined(__AVR__)
  #include <avr/pgmspace.h>
  #define ICM_TABLE_ATTR PROGMEM
  #define ICM_TABLE_BYTE(p) pgm_read_byte(p)
  #define ICM_TABLE_WORD(p) pgm_read_word(p)
#else
  #define ICM_TABLE_ATTR
  #define ICM_TABLE_BYTE(p) (*(p))
  #define ICM_TABLE_WORD(p) (*(p))
#endif

// SPI pin release (see setSpiPinRelease()) needs a way to stop the MCU from
// driving SCK and MOSI between transactions and to hand them back to the SPI
// peripheral afterwards. There is no Arduino API for that - pinMode() does not
// know about peripheral pin muxing on most cores - so each architecture gets
// its own small backend, built on what that core itself uses:
//
//   RP2040 / RP2350 (arduino-pico)  gpio_set_function(): SPI <-> NULL
//   AVR (ATmega328P/32U4/2560 ...)  DDR bit: with the SPI enabled as master,
//   megaAVR (Nano Every, ATmega4809) the direction of SCK and MOSI is the
//                                   user's (ATmega328P / ATmega4809
//                                   datasheets, SPI pin overrides); the
//                                   value is the SPI's
//   SAMD21 / SAMD51                 PINCFG.PMUXEN via pinMode() / pinPeripheral()
//   ESP32 family (arduino-esp32)    GPIO matrix: the pin's out-select register
//                                   is saved, the pin made an input, and the
//                                   saved routing written back
//   Teensy 3.x / LC / 4.x           the pin's mux (and on 4.x pad) register is
//                                   saved, pinMode(INPUT) switches the pin to
//                                   GPIO, and the saved registers are written
//                                   back - what SPI.begin() itself writes
//   STM32 (stm32duino)              pinMode(INPUT) <-> pinmap_pinout() with the
//                                   core's SPI pin map, as the SPI driver does
//
// Every other core compiles the release to nothing and falls back to SPI mode
// 3 (see begin()), which removes the SCK half of the fight while the I2C bus
// is idle. Sketches never notice either way.
#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)) && !defined(ARDUINO_ARCH_MBED)
  #define ICM_SPI_PIN_RELEASE 1
  #define ICM_REL_RP 1
  #include <hardware/gpio.h>
#elif defined(TEENSYDUINO) && defined(__arm__)
  // Before the AVR test: the Teensy platform defines ARDUINO_ARCH_AVR too.
  #define ICM_SPI_PIN_RELEASE 1
  #define ICM_REL_TEENSY 1
#elif defined(ARDUINO_ARCH_STM32)
  #define ICM_SPI_PIN_RELEASE 1
  #define ICM_REL_STM32 1
  #include "pinmap.h"
  #include "PeripheralPins.h"
#elif defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
  #define ICM_SPI_PIN_RELEASE 1
  #define ICM_REL_AVR 1
#elif defined(ARDUINO_ARCH_SAMD)
  #define ICM_SPI_PIN_RELEASE 1
  #define ICM_REL_SAMD 1
  #include "wiring_private.h"
#elif defined(ESP32)
  #define ICM_SPI_PIN_RELEASE 1
  #define ICM_REL_ESP32 1
  #include "driver/gpio.h"
  #include "soc/gpio_reg.h"
  #include "soc/soc_caps.h"
  #include "soc/spi_periph.h"
  #define ICM_ESP_OUT_CFG(pin) (GPIO_FUNC0_OUT_SEL_CFG_REG + 4u * (uint32_t)(pin))
#endif

// ArduinoCore-API based cores (arduino-pico, SAMD, mbed, Renesas) take an
// SPIMode enum in SPISettings; the classic cores (AVR, ESP32) take a uint8_t.
#if defined(ARDUINO_API_VERSION)
  #define ICM_SPI_MODE_T SPIMode
#else
  #define ICM_SPI_MODE_T uint8_t
#endif


void ICM42688P::setDebug(bool enable) { _debug = enable; }

void ICM42688P::setWire(TwoWire &wire) { _wire = &wire; }

void ICM42688P::setSPI(TriSenseSPIClass &spi) {
  if (&spi == _spi) return;
  deactivateSpiPinRelease();     // Hand the old bus's pins back first
  _spi = &spi;
  _spiStarted = false;           // begin() starts the new bus
  if (!_relConfigured) { _relSck = -1; _relMosi = -1; }
}

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
        _spi->begin(sckSclPin, misoSdaPin, mosiPin, _csPin);
      } else {
        _spi->begin();
      }
    #elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
      if (sckSclPin != -1 && misoSdaPin != -1 && mosiPin != -1) {
        _spi->setSCK(sckSclPin);
        _spi->setRX(misoSdaPin);
        _spi->setTX(mosiPin);
      }
      _spi->begin();
    #else
      _spi->begin(); 
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

    // Where SCK and MOSI cannot be released (no backend for this core, or the
    // pins are unknown) at least idle SCK HIGH, the level the module drives it
    // to whenever the I2C bus is idle: SPI mode 3. That removes the SCK half of
    // the fight except during I2C traffic. The IMU takes mode 0 and mode 3
    // alike (DS-000347, DEVICE_CONFIG.SPI_MODE), so the data is unchanged. A
    // mode set with setSpiMode() is never overridden.
    if (!_relActive && !_spiModeSet) _spiMode = (uint8_t)SPI_MODE3;

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
// path takes its direction from CS. On RP2040/RP2350, where switching a pin
// between SPI and high-impedance costs two register writes and the SCK idle
// level is held by a pull (applySckIdlePull()), the MCU never drives against
// the shifter at all:
//   1. beginTransaction()
//   2. CS LOW         - the shifter turns, the connector side becomes an input
//   3. SCK/MOSI -> SPI function, now driving into an input
//   4. transfer
//   5. SCK/MOSI -> released (GPIO_FUNC_NULL, high impedance)
//   6. CS HIGH        - the shifter turns back and drives the pins again,
//                       which is harmless now that nothing drives against it
//   7. endTransaction()
//
// The other backends have no pull-down to rely on to hold an idle-LOW SCK, so a released
// SCK would float in step 3 and a float followed by the SPI snapping it to its
// idle level can be a clock edge. There the pins are taken BEFORE CS falls and
// released AFTER CS rises: the SPI drives SCK at its idle level the whole time
// CS is low, and the two drivers overlap only for the few microseconds between
// the pin switch and the CS edge - a duty cycle of well under 1 %, instead of
// the near-100 % of a pin that is never released.
//
// A FIFO burst is a single transaction, so it pays for one take/release pair
// no matter how many bytes it moves.

void ICM42688P::spiSelect() {
  _spi->beginTransaction(SPISettings(_spiFreq, MSBFIRST, (ICM_SPI_MODE_T)_spiMode));
#if defined(ICM_REL_RP)
  if (_csPin != -1) digitalWrite(_csPin, LOW);
  if (_relActive) relTakePins();
#else
  if (_relActive) relTakePins();
  if (_csPin != -1) digitalWrite(_csPin, LOW);
#endif
}

void ICM42688P::spiDeselect() {
#if defined(ICM_REL_RP)
  if (_relActive) relReleasePins();
  if (_csPin != -1) digitalWrite(_csPin, HIGH);
#else
  if (_csPin != -1) digitalWrite(_csPin, HIGH);
  if (_relActive) relReleasePins();
#endif
  _spi->endTransaction();
}

// Hand SCK and MOSI to the SPI peripheral.
void ICM42688P::relTakePins() {
#if defined(ICM_REL_RP)
  gpio_set_function((uint)_relSck, GPIO_FUNC_SPI);
  gpio_set_function((uint)_relMosi, GPIO_FUNC_SPI);
#elif defined(ICM_REL_AVR)
  pinMode(_relSck, OUTPUT);       // value comes from the SPI, not PORT
  pinMode(_relMosi, OUTPUT);
#elif defined(ICM_REL_TEENSY)
  *portConfigRegister(_relSck)  = _relSckCfg;     // mux back to the SPI
  *portConfigRegister(_relMosi) = _relMosiCfg;
  #if defined(__IMXRT1062__)
  *portControlRegister(_relSck)  = _relSckPad;    // and its pad settings
  *portControlRegister(_relMosi) = _relMosiPad;
  #endif
#elif defined(ICM_REL_STM32)
  pinmap_pinout(digitalPinToPinName(_relSck), PinMap_SPI_SCLK);
  pinmap_pinout(digitalPinToPinName(_relMosi), PinMap_SPI_MOSI);
#elif defined(ICM_REL_SAMD)
  pinPeripheral(_relSck, g_APinDescription[_relSck].ulPinType);
  pinPeripheral(_relMosi, g_APinDescription[_relMosi].ulPinType);
#elif defined(ICM_REL_ESP32)
  // gpio_set_direction(OUTPUT) briefly routes the plain GPIO latch to the pin
  // before the saved SPI routing is written back, so the latch is set to the
  // SPI idle level first: no edge on SCK in that window.
  gpio_set_level((gpio_num_t)_relSck, _spiMode == (uint8_t)SPI_MODE3 ? 1 : 0);
  gpio_set_direction((gpio_num_t)_relSck, GPIO_MODE_INPUT_OUTPUT);
  REG_WRITE(ICM_ESP_OUT_CFG(_relSck), _relSckCfg);
  gpio_set_direction((gpio_num_t)_relMosi, GPIO_MODE_INPUT_OUTPUT);
  REG_WRITE(ICM_ESP_OUT_CFG(_relMosi), _relMosiCfg);
#endif
}

// Make SCK and MOSI high-impedance inputs.
void ICM42688P::relReleasePins() {
#if defined(ICM_REL_RP)
  gpio_set_function((uint)_relSck, GPIO_FUNC_NULL);
  gpio_set_function((uint)_relMosi, GPIO_FUNC_NULL);
#elif defined(ICM_REL_AVR) || defined(ICM_REL_SAMD) || defined(ICM_REL_TEENSY) || defined(ICM_REL_STM32)
  // AVR/SAMD: input, no pull-up. Teensy: mux to GPIO, input, no pull.
  // STM32: GPIO input, no pull - the alternate function is no longer driven.
  pinMode(_relSck, INPUT);
  pinMode(_relMosi, INPUT);
#elif defined(ICM_REL_ESP32)
  gpio_set_direction((gpio_num_t)_relSck, GPIO_MODE_INPUT);   // unroutes the pin
  gpio_set_direction((gpio_num_t)_relMosi, GPIO_MODE_INPUT);
#endif
}

#if defined(ICM_REL_ESP32)
// The out-select field of a pin's GPIO matrix register: which peripheral
// signal drives it. Returns the index of the SPI host whose clock (what = 0)
// or data-out (what = 1) signal that is, or -1. Host 0 is the flash SPI and is
// never ours.
static int icmEspSpiHostOf(int8_t pin, int what) {
  if (pin < 0 || pin >= SOC_GPIO_PIN_COUNT || !GPIO_IS_VALID_OUTPUT_GPIO(pin)) return -1;
  const uint32_t sel = REG_READ(ICM_ESP_OUT_CFG(pin)) & GPIO_FUNC0_OUT_SEL_V;
  for (int h = 1; h < SOC_SPI_PERIPH_NUM; h++) {
    const uint32_t sig = what ? spi_periph_signal[h].spid_out : spi_periph_signal[h].spiclk_out;
    if (sel == sig) return h;
  }
  return -1;
}
#endif

#if defined(ICM_REL_RP)
// Which RP2040/RP2350 SPI block the driver's bus object drives: 0 for SPI,
// 1 for SPI1, -1 for a sketch's own SPIClassRP2040 instance (then only
// explicitly named pins are used).
static int icmRpSpiIndex(const TriSenseSPIClass *spi) {
  if (spi == &SPI)  return 0;
  if (spi == &SPI1) return 1;
  return -1;
}
#endif

// Can these two pins be released? Checked when release is armed, i.e. after
// the bus's begin(), so on ESP32 the pins' actual routing can be checked too.
bool ICM42688P::spiPinsValid(int8_t sck, int8_t mosi) {
  if (sck < 0 || mosi < 0 || sck == mosi) return false;
#if defined(ICM_REL_RP)
  // GPIO function 1 (SPI) is fixed per pin: GPIO n belongs to SPI((n >> 3) & 1)
  // and n % 4 selects the signal - 0 RX, 1 CSn, 2 SCK, 3 TX. Checking against
  // that table means a wrong pin number can never route an SPI signal onto a
  // pin that is wired to something else. Both pins must belong to one block,
  // and to the block of the bus the driver was given, when that is known.
  if (sck >= (int8_t)NUM_BANK0_GPIOS || mosi >= (int8_t)NUM_BANK0_GPIOS) return false;
  if ((sck & 3) != 2 || (mosi & 3) != 3) return false;
  const int block = (sck >> 3) & 1;
  if (((mosi >> 3) & 1) != block) return false;
  const int idx = icmRpSpiIndex(_spi);
  return idx < 0 || idx == block;
#elif defined(ICM_REL_AVR)
  // Only the hardware SPI's own, fixed pins.
  return _spi == &SPI && sck == (int8_t)SCK && mosi == (int8_t)MOSI;
#elif defined(ICM_REL_TEENSY)
  // Both pins must currently be muxed to a peripheral, not to GPIO: that is
  // what the bus's begin() leaves on its SCK and MOSI, and it keeps release
  // away from a pin the sketch uses as plain I/O.
  if (sck >= (int8_t)CORE_NUM_DIGITAL || mosi >= (int8_t)CORE_NUM_DIGITAL) return false;
  #if defined(__IMXRT1062__)
  // IMXRT1062 mux: ALT5 is GPIO.
  return (*portConfigRegister(sck) & 7) != 5 && (*portConfigRegister(mosi) & 7) != 5;
  #else
  // Kinetis PORTx_PCRn.MUX: 0 = disabled/analog, 1 = GPIO, >= 2 peripherals.
  return ((*portConfigRegister(sck) >> 8) & 7) >= 2 && ((*portConfigRegister(mosi) >> 8) & 7) >= 2;
  #endif
#elif defined(ICM_REL_STM32)
  // Pins that can carry this signal at all, and are in alternate-function
  // mode right now (left there by the bus's begin()).
  {
    const PinName ps = digitalPinToPinName(sck), pm = digitalPinToPinName(mosi);
    if (ps == NC || pm == NC) return false;
    if (pinmap_find_function(ps, PinMap_SPI_SCLK) == (uint32_t)NC) return false;
    if (pinmap_find_function(pm, PinMap_SPI_MOSI) == (uint32_t)NC) return false;
    return LL_GPIO_GetPinMode(get_GPIO_Port(STM_PORT(ps)), STM_LL_GPIO_PIN(ps)) == LL_GPIO_MODE_ALTERNATE &&
           LL_GPIO_GetPinMode(get_GPIO_Port(STM_PORT(pm)), STM_LL_GPIO_PIN(pm)) == LL_GPIO_MODE_ALTERNATE;
  }
#elif defined(ICM_REL_SAMD)
  if (sck >= (int8_t)PINS_COUNT || mosi >= (int8_t)PINS_COUNT) return false;
  const EPioType ts = g_APinDescription[sck].ulPinType;
  const EPioType tm = g_APinDescription[mosi].ulPinType;
  return (ts == PIO_SERCOM || ts == PIO_SERCOM_ALT) && (tm == PIO_SERCOM || tm == PIO_SERCOM_ALT);
#elif defined(ICM_REL_ESP32)
  // Both pins must currently carry the clock and data-out of the same SPI host.
  const int h = icmEspSpiHostOf(sck, 0);
  return h > 0 && icmEspSpiHostOf(mosi, 1) == h;
#else
  return false;
#endif
}

// For begin() without explicit pins: find the pins the global SPI object is
// really using. Accepted only when that is unambiguous; otherwise release
// stays off rather than guess. No private SPIClass state is touched.
bool ICM42688P::detectSpiPins(int8_t &sck, int8_t &mosi) {
  sck = -1;
  mosi = -1;
#if defined(ICM_REL_RP)
  // After the bus's begin() the pins the core chose are the ones in SPI
  // function. Only the driver's own block counts: the other block being in
  // use is not ambiguous, but two SCK pins muxed to the same block would be.
  const int idx = icmRpSpiIndex(_spi);              // -1: own instance, name the pins
  int8_t foundSck = -1, foundMosi = -1;
  uint8_t nSck = 0, nMosi = 0;
  for (uint pin = 0; idx >= 0 && pin < NUM_BANK0_GPIOS; pin++) {
    if (gpio_get_function(pin) != GPIO_FUNC_SPI) continue;
    if ((int)((pin >> 3) & 1) != idx) continue;     // the other block
    if ((pin & 3) == 2) { foundSck = (int8_t)pin; nSck++; }
    if ((pin & 3) == 3) { foundMosi = (int8_t)pin; nMosi++; }
  }
  if (nSck == 1 && nMosi == 1) { sck = foundSck; mosi = foundMosi; }
#elif defined(ICM_REL_AVR) || defined(ICM_REL_TEENSY)
  // The core's default SPI pins. If the sketch moved SPI elsewhere (Teensy
  // setSCK()), these are not muxed to it and spiPinsValid() refuses them.
  if (_spi == &SPI) { sck = (int8_t)SCK; mosi = (int8_t)MOSI; }
#elif defined(ICM_REL_STM32)
  #if defined(PIN_SPI_SCK) && defined(PIN_SPI_MOSI)
  if (_spi == &SPI) { sck = (int8_t)PIN_SPI_SCK; mosi = (int8_t)PIN_SPI_MOSI; }
  #endif
#elif defined(ICM_REL_SAMD)
  if (_spi == &SPI) { sck = (int8_t)PIN_SPI_SCK; mosi = (int8_t)PIN_SPI_MOSI; }
  #if SPI_INTERFACES_COUNT > 1 && defined(PIN_SPI1_SCK) && defined(PIN_SPI1_MOSI)
  if (_spi == &SPI1) { sck = (int8_t)PIN_SPI1_SCK; mosi = (int8_t)PIN_SPI1_MOSI; }
  #endif
#elif defined(ICM_REL_ESP32)
  // The bus's begin() may have been given any pins, and SPIClass does not
  // say which host it is, so look for the one pin routed to an SPI clock and
  // the one routed to that host's data-out. With two SPI buses running this
  // is ambiguous and the pins have to be named.
  uint8_t nSck = 0, nMosi = 0;
  int8_t foundSck = -1, foundMosi = -1;
  for (int pin = 0; pin < SOC_GPIO_PIN_COUNT; pin++) {
    if (icmEspSpiHostOf((int8_t)pin, 0) > 0) { foundSck = (int8_t)pin; nSck++; }
    if (icmEspSpiHostOf((int8_t)pin, 1) > 0) { foundMosi = (int8_t)pin; nMosi++; }
  }
  if (nSck == 1 && nMosi == 1) { sck = foundSck; mosi = foundMosi; }
#endif
  if (sck >= 0 && mosi >= 0) return true;
  if (_debug) Serial.println(F("ICM: SPI pins not detected - pin release off. Use setSpiPinRelease()."));
  sck = -1;
  mosi = -1;
  return false;
}

// RP2040/RP2350 only. While released, SCK is held at its idle level by a pull
// resistor. Without it, the pin floats between CS falling and the SPI function
// taking over (and again between release and CS rising), and a float to the
// non-idle level followed by the snap back would be a clock edge the IMU
// counts as a bit. MOSI is only sampled on SCK edges, so its pulls are simply
// switched off. The other backends avoid the float by ordering instead.
void ICM42688P::applySckIdlePull() {
#if defined(ICM_REL_RP)
  if (_spiMode == (uint8_t)SPI_MODE3) gpio_pull_up((uint)_relSck);
  else                                gpio_pull_down((uint)_relSck);
#endif
}

void ICM42688P::activateSpiPinRelease() {
#if defined(ICM_SPI_PIN_RELEASE)
  if (_relActive) return;
  if (!_spiStarted || _bus != BUS_SPI || _csPin < 0) return;  // CS must be ours to order it
  if (!spiPinsValid(_relSck, _relMosi)) {
    if (_debug && (_relSck >= 0 || _relMosi >= 0))
      Serial.println(F("ICM: SCK/MOSI are not this SPI bus's pins - pin release off."));
    return;
  }
#if defined(ICM_REL_RP)
  applySckIdlePull();
  gpio_disable_pulls((uint)_relMosi);
#elif defined(ICM_REL_ESP32)
  // The routing to give back on every transaction.
  _relSckCfg  = REG_READ(ICM_ESP_OUT_CFG(_relSck));
  _relMosiCfg = REG_READ(ICM_ESP_OUT_CFG(_relMosi));
#elif defined(ICM_REL_TEENSY)
  // The mux (and pad) values the bus's begin() wrote, to write back.
  _relSckCfg  = *portConfigRegister(_relSck);
  _relMosiCfg = *portConfigRegister(_relMosi);
  #if defined(__IMXRT1062__)
  _relSckPad  = *portControlRegister(_relSck);
  _relMosiPad = *portControlRegister(_relMosi);
  #endif
#endif
  relReleasePins();
  _relActive = true;
#endif
}

void ICM42688P::deactivateSpiPinRelease() {
#if defined(ICM_SPI_PIN_RELEASE)
  if (!_relActive) return;
  relTakePins();
  _relActive = false;
#endif
}

bool ICM42688P::setSpiPinRelease(int8_t sckPin, int8_t mosiPin) {
  _relConfigured = true;
  deactivateSpiPinRelease();   // Hand the old pins back before switching

  if (sckPin < 0 || mosiPin < 0) {
    _relSck = -1;
    _relMosi = -1;
    return false;
  }
  _relSck = sckPin;
  _relMosi = mosiPin;
#if defined(ICM_SPI_PIN_RELEASE)
  if (!_spiStarted) return true;   // Armed by begin(), validated there
  activateSpiPinRelease();
  return _relActive;
#else
  if (_debug) Serial.println(F("ICM: setSpiPinRelease() - not supported on this core."));
  return false;
#endif
}

bool ICM42688P::isSpiPinReleaseActive() { return _relActive; }

bool ICM42688P::setSpiMode(uint8_t mode) {
  if (mode != (uint8_t)SPI_MODE0 && mode != (uint8_t)SPI_MODE3) return false;
  _spiMode = mode;
  _spiModeSet = true;
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
    _spi->transfer(startReg | 0x80); 
    for(size_t i=0; i<len; i++) buffer[i] = _spi->transfer(0x00);
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
    _spi->transfer(reg & 0x7F); 
    _spi->transfer(data);
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
    _spi->transfer(reg | 0x80); 
    data = _spi->transfer(0x00);
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

// Every supported ODR from fastest to slowest. The enum is not ordered by
// frequency - ODR_500HZ is 0xF, below ODR_12_5HZ's 0xB - so the order has to be
// spelled out. Shared by enforceBandwidthLimit() and stepDownODR().
static const uint8_t kOdrLadder[] ICM_TABLE_ATTR = {
  ODR_32KHZ, ODR_16KHZ, ODR_8KHZ, ODR_4KHZ, ODR_2KHZ, ODR_1KHZ,
  ODR_500HZ, ODR_200HZ, ODR_100HZ, ODR_50HZ, ODR_25HZ, ODR_12_5HZ
};
static const uint8_t kOdrLadderLen = sizeof(kOdrLadder);

// Rate in Hz indexed by the ODR register value (0 = reserved).
static const uint16_t kOdrHz[16] ICM_TABLE_ATTR = {
  0, 32000, 16000, 8000, 4000, 2000, 1000, 200, 100, 50, 25, 12, 0, 0, 0, 500
};

static inline ICM_ODR odrLadderAt(uint8_t i) { return (ICM_ODR)ICM_TABLE_BYTE(&kOdrLadder[i]); }

int ICM42688P::_getHzFromODR(ICM_ODR odr) {
  return ((uint8_t)odr < 16) ? (int)ICM_TABLE_WORD(&kOdrHz[(uint8_t)odr]) : 0;
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
  ICM_ODR safeODR = _requestedOdr;
  int requestedHz = _getHzFromODR(_requestedOdr);

  for (uint8_t i = 0; i < kOdrLadderLen; i++) {
    int targetHz = _getHzFromODR(odrLadderAt(i));
    if (targetHz > requestedHz) continue; 
    
    uint32_t requiredBps = bitsPerRead * targetHz;
    if (requiredBps <= maxAllowedBps) {
      safeODR = odrLadderAt(i);
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
  // A LITE build has no HiRes path (see TRISENSE_LITE). This used to test
  // ARDUINO_ARCH_AVR as well, which older Teensyduino releases define for
  // Teensy 3.x/4.x too, so those silently got 16-bit packets instead of HiRes.
  #if TRISENSE_LITE
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
  }

  enforceBandwidthLimit();
}

bool ICM42688P::stepDownODR() {
  const uint8_t n = kOdrLadderLen;

  uint8_t cur = n;
  for (uint8_t i = 0; i < n; i++) {
    if (odrLadderAt(i) == _odr) { cur = i; break; }
  }
  if (cur >= n - 1) return false;   // Unknown, or already at the bottom

  const ICM_ODR next = odrLadderAt(cur + 1);
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

  uint8_t countBuf[2];
  readRegisters(ICM42688_REG_FIFO_COUNTH, countBuf, 2);
  uint16_t fifoBytes = ((uint16_t)countBuf[0] << 8) | countBuf[1];

  // A count beyond what the part can physically report means a garbled read.
  if (fifoBytes > ICM42688_FIFO_COUNT_MAX) return 0;

  // Overflow detection rests on the FIFO count alone. In Stream mode a FIFO
  // that filled up stays full until it is read - only these reads drain it -
  // so every overflow is still visible here, at the next refill, as a count
  // within two packets of the 2 KB capacity (one packet is reserved by the
  // chip, and the read cache can add one or two; datasheet 6.3).
  //
  // The hardware FIFO_FULL bit in INT_STATUS is deliberately NOT used. On a
  // real module at 8 kHz in HiRes mode it read as set on almost every refill
  // while the FIFO held one or two packets and every sample arrived (fusion
  // rate == ODR), so it reported overflows that did not happen.
  const bool overflow =
      (uint32_t)fifoBytes + 2u * _fifoPacketSize >= ICM42688_FIFO_BYTES;

  if (overflow) {
    _fifoOverflowFlag = true;
    if (_fifoOverflowCount < 0xFFFFFFFFUL) _fifoOverflowCount++;

    // A full FIFO is not the same as lost data: a FIFO sitting full loses
    // nothing as long as the drain keeps pace, and one stalled loop loses
    // hundreds of packets while raising one flag. The part keeps its own
    // tally, so read that - only here, to keep two register reads out of the
    // normal path.
    //
    // Byte order: the datasheet's register description puts the LOW byte at
    // 0x6C; its register map table says the opposite. This follows the
    // description. If the lost count only ever moves in steps of 256, it is
    // the other way round.
    uint8_t lost[2];
    readRegisters(ICM42688_REG_FIFO_LOST_PKT0, lost, 2);
    const uint16_t raw = (uint16_t)lost[0] | ((uint16_t)lost[1] << 8);

    if (!_lostPacketPrimed) {
      _lostPacketPrimed = true;
    } else if (raw >= _lostPacketLastRaw) {
      _lostPacketTotal += (uint32_t)(raw - _lostPacketLastRaw);
    } else {
      // The counter went DOWN. That is a reset (FIFO flush, re-configuration,
      // or a counter that clears itself), not a wrap: a 16-bit wrap would need
      // 65536 lost packets between two refills. Treating it as a wrap - as an
      // earlier version did - added ~65536 phantom packets every time.
      _lostPacketTotal += raw;
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
#if !TRISENSE_LITE
    case FIFO_20BIT_HIRES: return readHardwareFIFOHires(ax, ay, az, gx, gy, gz);
#endif
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

  const int32_t raw[6] = { rawAx, rawAy, rawAz, rawGx, rawGy, rawGz };
  scaleSample(raw, _accelScaleFactor, _gyroScaleFactor, ax, ay, az, gx, gy, gz);
  return true;
}

// Raw counts to calibrated units, shared by all three read paths so the
// arithmetic exists once. Every raw value - 16-bit or 20-bit - converts to
// float exactly, so routing the 16-bit paths through int32_t changes nothing.
void ICM42688P::scaleSample(const int32_t raw[6], float aScale, float gScale,
                            float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  ax = ((float)raw[0] * aScale - accOffset[0]) * accScale[0];
  ay = ((float)raw[1] * aScale - accOffset[1]) * accScale[1];
  az = ((float)raw[2] * aScale - accOffset[2]) * accScale[2];

  gx = (float)raw[3] * gScale - gyrOffset[0];
  gy = (float)raw[4] * gScale - gyrOffset[1];
  gz = (float)raw[5] * gScale - gyrOffset[2];
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

  const int32_t raw[6] = { rawAx, rawAy, rawAz, rawGx, rawGy, rawGz };
  scaleSample(raw, _accelScaleFactor, _gyroScaleFactor, ax, ay, az, gx, gy, gz);
  return true;
}

// [VOLTINO FIX] PERFEKTNÍ 20-BIT PARSOVÁNÍ PODLE TDK DATASHEETU
#if !TRISENSE_LITE
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

  const int32_t raw[6] = { rawAx, rawAy, rawAz, rawGx, rawGy, rawGz };
  scaleSample(raw, scaleAccel20, scaleGyro20, ax, ay, az, gx, gy, gz);
  return true;
}
#endif

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