#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Constructors are constexpr only on AVR. There a global object is then laid
// out by the compiler instead of built by start-up code, which saves flash. On
// 32-bit cores the compiler would instead store the whole object - including
// the multi-KB zeroed FIFO buffer - as initialised data in flash, so it costs
// more than it saves there.
#ifndef TRISENSE_CTOR_CONSTEXPR
  #if defined(__AVR__)
    #define TRISENSE_CTOR_CONSTEXPR constexpr
  #else
    #define TRISENSE_CTOR_CONSTEXPR
  #endif
#endif

// The concrete class of the core's SPI objects - SPIClass on AVR/ESP32,
// SPIClassRP2040 on arduino-pico, SPIClassSAMD on SAMD, and so on. Taken from
// the global SPI object itself, so every core gets its own type and the
// driver can call that core's SPI methods (setSCK() on RP2040, begin(pins) on
// ESP32) on whichever bus the sketch hands it - SPI, SPI1, ...
template <typename T> struct TriSenseNoRef { typedef T type; };
template <typename T> struct TriSenseNoRef<T &> { typedef T type; };
typedef TriSenseNoRef<decltype(SPI)>::type TriSenseSPIClass;

// TRISENSE_LITE leaves out code that cannot run on the target anyway, to make
// room on 8-bit AVRs (Uno, Leonardo, Nano Every, Teensy 2.0 ...). It is on by
// default there and off everywhere else; behaviour is the same either way.
// Today it removes the 20-bit HiRes FIFO path - an AVR has always been given
// 16-bit packets instead - and sizes the FIFO buffer for 16-byte packets.
// Override with -DTRISENSE_LITE=0 or =1.
//
// Tested with __AVR__ alone, never ARDUINO_ARCH_AVR: older Teensyduino releases
// define ARDUINO_ARCH_AVR for Teensy 3.x/4.x too, which have the FPU and RAM for
// HiRes.
#if !defined(TRISENSE_LITE)
  #if defined(__AVR__)
    #define TRISENSE_LITE 1
  #else
    #define TRISENSE_LITE 0
  #endif
#endif

// Register definitions
#define ICM42688_REG_DEVICE_CONFIG 0x11
#define ICM42688_REG_DRIVE_CONFIG  0x13
#define ICM42688_REG_INT_CONFIG    0x14
#define ICM42688_REG_FIFO_CONFIG   0x16
#define ICM42688_REG_TEMP_DATA1    0x1D
#define ICM42688_REG_ACCEL_DATA_X1 0x1F
#define ICM42688_REG_GYRO_DATA_X1  0x25
#define ICM42688_REG_INT_STATUS    0x2D // Read-to-clear; bit 1 = FIFO_FULL
#define ICM42688_REG_FIFO_COUNTH   0x2E
#define ICM42688_REG_FIFO_COUNTL   0x2F
#define ICM42688_REG_FIFO_DATA     0x30
#define ICM42688_REG_SIGNAL_PATH_RESET 0x4B // [VOLTINO FIX] Added for FIFO flushing
#define ICM42688_REG_TMST_CONFIG   0x54
#define ICM42688_REG_APEX_CONFIG0  0x56
#define ICM42688_REG_SMD_CONFIG    0x57
#define ICM42688_REG_FIFO_CONFIG1  0x5F
#define ICM42688_REG_FIFO_CONFIG2  0x60
#define ICM42688_REG_FIFO_CONFIG3  0x61
#define ICM42688_REG_INT_SOURCE0   0x65
// INT_CONFIG1 (0x64). Both of these MUST be set once ODR reaches 4 kHz: the
// default interrupt pulse is 100 us and the mandatory de-assert window another
// 100 us, while a 4 kHz sample period is 250 us and a 32 kHz one just 31 us.
#define ICM42688_REG_INT_CONFIG1   0x64
#define ICM42688_BIT_INT_TPULSE_DURATION    0x40  // 1 = 8 us pulse (else 100 us)
#define ICM42688_BIT_INT_TDEASSERT_DISABLE  0x20  // 1 = no 100 us de-assert wait
#define ICM42688_REG_WHO_AM_I      0x75
#define ICM42688_REG_BANK_SEL      0x76
#define ICM42688_REG_PWR_MGMT0     0x4E
#define ICM42688_REG_GYRO_CONFIG0  0x4F
#define ICM42688_REG_ACCEL_CONFIG0 0x50

// --- Filter registers (DS-000347, sections 5.2-5.4 and the register map) ---
// Bank 0 - UI filter (the decimation filter after the AAF). Read-modify-write:
// the other fields of GYRO_CONFIG1 / ACCEL_CONFIG1 are left at their values.
#define ICM42688_REG_GYRO_CONFIG1       0x51  // [3:2] GYRO_UI_FILT_ORD, [1:0] GYRO_DEC2_M2_ORD; reset 0x16
#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52  // [7:4] ACCEL_UI_FILT_BW, [3:0] GYRO_UI_FILT_BW; reset 0x11
#define ICM42688_REG_ACCEL_CONFIG1      0x53  // [4:3] ACCEL_UI_FILT_ORD, [2:1] ACCEL_DEC2_M2_ORD; reset 0x0D
#define ICM42688_SHIFT_GYRO_UI_FILT_ORD   2
#define ICM42688_SHIFT_ACCEL_UI_FILT_ORD  3
// Bank 1 - gyro anti-alias filter
#define ICM42688_B1_GYRO_CONFIG_STATIC2  0x0B  // bit 1 GYRO_AAF_DIS, bit 0 GYRO_NF_DIS; reset 0xA0
#define ICM42688_B1_GYRO_CONFIG_STATIC3  0x0C  // [5:0] GYRO_AAF_DELT; reset 0x0D (13)
#define ICM42688_B1_GYRO_CONFIG_STATIC4  0x0D  // [7:0] GYRO_AAF_DELTSQR[7:0]; reset 0xAA (170)
#define ICM42688_B1_GYRO_CONFIG_STATIC5  0x0E  // [7:4] GYRO_AAF_BITSHIFT, [3:0] GYRO_AAF_DELTSQR[11:8]; reset 0x80
#define ICM42688_BIT_GYRO_AAF_DIS        0x02
// Bank 2 - accel anti-alias filter
#define ICM42688_B2_ACCEL_CONFIG_STATIC2 0x03  // [6:1] ACCEL_AAF_DELT, bit 0 ACCEL_AAF_DIS; reset 0x30 (24)
#define ICM42688_B2_ACCEL_CONFIG_STATIC3 0x04  // [7:0] ACCEL_AAF_DELTSQR[7:0]; reset 0x40
#define ICM42688_B2_ACCEL_CONFIG_STATIC4 0x05  // [7:4] ACCEL_AAF_BITSHIFT, [3:0] ACCEL_AAF_DELTSQR[11:8]; reset 0x62
#define ICM42688_BIT_ACCEL_AAF_DIS       0x01
// Power-on AAF settings: gyro DELT 13 = 585 Hz, accel DELT 24 = 1163 Hz.
#define ICM42688_AAF_GYRO_RESET_DELT     13
#define ICM42688_AAF_ACCEL_RESET_DELT    24
#define ICM42688_AAF_DELT_MAX            63
// UI_FILT_BW power-on value: 1 = max(400 Hz, ODR)/4
#define ICM42688_UI_FILT_BW_RESET        1

// FIFO packet header (byte 0) bits 3:2: 0b10 = the packet carries the ODR
// timestamp (TMST_CONFIG.TMST_EN, on at power-on, 1 us per LSB at TMST_RES = 0).
#define ICM42688_FIFO_HEADER_MSG         0x80  // Empty / message packet
#define ICM42688_FIFO_HEADER_ACCEL       0x40  // Packet carries accel data
#define ICM42688_FIFO_HEADER_GYRO        0x20  // Packet carries gyro data
#define ICM42688_FIFO_HEADER_20          0x10  // 20-bit packet (packet 4)
#define ICM42688_FIFO_HEADER_TMST_MASK   0x0C
#define ICM42688_FIFO_HEADER_TMST_ODR    0x08

#define ICM_ADDR_PRIMARY 0x68
#define ICM_ADDR_SECONDARY 0x69
#define WHO_AM_I_EXPECTED 0x47

// FIFO_FULL is bit 1 in BOTH registers, and FIFO_THS is bit 2 in both:
//   INT_STATUS  (0x2D) bit 1 = FIFO_FULL_INT      (bit 2 = FIFO_THS_INT)
//   INT_SOURCE0 (0x65) bit 1 = FIFO_FULL_INT1_EN  (bit 2 = FIFO_THS_INT1_EN)
#define ICM42688_INT_STATUS_FIFO_FULL   0x02
#define ICM42688_INT_SOURCE0_FIFO_FULL  0x02

// Hardware FIFO is 2 KB: 128 x 16-byte packets, or 102 x 20-byte packets.
#define ICM42688_FIFO_BYTES 2048

// FIFO_COUNT may legitimately exceed the 2 KB array. On top of that memory the
// part has a read cache two packets wide, so the datasheet puts the reachable
// total at 2048 bytes (2040 for 20-byte packets) plus one packet, and asks for
// 2080 bytes of driver allocation because bus timing is non-deterministic.
// Sanity-checking a count against 2048 therefore rejects counts that are
// perfectly valid - and a rejected count means zero packets returned, so the
// drain stalls exactly when the FIFO is at its fullest.
#define ICM42688_FIFO_COUNT_MAX 2080

// INTF_CONFIG0 (0x4C, bank 0) - FIFO count format and serial interface select.
// Bit layout per ICM-42688-P datasheet DS-000347, register map section
// "INTF_CONFIG0" (reset value 0x30):
//   bit 7    FIFO_HOLD_LAST_DATA_EN
//   bit 6    FIFO_COUNT_REC      1 = count in records, 0 = count in bytes
//   bit 5    FIFO_COUNT_ENDIAN   1 = big endian (reset value)
//   bit 4    SENSOR_DATA_ENDIAN  1 = big endian (reset value)
//   bit 3:2  reserved
//   bit 1:0  UI_SIFS_CFG         0x = reserved, 10 = disable SPI, 11 = disable I2C
#define ICM42688_REG_INTF_CONFIG0       0x4C
#define ICM42688_BIT_FIFO_COUNT_REC     0x40  // 1 = count in records, 0 = count in BYTES
#define ICM42688_BIT_FIFO_COUNT_ENDIAN  0x20  // 1 = big endian (power-on default)
#define ICM42688_MASK_UI_SIFS_CFG       0x03
#define ICM42688_UI_SIFS_DISABLE_I2C    0x03  // 0b11

// PWR_MGMT0 (0x4E). 0x0F = gyro + accel in Low Noise mode; 0x00 = both off
// (sleep). Datasheet timing rules this driver honours:
//   - after switching the gyro/accel on from OFF, no register writes for 200 us;
//   - the gyro must stay on for at least 45 ms once enabled;
//   - gyro start-up from sleep to valid data is 30 ms (typ.), the accel's is
//     shorter, so the gyro sets the wait after wake().
#define ICM42688_PWR_SENSORS_LN         0x0F
#define ICM42688_PWR_SENSORS_OFF        0x00
#define ICM42688_PWR_WRITE_HOLDOFF_US   200
#define ICM42688_GYRO_MIN_ON_MS         45
#define ICM42688_GYRO_STARTUP_MS        30

// FIFO_CONFIG1 (0x5F) bit 6. With this clear, a read that does not consume the
// whole FIFO is not resumable: the next read restarts from the beginning.
#define ICM42688_BIT_FIFO_RESUME_PARTIAL_RD 0x40

// Hardware counter of packets discarded on overflow in Stream mode (16-bit).
#define ICM42688_REG_FIFO_LOST_PKT0 0x6C
#define ICM42688_REG_FIFO_LOST_PKT1 0x6D

// Upper bound on any blocking calibration routine. These wait for the sensor to
// deliver samples, and a sensor that has stopped delivering must not hang the
// sketch: without a bound the call never returns and nothing is ever printed
// again, which is indistinguishable from a crash.
#define CALIBRATION_TIMEOUT_MS 10000UL

// How many times a blocking calibration may drop to the next ODR down and try
// again before giving up. Collecting a thousand samples takes well under a
// second at any rate the sensor offers, so hitting the timeout at all means the
// configured rate cannot be serviced - a lower one usually can.
#define CALIBRATION_ODR_FALLBACK_ATTEMPTS 4

enum ICM_BUS {
  BUS_I2C,
  BUS_SPI
};

enum ICM_ACCEL_FS {
  AFS_16G = 0,
  AFS_8G  = 1,
  AFS_4G  = 2,
  AFS_2G  = 3
};

enum ICM_GYRO_FS {
  GFS_2000DPS = 0,
  GFS_1000DPS = 1,
  GFS_500DPS  = 2,
  GFS_250DPS  = 3,
  GFS_125DPS  = 4
};

enum ICM_ODR {
  ODR_32KHZ  = 1,
  ODR_16KHZ  = 2,
  ODR_8KHZ   = 3,
  ODR_4KHZ   = 4,
  ODR_2KHZ   = 5,
  ODR_1KHZ   = 6,
  ODR_200HZ  = 7,
  ODR_100HZ  = 8,
  ODR_50HZ   = 9,
  ODR_25HZ   = 10,
  ODR_12_5HZ = 11,
  ODR_500HZ  = 15
};

enum ICM_FIFO_MODE {
  FIFO_NONE = 0,        // Standard register read (no FIFO)
  FIFO_16BIT = 1,       // 16-bit FIFO (Packet format 3)
  FIFO_20BIT_HIRES = 2  // 20-bit High-Res FIFO (Packet format 4)
};

// Order of the UI filter (GYRO_UI_FILT_ORD / ACCEL_UI_FILT_ORD). Power-on: 2nd.
enum ICM_UI_FILT_ORDER {
  UI_FILT_1ST_ORDER = 0,
  UI_FILT_2ND_ORDER = 1,
  UI_FILT_3RD_ORDER = 2
};

class ICM42688P {
public:
  // constexpr: all initial values live in the class (below), so a global
  // driver object is laid out by the compiler instead of being built by
  // start-up code. Same object, less flash on small parts.
  TRISENSE_CTOR_CONSTEXPR ICM42688P() {}
  
  // --- Initialization ---
  bool beginI2C(uint32_t freq = 0, uint8_t i2cAddr = ICM_ADDR_PRIMARY, int8_t sdaPin = -1, int8_t sclPin = -1);
  bool beginSPI(int8_t csPin, uint32_t freq = 0, int8_t sckPin = -1, int8_t misoPin = -1, int8_t mosiPin = -1);
  bool begin(ICM_BUS busType, int8_t csPin = -1, uint32_t freq = 0, uint8_t i2cAddr = ICM_ADDR_PRIMARY, int8_t sckSclPin = -1, int8_t misoSdaPin = -1, int8_t mosiPin = -1);
  
  void setDebug(bool enable);

  // --- SPI pin release (TriSense Pro level shifter workaround) ---
  // On the TriSense Pro the IMU's SCLK/SDI pass through an SN74LVC2T45 whose
  // DIR pin is the IMU's chip select. While CS is HIGH (IMU not selected) that
  // shifter DRIVES the module's SCK and MOSI pins with the I2C bus levels, so
  // an MCU that keeps SCK/MOSI as push-pull outputs between transactions
  // fights it - tens of mA, almost continuously. With pin release enabled the
  // driver gives SCK and MOSI back to the SPI peripheral only while CS is LOW
  // and parks them as high-impedance inputs the rest of the time. See README,
  // "SPI pin release".
  //
  // Implemented for RP2040 / RP2350 (arduino-pico), AVR, megaAVR (Nano
  // Every), SAMD, the ESP32 family, Teensy 3.x / LC / 4.x and STM32
  // (stm32duino); on other cores it is a no-op that returns false, and begin()
  // falls back to SPI mode 3 instead (see setSpiMode()). Pass -1 for either pin to
  // switch it off. May be called before or after begin(). If it is never
  // called, begin() enables it on its own whenever the pins are known: passed
  // to begin(), or unambiguously found after SPI.begin() (the pin mux on
  // RP2040/RP2350 and ESP32, the core's default SPI pins elsewhere - checked
  // to really be muxed to a peripheral on Teensy and STM32).
  // Returns true if release is active (or, before begin(), armed - the pins
  // are checked when begin() starts SPI).
  bool setSpiPinRelease(int8_t sckPin, int8_t mosiPin);
  bool isSpiPinReleaseActive();

  // SPI_MODE0 or SPI_MODE3 - the ICM-42688-P supports both. Mode 3 idles SCK
  // HIGH, the same level as an idle I2C SCL, so flipping CS does not put a LOW
  // edge onto the module's I2C bus. Default: mode 0 while pin release is
  // active, mode 3 where it is not (then an idle-HIGH SCK at least does not
  // fight the module while the I2C bus is idle). Calling this overrides the
  // default. Returns false (and changes nothing) for any other mode. Takes
  // effect on the next transaction.
  bool setSpiMode(uint8_t mode);
  uint8_t getSpiMode();

  // --- Power ---
  // sleep(): accel + gyro off (PWR_MGMT0 = 0x00). Register contents are kept.
  // wake(): restores the previous power mode, full-scale ranges, FIFO mode and
  // ODR, waits for the gyro to start up and flushes the FIFO so no stale or
  // start-up samples are handed out. Returns false if the chip does not answer.
  void sleep();
  bool wake();
  bool isSleeping();

  // Selects the SPI bus for BUS_SPI mode: SPI (default), SPI1, ... Must be
  // called BEFORE begin(). On RP2040/RP2350 set that bus's pins with its own
  // setSCK()/setTX()/setRX() first, or pass them to begin(). Ignored in
  // BUS_I2C mode. TriSense::beginAll() takes the bus as its last argument.
  void setSPI(TriSenseSPIClass &spi);
  TriSenseSPIClass &getSPI() { return *_spi; }

  // Selects the I2C bus for BUS_I2C mode. Must be called BEFORE begin().
  // Ignored in BUS_SPI mode. The whole TriSense module shares one bus, so
  // TriSense::beginAll() sets this for you.
  void setWire(TwoWire &wire);

  // --- Configuration ---
  void setODR(ICM_ODR odr);
  void setAccelFS(ICM_ACCEL_FS fs);
  void setGyroFS(ICM_GYRO_FS fs);
  void setFIFOMode(ICM_FIFO_MODE mode);
  
  int getODRHz();               // ODR actually in effect (may be below what you asked for)
  int getRequestedODRHz();      // ODR you asked for via setODR()

  // Drops to the next ODR down the ladder and makes that the new request, so
  // the bandwidth limiter will not raise it again. Returns false if already at
  // the bottom (12.5 Hz), leaving the rate untouched.
  //
  // This is the recovery path for a rate the system turns out not to be able to
  // service. Nothing calls it automatically during normal streaming - a rate
  // that silently sags under load is worse than one that stays put and tells
  // you - but the blocking calibration routines use it when they time out, and
  // a sketch that detects it is falling behind can call it directly.
  //
  // ODR is one of the few registers the datasheet allows to be changed while
  // the sensor is running, so no power-down cycle is needed. The FIFO is
  // flushed, because packets already in it were captured at the old rate.
  bool stepDownODR();
  
  // [VOLTINO FIX] Helper method to dynamically adapt fusion integration based on buffer state
  ICM_FIFO_MODE getFIFOMode(); 

  // [VOLTINO FIX] New rescue function added to header!
  void flushFIFO();

  // How many packets readFIFO() can hand out right now: whatever is still held
  // in the software burst buffer, plus whatever the hardware FIFO reports.
  // Costs one FIFO_COUNT read. Lets a caller size a batch BEFORE draining it,
  // so it can work out the per-sample dt without buffering the packets first.
  // Returns 1 in FIFO_NONE mode (a direct register read always has "one" sample).
  uint16_t availablePackets();

  // --- FIFO overflow monitoring ---
  // The hardware FIFO holds 2 KB. If the sketch does not drain it fast enough
  // the sensor discards new samples, and dropped packets are rotation that can
  // never be integrated - a permanent attitude error. On every buffer refill
  // the driver checks the FIFO count and counts the refills that found it
  // full (within two packets of capacity). The hardware FIFO_FULL bit in
  // INT_STATUS is not used - see fillFIFOBuffer() - and INT_STATUS is not read.
  // NOTHING is printed from the library: poll these from your sketch instead.
  // NOTE ON THE COUNT: this counts EVENTS - refills that found the FIFO full -
  // not lost samples. One stalled loop produces one event but may lose hundreds
  // of packets; a FIFO that merely sits full produces an event per refill while
  // losing nothing. To judge actual data loss, compare getActualFusionHz()
  // against getODRHz(): if they match, nothing is being lost.
  bool fifoOverflowed();            // True if an overflow occurred since the last call (self-clearing)
  uint32_t getFIFOOverflowCount();  // Total overflow EVENTS since boot / last reset
  void resetFIFOOverflowCount();

  // Packets the SENSOR reports having discarded, read straight from its own
  // FIFO_LOST_PKT registers. Unlike the event count above, this is an actual
  // quantity of lost samples measured by the hardware rather than inferred from
  // how full the FIFO looked, so it is the number to trust when deciding
  // whether overflow is costing you anything. The chip's counter is 16 bits and
  // wraps; this accumulates it into 32 across refills.
  uint32_t getLostPacketCount();
  void resetLostPacketCount();
  // Times the FIFO read lost packet alignment - a data packet whose header did
  // not say accel + gyro in the configured layout - and was flushed to recover.
  // The read that finds it returns false; samples resume with the next packet.
  // Should stay 0; it grows when the FIFO overflows mid-packet.
  uint32_t getFIFOResyncCount();

  // --- Anti-alias filter (AAF) ---
  // The AAF sits in front of the decimator and is what keeps vibration above
  // the output Nyquist from folding down into the band the fusion sees. Its
  // 3 dB frequency is set by three coefficients (AAF_DELT, AAF_DELTSQR,
  // AAF_BITSHIFT) that must come as a triple from the datasheet's AAF table
  // (DS-000347 section 5.3: 63 steps, 42 Hz ... 3979 Hz). These calls pick the
  // table row NEAREST to the requested frequency (a tie goes to the lower one)
  // and write all three coefficients, so an inconsistent triple is impossible.
  //
  //   hz = 0  disables that sensor's AAF (AAF_DIS = 1)
  //   hz < 42 / > 3979 is clamped to the first / last table row
  //
  // The coefficients live in register banks 1 (gyro) and 2 (accel). The write
  // selects the bank through REG_BANK_SEL and ALWAYS returns to bank 0. As the
  // datasheet asks (section 12.9), the gyro and accel are switched off while
  // these static registers change: if they are running, the call waits out the
  // 45 ms gyro minimum on-time, turns them off, writes, turns them back on,
  // waits the 30 ms gyro start-up and flushes the FIFO. Budget ~80 ms - call it
  // during setup, not in flight. Asleep (sleep()) the sensors stay off.
  //
  // Every coefficient is read back after the write. Returns false if the chip
  // did not take the value; the getters then report what the chip really holds.
  // The settings are kept across sleep()/wake(); begin() (soft reset) restores
  // the power-on defaults (gyro 585 Hz, accel 1163 Hz).
  bool setAntiAliasFilter(uint16_t gyroHz, uint16_t accelHz);
  bool setGyroAntiAliasFilter(uint16_t hz);
  bool setAccelAntiAliasFilter(uint16_t hz);
  uint16_t getGyroAntiAliasFilterHz();   // 3 dB frequency in effect; 0 = AAF disabled
  uint16_t getAccelAntiAliasFilterHz();  // 3 dB frequency in effect; 0 = AAF disabled
  // The table row the setters would choose for hz, without touching the chip:
  // returns its 3 dB frequency (0 for hz = 0) and, if given, its coefficients.
  static uint16_t antiAliasFilterLookup(uint16_t hz, uint8_t *delt = nullptr,
                                        uint16_t *deltSqr = nullptr, uint8_t *bitShift = nullptr);

  // --- UI filter (output low-pass, GYRO_ACCEL_CONFIG0 / *_CONFIG1) ---
  // The UI filter runs after the AAF at the output data rate. Its bandwidth is
  // a fraction of the ODR (Low Noise mode, DS-000347 register GYRO_ACCEL_CONFIG0):
  //   0: ODR/2   1: max(400 Hz, ODR)/4 (power-on)   2: max(400 Hz, ODR)/5
  //   3: max(400 Hz, ODR)/8   4: /10   5: /16   6: /20   7: max(400 Hz, ODR)/40
  // setUIFilter() picks the setting nearest to the requested bandwidth at the
  // ODR currently in effect (tie -> lower) and REMEMBERS the request: when the
  // ODR later changes (setODR, the bandwidth limiter, stepDownODR) the nearest
  // setting is chosen again for the new ODR. The getters return the nominal
  // bandwidth (Hz) of the setting in effect, for the ODR in effect; the actual
  // -3 dB point also depends on the filter order. The low-latency settings
  // 14/15 are not used. Section 12.9 allows only ODR, FSR and mode to change
  // while the sensors run, so a write here power-cycles them exactly like the
  // AAF setters (~80 ms, FIFO flushed) - also when an ODR change re-selects a
  // DIFFERENT UI setting; one that keeps the setting writes nothing. Returns
  // false if the read-back does not match.
  bool setUIFilter(float gyroHz, float accelHz);
  bool setUIFilterOrder(ICM_UI_FILT_ORDER gyroOrder, ICM_UI_FILT_ORDER accelOrder);
  float getGyroUIFilterHz();
  float getAccelUIFilterHz();
  ICM_UI_FILT_ORDER getGyroUIFilterOrder() { return (ICM_UI_FILT_ORDER)_uiGyroOrd; }
  ICM_UI_FILT_ORDER getAccelUIFilterOrder() { return (ICM_UI_FILT_ORDER)_uiAccelOrd; }

  // --- FIFO timestamp ---
  // Each FIFO packet carries a 16-bit ODR timestamp (1 us per LSB at the
  // power-on TMST_CONFIG, sensor's own clock, wraps every 65.536 ms). These
  // report the one in the packet the last readIMU()/readFIFO() returned.
  // The difference of two consecutive timestamps (mod 65536) is the true
  // sample spacing: a jump of ~2x the ODR period means a sample went missing.
  // hasFIFOTimestamp() is false in FIFO_NONE mode, before the first packet,
  // and when the packet header says the field holds no ODR timestamp.
  bool hasFIFOTimestamp() { return _fifoTsValid; }
  uint16_t getFIFOTimestamp() { return _fifoTs; }

  // --- Data reading ---
  bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  float readTemperature();

  // Wrappers and specific methods for direct calls
  bool readSensorData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  bool readHardwareFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
#if !TRISENSE_LITE
  bool readHardwareFIFOHires(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
#endif
  bool readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

  // --- Sample tap ---
  // Called with every sample readIMU()/readFIFO() hands out, in the same
  // calibrated units (g, deg/s), after the read succeeded. Runs in the
  // reader's context - keep it short and non-blocking. nullptr disables it.
  //
  // Lets a second consumer (e.g. a vibration analyser) see the full sample
  // stream while the fusion code stays the only one draining the FIFO: a
  // second readFIFO() caller would steal packets from the first.
  typedef void (*SampleTap)(const float acc_g[3], const float gyr_dps[3], void *ctx);
  void setSampleTap(SampleTap fn, void *ctx = nullptr) { _sampleTap = fn; _sampleTapCtx = ctx; }

  // --- Calibration ---
  // Selects register bank 0, the bank every other call in this driver expects.
  // Kept as a recovery hook: if a sketch talks to the chip directly and leaves
  // it parked in bank 1-4, subsequent driver reads would silently return the
  // wrong registers, and this puts it back.
  //
  // It does NOT clear the chip's OFFSET_USER registers (bank 4, 0x77-0x7F) -
  // the name is historical. In practice nothing needs it to: this driver never
  // programs those registers, applying accel/gyro bias in software instead (see
  // setAccelOffset / setGyroOffset), so they stay at their power-on zero anyway.
  void resetHardwareOffsets();
  // --- Calibration storage ---
  // Every offset and scale below is expressed in the SENSOR's OWN axes and is
  // applied to the raw sample before anything else sees it:
  //
  //     accel_out = (accel_raw - accOffset) * accScale
  //     gyro_out  =  gyro_raw  - gyrOffset
  //
  // This is deliberately the ONLY place an accel/gyro bias is stored. The
  // fusion layer's mount remap (TriSenseFusion::setMountOrientation) happens
  // afterwards, on the already-corrected sample, so a value produced by one of
  // the getters below can be saved and restored through the matching setter at
  // any mount orientation.
  void autoCalibrateGyro(uint16_t samples = 750);   // Board still -> gyrOffset
  void autoCalibrateAccel();                        // 6-point sphere fit -> accOffset + accScale
  
  void setGyroSoftwareOffset(float ox, float oy, float oz);
  void setAccelSoftwareOffset(float ox, float oy, float oz);
  void setAccelSoftwareScale(float sx, float sy, float sz);

  void getGyroSoftwareOffset(float &ox, float &oy, float &oz);
  void getAccelSoftwareOffset(float &ox, float &oy, float &oz);
  void getAccelSoftwareScale(float &sx, float &sy, float &sz);

  // Short names for backward compatibility
  void setGyroOffset(float ox, float oy, float oz);
  void setAccelOffset(float ox, float oy, float oz);
  void setAccelScale(float sx, float sy, float sz);
  void getGyroOffset(float &ox, float &oy, float &oz);
  void getAccelOffset(float &ox, float &oy, float &oz);
  void getAccelScale(float &sx, float &sy, float &sz);

  float getAccelOffsetX();
  float getAccelOffsetY();
  float getAccelOffsetZ();
  float getAccelScaleX();
  float getAccelScaleY();
  float getAccelScaleZ();
  float getGyroOffsetX();
  float getGyroOffsetY();
  float getGyroOffsetZ();

private:
  SampleTap _sampleTap = nullptr;
  void *_sampleTapCtx = nullptr;

  TwoWire* _wire = &Wire;
  TriSenseSPIClass* _spi = &SPI;
  ICM_BUS _bus = BUS_I2C;
  int8_t _csPin = -1;
  uint8_t _i2cAddr = ICM_ADDR_PRIMARY;
  uint32_t _spiFreq = 0;
  uint8_t _spiMode = (uint8_t)SPI_MODE0;

  // SPI pin release state (see setSpiPinRelease()).
  int8_t _relSck = -1;
  int8_t _relMosi = -1;
  bool _relConfigured = false;   // setSpiPinRelease() was called - no auto-detect
  bool _relActive = false;       // Pins are currently being released between transactions
  bool _spiStarted = false;
  bool _spiModeSet = false;      // setSpiMode() was called - keep the sketch's choice
  uint32_t _relSckCfg = 0;       // ESP32: GPIO matrix routing, Teensy: mux register
  uint32_t _relMosiCfg = 0;      //   of SCK / MOSI, saved to write back
  uint32_t _relSckPad = 0;       // Teensy 4.x: pad registers, likewise
  uint32_t _relMosiPad = 0;

  // Remembered so wake() can put everything back.
  ICM_ACCEL_FS _accelFs = AFS_16G;
  ICM_GYRO_FS _gyroFs = GFS_2000DPS;
  uint8_t _pwrMgmt0 = ICM42688_PWR_SENSORS_LN;
  bool _sleeping = false;
  uint32_t _powerOnMs = 0;
  // _requestedOdr is what the sketch asked for; _odr is what the bus can
  // actually sustain in the current FIFO mode. Keeping them apart lets the
  // requested rate be restored when the packet size (and so the bandwidth
  // demand) drops again - see enforceBandwidthLimit().
  ICM_ODR _requestedOdr = ODR_1KHZ;
  ICM_ODR _odr = ODR_1KHZ;
  ICM_FIFO_MODE _fifoMode = FIFO_NONE;
  bool _debug = false;

  float _accelScaleFactor = 1.0f / 2048.0f;
  float _gyroScaleFactor = 1.0f / 16.4f;

  float gyrOffset[3] = {0,0,0};
  float accOffset[3] = {0,0,0};
  float accScale[3]  = {1,1,1};

  // --- FIFO burst buffer ---
  // readFIFO() still hands out one packet per call, but the bytes are fetched
  // from the sensor a whole burst at a time: one FIFO_COUNT read plus one bulk
  // FIFO_DATA read serve up to FIFO_BURST_PACKETS packets, instead of two bus
  // transactions per single packet.
#if TRISENSE_LITE
  static const uint8_t FIFO_MAX_PACKET_SIZE = 16;   // No HiRes, so no 20-byte packets
#else
  static const uint8_t FIFO_MAX_PACKET_SIZE = 20;
#endif
  // Sizing this BELOW the hardware FIFO's capacity (102 x 20-byte packets) means
  // a full FIFO needs several round trips to empty, each paying its own
  // INT_STATUS + FIFO_COUNT reads - extra latency at exactly the moment the
  // driver is already behind. Where the RAM exists, one burst therefore covers
  // the whole FIFO. Override with -DTRISENSE_FIFO_BURST_PACKETS=n.
#if defined(TRISENSE_FIFO_BURST_PACKETS)
  static const uint8_t FIFO_BURST_PACKETS = TRISENSE_FIFO_BURST_PACKETS;
#elif defined(__AVR__)   // Not ARDUINO_ARCH_AVR - older Teensyduino sets it on Teensy 3.x/4.x
  static const uint8_t FIFO_BURST_PACKETS = 8;    // 160 B - an Uno has 2 KB total
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350) || defined(ESP32) \
   || defined(ARDUINO_ARCH_SAMD51) || defined(__IMXRT1062__)
  static const uint8_t FIFO_BURST_PACKETS = 128;  // 2560 B - drains a full FIFO in one go
                                                  // (2048/16 = 128 packets, 2048/20 = 102)
#else
  static const uint8_t FIFO_BURST_PACKETS = 32;   // 640 B - conservative default
#endif
  uint8_t _fifoBuf[FIFO_BURST_PACKETS * FIFO_MAX_PACKET_SIZE] = {};
  uint8_t _fifoBufCount = 0;   // Packets currently held in _fifoBuf
  uint8_t _fifoBufIndex = 0;   // Next packet to hand out
  uint8_t _fifoPacketSize = 16;

  bool _fifoOverflowFlag = false;
  uint32_t _fifoOverflowCount = 0;
  uint32_t _lostPacketTotal = 0;
  uint32_t _fifoResyncCount = 0;    // Alignment lost and recovered, see nextFIFOPacket()    // Accumulated across wraps of the 16-bit register
  uint16_t _lostPacketLastRaw = 0;  // Previous raw reading, to detect the wrap
  bool _lostPacketPrimed = false;

  // Filter state, mirrored from the chip (power-on values until changed).
  uint8_t _aafGyroDelt = ICM42688_AAF_GYRO_RESET_DELT;   // 0 = disabled
  uint8_t _aafAccelDelt = ICM42688_AAF_ACCEL_RESET_DELT; // 0 = disabled
  uint8_t _uiGyroBw = ICM42688_UI_FILT_BW_RESET;
  uint8_t _uiAccelBw = ICM42688_UI_FILT_BW_RESET;
  uint8_t _uiGyroOrd = UI_FILT_2ND_ORDER;
  uint8_t _uiAccelOrd = UI_FILT_2ND_ORDER;
  float _uiGyroReqHz = 0;    // 0 = never requested: the ODR path leaves the UI filter alone
  float _uiAccelReqHz = 0;
  uint16_t _fifoTs = 0;
  bool _fifoTsValid = false;

  void resetFilterState();
  unsigned long calibrationReadPeriodUs();     // Flushes the FIFO; 0 = read as fast as samples arrive
  bool sensorsOffForConfig();                  // Section 12.9 bracket; true if they were running
  void sensorsOnAfterConfig(bool wasRunning);
  bool writeAntiAliasFilter(bool doGyro, uint8_t gyroDelt, bool doAccel, uint8_t accelDelt);
  bool applyUIFilterBandwidth();
  float uiFilterHz(uint8_t bw);
  uint8_t uiFilterBwFor(float hz);
  void noteFIFOTimestamp(const uint8_t *packet, uint8_t tsOffset);

  void scaleSample(const int32_t raw[6], float aScale, float gScale,
                   float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
  const uint8_t* nextFIFOPacket();                 // Next usable packet, skipping message packets
  uint8_t fillFIFOBuffer();                        // Refill from hardware; returns packets loaded
  void readFIFOBytes(uint8_t *buf, size_t len);    // Bulk FIFO_DATA read (chunked on I2C)
  void invalidateFIFOBuffer();

  // Every SPI transaction goes through these two, so the CS / pin-release
  // ordering lives in exactly one place.
  void spiSelect();
  void spiDeselect();
  bool spiPinsValid(int8_t sck, int8_t mosi);
  bool detectSpiPins(int8_t &sck, int8_t &mosi);
  void activateSpiPinRelease();
  void relTakePins();
  void relReleasePins();
  void deactivateSpiPinRelease();
  void applySckIdlePull();

  // Soft reset + WHO_AM_I + interface setup. The only place that resets the
  // chip, so the I2C-disable can never be lost to a reset.
  bool resetDevice();
  void applyInterfaceConfig();

  void writeRegister(uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t reg);
  void readRegisters(uint8_t reg, uint8_t *buf, size_t len);
  void setBank(uint8_t bank);

  void enforceBandwidthLimit();
  int _getHzFromODR(ICM_ODR odr);
};