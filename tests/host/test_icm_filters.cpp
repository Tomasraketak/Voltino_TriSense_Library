// Register-level host tests for the ICM-42688-P anti-alias filter (AAF), UI
// filter and FIFO bookkeeping. The driver runs against icm_stub.cpp, which
// models the five register banks and logs every access.
//
//   make -C tests/host test
#include "Arduino.h"
#include "ICM42688P_voltino.h"
#include "icm_stub.h"
#include <vector>

static int fails = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL  %s:%d  ", __FILE__, __LINE__); \
  printf(__VA_ARGS__); printf("\n"); } } while (0)

static void section(const char *name) { printf("=== %s\n", name); }

// Value of the LAST write to (bank, reg) in the log; -1 if none.
static int lastWrite(uint8_t bank, uint8_t reg) {
  int v = -1;
  for (size_t i = 0; i < stub_log_size(); i++) {
    const StubAccess &a = stub_log(i);
    if (a.write && a.bank == bank && a.reg == reg) v = a.val;
  }
  return v;
}

// Bank discipline of everything in the log:
//  - every write to bank 1/2 happened with gyro + accel off (PWR_MGMT0 = 0)
//    when expectOff is set,
//  - while bank 1 / 2 was selected the driver touched only the AAF registers
//    (and REG_BANK_SEL) - nothing that it meant for bank 0,
//  - the last REG_BANK_SEL write is 0 and the chip ends in bank 0.
static void checkBankDiscipline(bool expectOff) {
  int lastSel = -1;
  for (size_t i = 0; i < stub_log_size(); i++) {
    const StubAccess &a = stub_log(i);
    if (a.reg == 0x76) { if (a.write) lastSel = a.val; continue; }
    if (a.bank == 1)
      CHECK(a.reg >= 0x0B && a.reg <= 0x0E, "bank 1 access to unexpected reg 0x%02X", a.reg);
    if (a.bank == 2)
      CHECK(a.reg >= 0x03 && a.reg <= 0x05, "bank 2 access to unexpected reg 0x%02X", a.reg);
    CHECK(a.bank <= 2, "access in bank %u", a.bank);
    if (expectOff && a.write && a.bank != 0)
      CHECK(a.pwr == 0x00, "bank %u reg 0x%02X written with PWR_MGMT0 = 0x%02X", a.bank, a.reg, a.pwr);
  }
  CHECK(lastSel == 0, "last REG_BANK_SEL write was %d, expected 0", lastSel);
  CHECK(stub_bank() == 0, "chip left in bank %u", stub_bank());
}

static bool beginImu(ICM42688P &imu) {
  stub_reset();
  imu.setSpiPinRelease(-1, -1);
  return imu.begin(BUS_SPI, 17, 10000000);
}

// Expected register bytes for one AAF table row.
struct AafCase {
  const char *name;
  uint16_t gyroReq, accelReq;   // What the sketch asks for
  uint16_t gyroHz, accelHz;     // Table row that must be chosen (0 = disabled)
  uint8_t gDelt; uint16_t gSqr; uint8_t gShift;
  uint8_t aDelt; uint16_t aSqr; uint8_t aShift;
};

static void testAafCase(const AafCase &c) {
  section(c.name);
  ICM42688P imu;
  CHECK(beginImu(imu), "begin");
  stub_set_reg(1, 0x0B, 0xA1);   // GYRO_NF_DIS set + reserved bits: must survive the RMW
  stub_log_clear();

  const bool ok = imu.setAntiAliasFilter(c.gyroReq, c.accelReq);
  CHECK(ok, "setAntiAliasFilter returned false");
  CHECK(imu.getGyroAntiAliasFilterHz() == c.gyroHz, "gyro AAF %u Hz, expected %u",
        imu.getGyroAntiAliasFilterHz(), c.gyroHz);
  CHECK(imu.getAccelAntiAliasFilterHz() == c.accelHz, "accel AAF %u Hz, expected %u",
        imu.getAccelAntiAliasFilterHz(), c.accelHz);

  // Gyro - bank 1
  if (c.gyroHz == 0) {
    CHECK(stub_reg(1, 0x0B) == 0xA3, "GYRO_CONFIG_STATIC2 = 0x%02X, expected 0xA3 (AAF_DIS)", stub_reg(1, 0x0B));
    CHECK(lastWrite(1, 0x0C) == -1, "coefficients written although the AAF is being disabled");
  } else {
    CHECK(lastWrite(1, 0x0C) == c.gDelt, "GYRO_AAF_DELT write %d, expected %u", lastWrite(1, 0x0C), c.gDelt);
    CHECK(lastWrite(1, 0x0D) == (c.gSqr & 0xFF), "GYRO_AAF_DELTSQR[7:0] write %d, expected %u",
          lastWrite(1, 0x0D), c.gSqr & 0xFF);
    CHECK(lastWrite(1, 0x0E) == ((c.gShift << 4) | (c.gSqr >> 8)), "GYRO_CONFIG_STATIC5 write 0x%02X, expected 0x%02X",
          lastWrite(1, 0x0E), (c.gShift << 4) | (c.gSqr >> 8));
    CHECK(lastWrite(1, 0x0B) == 0xA1, "GYRO_CONFIG_STATIC2 write 0x%02X, expected 0xA1", lastWrite(1, 0x0B));
  }
  // Accel - bank 2
  if (c.accelHz == 0) {
    CHECK(stub_reg(2, 0x03) == 0x31, "ACCEL_CONFIG_STATIC2 = 0x%02X, expected 0x31", stub_reg(2, 0x03));
    CHECK(lastWrite(2, 0x04) == -1, "accel coefficients written although the AAF is being disabled");
  } else {
    CHECK(lastWrite(2, 0x03) == (c.aDelt << 1), "ACCEL_CONFIG_STATIC2 write 0x%02X, expected 0x%02X",
          lastWrite(2, 0x03), c.aDelt << 1);
    CHECK(lastWrite(2, 0x04) == (c.aSqr & 0xFF), "ACCEL_AAF_DELTSQR[7:0] write %d, expected %u",
          lastWrite(2, 0x04), c.aSqr & 0xFF);
    CHECK(lastWrite(2, 0x05) == ((c.aShift << 4) | (c.aSqr >> 8)), "ACCEL_CONFIG_STATIC4 write 0x%02X, expected 0x%02X",
          lastWrite(2, 0x05), (c.aShift << 4) | (c.aSqr >> 8));
  }
  // Nothing landed in bank 0 at the AAF addresses (would mean a missed bank switch).
  CHECK(lastWrite(0, 0x0B) == -1 && lastWrite(0, 0x0C) == -1 && lastWrite(0, 0x03) == -1,
        "AAF value written to bank 0");

  checkBankDiscipline(true);
  // Sensors were off for the static writes and are back on afterwards.
  CHECK(lastWrite(0, 0x4E) == 0x0F, "PWR_MGMT0 not restored (last write %d)", lastWrite(0, 0x4E));
  CHECK(stub_reg(0, 0x4E) == 0x0F, "sensors left off");
}

static void testDefaultsAndLookup() {
  section("power-on defaults and table lookup");
  ICM42688P imu;
  CHECK(beginImu(imu), "begin");
  CHECK(imu.getGyroAntiAliasFilterHz() == 585, "default gyro AAF %u", imu.getGyroAntiAliasFilterHz());
  CHECK(imu.getAccelAntiAliasFilterHz() == 1163, "default accel AAF %u", imu.getAccelAntiAliasFilterHz());
  CHECK(imu.getGyroUIFilterOrder() == UI_FILT_2ND_ORDER, "default gyro UI order");
  CHECK(imu.getAccelUIFilterOrder() == UI_FILT_2ND_ORDER, "default accel UI order");

  uint8_t d; uint16_t sq; uint8_t sh;
  CHECK(ICM42688P::antiAliasFilterLookup(0, &d) == 0 && d == 0, "0 Hz -> disabled");
  CHECK(ICM42688P::antiAliasFilterLookup(1, &d) == 42 && d == 1, "1 Hz clamps to 42 Hz");
  CHECK(ICM42688P::antiAliasFilterLookup(63, &d) == 42 && d == 1, "63 Hz is a tie 42/84 -> lower");
  CHECK(ICM42688P::antiAliasFilterLookup(64, &d) == 84 && d == 2, "64 Hz -> 84 Hz");
  CHECK(ICM42688P::antiAliasFilterLookup(585, &d, &sq, &sh) == 585 && d == 13 && sq == 170 && sh == 8,
        "585 Hz -> DELT 13 / 170 / 8 (power-on gyro value)");
  CHECK(ICM42688P::antiAliasFilterLookup(1163, &d, &sq, &sh) == 1163 && d == 24 && sq == 576 && sh == 6,
        "1163 Hz -> DELT 24 / 576 / 6 (power-on accel value)");
  CHECK(ICM42688P::antiAliasFilterLookup(65535, &d, &sq, &sh) == 3979 && d == 63 && sq == 3968 && sh == 3,
        "65535 Hz clamps to 3979 Hz");
  // Monotonic table, DELTSQR fits 12 bits, BITSHIFT fits 4 bits.
  uint16_t prev = 0;
  bool mono = true;
  for (uint16_t hz = 42; hz <= 3979; hz++) {
    uint16_t f = ICM42688P::antiAliasFilterLookup(hz, &d, &sq, &sh);
    if (f < prev || sq > 0x0FFF || sh > 15 || d < 1 || d > 63) mono = false;
    prev = f;
  }
  CHECK(mono, "lookup monotonic and coefficients in range");
}

static void testReadbackFailure() {
  section("read-back mismatch is reported, bank 0 still restored");
  ICM42688P imu;
  CHECK(beginImu(imu), "begin");
  stub_set_readonly(1, 0x0D, true);   // GYRO_AAF_DELTSQR[7:0] does not take the write
  stub_log_clear();
  CHECK(!imu.setGyroAntiAliasFilter(258), "setGyroAntiAliasFilter should fail");
  CHECK(imu.getGyroAntiAliasFilterHz() == 0xFFFF, "getter must report unknown, got %u",
        imu.getGyroAntiAliasFilterHz());
  CHECK(imu.getAccelAntiAliasFilterHz() == 1163, "accel untouched");
  CHECK(lastWrite(2, 0x03) == -1, "gyro-only call wrote bank 2");
  checkBankDiscipline(true);
}

static void testWhileAsleep() {
  section("AAF while asleep: no power cycle");
  ICM42688P imu;
  CHECK(beginImu(imu), "begin");
  sim_advance_us(100000);
  imu.sleep();
  stub_log_clear();
  CHECK(imu.setAccelAntiAliasFilter(213), "setAccelAntiAliasFilter");
  CHECK(imu.getAccelAntiAliasFilterHz() == 213, "accel AAF %u", imu.getAccelAntiAliasFilterHz());
  CHECK(lastWrite(0, 0x4E) == -1, "PWR_MGMT0 touched while asleep");
  CHECK(lastWrite(1, 0x0C) == -1, "accel-only call wrote bank 1");
  checkBankDiscipline(true);
  CHECK(imu.wake(), "wake");
  CHECK(stub_reg(2, 0x03) == (5 << 1) && stub_reg(2, 0x04) == 25 && stub_reg(2, 0x05) == 0xA0,
        "bank 2 holds 213 Hz after wake");
}

static void testUIFilter() {
  section("UI filter: nearest bandwidth, follows the ODR, order");
  ICM42688P imu;
  CHECK(beginImu(imu), "begin");
  imu.setODR(ODR_1KHZ);
  stub_log_clear();

  // 1 kHz: 500, 250, 200, 125, 100, 62.5, 50, 25 Hz for settings 0..7
  CHECK(imu.setUIFilter(100, 250), "setUIFilter");
  CHECK(lastWrite(0, 0x52) == 0x14, "GYRO_ACCEL_CONFIG0 = 0x%02X, expected 0x14", lastWrite(0, 0x52));
  CHECK(imu.getGyroUIFilterHz() == 100.0f, "gyro UI %.1f", imu.getGyroUIFilterHz());
  CHECK(imu.getAccelUIFilterHz() == 250.0f, "accel UI %.1f", imu.getAccelUIFilterHz());
  for (size_t i = 0; i < stub_log_size(); i++)   // Bank 0 registers only: no bank switch at all
    CHECK(stub_log(i).bank == 0 && stub_log(i).reg != 0x76, "UI filter left bank 0");

  CHECK(imu.setUIFilter(70, 0), "setUIFilter gyro only");   // 62.5 is nearest; accel stays
  CHECK(lastWrite(0, 0x52) == 0x15, "GYRO_ACCEL_CONFIG0 = 0x%02X, expected 0x15", lastWrite(0, 0x52));

  // Requests are re-resolved when the ODR changes. 200 Hz ODR: max(400, ODR)
  // = 400, so 100, 100, 80, 50, 40, 25, 20, 10 Hz; gyro 70 -> 80 (setting 2),
  // accel 250 -> 100 (setting 0, the first of the two 100 Hz settings).
  stub_log_clear();
  imu.setODR(ODR_200HZ);
  CHECK(lastWrite(0, 0x52) == 0x02, "after ODR change GYRO_ACCEL_CONFIG0 = 0x%02X, expected 0x02",
        lastWrite(0, 0x52));
  CHECK(imu.getGyroUIFilterHz() == 80.0f, "gyro UI %.1f", imu.getGyroUIFilterHz());

  // Order: GYRO_CONFIG1 0x16 -> bits 3:2 = 10; ACCEL_CONFIG1 0x0D -> bits 4:3 = 00.
  CHECK(imu.setUIFilterOrder(UI_FILT_3RD_ORDER, UI_FILT_1ST_ORDER), "setUIFilterOrder");
  CHECK(stub_reg(0, 0x51) == 0x1A, "GYRO_CONFIG1 = 0x%02X, expected 0x1A", stub_reg(0, 0x51));
  CHECK(stub_reg(0, 0x53) == 0x05, "ACCEL_CONFIG1 = 0x%02X, expected 0x05", stub_reg(0, 0x53));
  CHECK(imu.getGyroUIFilterOrder() == UI_FILT_3RD_ORDER && imu.getAccelUIFilterOrder() == UI_FILT_1ST_ORDER,
        "order getters");
  CHECK(!imu.setUIFilterOrder((ICM_UI_FILT_ORDER)3, UI_FILT_1ST_ORDER), "reserved order rejected");

  // Without setUIFilter() the ODR path does not touch GYRO_ACCEL_CONFIG0.
  ICM42688P fresh;
  CHECK(beginImu(fresh), "begin");
  stub_log_clear();
  fresh.setODR(ODR_2KHZ);
  CHECK(lastWrite(0, 0x52) == -1, "GYRO_ACCEL_CONFIG0 written without a request");
}

static void packet16(uint8_t *p, uint8_t header, uint16_t ts) {
  memset(p, 0, 16);
  p[0] = header;
  p[2] = 0x08;               // accel X = 8 LSB
  p[14] = (uint8_t)(ts >> 8);
  p[15] = (uint8_t)ts;
}

static void testFifoTimestampAndLost() {
  section("FIFO timestamp and lost-packet counter");
  ICM42688P imu;
  CHECK(beginImu(imu), "begin");
  imu.setFIFOMode(FIFO_16BIT);
  imu.resetLostPacketCount();
  CHECK(!imu.hasFIFOTimestamp(), "no timestamp before the first packet");

  uint8_t p[16];
  stub_set_lost(5);                       // Losses from before: baseline, not counted
  packet16(p, 0x68, 0x1234); stub_fifo_push(p, 16);
  packet16(p, 0x60, 0x0000); stub_fifo_push(p, 16);   // Header bits 3:2 = 00: no ODR timestamp
  float ax, ay, az, gx, gy, gz;
  CHECK(imu.readIMU(ax, ay, az, gx, gy, gz), "read packet 1");
  CHECK(imu.hasFIFOTimestamp() && imu.getFIFOTimestamp() == 0x1234, "timestamp 0x%04X",
        imu.getFIFOTimestamp());
  CHECK(imu.readIMU(ax, ay, az, gx, gy, gz), "read packet 2");
  CHECK(!imu.hasFIFOTimestamp(), "packet without ODR timestamp flagged");
  CHECK(imu.getLostPacketCount() == 0, "baseline counted as loss: %u", (unsigned)imu.getLostPacketCount());

  // First overflow: a full FIFO plus 172 packets dropped by the chip.
  stub_set_lost(5 + 172);
  for (int i = 0; i < 128; i++) { packet16(p, 0x68, (uint16_t)(0x2000 + i * 1000)); stub_fifo_push(p, 16); }
  CHECK(imu.readIMU(ax, ay, az, gx, gy, gz), "read after overflow");
  CHECK(imu.fifoOverflowed(), "overflow reported");
  CHECK(imu.getLostPacketCount() == 172, "first overflow lost %u packets, expected 172",
        (unsigned)imu.getLostPacketCount());
  CHECK(imu.getFIFOTimestamp() == 0x2000, "timestamp of the first packet in the burst");
}

int main() {
  // Rows: DS-000347 section 5.3. Gyro / accel requests chosen to hit exact
  // rows, nearest-row rounding and both ends of the table.
  const AafCase cases[] = {
    { "AAF 258 Hz gyro / 213 Hz accel (exact rows)", 258, 213, 258, 213,
      6, 36, 10,   5, 25, 10 },
    { "AAF 1000 Hz gyro / 500 Hz accel (nearest: 997 / 488)", 1000, 500, 997, 488,
      21, 440, 6,  11, 122, 8 },
    { "AAF lower edge: 10 Hz gyro -> 42 Hz, 100 Hz accel -> 84 Hz", 10, 100, 42, 84,
      1, 1, 15,    2, 4, 13 },
    { "AAF upper edge: 3979 Hz gyro, 5000 Hz accel -> 3979 Hz", 3979, 5000, 3979, 3979,
      63, 3968, 3, 63, 3968, 3 },
    { "AAF disabled (0 Hz) on both", 0, 0, 0, 0,
      0, 0, 0,     0, 0, 0 },
  };
  testDefaultsAndLookup();
  for (const AafCase &c : cases) testAafCase(c);
  testReadbackFailure();
  testWhileAsleep();
  testUIFilter();
  testFifoTimestampAndLost();

  printf("%d checks, %d failed\n", checks, fails);
  printf(fails ? "FAILED\n" : "ALL PASSED\n");
  return fails ? 1 : 0;
}
