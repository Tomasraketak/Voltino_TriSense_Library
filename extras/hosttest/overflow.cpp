// FIFO overflow detection against the simulated FIFO, which drops samples
// exactly when it is full and counts them in FIFO_LOST_PKT.
//
//   make overflow && ./overflow
//
// A loop that keeps up must report no overflow at all; a stalled loop must
// report it, with the lost-packet count the simulated chip produced.
#include "Arduino.h"
#include "TriSense.h"

void sim_reset(int motion, uint32_t seed);
void sim_motion(int motion, uint32_t seed);

static int fails = 0;
static void check(bool ok, const char *what) {
  printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}

static void scenario(const char *name, ICM_FIFO_MODE fifo, ICM_ODR odr, uint32_t stallMs, uint32_t expectLost) {
  printf("=== %s\n", name);
  sim_reset(0, 1);
  TriSense sensor;
  AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);
  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) { check(false, "beginAll"); return; }
  sensor.imu.setFIFOMode(fifo);
  sensor.imu.setODR(odr);
  sensor.imu.flushFIFO();
  sensor.imu.resetFIFOOverflowCount();
  sensor.imu.resetLostPacketCount();

  for (int i = 0; i < 4000; i++) { sim_advance_us(250); fusion.update(); }   // 1 s, keeping up
  check(!sensor.imu.fifoOverflowed() && sensor.imu.getFIFOOverflowCount() == 0,
        "no overflow reported while the loop keeps up");

  if (stallMs) {
    // The first overflow event only primes the lost-packet baseline, so stall
    // twice and count the second one.
    sim_advance_us(stallMs * 1000); fusion.update();
    for (int i = 0; i < 400; i++) { sim_advance_us(250); fusion.update(); }
    uint32_t before = sensor.imu.getLostPacketCount();
    sim_advance_us(stallMs * 1000); fusion.update();
    for (int i = 0; i < 400; i++) { sim_advance_us(250); fusion.update(); }
    uint32_t lost = sensor.imu.getLostPacketCount() - before;
    printf("     events %u, lost in 2nd stall %u (expected ~%u)\n",
           (unsigned)sensor.imu.getFIFOOverflowCount(), (unsigned)lost, (unsigned)expectLost);
    check(sensor.imu.fifoOverflowed() && sensor.imu.getFIFOOverflowCount() >= 2, "stall reported");
    check(lost + 3 >= expectLost && lost <= expectLost + 3, "lost-packet count matches the stall");
  }
}

int main() {
  scenario("16-bit 1 kHz, no stall", FIFO_16BIT, ODR_1KHZ, 0, 0);
  scenario("HiRes 8 kHz, no stall", FIFO_20BIT_HIRES, ODR_8KHZ, 0, 0);
  scenario("16-bit 1 kHz, 300 ms stall", FIFO_16BIT, ODR_1KHZ, 300, 300 - 128);
  scenario("HiRes 8 kHz, 30 ms stall", FIFO_20BIT_HIRES, ODR_8KHZ, 30, 240 - 102);
  printf(fails ? "%d FAILED\n" : "ALL PASSED\n", fails);
  return fails ? 1 : 0;
}
