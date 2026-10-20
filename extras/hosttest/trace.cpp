// Runs the real driver and fusion code against the simulated module and prints
// a trace. Two builds of the library that are meant to behave identically must
// produce byte-identical traces: every logged float is printed as its exact
// bit pattern as well as in decimal.
//
//   make            build ./trace from ../../src
//   ./trace > a.txt
//   make SRC=/path/to/other/src && ./trace > b.txt && cmp a.txt b.txt
#include "Arduino.h"
#include "TriSense.h"

void sim_reset(int motion, uint32_t seed);
void sim_motion(int motion, uint32_t seed);

static uint32_t bits(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static void logf4(const char *tag, float a, float b, float c, float d = 0) {
  printf("%s %08x %08x %08x %08x  %.6f %.6f %.6f %.6f\n", tag, bits(a), bits(b), bits(c), bits(d), a, b, c, d);
}

static void run(TriSense &sensor, TriSenseFusion &fusion, int seconds, uint32_t loopUs) {
  uint32_t jitter = 7;
  for (uint32_t t = 0, k = 0; t < (uint32_t)seconds * 1000000u; t += loopUs, k++) {
    jitter = jitter * 1103515245u + 12345u;
    sim_advance_us(loopUs + ((jitter >> 16) % 300));
    fusion.update();
    if (k % 25 == 0) {
      float r, p, y;
      fusion.getOrientationDegrees(r, p, y);
      logf4("rpy", r, p, y, fusion.getMagHeadingDegrees());
      float ax, ay, az;
      fusion.getGlobalLinearAcceleration(ax, ay, az, ACCEL_UNIT_MS2);
      logf4("gla", ax, ay, az);
      fusion.getLinearAcceleration(ax, ay, az);
      logf4("lin", ax, ay, az, fusion.getActualFusionHz());
      TriSenseDataSnapshot s;
      sensor.getSnapshot(s);
      logf4("snA", s.accelX, s.accelY, s.accelZ, s.pressure);
      logf4("snG", s.gyroX, s.gyroY, s.gyroZ, s.temperature);
      logf4("snM", s.magX, s.magY, s.magZ);
    }
  }
}

static void scenario(const char *name, TriSenseMode mode, ICM_FIFO_MODE fifo, ICM_ODR odr,
                     bool advanced, int motion, TriSenseOrientation mount, uint32_t loopUs) {
  printf("=== %s\n", name);
  sim_reset(motion, 4242);
  TriSense sensor;
  AdvancedTriFusion adv(&sensor.imu, &sensor.mag);
  SimpleTriFusion simple(&sensor.imu, &sensor.mag);
  TriSenseFusion &fusion = advanced ? (TriSenseFusion &)adv : (TriSenseFusion &)simple;

  if (!sensor.beginAll(mode, 17, 10000000)) { printf("beginAll FAILED\n"); return; }
  sensor.imu.setFIFOMode(fifo);
  sensor.imu.setODR(odr);
  printf("odr %d fifo %d\n", sensor.imu.getODRHz(), (int)sensor.imu.getFIFOMode());

  float hard[3] = { 6.0f, -4.0f, 3.0f };
  float soft[3][3] = { { 1.01f, 0.002f, 0 }, { 0.002f, 0.99f, 0 }, { 0, 0, 1.0f } };
  fusion.setMagCalibration(hard, soft);
  fusion.setMountOrientation(mount);
  fusion.setDeclination(4.5f);
  fusion.setDynamicGyroBias(true, 0.0005f);
  fusion.setMagGaussian(50.0f, 6.0f, 20.0f);
  if (!advanced) simple.enableLightweightGravity(true, 0.03f);

  sim_motion(1, 99);                     // calibrate while still
  sensor.imu.flushFIFO();
  sensor.autoCalibrateGyro(200);
  fusion.initOrientation(100);
  {
    float gx, gy, gz;
    sensor.imu.getGyroOffset(gx, gy, gz);
    logf4("cal", gx, gy, gz);
  }
  sim_motion(motion, 777);
  sensor.imu.flushFIFO();
  run(sensor, fusion, 20, loopUs);
}

int main() {
  setvbuf(stdout, NULL, _IOLBF, 0);
  scenario("adv hybrid fifo16 1k", MODE_HYBRID, FIFO_16BIT, ODR_1KHZ, true, 0, ORIENTATION_Z_UP, 2000);
  scenario("adv hybrid hires 8k", MODE_HYBRID, FIFO_20BIT_HIRES, ODR_8KHZ, true, 0, ORIENTATION_Z_UP, 2500);
  scenario("adv hybrid fifo16 tumble", MODE_HYBRID, FIFO_16BIT, ODR_2KHZ, true, 2, ORIENTATION_X_UP, 1500);
  scenario("adv hybrid no fifo", MODE_HYBRID, FIFO_NONE, ODR_1KHZ, true, 0, ORIENTATION_Z_UP, 1000);
  scenario("adv i2c fifo16 200", MODE_I2C, FIFO_16BIT, ODR_200HZ, true, 0, ORIENTATION_Y_UP, 5000);
  scenario("simple hybrid fifo16", MODE_HYBRID, FIFO_16BIT, ODR_1KHZ, false, 0, ORIENTATION_Z_UP, 2000);
  scenario("simple no fifo", MODE_HYBRID, FIFO_NONE, ODR_500HZ, false, 2, ORIENTATION_Z_DOWN, 2000);
  return 0;
}
