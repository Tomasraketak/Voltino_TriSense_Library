// Attitude error against the simulator's ground truth, for comparing builds
// that are NOT meant to be bit-identical (feature switches, TRISENSE_LITE).
//
//   make accuracy && ./accuracy
//
// Prints, per scenario, the RMS and worst tilt error (angle between the true
// and the estimated gravity direction) and the RMS heading error over the
// last 30 s of a 60 s run (the first half lets the filter converge from the
// jump between the still calibration pose and the moving one; the constant
// offset between magnetic and simulator north is removed first). Z_UP mount, so sensor and body frames coincide.
#include "Arduino.h"
#include "TriSense.h"
#include <math.h>

void sim_reset(int motion, uint32_t seed);
void sim_motion(int motion, uint32_t seed);
void sim_truth(double q[4]);

static void gravityBody(double w, double x, double y, double z, double g[3]) {
  g[0] = 2 * (x * z - w * y); g[1] = 2 * (w * x + y * z); g[2] = w * w - x * x - y * y + z * z;
}
static double yawOf(double w, double x, double y, double z) {
  return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

static void scenario(const char *name, ICM_FIFO_MODE fifo, ICM_ODR odr, int motion, uint32_t loopUs) {
  sim_reset(motion, 4242);
  TriSense sensor;
  AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);
  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) { printf("beginAll FAILED\n"); return; }
  sensor.imu.setFIFOMode(fifo);
  sensor.imu.setODR(odr);
  float hard[3] = { 6.0f, -4.0f, 3.0f };
  float soft[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
  fusion.setMagCalibration(hard, soft);
  fusion.setMagGaussian(49.0f, 6.0f, 20.0f);
  fusion.setDynamicGyroBias(true, 0.0005f);
  sim_motion(1, 99);
  sensor.imu.flushFIFO();
  sensor.autoCalibrateGyro(200);
  fusion.initOrientation(100);
  sim_motion(motion, 777);
  sensor.imu.flushFIFO();

  double tiltSq = 0, tiltMax = 0, n = 0;
  double yawErr[4000]; int ny = 0;
  for (uint32_t t = 0, k = 0; t < 60000000u; t += loopUs, k++) {
    sim_advance_us(loopUs);
    fusion.update();
    if (k % 50) continue;
    double tq[4]; sim_truth(tq);
    float w, x, y, z; fusion.getQuaternion(w, x, y, z);
    double gt[3], ge[3];
    gravityBody(tq[0], tq[1], tq[2], tq[3], gt);
    gravityBody(w, x, y, z, ge);
    double c = gt[0] * ge[0] + gt[1] * ge[1] + gt[2] * ge[2];
    if (c > 1) c = 1;
    if (t < 30000000u) continue;   // Let the jump from the still pose settle
    double e = acos(c) * 180 / M_PI;
    tiltSq += e * e; n++; if (e > tiltMax) tiltMax = e;
    if (ny < 4000) {
      double d = yawOf(w, x, y, z) - yawOf(tq[0], tq[1], tq[2], tq[3]);
      while (d > M_PI) d -= 2 * M_PI; while (d < -M_PI) d += 2 * M_PI;
      yawErr[ny++] = d * 180 / M_PI;
    }
  }
  double mean = 0; for (int i = 0; i < ny; i++) mean += yawErr[i]; mean /= ny;
  double ys = 0; for (int i = 0; i < ny; i++) ys += (yawErr[i] - mean) * (yawErr[i] - mean);
  printf("%-22s tilt rms %6.3f max %6.3f deg | heading rms %6.3f deg (offset %.1f)\n",
         name, sqrt(tiltSq / n), tiltMax, sqrt(ys / ny), mean);
}

int main() {
  scenario("fifo16 1k wander", FIFO_16BIT, ODR_1KHZ, 0, 2000);
  scenario("fifo16 2k tumble", FIFO_16BIT, ODR_2KHZ, 2, 1500);
  scenario("no fifo 1k wander", FIFO_NONE, ODR_1KHZ, 0, 1000);
  return 0;
}
