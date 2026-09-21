/*
 * Example: SimpleFusion.ino
 * Demonstration of simple sensor fusion.
 */

#include <TriSense.h>

TriSense sensor;
SimpleTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
// 10 Hz is plenty for a human to read, and it matters more than it looks: a
// ~190-character line takes about 17 ms to clear at 115200 baud, while the FIFO
// fills in 12.8 ms at 8 kHz. Printing every 50 ms overruns it on its own - see
// docs/GUIDE.md, "Writing a loop that keeps up".
const unsigned long printInterval = 100000; // 10Hz Serial output

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) {
    Serial.println("Failed to initialize sensors!");
    while (1);
  }

  // Simple fusion is fast, ODR can be kept standard
  sensor.imu.setODR(ODR_4KHZ); 

  // Fast Gyro Calibration
  Serial.println("Calibrating Gyro... Keep still!");
  sensor.autoCalibrateGyro(500); 

  // NOTE: SimpleTriFusion integrates the gyro only - it never reads the
  // magnetometer in update(), so yaw WILL drift and these values do not correct
  // it. They still matter, because initOrientation() below uses the
  // magnetometer once to seed the starting heading. For a heading that stays
  // referenced to north, use AdvancedTriFusion.
  fusion.setMagHardIron(-46.02, -0.85, -46.00);
  float softIron[3][3] = {
    {0.965, 0.008, -0.002},
    {0.008, 0.981, 0.139},
    {-0.002, 0.139, 1.077}
  };
  fusion.setMagSoftIron(softIron);
  fusion.setDeclination(5.6); 

  Serial.println("Calibrating initial orientation...");
  fusion.initOrientation();
}

void loop() {
  if (fusion.update()) {
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      lastPrint = now;
      
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);
      
      Serial.print("R: "); Serial.print(roll, 1);
      Serial.print("\tP: "); Serial.print(pitch, 1);
      Serial.print("\tY: "); Serial.println(yaw, 1);
    }
  }
}