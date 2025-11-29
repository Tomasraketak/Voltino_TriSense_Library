// Example: SimpleFusion.ino
// This example demonstrates simple sensor fusion using TriSense library.
// It initializes all sensors in hybrid mode and performs gyro integration with initial orientation.

#include <TriSense.h>

TriSense sensor;
SimpleTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 50000; // 20Hz

void setup() {
  Serial.begin(115200);
  delay(200);

  // Initialize all sensors in hybrid mode (IMU on SPI)
  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Failed to initialize sensors!");
    while (1);
  }

  // Custom offsets (optional, replace with your values)
  fusion.setGyroOffsets(-0.658, -0.31266, 0.25129);
  fusion.setMagHardIron(-46.02, -0.85, -46.00);
  float softIron[3][3] = {
    {0.965, 0.008, -0.002},
    {0.008, 0.981, 0.139},
    {-0.002, 0.139, 1.077}
  };
  fusion.setMagSoftIron(softIron);
  fusion.setDeclination(5.0 + 1.0 / 60.0);

  // Initialize orientation
  Serial.println("Do not move the sensor, calibrating...");
  fusion.initOrientation();
  Serial.println("Calibration done.");
}

void loop() {
  if (fusion.update()) {
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);
      Serial.print("Roll: "); Serial.print(roll, 2);
      Serial.print(" Pitch: "); Serial.print(pitch, 2);
      Serial.print(" Yaw: "); Serial.println(yaw, 2);
      lastPrint = now;
    }
  }
}