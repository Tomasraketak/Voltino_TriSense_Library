/*
 * Example: GlobalAcceleration.ino (Updated for v1.3.0)
 *
 * Description:
 * Extracts pure Global Acceleration (World Frame Acceleration)
 * compensated for sensor tilt using AdvancedTriFusion.
 *
 * The world-frame acceleration is built from the MEAN of every accelerometer
 * sample the FIFO delivered since the last update() - not just the newest one -
 * so vibration is low-passed and no samples are wasted. See README.
 *
 * Also shows the two things you need if you intend to integrate acceleration:
 *   - setLocalGravity()   -> gravity differs by latitude/altitude, and any error
 *                            here becomes a constant offset you integrate twice
 *   - FIFO overflow check -> dropped IMU packets are unrecoverable lost motion,
 *                            so a log with overflows in it cannot be trusted
 */

#include <TriSense.h>

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 40000; // 25Hz output

void setup() {
  Serial.begin(115200);
  delay(1000); 

  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Sensor init failed!");
    while (1) delay(100);
  }

  // Quick calibration for demonstration
  sensor.autoCalibrateGyro(500);

  // Local gravity for your location. 9.80665 is the standard value; the real
  // figure ranges from ~9.780 (equator) to ~9.832 (poles). Prague is ~9.8127.
  // Only matters when you ask for ACCEL_UNIT_MS2 or integrate the result.
  fusion.setLocalGravity(9.8127f);

  Serial.println("Calibrating initial orientation...");
  fusion.initOrientation();
  Serial.println("System Running. Format: R, P, Y | GlobalAcc(G) | LinearAcc(G)");
}

void loop() {
  if (fusion.update()) {
    
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      lastPrint = now;
      
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);

      // Get GLOBAL ACCELERATION [G] (gravity included, e.g. Z ~= +1G at rest)
      float ax_g, ay_g, az_g;
      fusion.getGlobalAcceleration(ax_g, ay_g, az_g);

      // Get GLOBAL LINEAR ACCELERATION [G] - same as above but with gravity
      // already removed by the library, ready for dead-reckoning integration.
      float ax_lin, ay_lin, az_lin;
      fusion.getGlobalLinearAcceleration(ax_lin, ay_lin, az_lin);

      Serial.print("R:"); Serial.print(roll, 1);
      Serial.print("\tP:"); Serial.print(pitch, 1);
      Serial.print("\tY:"); Serial.print(yaw, 1);
      
      Serial.print("  |  GlobalAcc(G) -> X:"); Serial.print(ax_g, 3);
      Serial.print(" Y:"); Serial.print(ay_g, 3);
      Serial.print(" Z:"); Serial.print(az_g, 3);

      Serial.print("  |  LinearAcc(G) -> X:"); Serial.print(ax_lin, 3);
      Serial.print(" Y:"); Serial.print(ay_lin, 3);
      Serial.print(" Z:"); Serial.print(az_lin, 3);

      // Warn if the IMU FIFO overflowed since the last check. Those samples are
      // gone for good, so any integration across this interval has a permanent
      // error. If you see this, lower the ODR or speed up the loop (a blocking
      // Serial.print at a low baud rate is the usual culprit).
      if (sensor.imu.fifoOverflowed()) {
        Serial.print("  !! FIFO OVERFLOW (total ");
        Serial.print(sensor.imu.getFIFOOverflowCount());
        Serial.print(")");
      }
      Serial.println();
    }
  }
}