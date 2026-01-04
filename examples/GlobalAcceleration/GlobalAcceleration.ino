/*
 * Example: AdvancedFusion_GlobalAccel.ino
 *
 * Description:
 * Demonstration of using advanced fusion (AdvancedTriFusion) together with
 * Global Acceleration (World Frame Acceleration) calculation.
 *
 * Features:
 * - Sensor fusion of gyroscope, accelerometer, and magnetometer data.
 * - Orientation output (Roll, Pitch, Yaw).
 * - Global acceleration output (compensated for sensor tilt).
 * -> If the sensor is stationary (at any tilt), the Z-axis should show approx 1.00 G.
 */

#include <TriSense.h>

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 40000; // 25Hz output (40ms)

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial connection initialization

  // 1. Sensor Initialization
  // CS pin 17 for Raspberry Pi Pico
  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Sensor init failed!");
    while (1) {
      delay(100); // LED blink or wait loop
    }
  }



  // =============================================================
  // 2. CALIBRATION DATA
  // (Insert your measured values from calibration scripts here)
  // =============================================================

  // -- ACCELEROMETER (ICM-42688-P) --
  // Example: sensor.imu.setAccelOffset(0.012, -0.005, 0.001);
  sensor.imu.setAccelOffset(0.0, 0.0, 0.0); 
  sensor.imu.setAccelScale(1.0, 1.0, 1.0);

  // -- GYROSCOPE --
  // Fast startup calibration (sensor must be stationary!)
  Serial.println("Calibrating Gyro... Keep still!");
  sensor.autoCalibrateGyro(1000);
  
  //sensor.imu.setGyroOffset(-0.5716, -0.2940, 0.1374);  // Uncomment if you have already measured offset

  // -- MAGNETOMETER (AK09918C) --
  fusion.setMagHardIron(-46.02, -0.85, -46.00);
  
  float softIron[3][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  };
  fusion.setMagSoftIron(softIron);
  
  // Magnetic declination (approx 5.6 deg for Prague)
  fusion.setDeclination(5.6); 

  // =============================================================
  // 3. FUSION PARAMETERS
  // =============================================================
  
  // Accelerometer trust (Ref 1G, Sigma noise)
 
  // Initialize orientation
  Serial.println("Calibrating initial orientation...");
  fusion.initOrientation();
  
  Serial.println("System Running. Format: R, P, Y | Ax_G, Ay_G, Az_G");
}

void loop() {
  // Fusion update (reads data, computes orientation)
  if (fusion.update()) {
    
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      
      // 1. Get orientation (degrees)
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);

      // 2. Get GLOBAL ACCELERATION [G]
      // (This function calculates acceleration in the Earth frame)
      float ax_g, ay_g, az_g;
      fusion.getGlobalAcceleration(ax_g, ay_g, az_g);

      // --- Optional: Pure Linear Acceleration ---
      // If you want to remove gravity, uncomment this line:
      // az_g -= 1.0f; 

      // Print data to Serial Plotter / Monitor
      // Orientation
      Serial.print("R:"); Serial.print(roll, 1);
      Serial.print("\tP:"); Serial.print(pitch, 1);
      Serial.print("\tY:"); Serial.print(yaw, 1);
      
      // Global Acceleration
      // When stationary, X and Y should be close to 0.00 and Z close to 1.00
      Serial.print("\t| GA_X:"); Serial.print(ax_g, 3);
      Serial.print("\tGA_Y:"); Serial.print(ay_g, 3);
      Serial.print("\tGA_Z:"); Serial.println(az_g, 3);
      
      lastPrint = now;
    }
  }
}