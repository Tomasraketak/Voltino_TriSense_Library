/*
 * Example: AdvancedFusion_Improved.ino
 * Advanced sensor fusion (Complementary Filter with Gaussian confidence)
 * using precise 6-point calibration.
 * * HW: Raspberry Pi Pico 2 + Voltino TriSense
 */

#include <TriSense.h>

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 20000; // 50Hz Serial output

void setup() {
  Serial.begin(115200);
  delay(500);

  // 1. Sensor Initialization
  
  // OPTION A: Hybrid Mode (Recommended for Voltino)
  // AK09918C + BMP580 on I2C, ICM-42688-P on SPI (CS 17, 10MHz)
  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) {
  
  // OPTION B: I2C Only Mode
  // Use this if you want to save wires or pins.
  // if (!sensor.beginAll(MODE_I2C)) {
  
    Serial.println("Sensor init failed!");
    while (1);
  }

  // Set ODR for IMU (Turbo mode for SPI, max 1kHz for I2C)
  sensor.imu.setODR(ODR_8KHZ);

  // =============================================================
  // 2. CALIBRATION DATA (IMPORTANT)
  // =============================================================

  // --- A) ACCELEROMETER (ICM-42688-P) ---
  // Obtain these values using 'TriSense_Calibration.ino' (option 'a').
  // Example output:
  // sensor.imu.setAccelOffset(0.01234, -0.00567, 0.00123);
  // sensor.imu.setAccelScale(1.00100, 0.99950, 1.00200);
  
  // PASTE YOUR VALUES HERE:
  sensor.imu.setAccelOffset(0.0, 0.0, 0.0); // <-- Replace with your offsets
  sensor.imu.setAccelScale(1.0, 1.0, 1.0);  // <-- Replace with your scale factors

  // --- B) GYROSCOPE ---
  // Calibrate gyro on every startup to remove boot drift.
  // Sensor must be stationary!
  Serial.println("Calibrating Gyro... Keep still!");
  sensor.autoCalibrateGyro(800);

  // --- C) MAGNETOMETER (AK09918C) ---
  // Obtain these values using MotionCal and 'MotionCal_Bridge.ino'
  fusion.setMagHardIron(-46.02, -0.85, -46.00); // <-- Replace (Hard Iron)
  
  float softIron[3][3] = {
    {1.0, 0.0, 0.0}, // <-- Replace with Soft Iron Matrix
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
  };
  fusion.setMagSoftIron(softIron);
  
  // Magnetic Declination (approx 5.6 for Prague/Central Europe)
  // Look up your location at: magnetic-declination.com
  fusion.setDeclination(5.6);

  // =============================================================
  // 3. FUSION PARAMETERS TUNING (OPTIONAL)
  // =============================================================
  
  // Accelerometer trust vs. Gyro drift
  // Reference 1G, Sigma (noise allowed)
  //fusion.setAccelGaussian(1.0, 0.02);
  
  // Magnetometer trust
  // Reference strength (uT), Sigma
  //fusion.setMagGaussian(48.0, 4.0);    
  
  // Correction Gains (0.0 to 1.0)
  // Higher = faster correction but more noise susceptibility
  //fusion.setMaxGains(0.4, 0.4);
  
  // Integral term to fix long-term yaw drift
  //fusion.setYawKi(0.0075); 

  // =============================================================

  Serial.println("Do not move sensor board!");
  Serial.println("Calibrating initial orientation...");
      delay(2000);

  // initOrientation() uses the calibrated accelerometer and magnetometer data
  // to set the starting quaternion.
  fusion.initOrientation();
  
  Serial.println("Calibration Done, System Running.");
}

void loop() {
  // Fusion Update: reads data, integrates gyro, applies corrections
  if (fusion.update()) {
    
    // Data Output
    unsigned long now = micros();
    if (now - lastPrint >= printInterval) {
      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);
      
      // Output format compatible with Serial Plotter
      Serial.print("R:"); Serial.print(roll, 1);
      Serial.print("\tP:"); Serial.print(pitch, 1);
      Serial.print("\tY:"); Serial.println(yaw, 1);
      
      lastPrint = now;
    }
  }
}