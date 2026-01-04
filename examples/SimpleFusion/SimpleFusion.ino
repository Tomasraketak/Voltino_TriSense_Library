/*
 * Example: SimpleFusion.ino
 * Demonstration of simple sensor fusion using the new calibration system.
 */

#include <TriSense.h>

TriSense sensor;
SimpleTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrint = 0;
const unsigned long printInterval = 50000; // 20Hz output (50000 us)

void setup() {
  Serial.begin(115200);
  delay(500);

  // 1. Sensor Initialization
  
  // OPTION A: Hybrid Mode (Recommended for better results)
  // AK09918C + BMP580 on I2C, ICM-42688-P on SPI (CS pin 17, 10MHz)
  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) {
  
  // OPTION B: I2C Only Mode
  // If you want to run everything on I2C (slower, but fewer wires), uncomment below:
  // if (!sensor.beginAll(MODE_I2C)) {
  
    Serial.println("Failed to initialize sensors!");
    while (1);
  }

  // Set ODR for IMU
  sensor.imu.setODR(ODR_4KHZ); 

  // =============================================================
  // 2. CALIBRATION (Insert values from TriSense_Calibration.ino)
  // =============================================================
  
  // A) Accelerometer (Bias + Scale)
  // Run TriSense_Calibration.ino, press 'a', and copy the code output here.
  // If values are 0/1, it means no calibration is applied.
  sensor.imu.setAccelOffset(0.00, 0.00, 0.00);
  sensor.imu.setAccelScale(1.00, 1.00, 1.00);

  // B) Gyroscope
  // You can set fixed offset values if known:
  // sensor.imu.setGyroOffset(0.52, -0.12, 0.05);
  
  // ... OR perform fast calibration on every startup (Recommended):
  // Sensor must be stationary during this phase!
  Serial.println("Calibrating Gyro... Keep still!");
  sensor.autoCalibrateGyro(500); 

  // C) Magnetometer (Hard/Soft Iron)
  // Run MotionCal or similar tool to get these values.
  fusion.setMagHardIron(-46.02, -0.85, -46.00);
  
  float softIron[3][3] = {
    {0.965, 0.008, -0.002},
    {0.008, 0.981, 0.139},
    {-0.002, 0.139, 1.077}
  };
  fusion.setMagSoftIron(softIron);
  
  // Magnetic Declination (Change according to your location)
  // Example for Prague/Central Europe (~5.6 degrees)
  fusion.setDeclination(5.6); 

  // =============================================================

  // Initialize orientation algorithm
  Serial.println("Initializing orientation...");
  fusion.initOrientation();
  Serial.println("Ready.");
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