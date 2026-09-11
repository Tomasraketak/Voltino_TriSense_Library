/*
 * Example: IMUcalibration.ino (Updated for v1.2.0)
 * This tool is used for calibrating sensors on the Voltino TriSense board.
 * * INSTRUCTIONS:
 * 1. Open Serial Monitor (115200 baud).
 * 2. Send commands:
 * 'g' -> Automatic Gyro Calibration (Sensor must be still!)
 * 'a' -> 6-Point Accelerometer Calibration (Sphere fit)
 */

#include <TriSense.h>

TriSense sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(500);

  Serial.println("Initializing TriSense...");

  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) {
    Serial.println("Initialization failed! Check wiring.");
    while (1);
  }
  
  sensor.imu.setODR(ODR_500HZ); 

  Serial.println("--------------------------------------");
  Serial.println("Voltino TriSense Calibration Tool v1.2");
  Serial.println("--------------------------------------");
  Serial.println("[g] -> Calibrate GYRO (Keep still!)");
  Serial.println("[a] -> Calibrate ACCELEROMETER (6 positions)");
  Serial.println("--------------------------------------");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'g') {
      sensor.autoCalibrateGyro(1000); 
    } 
    else if (c == 'a') {
      sensor.autoCalibrateAccel(); 
    }
  }
}