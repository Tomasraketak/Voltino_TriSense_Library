/*
 * Example: TriSense_Calibration.ino
 * This tool is used for calibrating sensors on the Voltino TriSense board.
 * * INSTRUCTIONS:
 * 1. Open Serial Monitor (115200 baud).
 * 2. Send commands:
 * 'g' -> Automatic Gyro Calibration (Sensor must be still!)
 * 'a' -> 6-Point Accelerometer Calibration (Follow instructions in terminal)
 * 'x' -> Reset all HW offsets (Factory chip bias)
 * * 3. Copy the obtained values (code) into the setup() function of your project.
 */

#include <TriSense.h>

TriSense sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(500);

  Serial.println("Initializing TriSense...");

  // Initialization in Hybrid mode (most common use)
  // 3rd parameter sets the SPI frequency (e.g., 10000000 = 10 MHz for RP2040/ESP32)
  // NOTE: Standard Arduino (AVR like Uno/Nano) handles max 4MHz (4000000)!
  // If using I2C mode, change to: sensor.beginAll(MODE_I2C);
  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) {
    Serial.println("Initialization failed! Check wiring.");
    while (1);
  }
  
  // Set Output Data Rate for calibration
  sensor.imu.setODR(ODR_500HZ); 

  Serial.println("--------------------------------------");
  Serial.println("TriSense Calibration Tool");
  Serial.println("--------------------------------------");
  Serial.println("[g] -> Calibrate GYRO (Keep still!)");
  Serial.println("[a] -> Calibrate ACCELEROMETER (6 positions)");
  Serial.println("[x] -> RESET chip offsets");
  Serial.println("--------------------------------------");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    // --- GYRO CALIBRATION ---
    if (c == 'g') {
      // Calls wrapper in TriSense which triggers logic in ICM42688P
      sensor.autoCalibrateGyro(1000); 
    } 
    
    // --- ACCELEROMETER CALIBRATION ---
    else if (c == 'a') {
      // Starts interactive wizard (requires 'y' confirmation for each position)
      sensor.autoCalibrateAccel();
    }
    
    // --- RESET OFFSETS ---
    else if (c == 'x') {
      sensor.resetHardwareOffsets();
      Serial.println("Hardware registers cleared (Offset = 0).");
    }
  }
}