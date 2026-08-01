/*
 * Example: RawDataI2C.ino (Updated for v1.3.0)
 *
 * Description:
 * Example of reading raw physical data (without extra calibration)
 * from all sensors in I2C mode using the new unified Snapshot API.
 *
 * Units:
 * - Accelerometer: g (gravitational acceleration) and m/s^2
 * - Gyroscope:     dps (degrees per second)
 * - Magnetometer:  uT (micro Tesla)
 * - Pressure:      Pa (Pascal)
 * - Temperature:   C (Celsius)
 * - Altitude:      m (meters)
 */

#include <TriSense.h>

TriSense sensor;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); 

  Serial.println("Initializing TriSense in I2C mode...");

  if (!sensor.beginAll(MODE_I2C)) {
    Serial.println("Error: Sensors not found! Check wiring.");
    while (1) delay(100);
  }

  // Clear hardware offsets to see true "factory raw" data
  sensor.resetHardwareOffsets();
  Serial.println("System Ready. Reading unified snapshot...");
}

void loop() {
  TriSenseDataSnapshot data;
  
  // getSnapshot() fetches synchronous data from IMU, Mag, and Baro at once
  if (sensor.getSnapshot(data)) {
    
    if (millis() - lastPrint >= 100) { // Print at 10Hz to avoid serial spam
      lastPrint = millis();

      // --- DATA OUTPUT ---
      Serial.print("A [g]: ");
      Serial.print(data.accelX, 2); Serial.print(", ");
      Serial.print(data.accelY, 2); Serial.print(", ");
      Serial.print(data.accelZ, 2);
      
      Serial.print(" | G [dps]: ");
      Serial.print(data.gyroX, 1); Serial.print(", ");
      Serial.print(data.gyroY, 1); Serial.print(", ");
      Serial.print(data.gyroZ, 1);

      Serial.print(" | M [uT]: ");
      Serial.print(data.magX, 1); Serial.print(", ");
      Serial.print(data.magY, 1); Serial.print(", ");
      Serial.print(data.magZ, 1);

      Serial.print(" | Baro: ");
      Serial.print(data.pressure); Serial.print(" Pa, ");
      Serial.print(data.temperature); Serial.print(" C");

      // Altitude needs a sea-level reference pressure in PASCALS (not hPa).
      // 101325.0 Pa is the ISA standard day; substitute your local QNH
      // (hPa from a weather report x 100) for an accurate absolute altitude.
      Serial.print(" | Alt: ");
      Serial.print(sensor.readAltitude(101325.0f), 1); Serial.println(" m");
    }
  }
}