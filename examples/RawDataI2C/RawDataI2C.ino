/*
 * Example: RawDataI2C.ino
 *
 * Description:
 * Example of reading raw physical data (without extra calibration)
 * from all sensors in I2C mode.
 *
 * Units:
 * - Accelerometer: g (gravitational acceleration) and m/s^2
 * - Gyroscope:     dps (degrees per second)
 * - Magnetometer:  uT (micro Tesla)
 * - Pressure:      Pa (Pascal)
 * - Temperature:   C (Celsius)
 * - Altitude:      m (meters, based on standard sea level pressure)
 */

#include <TriSense.h>

TriSense sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for USB Serial (for Pico)

  Serial.println("Initializing TriSense in I2C mode...");

  // 1. Initialize all sensors in I2C mode
  // In this mode, IMU, Mag, and Baro share the same bus.
  // SPI parameters (CS pin, freq) are ignored.
  if (!sensor.beginAll(MODE_I2C)) {
    Serial.println("Error: Sensors not found! Check wiring.");
    while (1) delay(100);
  }

  // 2. Reset HW offsets
  // Clear offsets stored directly in the ICM-42688-P chip to ensure
  // we are seeing true "factory raw" data.
  sensor.resetHardwareOffsets();
  
  // Set a reasonable sample rate for Serial output
  sensor.imu.setODR(ODR_50HZ); 

  Serial.println("Sensors ready.");
  Serial.println("Format: Accel[g] | Gyro[dps] | Mag[uT] | Baro[Pa/m/C]");
}

void loop() {
  // IMU Variables
  float ax, ay, az; // g
  float gx, gy, gz; // dps

  // 1. Read IMU (Accelerometer + Gyro)
  // readFIFO returns true only when new data is available in the buffer
  if (sensor.imu.readFIFO(ax, ay, az, gx, gy, gz)) {
    
    // 2. Read Magnetometer (AK09918C)
    // If mag is ready, data is loaded into internal variables sensor.mag.x/y/z
    sensor.mag.readData(); 
    
    // 3. Read Barometer (BMP580)
    // BMP580 is very fast, we can read directly
    float temp = sensor.bmp.readTemperature();
    float press = sensor.bmp.readPressure();
    float alt = sensor.bmp.readAltitude(101325.0); // 101325 Pa = Standard sea level pressure

    // --- DATA OUTPUT ---
    
    // A) Accelerometer
    Serial.print("A [g]: ");
    Serial.print(ax, 2); Serial.print(", ");
    Serial.print(ay, 2); Serial.print(", ");
    Serial.print(az, 2);
    
    // Convert to m/s^2 (1g = ~9.80665 m/s^2)
    Serial.print(" | [m/s2]: ");
    Serial.print(ax * 9.81, 1); Serial.print(", ");
    Serial.print(ay * 9.81, 1); Serial.print(", ");
    Serial.print(az * 9.81, 1);

    // B) Gyroscope
    Serial.print(" | G [dps]: ");
    Serial.print(gx, 1); Serial.print(", ");
    Serial.print(gy, 1); Serial.print(", ");
    Serial.print(gz, 1);

    // C) Magnetometer
    Serial.print(" | M [uT]: ");
    Serial.print(sensor.mag.x, 1); Serial.print(", ");
    Serial.print(sensor.mag.y, 1); Serial.print(", ");
    Serial.print(sensor.mag.z, 1);

    // D) Barometer (Pressure, Temp, Altitude)
    Serial.print(" | Alt: "); Serial.print(alt, 1); Serial.print("m");
    Serial.print(" P: "); Serial.print(press / 100.0); Serial.print("hPa"); // Pa -> hPa
    Serial.print(" T: "); Serial.print(temp, 1); Serial.println("C");

    // Slow down output to make it readable 
    // (IMU has a FIFO buffer, so we won't lose data, just read it later or skip)
    delay(100); 
  }
}