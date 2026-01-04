/*
 * Example: MotionCal_Bridge.ino
 * Sends raw sensor data from Voltino TriSense to the MotionCal application.
 * * DOWNLOAD MOTIONCAL: https://www.pjrc.com/store/prop_shield.html
 *
 * * INSTRUCTIONS:
 * 1. Upload this sketch to your Raspberry Pi Pico 2.
 * 2. Run the MotionCal application on your PC.
 * 3. Select the correct COM port.
 * 4. Rotate the sensor in all directions until the sphere is filled with dots.
 * 5. Copy the resulting values (Hard Iron, Soft Iron) into your setup code.
 */

#include <TriSense.h>

TriSense sensor;

// Variables to store the latest readings
float ax, ay, az;
float gx, gy, gz;

void setup() {
  // Preferred baud rate
  Serial.begin(115200);
  delay(500);

  // Initialize sensors in I2C MODE
  // In this mode, all sensors (IMU, Mag, Baro) share the I2C bus.
  // SPI parameters are ignored here.
  if (!sensor.beginAll(MODE_I2C)) {
    Serial.println("Error: Sensors not found!");
    while (1) delay(10);
  }

  // IMPORTANT: For MotionCal we want the "rawest" data possible.
  // We reset HW offsets in ICM so the chip doesn't pre-process data.
  sensor.resetHardwareOffsets();

  // Set Magnetometer ODR to maximum (100 Hz) for smooth painting in MotionCal
  sensor.mag.setODR(100);
}

void loop() {
  // 1. Read IMU data (Accelerometer + Gyro)
  // readFIFO returns true if new data is available
  // We buffer this data globally so it's ready when the Magnetometer is ready.
  float tax, tay, taz, tgx, tgy, tgz;
  if (sensor.imu.readFIFO(tax, tay, taz, tgx, tgy, tgz)) {
    ax = tax; ay = tay; az = taz;
    gx = tgx; gy = tgy; gz = tgz;
  }

  // 2. Read Magnetometer data
  // The Mag is slower (100Hz) than the IMU. We send data to PC
  // only when we have a new Mag sample to avoid duplicate points in MotionCal.
  if (sensor.mag.readData()) {
    
    // 3. Format for MotionCal
    // MotionCal expects format: "Raw:ax,ay,az,gx,gy,gz,mx,my,mz"
    // It expects integers, so we convert floats back to "pseudo-raw" values.
    
    Serial.print("Raw:");
    
    // Accel: Library returns [g]. Multiply by 8192 (Approx. raw LSB for 4G range)
    Serial.print((int)(ax * 8192)); Serial.print(",");
    Serial.print((int)(ay * 8192)); Serial.print(",");
    Serial.print((int)(az * 8192)); Serial.print(",");

    // Gyro: Library returns [deg/s]. Multiply by 16 (Approx. raw sensitivity)
    // MotionCal uses gyro only for visual rotation, not critical for mag calibration.
    Serial.print((int)(gx * 16));   Serial.print(",");
    Serial.print((int)(gy * 16));   Serial.print(",");
    Serial.print((int)(gz * 16));   Serial.print(",");

    // Mag: Library returns [uT]. Multiply by 10 to get reasonable integers.
    // (e.g., 40 uT -> 400). Sufficient resolution for the tool.
    Serial.print((int)(sensor.mag.x * 10)); Serial.print(",");
    Serial.print((int)(sensor.mag.y * 10)); Serial.print(",");
    Serial.print((int)(sensor.mag.z * 10));
    
    Serial.println();
    
    // Small delay to prevent serial buffer saturation, 
    delay(40); 
  }
}