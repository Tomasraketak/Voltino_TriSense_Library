/*
 * Example: MotionCal.ino
 * Sends raw sensor data from Voltino TriSense to the MotionCal application.
 * DOWNLOAD MOTIONCAL: https://www.pjrc.com/store/prop_shield.html
 */

#include <TriSense.h>

TriSense sensor;

// Variables to store the latest IMU readings
float ax, ay, az;
float gx, gy, gz;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!sensor.beginAll(MODE_I2C)) {
    Serial.println("Error: Sensors not found!");
    while (1) delay(10);
  }

  // MotionCal only ever looks at the magnetometer locus, and mag readings never
  // pass through the IMU's offset/scale registers - so nothing needs clearing here.
}

void loop() {
  // 1. Always drain IMU FIFO to have the freshest attitude data
  float tax, tay, taz, tgx, tgy, tgz;
  if (sensor.imu.readFIFO(tax, tay, taz, tgx, tgy, tgz)) {
    ax = tax; ay = tay; az = taz;
    gx = tgx; gy = tgy; gz = tgz;
  }

  // 2. Output data to PC ONLY when the 100Hz Magnetometer fires a new sample.
  if (sensor.mag.readData()) {
    
    // MotionCal expects format: "Raw:ax,ay,az,gx,gy,gz,mx,my,mz" (integers)
    Serial.print("Raw:");
    
    // Accel: Multiply by 8192 (Approx. raw LSB for 4G range)
    Serial.print((int)(ax * 8192)); Serial.print(",");
    Serial.print((int)(ay * 8192)); Serial.print(",");
    Serial.print((int)(az * 8192)); Serial.print(",");

    // Gyro: Multiply by 16 (Approx. raw sensitivity)
    Serial.print((int)(gx * 16));   Serial.print(",");
    Serial.print((int)(gy * 16));   Serial.print(",");
    Serial.print((int)(gz * 16));   Serial.print(",");

    // Mag: MotionCal expects values ~10 times larger than uT (raw format)
    Serial.print((int)(sensor.mag.x * 10)); Serial.print(",");
    Serial.print((int)(sensor.mag.y * 10)); Serial.print(",");
    Serial.println((int)(sensor.mag.z * 10));
  }
}