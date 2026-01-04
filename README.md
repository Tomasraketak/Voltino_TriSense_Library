# Voltino TriSense Library

## Description
The **Voltino TriSense** library is a high-performance Arduino library designed for the **Voltino TriSense Pro** sensor board. It integrates support for the onboard sensors:
* **BMP580** (High-precision pressure and temperature)
* **AK09918C** (3-axis Magnetometer)
* **ICM-42688-P** (6-axis IMU with Accelerometer and Gyroscope)

This library allows for easy initialization of all sensors simultaneously in either **I2C mode** or **Hybrid mode** (AK09918C and BMP580 on I2C, ICM42688P on SPI for maximum speed).

It features a robust **Sensor Fusion** engine (Madgwick/Mahony based Complementary Filter) with dynamic Gaussian gain adjustment, providing stable orientation (Roll, Pitch, Yaw) and Global Acceleration vectors.

## Key Features
* **Unified Initialization:** One line of code to set up all sensors.
* **Hybrid Mode Support:** Runs the IMU on SPI (up to 10MHz) while keeping other sensors on I2C.
* **Sphere Fit Calibration:** Built-in 6-point accelerometer calibration algorithm.
* **Advanced Sensor Fusion:**
    * Quaternion-based orientation estimation.
    * **Global Acceleration:** Calculates linear acceleration in the Earth frame (removing sensor tilt).
    * **Dynamic Gains:** Uses Gaussian functions to trust sensors less during dynamic motion or magnetic anomalies.
    * **Yaw Drift Correction:** Integral term (Ki) to minimize long-term heading drift.
* **MotionCal Compatibility:** Dedicated example for easy magnetometer calibration using visual tools.

## Installation
1.  Download the latest release as a ZIP file from the GitHub repository.
2.  Open the Arduino IDE.
3.  Go to **Sketch > Include Library > Add .ZIP Library...** and select the downloaded ZIP file.
4.  The library should now be available in **Sketch > Include Library > Voltino TriSense**.

## Dependencies
* `Wire` (Standard Arduino I2C)
* `SPI` (Standard Arduino SPI)

## Usage

### 1. Including the Library
#include <TriSense.h>
2. Initializing Sensors
You can initialize all sensors at once using beginAll().

Parameters:

mode: MODE_HYBRID (Recommended) or MODE_I2C.

spiCsPin: Chip Select pin for the IMU (Default is 17 for Voltino/Pico).

spiFreq: SPI Frequency in Hz (Default 10000000 = 10 MHz).


TriSense sensor;

void setup() {
  Serial.begin(115200);

  // Initialize in Hybrid Mode:
  // - AK09918C & BMP580 on I2C
  // - ICM42688P on SPI (Pin 17, 10 MHz)
  if (!sensor.beginAll(MODE_HYBRID, 17, 10000000)) {
    Serial.println("Failed to initialize sensors!");
    while (1);
  }

  // Set ODR (Output Data Rate)
  sensor.imu.setODR(ODR_4KHZ); // Turbo mode for SPI
}
3. Reading Raw Data
Access individual sensor objects via sensor.bmp, sensor.mag, and sensor.imu.

// BMP580
float temp = sensor.bmp.readTemperature();
float press = sensor.bmp.readPressure();
float alt = sensor.bmp.readAltitude(101325);

// AK09918C
if (sensor.mag.readData()) {
    float mx = sensor.mag.x;
    float my = sensor.mag.y;
    float mz = sensor.mag.z;
}

// ICM-42688-P
float ax, ay, az, gx, gy, gz;
if (sensor.imu.readFIFO(ax, ay, az, gx, gy, gz)) {
    // Data is available in variables
}
Sensor Fusion & Orientation
The library provides AdvancedTriFusion for high-quality orientation data.

Setup

AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

void setup() {
    // ... init sensors ...

    // Set Calibration Data (See Calibration section)
    sensor.imu.setAccelOffset(0.01, -0.02, 0.05);
    sensor.imu.setAccelScale(1.00, 1.00, 1.00);
    fusion.setMagHardIron(-40.0, 12.5, 5.0);
    
    // Set Magnetic Declination (e.g., 5.6 degrees for Central Europe)
    fusion.setDeclination(5.6);

    // Initialize Orientation
    fusion.initOrientation();
}
Loop

void loop() {
    if (fusion.update()) {
        float roll, pitch, yaw;
        fusion.getOrientationDegrees(roll, pitch, yaw);
        
        Serial.print("Roll: "); Serial.print(roll);
        Serial.print(" Pitch: "); Serial.print(pitch);
        Serial.print(" Yaw: "); Serial.println(yaw);
    }
}
Global Acceleration
Calculate acceleration relative to the Earth (removing gravity and sensor tilt).

Stationary: Z ≈ 1.0 G, X/Y ≈ 0.0 G.

Linear Motion: Shows pure movement acceleration.


float ax_g, ay_g, az_g;
fusion.getGlobalAcceleration(ax_g, ay_g, az_g);
Calibration
To get accurate data, you must calibrate the sensors. The library includes tools to make this easy.

1. Gyroscope & Accelerometer
Use the example: TriSense_Calibration.ino

Upload the sketch.

Open Serial Monitor.

Send 'g' to auto-calibrate Gyro bias (keep sensor still).

Send 'a' to start the 6-point Accelerometer Calibration. Follow the on-screen instructions to rotate the sensor. It uses a "Sphere Fit" algorithm to calculate precise Offset and Scale.

Copy the generated code into your project's setup().

2. Magnetometer
Use the example: MotionCal_Bridge.ino

Upload the sketch.

Download the MotionCal tool (by Paul Stoffregen).

Run MotionCal and connect to the Serial port.

Rotate the sensor in all directions until the sphere is complete.

MotionCal will give you Hard Iron and Soft Iron values.

Copy these values into fusion.setMagHardIron(...) and fusion.setMagSoftIron(...).

Examples included
SimpleFusion: Basic fusion setup.

AdvancedFusion_Improved: Complete implementation with calibration placeholders and best practices.

AdvancedFusion_GlobalAccel: Demonstrates how to get Earth-frame acceleration.

TriSense_Calibration: Tool for calibrating Gyro and Accel.

MotionCal_Bridge: Tool for sending raw data to MotionCal application.

License
This library is released under the MIT License. Developed by VoltinoLabs (Tomas Michal).