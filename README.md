```markdown
# Voltino TriSense Library

## Overview
**Voltino TriSense** is a high-performance Arduino library designed specifically for the **Voltino TriSense Pro** sensor board. It provides a unified interface for initializing and reading data from state-of-the-art environmental and motion sensors.

Beyond simple data reading, this library features a robust **Advanced Sensor Fusion** engine. It combines high-speed IMU data with Magnetometer readings using an adaptive quaternion-based complementary filter to provide stable, drift-free orientation (Roll, Pitch, Yaw) and Earth-frame Global Acceleration.

## Hardware Support
The library supports the following sensors onboard the Voltino TriSense Pro:
* **ICM-42688-P:** High-precision 6-Axis MEMS MotionTracking™ (Accelerometer + Gyroscope).
* **AK09918C:** High-sensitivity 3-Axis Magnetometer with compass capabilities.
* **BMP580:** High-performance barometric pressure and temperature sensor.

## Key Features

### 🚀 Hybrid Bus Architecture
To maximize performance, the library supports a **Hybrid Mode**.
* **SPI (up to 10 MHz):** Used for the **ICM-42688-P** to achieve high sampling rates (up to 8kHz/32kHz) and low latency.
* **I2C:** Used for the **AK09918C** and **BMP580** for standard communication.
* *Note:* A standard I2C-only mode is also available for compatibility.

### 🧠 Advanced Sensor Fusion (AHRS)
The `AdvancedTriFusion` class implements a sophisticated sensor fusion algorithm:
* **Adaptive Gaussian Gains:** Dynamically adjusts trust in the Accelerometer and Magnetometer based on motion intensity and magnetic anomalies.
* **Gyro Bias Learning:** Automatically estimates and corrects gyroscope drift in the Yaw axis using an integral (Ki) controller.
* **Tilt Compensation:** Ensures accurate compass heading even when the device is tilted steeply.
* **Global Acceleration:** Calculates linear acceleration vectors relative to the Earth, removing gravity and sensor tilt.

### 🛠️ Calibration Tools
* **Sphere Fit:** Built-in algorithm for 6-point Accelerometer calibration (Offset & Scale).
* **MotionCal Support:** Compatible with the visual MotionCal tool for Hard/Soft Iron Magnetometer calibration.

---

## Installation

1.  Download the latest release `.zip` file from the GitHub repository.
2.  Open Arduino IDE.
3.  Go to **Sketch > Include Library > Add .ZIP Library...**
4.  Select the downloaded file.

**Dependencies:**
* `Wire.h`
* `SPI.h`

---

## Getting Started

### 1. Initialization
You can initialize all sensors with a single call.

```cpp
#include <TriSense.h>

TriSense sensor;

void setup() {
  Serial.begin(115200);

  // Initialize in Hybrid Mode (Recommended for performance)
  // - ICM42688P on SPI (CS Pin 17, 10 MHz)
  // - BMP580 & AK09918C on I2C
  bool success = sensor.beginAll(MODE_HYBRID, 17, 10000000);

  if (!success) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }

  // Set IMU Output Data Rate (ODR)
  sensor.imu.setODR(ODR_1KHZ); 
}

```

### 2. Reading Raw Data

Access individual sensor instances directly via `sensor.bmp`, `sensor.mag`, and `sensor.imu`.

```cpp
void loop() {
  // --- BMP580 (Pressure/Temp) ---
  float temp = sensor.bmp.readTemperature();
  float press = sensor.bmp.readPressure();
  float alt = sensor.bmp.readAltitude(101325); // 101325 Pa as sea level standard

  // --- AK09918C (Magnetometer) ---
  if (sensor.mag.readData()) {
      float mx = sensor.mag.x;
      float my = sensor.mag.y;
      float mz = sensor.mag.z;
  }

  // --- ICM-42688-P (Accel/Gyro) ---
  float ax, ay, az, gx, gy, gz;
  // readFIFO returns true if new data is available
  if (sensor.imu.readFIFO(ax, ay, az, gx, gy, gz)) {
      Serial.print("Accel Z: "); Serial.println(az);
  }
}

```

---

## Using Sensor Fusion

To get stable orientation (Roll, Pitch, Yaw), use the `AdvancedTriFusion` class.

### 1. Setup and Calibration

Accurate fusion requires calibration constants.

```cpp
// Pass pointers to the sensor objects
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

void setup() {
  sensor.beginAll(MODE_HYBRID);

  // --- Apply Calibration Data ---
  // (Obtain these values using the calibration examples provided)
  
  // 1. Accelerometer (Offset & Scale)
  sensor.imu.setAccelOffset(0.02, -0.01, 0.05); 
  sensor.imu.setAccelScale(1.00, 1.00, 1.00);

  // 2. Magnetometer (Hard Iron & Soft Iron)
  fusion.setMagHardIron(15.5, -40.2, 5.0);
  float softIron[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  fusion.setMagSoftIron(softIron);

  // 3. Magnetic Declination (Important for True North)
  // Look up your location (e.g., 5.6 degrees for Central Europe)
  fusion.setDeclination(5.6);

  // Initialize the orientation filter (waits for stable data)
  fusion.initOrientation();
}

```

### 2. The Loop

Call `update()` as fast as possible in the loop.

```cpp
void loop() {
  if (fusion.update()) {
    float roll, pitch, yaw;
    fusion.getOrientationDegrees(roll, pitch, yaw);

    // Get Global Acceleration (Earth Frame)
    float ax_g, ay_g, az_g;
    fusion.getGlobalAcceleration(ax_g, ay_g, az_g);

    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" Pitch: "); Serial.print(pitch);
    Serial.print(" Yaw: "); Serial.println(yaw);
  }
}

```

---

## Deep Dive: How AdvancedFusion Works

The `AdvancedTriFusion` algorithm is a custom implementation of a Complementary Filter augmented with statistical probability (Gaussian functions). Here is the logic flow:

1. **Gyroscope Integration (Prediction):** The core orientation is calculated by integrating the gyroscope angular rates. This provides extremely fast response to movement but suffers from drift over time.
2. **Accelerometer Correction (Roll/Pitch):** The algorithm measures the gravity vector. It calculates the error between the *estimated* gravity (from the current quaternion) and the *measured* gravity.
* **Adaptive Gain:** If the total acceleration vector length is not close to 1G (e.g., during vibration or freefall), the algorithm assumes external forces are present. It uses a **Gaussian function** to reduce the Accelerometer's influence (`Gain -> 0`), preventing the horizon from tilting incorrectly during movement.


3. **Magnetometer Correction (Yaw):**
The algorithm aligns the Heading (Yaw) with magnetic North.
* **Magnetic Anomaly Rejection:** Similar to the accelerometer, if the measured magnetic field strength differs significantly from the calibrated reference (`magRef`), the fusion engine ignores the magnetometer to prevent glitches near motors or iron objects.
* **Tilt Compensation:** The magnetometer gain is dynamically scaled based on the current Pitch/Roll angles. This prevents the "gimbal lock" effect where pitch changes could mistakenly alter Yaw.


4. **Gyro Bias Learning (Ki):**
The system tracks the difference between the Gyro-predicted Yaw and the Magnetometer-corrected Yaw. This error is fed into an integral controller (`Ki`) which slowly adjusts a `gyroBiasZ` variable. This effectively "learns" the gyroscope's resting drift and subtracts it, resulting in a stable heading even if the magnetometer is temporarily unavailable.

---

## Calibration Guide

To achieve professional-grade results, you must calibrate the sensors.

### 1. Gyroscope & Accelerometer

Use the example sketch `TriSense_Calibration.ino`.

* **Gyro:** Keep the board perfectly still and press 'g'. The code will average samples to find the zero-rate offset.
* **Accel:** Press 'a' to start the guided calibration. You will be asked to hold the sensor in 6 orientations (Z+, Z-, Y+, Y-, X+, X-). The library calculates the Offset and Scale Matrix to map the sensor data to a perfect unit sphere.

### 2. Magnetometer

Use the example sketch `MotionCal_Bridge.ino`.

1. Upload the sketch to your board.
2. Download the **MotionCal** software (by Paul Stoffregen).
3. Connect MotionCal to the board's serial port.
4. Rotate the board in figure-8 motions and all directions.
5. MotionCal will visualize the magnetic field sphere and generate **Hard Iron** (offsets) and **Soft Iron** (matrix) values.
6. Copy these values into your `setup()` code.

---

## Library Reference

### `TriSense` Class

* `beginAll(mode, csPin, freq)`: Initialize all hardware.
* `resetHardwareOffsets()`: Clears internal IMU registers.
* `autoCalibrateGyro(samples)`: Calculates and applies gyro offsets internally in the IMU.

### `AdvancedTriFusion` Class

* `setAccelGaussian(ref, sigma)`: Tune how strictly the filter rejects external acceleration (Default ref=1.0, sigma=0.02).
* `setMagGaussian(ref, sigma)`: Tune magnetic interference rejection.
* `setYawKi(ki)`: Adjust the speed of gyro bias learning (Higher = faster correction, Lower = smoother).
* `getGlobalAcceleration(x, y, z)`: Returns linear acceleration in Gs, with gravity removed, rotated to the Earth frame.

## License

MIT License. Developed by VoltinoLabs.

```

```