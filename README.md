TriSense Library
Description
The TriSense library is an Arduino library designed for the Voltino TriSense Pro sensor board. It integrates support for the onboard sensors: BMP580 (pressure and temperature), AK09918C (magnetometer), and ICM42688P (IMU with accelerometer and gyroscope).
This library allows for easy initialization of all sensors simultaneously in either full I2C mode or hybrid mode (AK09918C and BMP580 on I2C, ICM42688P on SPI). It preserves the ability to initialize and use sensors individually. Additionally, it provides sensor fusion capabilities for orientation estimation using quaternion-based methods, including a simple gyro integration fusion and an advanced complementary filter with dynamic gains.
Key features:

Unified sensor initialization
Support for I2C and SPI communication modes
Access to individual sensor functions from original libraries
Quaternion-based sensor fusion with customizable offsets (gyro, accel, mag hard/soft iron)
Options for custom Gaussian parameters in advanced fusion
Example sketches for basic usage and fusion

Installation

Download the latest release as a ZIP file from the GitHub repository.
Open the Arduino IDE.
Go to Sketch > Include Library > Add .ZIP Library... and select the downloaded ZIP file.
The library should now be available in Sketch > Include Library > TriSense.

Alternatively, clone the repository directly into your Arduino libraries folder.
Dependencies

Wire (for I2C)
SPI (for hybrid mode)

These are standard Arduino libraries and do not need separate installation.
Usage
Including the Library
C++#include <TriSense.h>
Initializing All Sensors
You can initialize all sensors at once using beginAll().
C++TriSense sensor;

void setup() {
  Serial.begin(115200);

  // Initialize in hybrid mode (IMU on SPI pin 17)
  if (!sensor.beginAll(MODE_HYBRID, 17)) {
    Serial.println("Failed to initialize sensors!");
    while (1);
  }

  // Now access sensors via sensor.bmp, sensor.mag, sensor.imu
}

MODE_I2C: All sensors on I2C.
MODE_HYBRID: AK09918C and BMP580 on I2C, ICM42688P on SPI.

Initializing Individual Sensors
You can also initialize sensors separately:
C++TriSense sensor;

void setup() {
  if (!sensor.beginBMP()) {
    // Error
  }
  if (!sensor.beginMAG()) {
    // Error
  }
  if (!sensor.beginIMU(BUS_SPI, 17)) {
    // Error
  }
}
Reading Sensor Data
Access individual sensors:

BMP580: sensor.bmp.readTemperature(), sensor.bmp.readPressure(), sensor.bmp.readAltitude(seaLevelPressure)
AK09918C: sensor.mag.readData(), then access sensor.mag.x, y, z; sensor.mag.getHeading()
ICM42688P: sensor.imu.readFIFO(ax, ay, az, gx, gy, gz)

Refer to the original sensor libraries for full functionality.
Sensor Fusion
The library includes two fusion classes:
SimpleTriFusion
Basic gyro integration with initial orientation from accel and mag.
C++SimpleTriFusion fusion(&sensor.imu, &sensor.mag);

// Set custom offsets (optional)
fusion.setGyroOffsets(-0.658, -0.31266, 0.25129);
fusion.setMagHardIron(-46.02, -0.85, -46.00);
float softIron[3][3] = { /* values */ };
fusion.setMagSoftIron(softIron);
fusion.setDeclination(5.0167);

// Initialize orientation
fusion.initOrientation();

// In loop:
if (fusion.update()) {
  float roll, pitch, yaw;
  fusion.getOrientationDegrees(roll, pitch, yaw);
  // Use roll, pitch, yaw
}
AdvancedTriFusion
Complementary filter with dynamic Gaussian gains, tilt-dependent mag correction, and yaw bias estimation.
C++AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

// Set offsets as above

// Custom parameters (optional)
fusion.setAccelGaussian(1.0, 0.0175);
fusion.setMagGaussian(50.88, 3.5);
fusion.setMagTiltSigma(15.0);
fusion.setYawKi(0.005);
fusion.setMaxGains(0.5, 0.5);

// Initialize and update similar to simple fusion
See the example sketches SimpleFusion.ino and AdvancedFusion.ino for complete usage.
Configuration

Sensor ODRs: Set using setODR() on individual sensors.
Fusion customization: Offsets, declination, Gaussian sigmas, etc., as shown above.

Examples

SimpleFusion.ino: Demonstrates basic sensor fusion.
AdvancedFusion.ino: Demonstrates advanced complementary filter fusion.

License
This library is released under the MIT License. See LICENSE file for details.