/*
 * Example: CustomPins.ino (Voltino TriSense v1.3.0)
 *
 * Description:
 * Shows how to route the ICM-42688-P (SPI) and the AK09918C / BMP580 (I2C)
 * to non-default GPIO pins instead of the board's default SPI/I2C bus pins.
 * This is needed whenever the TriSense module is wired to arbitrary pins,
 * e.g. on a flight computer / custom PCB.
 *
 * Pin remapping is only possible on cores that support it:
 *   - RP2040 / RP2350 (Arduino-Pico core): SPI.setSCK()/setTX()/setRX() and
 *     Wire.setSDA()/setSCL() must be called BEFORE SPI.begin()/Wire.begin().
 *     TriSense::beginAll() calls SPI.begin()/Wire.begin() internally without
 *     pin arguments, so it automatically picks up the pins configured here.
 *   - ESP32: I2C pins can be pre-configured the same way via Wire.setPins().
 *     The SPIClass has no persistent "set pins then begin" API though, so the
 *     ICM-42688-P must be initialized directly via sensor.imu.begin(), passing
 *     the SCK/MISO/MOSI pins explicitly, instead of using beginAll().
 *   - AVR (Uno/Nano/Mega) and other classic cores: SPI/I2C pins are fixed in
 *     hardware and cannot be remapped, so this example falls back to the
 *     default wiring there.
 *
 * This example also demonstrates a few newer TriSenseFusion features that
 * are not covered by the other examples:
 *   - setMagCalibration()   -> apply hard/soft iron in a single call
 *   - setMountOrientation() -> remap axes for a sensor mounted sideways/upside down
 *   - setDynamicGyroBias()  -> continuous in-flight gyro drift learning
 *   - getGlobalLinearAcceleration() -> world-frame acceleration with gravity removed
 *   - getActualFusionHz()   -> measured update rate of the fusion loop
 *   - getMagHeadingDegrees() -> tilt-compensated magnetometer-only heading
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TriSense.h>

// ---------------------------------------------------------------------------
// EDIT THESE FOR YOUR WIRING
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  #define TRISENSE_SCK   2   // SPI0 SCK
  #define TRISENSE_MOSI  3   // SPI0 TX
  #define TRISENSE_MISO  4   // SPI0 RX
  #define TRISENSE_CS    5   // ICM-42688-P chip select
  #define TRISENSE_SDA   6   // I2C SDA (AK09918C + BMP580)
  #define TRISENSE_SCL   7   // I2C SCL (AK09918C + BMP580)
#elif defined(ESP32)
  #define TRISENSE_SCK   18
  #define TRISENSE_MOSI  23
  #define TRISENSE_MISO  19
  #define TRISENSE_CS    5
  #define TRISENSE_SDA   21
  #define TRISENSE_SCL   22
#else
  // AVR / other classic cores: pins are fixed in hardware, only the CS pin is configurable.
  #define TRISENSE_CS    17
#endif
// ---------------------------------------------------------------------------

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

unsigned long lastPrintTime = 0;

bool beginWithCustomPins() {
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  // Route I2C (AK09918C + BMP580) to the custom pins.
  Wire.setSDA(TRISENSE_SDA);
  Wire.setSCL(TRISENSE_SCL);

  // Route SPI0 (ICM-42688-P) to the custom pins.
  SPI.setSCK(TRISENSE_SCK);
  SPI.setTX(TRISENSE_MOSI);
  SPI.setRX(TRISENSE_MISO);

  // beginAll() internally calls Wire.begin()/SPI.begin() with no pin
  // arguments, so the pin mapping configured above is kept.
  return sensor.beginAll(MODE_HYBRID, TRISENSE_CS, 10000000);

#elif defined(ESP32)
  // Route I2C (AK09918C + BMP580) to the custom pins. Must happen before
  // Wire.begin(), which beginBMP()/beginMAG() call internally.
  Wire.setPins(TRISENSE_SDA, TRISENSE_SCL);

  // ESP32's SPIClass can't be pre-configured like RP2040's, so the custom
  // SCK/MISO/MOSI pins have to be passed directly into imu.begin() instead
  // of going through the beginAll() shortcut.
  bool ok = sensor.imu.begin(BUS_SPI, TRISENSE_CS, 10000000, ICM_ADDR_PRIMARY,
                              TRISENSE_SCK, TRISENSE_MISO, TRISENSE_MOSI);
  ok = sensor.beginBMP() && ok;
  ok = sensor.beginMAG() && ok;
  return ok;

#else
  // AVR and other classic cores: hardware SPI/I2C pins are fixed, no remapping possible.
  Serial.println("This core has fixed SPI/I2C pins - using default wiring.");
  return sensor.beginAll(MODE_HYBRID, TRISENSE_CS, 8000000);
#endif
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=========================================");
  Serial.println("Voltino TriSense - Custom SPI/I2C Pins Demo");
  Serial.println("=========================================");

  if (!beginWithCustomPins()) {
    Serial.println("ERROR: Sensor not found! Check the custom pin wiring above.");
    while (1) delay(100);
  }

  // 20-bit High-Res FIFO + 8kHz ODR (auto-downgraded on AVR / slower buses).
  sensor.imu.setFIFOMode(FIFO_20BIT_HIRES);
  sensor.imu.setODR(ODR_8KHZ);
  sensor.imu.flushFIFO();

  // --- Magnetometer calibration (from MotionCal) applied in one call ---
  float magHardIron[3] = {-80.40f, -6.88f, -23.86f};
  float magSoftIron[3][3] = {
    {1.004f, 0.001f, 0.005f},
    {0.001f, 1.002f, 0.027f},
    {0.005f, 0.027f, 0.994f}
  };
  fusion.setMagCalibration(magHardIron, magSoftIron);
  fusion.setDeclination(4.8f);              // Local magnetic declination
  fusion.setMagGaussian(57.18f, 6.0f, 15.0f); // Expected field strength, sigma, tilt sigma
  fusion.setMaxGains(0.1f, 0.15f);            // Max accel / mag correction gains

  // Adjust if the module is mounted sideways or upside down.
  fusion.setMountOrientation(ORIENTATION_Z_UP);

  // Continuously learn the gyro's resting drift while flying.
  fusion.setDynamicGyroBias(true, 0.0001f);

  Serial.println("Running static calibration - keep the module still!");
  fusion.initOrientation(1000);
  Serial.println("Ready.");
}

void loop() {
  if (fusion.update()) {
    unsigned long now = millis();
    if (now - lastPrintTime >= 20) { // 50Hz output
      lastPrintTime = now;

      float roll, pitch, yaw;
      fusion.getOrientationDegrees(roll, pitch, yaw);

      // Magnetometer-only heading (tilt-compensated, no gyro integration) -
      // handy to compare against the fused yaw above to catch magnetic
      // interference or a bad hard/soft-iron calibration.
      float yawMagOnly = fusion.getMagHeadingDegrees();

      // World-frame acceleration, gravity included / removed, in m/s^2.
      float gAx, gAy, gAz;
      fusion.getGlobalAcceleration(gAx, gAy, gAz, ACCEL_UNIT_MS2);

      float glAx, glAy, glAz;
      fusion.getGlobalLinearAcceleration(glAx, glAy, glAz, ACCEL_UNIT_MS2);

      Serial.print("Roll:"); Serial.print(roll, 1);
      Serial.print(" Pitch:"); Serial.print(pitch, 1);
      Serial.print(" Yaw:"); Serial.print(yaw, 1);
      Serial.print(" Yaw_MagOnly:"); Serial.print(yawMagOnly, 1);

      Serial.print(" | GlobalAcc(m/s^2) X:"); Serial.print(gAx, 3);
      Serial.print(" Y:"); Serial.print(gAy, 3);
      Serial.print(" Z:"); Serial.print(gAz, 3);

      Serial.print(" | LinearAcc(m/s^2) X:"); Serial.print(glAx, 3);
      Serial.print(" Y:"); Serial.print(glAy, 3);
      Serial.print(" Z:"); Serial.print(glAz, 3);

      Serial.print(" | FusionHz:"); Serial.println(fusion.getActualFusionHz(), 1);
    }
  }
}
