/*
 * Example: LowPower.ino
 *
 * Description:
 * Runs the TriSense module at reduced rates and puts all three sensors to
 * sleep between bursts of measurements:
 *
 *   - TriSenseConfig     -> magnetometer, barometer and IMU rates chosen at
 *                           beginAll() instead of the 100 Hz / 240 Hz / kHz
 *                           defaults
 *   - baroConfigValid()  -> confirms the barometer can actually deliver the
 *                           OSR/ODR pair that was asked for
 *   - sleep() / wake()   -> IMU off, magnetometer power-down, barometer
 *                           standby; wake() restores every previous setting
 *   - SPI pin release    -> on RP2040/RP2350 the SCK/MOSI pins are parked as
 *                           inputs between IMU transactions, which removes the
 *                           tens-of-mA fight with the TriSense Pro's level
 *                           shifter (see README, "SPI pin release")
 *
 * What remains while asleep is dominated by the module's ON LED.
 *
 * Wiring: hybrid mode - ICM-42688-P on SPI (CS below), AK09918C + BMP580 on
 * the default I2C bus.
 */

#include <TriSense.h>

#define TRISENSE_CS 17

// Leave both at -1 to let the driver find the SCK/MOSI pins on its own after
// SPI.begin(). Set them if you remapped SPI and want to be explicit.
#define TRISENSE_SCK  -1
#define TRISENSE_MOSI -1

const unsigned long AWAKE_MS = 5000;
const unsigned long ASLEEP_MS = 5000;

TriSense sensor;
unsigned long phaseStart = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Voltino TriSense - Low Power demo");

  TriSenseConfig cfg;               // Defaults = the classic beginAll() rates
  cfg.magOdr      = AK_ODR_10HZ;    // 100 Hz -> 10 Hz
  cfg.baroOsrP    = BMP580_OSR_x8;  // More oversampling per sample...
  cfg.baroOsrT    = BMP580_OSR_x1;
  cfg.baroOdr     = BMP580_ODR_10Hz; // ...but far fewer samples
  cfg.imuOdr      = ODR_100HZ;
  cfg.imuFifoMode = FIFO_16BIT;
  cfg.spiSckPin   = TRISENSE_SCK;
  cfg.spiMosiPin  = TRISENSE_MOSI;
  cfg.spiMode     = SPI_MODE3;      // SCK idles HIGH like the I2C SCL

  if (!sensor.beginAll(MODE_HYBRID, cfg, TRISENSE_CS, 4000000)) {
    Serial.println("Error: Sensors not found! Check wiring.");
    while (1) delay(100);
  }

  if (!sensor.baroConfigValid()) {
    Serial.println("WARNING: barometer cannot meet this OSR/ODR pair - lower one of them.");
  }

  Serial.print("SPI pin release: ");
  Serial.println(sensor.imu.isSpiPinReleaseActive() ? "active" : "off (not supported on this core / pins unknown)");

  phaseStart = millis();
}

void loop() {
  const unsigned long now = millis();

  if (sensor.isSleeping()) {
    if (now - phaseStart >= ASLEEP_MS) {
      Serial.println("--- wake ---");
      if (!sensor.wake()) Serial.println("Error: IMU did not answer after wake().");
      phaseStart = millis();
    }
    // A real application would put the MCU to sleep here as well.
    delay(10);
    return;
  }

  if (now - phaseStart >= AWAKE_MS) {
    Serial.println("--- sleep ---");
    sensor.sleep();
    phaseStart = millis();
    return;
  }

  TriSenseDataSnapshot data;
  if (sensor.getSnapshot(data) && now - lastPrint >= 500) {
    lastPrint = now;
    Serial.print("A [g]: ");
    Serial.print(data.accelX, 2); Serial.print(", ");
    Serial.print(data.accelY, 2); Serial.print(", ");
    Serial.print(data.accelZ, 2);
    Serial.print(" | M [uT]: ");
    Serial.print(data.magX, 1); Serial.print(", ");
    Serial.print(data.magY, 1); Serial.print(", ");
    Serial.print(data.magZ, 1);
    Serial.print(" | P: ");
    Serial.print(data.pressure); Serial.println(" Pa");
  }
}
