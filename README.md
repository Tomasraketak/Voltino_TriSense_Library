# Voltino TriSense Library

## Overview

**Voltino TriSense** is a high-performance Arduino library designed specifically for the **Voltino TriSense Pro** sensor board. It provides a unified interface for initializing and reading data from state-of-the-art environmental and motion sensors.

The library features **two Sensor Fusion engines** — `SimpleTriFusion` (lightweight, gyro-only) and `AdvancedTriFusion` (full AHRS with adaptive quaternion-based complementary filtering) — to provide stable, drift-free orientation (Roll, Pitch, Yaw) and Earth-frame Global Acceleration.

> **New in v1.3.0:** Tilt-compensated magnetometer-only heading (`getMagHeadingDegrees()`), full-batch accelerometer averaging for world-frame acceleration, corrected yaw gyro-bias learning, FIFO overflow detection, burst FIFO reads (~18× fewer bus transactions), and a custom SPI/I2C pin-mapping example.

---

## Hardware Support

The library supports the following sensors onboard the Voltino TriSense Pro:

| Sensor | Type | Bus | Key Specs |
|--------|------|-----|-----------|
| **ICM-42688-P** | 6-Axis IMU (Accel + Gyro) | SPI (up to 12 MHz) or I2C | ODR up to 32 kHz, 20-bit high-resolution FIFO |
| **AK09918C** | 3-Axis Magnetometer | I2C | ODR up to 100 Hz, 0.15 µT/LSB sensitivity |
| **BMP580** | Barometric Pressure + Temp | I2C | ODR up to 240 Hz, IIR filter, smart caching |

---

## Key Features

### 🚀 Hybrid Bus Architecture

To maximize performance, the library supports a **Hybrid Mode**:

- **SPI (up to 12 MHz)** — Used for the **ICM-42688-P** to achieve high sampling rates (up to 8 kHz on AVR, 32 kHz on ESP32/RP2040) and low latency.
- **I2C** — Used for the **AK09918C** and **BMP580** for standard communication.
- *A standard I2C-only mode is also available for compatibility.*

Platform-specific pin configuration is supported for **ESP32**, **RP2040**, and **RP2350** architectures.

### 🧠 Dual Sensor Fusion Engines

| Feature | `SimpleTriFusion` | `AdvancedTriFusion` |
|---------|-------------------|---------------------|
| Gyro Integration | ✅ | ✅ |
| Accelerometer Correction | ❌ | ✅ (adaptive Gaussian) |
| Magnetometer Correction | ❌ | ✅ (tilt-compensated) |
| Gyro Bias Learning (Yaw) | ❌ | ✅ (Ki controller) |
| Magnetic Anomaly Rejection | ❌ | ✅ |
| Computational Cost | Very Low | Moderate |
| Use Case | High-rate rotation tracking | Full AHRS orientation |

### ⚙️ Automatic Architecture Detection

The library auto-detects your platform and sets optimal defaults:

| Platform | Math Type | Default SPI ODR | Calibration Samples |
|----------|-----------|-----------------|---------------------|
| **AVR** (Uno, Mega, Nano) | `float` | 500 Hz | 200 |
| **ESP32** | `float` | 8 kHz | 1000 |
| **RP2040 / RP2350** | `float` | 8 kHz | 1000 |
| **Other** | `float` | 1 kHz | 500 |

`float` is the default everywhere so the hardware FPU is used; `double` is selected only when you define `FORCE_FUSION_DOUBLE`.

You can override precision manually by defining `FORCE_FUSION_FLOAT` or `FORCE_FUSION_DOUBLE` before including the library.

### 📦 TriSense Data Snapshot

The new `getSnapshot()` method returns all sensor data in a single synchronous call:

```cpp
TriSenseDataSnapshot data;
sensor.getSnapshot(data);
// data.accelX, data.gyroX, data.magX, data.pressure, data.temperature ...
```

### 🧮 Full-Batch Accelerometer Averaging

Each `update()` drains every packet waiting in the IMU FIFO. The gyroscope is integrated **per sample**, so no rotation is ever lost. The accelerometer is *accumulated* across the whole batch and `lastAx/lastAy/lastAz` is set to the **batch mean**, which is what `getGlobalAcceleration()` and `getGlobalLinearAcceleration()` then rotate into the world frame.

This matters because the FIFO delivers far more samples than a typical print/log loop consumes — at 8 kHz ODR polled at 50 Hz, roughly 160 samples arrive per iteration. Publishing only the newest packet would throw away ~99% of the data and hand you a single vibration-contaminated instant. Averaging keeps every sample, acts as a low-pass filter against airframe vibration, and gives `AdvancedTriFusion`'s gravity estimate a steadier input.

> **Caveat:** the mean is taken in the **body frame** and then rotated by the quaternion as it stands at the *end* of the batch. If the board rotates significantly during one batch, that single rotation is an approximation. Keep your `update()` call rate high (batches short) if you care about world-frame accuracy under high spin rates.

### 🛠️ Smart BMP580 Caching

The BMP580 driver uses an **adaptive cache** that intelligently re-reads the sensor only when the cached data has expired — based on the actual ODR frequency. This saves I²C bus bandwidth and reduces power consumption.

### 🎯 ICM-42688-P 20-Bit High-Resolution FIFO

Supports three FIFO modes:

- `FIFO_NONE` — Direct register reads (simple, minimal latency)
- `FIFO_16BIT` — 16-bit FIFO packet mode (balanced)
- `FIFO_20BIT_HIRES` — 20-bit high-resolution mode (16× precision, **automatically disabled on AVR** to save RAM)

### 📈 MCU-Clocked Adaptive Timebase

Integration timing is derived from the **MCU's own `micros()`**, never from the IMU's internal RC oscillator. On every `update()` the engine measures the wall-clock interval since the previous call, divides it by the number of FIFO packets actually drained, and uses that as the per-sample `dt`. The result is clamped to within ±2× of the nominal ODR period so a stalled loop or a bus hiccup can't inject a wild time-step.

This keeps quaternion integration correct under CPU load and bus contention without trusting the sensor's clock. `getActualFusionHz()` reports the measured update rate of the fusion loop.

---

## Installation

1.  Download the latest release `.zip` file from the GitHub repository.
2.  Open Arduino IDE.
3.  Go to **Sketch → Include Library → Add .ZIP Library...**
4.  Select the downloaded file.

**Dependencies:**
- `Wire.h`
- `SPI.h`

---

## Getting Started

### 1. Initialization

```cpp
#include <TriSense.h>

TriSense sensor;

void setup() {
  Serial.begin(115200);

  // --- Hybrid Mode (recommended) ---
  // ICM-42688-P on SPI (CS pin 17), BMP580 & AK09918C on I2C
  bool success = sensor.beginAll(MODE_HYBRID, 17, 10000000);

  // --- I2C-only Mode ---
  // bool success = sensor.beginAll(MODE_I2C);

  if (!success) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }
}
```

### 2. Reading Raw Data

Access individual sensor instances via `sensor.bmp`, `sensor.mag`, and `sensor.imu`.

```cpp
void loop() {
  // --- BMP580 (Pressure / Temp / Altitude) ---
  float temp  = sensor.bmp.readTemperature();
  float press = sensor.bmp.readPressure();
  float alt   = sensor.bmp.readAltitude(101325.0f); // sea level pressure in Pa

  // --- AK09918C (Magnetometer) ---
  if (sensor.mag.readData()) {
    Serial.print("Mag: ");
    Serial.print(sensor.mag.x, 2); Serial.print(", ");
    Serial.print(sensor.mag.y, 2); Serial.print(", ");
    Serial.println(sensor.mag.z, 2);
  }

  // --- ICM-42688-P (Accel/Gyro) ---
  float ax, ay, az, gx, gy, gz;
  if (sensor.imu.readFIFO(ax, ay, az, gx, gy, gz)) {
    Serial.print("Accel Z: "); Serial.println(az);
  }
}
```

### 3. Unified Snapshot

Read all sensors in one call using the data snapshot structure:

```cpp
void loop() {
  TriSenseDataSnapshot data;
  if (sensor.getSnapshot(data)) {
    Serial.print("A:"); Serial.print(data.accelX);
    Serial.print(" G:"); Serial.print(data.gyroX);
    Serial.print(" M:"); Serial.print(data.magX);
    Serial.print(" P:"); Serial.println(data.pressure);
  }
}
```

---

## Using Sensor Fusion

The library offers **two fusion backends**. Choose based on your needs.

### SimpleTriFusion — Lightweight Gyro-Only Integration

This engine integrates gyroscope data into a quaternion orientation. It does **not** use the accelerometer or magnetometer for correction, making it ideal for high-speed rotation tracking where absolute heading is not required.

```cpp
#include <TriSense.h>

TriSense sensor;
SimpleTriFusion fusion(&sensor.imu, &sensor.mag); // mag pointer required but unused

void setup() {
  sensor.beginAll(MODE_HYBRID);
  
  // Apply gyro offsets (from calibration)
  sensor.imu.setGyroOffset(-0.05, 0.02, 0.10);
  
  // Initialize orientation (averages initial samples)
  fusion.initOrientation();
}

void loop() {
  if (fusion.update()) {
    float roll, pitch, yaw;
    fusion.getOrientationDegrees(roll, pitch, yaw);
    Serial.print("R:"); Serial.print(roll);
    Serial.print(" P:"); Serial.print(pitch);
    Serial.print(" Y:"); Serial.println(yaw);
  }
}
```

### AdvancedTriFusion — Full AHRS with Adaptive Gains

This engine combines gyroscope integration with accelerometer and magnetometer corrections using an adaptive complementary filter. It provides drift-free Roll, Pitch, Yaw, and Earth-frame Global Acceleration.

```cpp
#include <TriSense.h>

TriSense sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

void setup() {
  sensor.beginAll(MODE_HYBRID);

  // --- Apply Calibration Data ---

  // 1. Accelerometer (Offset & Scale from 6-point calibration)
  sensor.imu.setAccelOffset(0.02, -0.01, 0.05);
  sensor.imu.setAccelScale(1.00, 1.00, 1.00);

  // 2. Gyro offsets
  sensor.imu.setGyroOffset(-0.05, 0.02, 0.10);

  // 3. Magnetometer (Hard Iron & Soft Iron from MotionCal)
  fusion.setMagHardIron(15.5, -40.2, 5.0);
  float softIron[3][3] = {{1.0, 0.0, 0.0},
                          {0.0, 1.0, 0.0},
                          {0.0, 0.0, 1.0}};
  fusion.setMagSoftIron(softIron);

  // 4. Magnetic Declination (e.g., 5.6° for Central Europe)
  fusion.setDeclination(5.6);

  // 5. Tune adaptive gains (optional)
  fusion.setAccelGaussian(1.0f, 0.05f);       // trust accel when |total| ≈ 1G
  fusion.setMagGaussian(50.88f, 3.5f, 15.0f); // trust mag when |field| ≈ 50.88 µT
  fusion.setYawKi(0.005f);                     // gyro bias learning speed

  // Initialize the orientation filter
  fusion.initOrientation();
}

void loop() {
  if (fusion.update()) {
    float roll, pitch, yaw;
    fusion.getOrientationDegrees(roll, pitch, yaw);

    // Earth-frame Global Acceleration (gravity removed)
    float ax_g, ay_g, az_g;
    fusion.getGlobalAcceleration(ax_g, ay_g, az_g);

    Serial.print("Roll: ");   Serial.print(roll);
    Serial.print(" Pitch: "); Serial.print(pitch);
    Serial.print(" Yaw: ");   Serial.println(yaw);
  }
}
```

---

## Deep Dive: How the Fusion Engines Work

### SimpleTriFusion — Pure Gyro Integration

1. **FIFO read loop:** Reads all available IMU samples from the FIFO buffer.
2. **Gyro integration:** Each sample is integrated into the quaternion at time-step `_realDt`.
3. **ODR tracking:** The engine measures the actual sample rate and adapts `_realDt` via a low-pass filter to compensate for CPU load variations.
4. **Result:** A fast, low-latency orientation that drifts slowly over time (gyro-only).

### AdvancedTriFusion — Adaptive Complementary Filter

This is a custom implementation of a quaternion-based complementary filter augmented with statistical probability (Gaussian functions).

**1. Gyroscope Integration (Prediction)**

The core orientation is updated by integrating angular rates into the quaternion:

$$\dot{q} = \frac{1}{2} q \otimes \omega$$

where $\omega$ is the gyroscope vector (rad/s). The gyro bias for the Z-axis (`gyroBiasZ`) is subtracted before integration if the Ki controller has learned a value.

**2. Accelerometer Correction (Roll/Pitch)**

The algorithm compares the *estimated* gravity vector (from the current quaternion) with the *measured* gravity vector. The error is computed as the cross-product:

$$e_{acc} = \hat{a}_{measured} \times \hat{a}_{estimated}$$

- **Adaptive Gaussian Gain:** If the total acceleration magnitude deviates from 1 G (e.g., during vibration, free-fall, or linear acceleration), a Gaussian function reduces the accelerometer influence to zero:

$$G_{acc} = \exp\left(-\frac{(||a|| - 1.0)^2}{2 \cdot \sigma_{acc}^2}\right) \cdot Gain_{max}$$

This prevents the horizon from tilting incorrectly during dynamic motion.

**3. Magnetometer Correction (Yaw)**

The engine aligns heading with magnetic North using tilt-compensated magnetic readings:

- **Tilt Compensation:** Raw magnetometer data is rotated by the current roll/pitch to produce horizontal components, from which the corrected yaw is derived.
- **Magnetic Anomaly Rejection:** If the measured magnetic field strength differs significantly from the calibrated reference (50.88 µT), the magnetometer is ignored via a second Gaussian function.
- **Tilt-Dependent Scaling:** The magnetometer gain is further scaled by a Gaussian centered at 0° tilt, preventing gimbal-lock artifacts when the board is steeply pitched or rolled:

$$G_{mag} = G_{mag\_raw} \cdot G_{tilt\_roll} \cdot G_{tilt\_pitch}$$

**4. Gyro Bias Learning (Ki Controller)**

The error between the gyro-predicted yaw and the magnetometer-corrected yaw is tracked:

$$\Delta_{yaw} = yaw_{corr} - yaw_{estimate}$$

This error drives an integral controller (`Ki`) which adjusts `gyroBiasZ`. All three `gyroBias[]` components are held in **deg/s**, matching the units of the gyro samples they are subtracted from, so the radian error is converted on the way in:

$$bias_z \mathrel{-}= K_i \cdot \Delta_{yaw} \cdot G_{mag} \cdot dt \cdot \frac{180}{\pi}$$

Wind-up is prevented by **bounding** the accumulated bias to ±`setMaxGyroBias()` (default 5 dps). It is deliberately *not* reset on sign change: near convergence the yaw error oscillates about zero, so zeroing on every crossing would wipe the accumulator continuously and the controller could never settle on the steady-state bias it exists to find. Bounded integration lets it converge, giving a stable heading even when the magnetometer is temporarily unavailable.

**5. MCU-Clocked Timebase**

Both engines take their time-step from the MCU's `micros()`, never the IMU's internal RC oscillator. Per `update()`, the elapsed wall-clock interval is divided by the number of packets drained from the FIFO to get the per-sample `dt`, then clamped to within ±2× of the nominal ODR period so a stalled loop cannot inject a wild time-step.

---

## Calibration Guide

To achieve professional-grade results, calibrate the sensors.

### 1. Gyroscope Calibration

Use `autoCalibrateGyro()`:

```cpp
void setup() {
  sensor.beginAll(MODE_HYBRID);
  sensor.autoCalibrateGyro(750); // 750 samples (default varies by platform)
  // Offsets are automatically applied to sensor.imu
}
```

Or manually: Keep the board perfectly still. The code averages gyro samples to find zero-rate offsets and stores them as `gyrOffset[3]`.

### 2. Accelerometer — 6-Point Sphere Fit

Use `autoCalibrateAccel()`:

```cpp
void setup() {
  sensor.beginAll(MODE_HYBRID);
  sensor.autoCalibrateAccel();
}
```

The guided calibration asks you to hold the sensor in **6 orientations** (Z+ down, Z- up, Y+ left, Y- right, X+ forward, X- backward). It uses a **gradient descent sphere-fit** algorithm (2000 iterations) to compute:

- **Offset** (bias) — centers the sphere at origin
- **Scale** (sensitivity) — maps the ellipsoid to a unit sphere

The results are printed to Serial — copy them into your `setup()`:

```
--- COPY TO SETUP() ---
IMU.setAccelOffset(0.02345, -0.01123, 0.05167);
IMU.setAccelScale(1.00123, 0.99876, 1.00234);
-----------------------
```

### 3. Magnetometer — MotionCal Tool

Use the `MotionCal_Bridge.ino` example:

1. Upload the sketch to your board.
2. Download **MotionCal** (by Paul Stoffregen).
3. Connect MotionCal to the board's serial port.
4. Rotate the board in figure-8 motions in all directions.
5. MotionCal visualizes the magnetic field sphere and generates **Hard Iron** (offsets) and **Soft Iron** (matrix) values.
6. Copy these into your `setup()` using `setMagHardIron()` and `setMagSoftIron()`.

---

## ICM-42688-P FIFO Modes

The IMU driver supports three reading modes:

| Mode | Set Via | Precision | RAM Usage | Best For |
|------|---------|-----------|-----------|----------|
| **Direct Register** | `setFIFOMode(FIFO_NONE)` | 16-bit | Minimal | Low-rate polling |
| **16-bit FIFO** | `setFIFOMode(FIFO_16BIT)` | 16-bit | Low | General use (default) |
| **20-bit High-Res** | `setFIFOMode(FIFO_20BIT_HIRES)` | 20-bit (16×) | Moderate | Precision analysis |

> **Note:** High-res mode is **automatically disabled on AVR** (Arduino Uno, Nano, Mega) to conserve RAM. It forces the accelerometer to ±16 G and gyroscope to ±2000 dps ranges.

In 20-bit mode the packet's 20-bit field is the 16-bit sample shifted left by 4 with the extension nibble appended below it, so the effective sensitivity is 16× the 16-bit value: **32768 LSB/g** and **262.4 LSB/dps**.

### 🚚 Burst FIFO Reads

`readFIFO()` still returns one packet per call, but bytes are fetched from the sensor a **whole burst at a time**: a single `FIFO_COUNT` read plus one bulk `FIFO_DATA` read serve up to 32 packets (8 on AVR), which are then parsed from RAM with no further bus traffic. Draining a full 128-packet FIFO costs **~14 bus transactions instead of 256**.

Faster draining directly reduces the chance of overflowing the FIFO in the first place. On I2C the bulk read is automatically split into Wire-buffer-sized chunks; the FIFO read pointer advances per byte, so packets still stream consecutively.

### 🧭 Magnetometer Calibration Order

Hard iron is a fixed offset in the magnetometer's **own physical axes**, and soft iron a linear distortion in those same axes — both are properties of the sensor and whatever magnetic material sits next to it. MotionCal therefore fits them in raw sensor axes, and the library removes them **before** `setMountOrientation()`'s axis remapping. Only the corrected, physically-real field vector is rotated into the mount frame.

The order is not interchangeable. Remapping first applies each hard-iron offset to the wrong axis for every orientation except `ORIENTATION_Z_UP` (where the remap is the identity), leaving a residual constant offset in the horizontal plane. That off-centres the heading locus, so heading sensitivity varies with direction — a given rotation reads too large in one sector and too small in the opposite one. If the residual offset exceeds the horizontal field strength (~20 µT in central Europe), the locus stops enclosing the origin and heading cannot sweep a full 360° at all.

To sanity-check a calibration, print the corrected field magnitude while rotating the board:

```cpp
float m = sqrt(fusion.lastMx*fusion.lastMx +
               fusion.lastMy*fusion.lastMy +
               fusion.lastMz*fusion.lastMz);
```

It should stay near-constant in every orientation, at your location's total field strength (~48–49 µT in Prague). A magnitude that swings with attitude means the calibration has not converged.

### 🚨 FIFO Overflow Detection

The hardware FIFO is 2 KB — **128 packets** at 16 bytes, or **102** at 20 bytes. At 8 kHz ODR it fills from empty in about **16 ms**. If your loop is slower than that, the sensor starts discarding samples, and a dropped packet is rotation that can never be integrated: a permanent, silently-accumulating attitude error.

Worse, it hides. The per-sample `dt` is computed as *elapsed time ÷ packets actually read*, so if 160 samples were produced but only 128 were read, the timing still looks perfectly consistent while a quarter of the motion has vanished.

Two independent detectors run on every buffer refill, and either one raises the flag:

1. The hardware's latched `FIFO_FULL` bit in `INT_STATUS`. `setFIFOMode()` enables `FIFO_FULL_INT1_EN` in `INT_SOURCE0` so the bit actually latches — the status bit does **not** latch for a disabled source, so without that write this detector reports nothing. (It also makes the INT1 pin pulse on FIFO full, harmless when the pin is unused.)
2. A config-free backstop: a FIFO count within one packet of the 2 KB capacity means the buffer is saturated. This needs no interrupt configuration and keeps working even if a sketch overwrites `INT_SOURCE0`.

Because detector 2 fires at saturation, it can flag one sample before the first packet is actually lost — treat the warning as "the FIFO hit its limit", which is the actionable condition either way.

**The library never prints anything** — poll it from your sketch:

```cpp
if (sensor.imu.fifoOverflowed()) {                  // self-clearing since last call
  Serial.print("FIFO overflow! total=");
  Serial.println(sensor.imu.getFIFOOverflowCount());
}
```

The usual cause is a blocking `Serial.print` at a low baud rate. If you see overflows, raise the baud rate (921600), log binary, or lower the ODR.

> `INT_STATUS` is read-to-clear, and the driver reads it once per FIFO refill. If your sketch also drives the INT pins and inspects `INT_STATUS` itself, the driver will have consumed the flags first.

---

## API Reference

### `TriSense` Class

| Method | Description |
|--------|-------------|
| `beginAll(mode, csPin, freq)` | Initialize all hardware (Hybrid or I2C-only) |
| `beginBMP(addr)` | Initialize BMP580 individually |
| `beginMAG()` | Initialize AK09918C individually |
| `beginIMU(busType, csPin, freq)` | Initialize ICM-42688-P individually |
| `resetHardwareOffsets()` | Clear internal IMU offset registers |
| `autoCalibrateGyro(samples)` | Compute and apply gyro software offsets |
| `autoCalibrateAccel()` | Guided 6-point accelerometer calibration |
| `getSnapshot(data)` | Read all sensors into a `TriSenseDataSnapshot` struct |
| `readPressure()` | Get pressure in Pa (smart-cached) |
| `readTemperature()` | Get temperature in °C (smart-cached) |
| `readAltitude(seaLevelPa)` | Get altitude in meters. Reference pressure is in **Pascals** (default `101325.0`) |

### `ICM42688P` Class

| Method | Description |
|--------|-------------|
| `begin(busType, csPin, freq, i2cAddr, sck, miso, mosi)` | Full initialization with pin mapping |
| `beginI2C(freq, addr, sda, scl)` | I2C-specific init |
| `beginSPI(csPin, freq, sck, miso, mosi)` | SPI-specific init |
| `setODR(odr)` | Set output data rate |
| `setAccelFS(fs)` | Set accelerometer full-scale range |
| `setGyroFS(fs)` | Set gyroscope full-scale range |
| `setFIFOMode(mode)` | Select FIFO reading mode |
| `flushFIFO()` | Discard all buffered samples (hardware and software) |
| `fifoOverflowed()` | True if the FIFO overflowed since the last call (self-clearing) |
| `getFIFOOverflowCount()` | Total overflow events since boot |
| `resetFIFOOverflowCount()` | Clear the overflow counter and flag |
| `readIMU(ax, ay, az, gx, gy, gz)` | Read based on current FIFO mode |
| `readFIFO(ax, ay, az, gx, gy, gz)` | Alias for `readIMU()` |
| `readTemperature()` | Read internal IMU temperature |
| `setAccelOffset(x, y, z)` | Set software accelerometer offsets |
| `setAccelScale(x, y, z)` | Set software accelerometer scale factors |
| `setGyroOffset(x, y, z)` | Set software gyroscope offsets |
| `getODRHz()` | Get current ODR in Hz |
| `autoCalibrateGyro(samples)` | Auto-calibrate gyroscope bias |
| `autoCalibrateAccel()` | 6-point sphere fit calibration |

### `BMP580` Class

| Method | Description |
|--------|-------------|
| `begin(addr)` | Initialize with I2C address (auto-failover to secondary) |
| `setI2CSpeed(speed)` | Set I2C clock speed (100k, 400k, or 1 MHz) |
| `setOversampling(osr_p, osr_t)` | Set pressure and temp oversampling |
| `setODR(odr)` | Set output data rate (0.125 Hz to 240 Hz) |
| `setPowerMode(mode)` | Set power mode (Standby, Normal, Forced, Continuous) |
| `setIIRFilter(iir_p, iir_t)` | Set IIR filter coefficients for pressure/temp |
| `readTemperature()` | Get temperature (°C) — smart-cached |
| `readPressure()` | Get pressure (Pa) — smart-cached |
| `readAltitude(seaLevel)` | Get altitude (m) using cached pressure |

### `AK09918C` Class

| Method | Description |
|--------|-------------|
| `begin(odr, wire)` | Initialize with I2C bus reference |
| `beginI2C(freq, sda, scl, odr)` | Standalone init with optional pin config |
| `readData()` | Read magnetic data (early-exit on ST1) |
| `setODR(odr)` | Set output data rate (10, 20, 50, or 100 Hz) |
| `setMode(mode)` | Set operation mode (Power-down, Single, Self-test) |
| `softReset()` | Hardware reset |
| `computeHeading(mx, my, mz, rollDeg, pitchDeg, declinationDeg)` *(static)* | Tilt-compensated heading (0–360°) from calibrated field components + roll/pitch |

**Public members:**
- `float x, y, z` — Field strength in µT
- `int16_t x_raw, y_raw, z_raw` — Raw ADC values
- `bool overflow` — Magnetic saturation flag

### `TriSenseFusion` Base Class

| Method | Description |
|--------|-------------|
| `initOrientation(samples)` | Initialize quaternion from initial accel/mag readings |
| `calibrateAccelStatic(samples)` | Simple static gravity offset calibration |
| `getOrientationDegrees(roll, pitch, yaw)` | Get orientation in degrees (0–360° yaw) |
| `getMagHeadingDegrees()` | Tilt-compensated, magnetometer-only heading (0–360°), independent of the fused/gyro yaw |
| `getGlobalAcceleration(x, y, z, unit)` | World-frame acceleration, **gravity included** (batch-averaged, see above) |
| `getGlobalLinearAcceleration(x, y, z, unit)` | Same as above with gravity removed — the quantity to integrate for dead reckoning |
| `getLinearAcceleration(x, y, z, unit)` | Body-frame acceleration with gravity removed |
| `getActualFusionHz()` | Measured update rate of the fusion loop |
| `setLocalGravity(g)` | Set local gravity in m/s² (default `9.80665`; use your latitude's real value) |
| `setDynamicGyroBias(enable, ki)` | Enable in-flight gyro drift learning on X/Y |
| `setMaxGyroBias(maxDps)` | Anti-windup bound on the learned gyro bias (default `5.0` dps) |
| `setAccelGaussian(ref, sigma)` | Tune accelerometer trust function |
| `setMagGaussian(ref, sigma)` | Tune magnetometer trust function |
| `setMagGaussian(ref, sigma, tiltSigma)` | Tune with explicit tilt sigma |
| `setMagTiltSigma(sigmaDeg)` | Set tilt-dependent magnetometer gain sigma |
| `setDeclination(deg)` | Set magnetic declination for true north |
| `setMagHardIron(x, y, z)` | Set magnetometer hard-iron offsets |
| `setMagSoftIron(matrix[3][3])` | Set magnetometer soft-iron matrix |
| `setYawKi(ki)` | Set yaw bias learning rate |
| `setMaxGains(accelGain, magGain)` | Set maximum correction gains |
| `setMagCheckInterval(ms)` | Set how often to poll the magnetometer |

### `SimpleTriFusion` — `update()`

Processes all available IMU FIFO data, integrates gyroscope into quaternion, and updates the ODR tracker. Returns `true` if at least one new sample was processed.

### `AdvancedTriFusion` — `update()`

Processes all available IMU FIFO data, performs gyro integration with bias subtraction, periodically reads the magnetometer, and applies the complementary correction (accelerometer + magnetometer) with adaptive Gaussian gains. Returns `true` if at least one new IMU sample was processed.

---

## Configuration Macros

Define these *before* including `TriSense.h` to override defaults:

| Macro | Effect |
|-------|--------|
| `FORCE_FUSION_FLOAT` | Force `FUSION_MATH_TYPE` to `float` |
| `FORCE_FUSION_DOUBLE` | Force `FUSION_MATH_TYPE` to `double` |

---

## License

MIT License. Developed by VoltinoLabs.