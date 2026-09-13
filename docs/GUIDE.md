# Voltino TriSense — Practical Guide

A working guide to the TriSense module and this library: how to wire it, how to
choose a configuration, how to calibrate it, and how to read the numbers that
come out. For the exhaustive method-by-method listing, see the
[API Reference in the README](../README.md#api-reference).

---

## Contents

1. [What is on the board](#1-what-is-on-the-board)
2. [Wiring and bus modes](#2-wiring-and-bus-modes)
3. [First sketch](#3-first-sketch)
4. [Choosing ODR and FIFO mode](#4-choosing-odr-and-fifo-mode)
5. [Writing a loop that keeps up](#5-writing-a-loop-that-keeps-up)
6. [Calibration, in order](#6-calibration-in-order)
7. [Reading the outputs](#7-reading-the-outputs)
8. [Choosing a fusion engine](#8-choosing-a-fusion-engine)
9. [Tuning the AHRS](#9-tuning-the-ahrs)
10. [Troubleshooting](#10-troubleshooting)
11. [Platform notes](#11-platform-notes)

---

## 1. What is on the board

| Sensor | Measures | Native rate | Bus |
|---|---|---|---|
| **ICM-42688-P** | 3-axis accelerometer + 3-axis gyroscope | up to 32 kHz | I2C or SPI |
| **AK09918C** | 3-axis magnetometer | 100 Hz | I2C only |
| **BMP580** | barometric pressure + temperature | up to 240 Hz | I2C only |

Two things follow from that table and shape everything else:

- **The magnetometer is the slow one.** At 100 Hz it produces a new sample every
  10 ms. Any loop faster than that will mostly find no new magnetometer data,
  which is normal and not an error.
- **Only the IMU can use SPI.** The other two are I2C-only, which is why the
  library's fastest configuration is *hybrid*: IMU on SPI, the rest on I2C.

### Axis convention

All three sensors are read in the **sensor's own axes**. If the module is not
mounted flat, tell the fusion engine with `setMountOrientation()` — do not try to
compensate by swapping calibration values, which will not work (see
[§6](#6-calibration-in-order)).

The fused output uses a Z-up, right-handed frame: **roll** about X, **pitch**
about Y, **yaw** about Z, with yaw reported 0–360° clockwise from magnetic north
plus your declination.

---

## 2. Wiring and bus modes

### Mode selection

```cpp
sensor.beginAll(MODE_I2C);                        // everything on I2C
sensor.beginAll(MODE_HYBRID, csPin, spiHz);       // IMU on SPI, mag+baro on I2C
```

| Mode | IMU bus | Practical IMU ceiling | Use when |
|---|---|---|---|
| `MODE_I2C` | I2C @ 400 kHz | ~1 kHz | simple wiring, low rate is fine |
| `MODE_HYBRID` | SPI @ up to 10 MHz | 8 kHz+ | you want high-rate gyro integration |

`MODE_I2C` needs four wires; `MODE_HYBRID` needs seven. There is no mode that
puts the magnetometer or barometer on SPI, because those parts do not have it.

### Choosing the I2C bus

Every I2C device on the module shares one bus, so it is selected once:

```cpp
sensor.beginAll(MODE_HYBRID, 5, 10000000, Wire1);   // module wired to Wire1
```

### Remapping pins

On cores that support it, configure the pins **before** `beginAll()`, which calls
`begin()` without pin arguments and so inherits whatever you set:

```cpp
// RP2040 / RP2350
SPI.setSCK(2); SPI.setTX(3); SPI.setRX(4);
Wire.setSDA(16); Wire.setSCL(17);
sensor.beginAll(MODE_HYBRID, 5, 10000000);
```

ESP32 can pre-set I2C pins with `Wire.setPins()`, but its `SPIClass` has no
persistent equivalent — there, initialise the IMU directly with
`sensor.imu.begin()` and pass the SPI pins explicitly. AVR pins are fixed in
hardware. See `examples/CustomPins`.

---

## 3. First sketch

```cpp
#include <TriSense.h>

TriSense sensor;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  if (!sensor.beginAll(MODE_I2C)) {
    Serial.println("TriSense not found - check wiring and addresses");
    while (1) { }
  }
}

void loop() {
  TriSenseDataSnapshot d;
  sensor.getSnapshot(d);

  Serial.print("accel g  "); Serial.print(d.accelX, 3); Serial.print(' ');
  Serial.print(d.accelY, 3); Serial.print(' '); Serial.println(d.accelZ, 3);

  delay(100);
}
```

### What `getSnapshot()` guarantees

It fills **every** field with the newest value available, refreshing whatever the
hardware has ready and reusing the last good value for the rest. It does *not*
fail merely because one sensor had nothing new.

```cpp
if (d.magFresh) { /* this magnetometer reading is new */ }
if (d.baroAgeUs > 500000) { /* barometer has not updated in 0.5 s */ }
```

- `imuFresh` / `magFresh` / `baroFresh` — was this block updated by this call?
- `imuAgeUs` / `magAgeUs` / `baroAgeUs` — microseconds since it last was.
  `0xFFFFFFFF` means *never read*.
- The **return value** means "every block has produced at least one valid reading",
  i.e. the snapshot is fully meaningful. Check it once after startup, not every loop.

---

## 4. Choosing ODR and FIFO mode

This is the decision that causes the most trouble, so it gets its own section.

### FIFO modes

| Mode | Packet | Resolution | Notes |
|---|---|---|---|
| `FIFO_NONE` | 13 B | 16-bit | direct register reads, no buffering |
| `FIFO_16BIT` | 16 B | 16-bit | hardware FIFO, selectable full-scale |
| `FIFO_20BIT_HIRES` | 20 B | 20-bit | hardware FIFO, **fixed at ±16 g / ±2000 dps** |

`FIFO_20BIT_HIRES` gives 16× the resolution of the 16-bit modes. Because the
full scale is locked wide, you get that extra resolution *and* keep the full
range — there is no trade-off to make. It is the recommended mode wherever the
RAM and bus allow.

Without a FIFO the loop must poll faster than the ODR or samples are simply
missed; with one, the sensor buffers 2 KB for you and the library drains it.

### The hardware FIFO is 2048 bytes

That is the number everything else follows from:

| FIFO mode | Packets that fit | @ 2 kHz | @ 4 kHz | @ 8 kHz |
|---|---|---|---|---|
| `FIFO_16BIT` (16 B) | 128 | 64 ms | 32 ms | 16 ms |
| `FIFO_20BIT_HIRES` (20 B) | 102 | 51 ms | 25 ms | **12.8 ms** |

**Read that last row as a deadline.** At 8 kHz in HiRes mode, if your loop ever
goes 12.8 ms without calling `update()`, the FIFO fills and the sensor starts
discarding samples. This — not bus bandwidth — is what actually limits the
usable ODR in practice.

### A realistic ODR budget

| ODR | What it costs you | Sensible when |
|---|---|---|
| 100–200 Hz | nothing | general orientation, robotics, IoT |
| 1 kHz | comfortable on any 32-bit MCU | drones, stabilisation, most real work |
| 2 kHz | 51 ms of slack in HiRes | fast dynamics, needs a tidy loop |
| 4 kHz | 25 ms of slack | you have measured your loop |
| 8 kHz | 12.8 ms of slack; no room for a blocking `Serial.print` | high-vibration analysis, RP2350 class only |
| 16 kHz | 6.4 ms of slack; works, but the bus is now a real factor | short high-rate captures |
| 32 kHz | 3.2 ms of slack; 5.12 Mbit/s of payload on a 10 MHz link | at or past the limit — see below |

### When a rate turns out to be too fast

The top of that table is genuinely marginal, and which side of the line you land
on depends on your board, your SPI clock and what else your loop does. 16 kHz
has been seen working on an RP2350 over hybrid SPI at 10 MHz; 32 kHz leaves
3.2 ms to service a FIFO that is also competing with a magnetometer read.

Nothing is locked out. Every ODR the sensor offers can be selected, and if the
rate proves unserviceable the library steps down instead of failing:

```
Voltino TriSense: 32000 Hz could not be serviced - falling back to 16000 Hz.
```

The blocking calibration routines do this automatically — they time out, drop
one rung, and retry, up to four times. During normal streaming nothing changes
the rate behind your back: a rate that silently sags under load is worse than
one that stays put and tells you. If your own loop detects it is falling behind,
call it yourself:

```cpp
if (sensor.imu.getLostPacketCount() > threshold) {
  sensor.imu.stepDownODR();     // false once 12.5 Hz is reached
}
```

As a starting point rather than a limit: **8 kHz** over hybrid SPI and **1 kHz**
over I2C are comfortable on a 32-bit MCU, with room left for the rest of your
sketch.

`setODR()` may quietly reduce the rate if the *bus* cannot carry it — call
`getODRHz()` for what is in effect and `getRequestedODRHz()` for what you asked
for. Note that this check models bus serialisation only. It cannot know how
often your sketch calls `update()`, so it will happily accept 8 kHz on a loop
that stalls for 40 ms at a time. That part is yours to budget.

---

## 5. Writing a loop that keeps up

At high ODR the shape of your loop matters more than the MCU you chose.

### The rule

> **Call `fusion.update()` unconditionally, every iteration. Print on a timer,
> and never inside the branch that depends on new data.**

```cpp
void loop() {
  fusion.update();                  // ALWAYS - this drains the FIFO

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 100) {   // 10 Hz is plenty for a human
    lastPrint = millis();
    float roll, pitch, yaw;
    fusion.getOrientationDegrees(roll, pitch, yaw);
    Serial.print(roll, 1);  Serial.print(' ');
    Serial.print(pitch, 1); Serial.print(' ');
    Serial.println(yaw, 1);
  }
}
```

### What blocks, and for how long

| Operation | Typical cost |
|---|---|
| `Serial.print()` of a 200-character line @ 115200 baud | **~17 ms** |
| the same over USB CDC with a slow or unread host | **tens of ms, unbounded** |
| `readPressure()` when the BMP580 cache expires | ~1 ms (I2C) |
| `_mag->readData()` full read @ 100 kHz I2C | ~1 ms |
| SD card write | 10–100 ms |
| `delay(n)` | exactly as bad as it looks |

Compare those against the deadline table in §4. A single long status line at
8 kHz overruns the FIFO on its own — this is by far the most common cause of
overflow, and it is a property of the sketch, not of the library.

### Practical fixes, in order of preference

1. **Print less often, and print less.** 10 Hz of output is plenty for a human.
2. **Raise the serial speed** — `Serial.begin(921600)` cuts the cost 8×.
3. **Lower the ODR.** 8 kHz is rarely needed. Integrating a full second of
   continuous 500°/s rotation leaves 0.32° of truncation error at 100 Hz but
   only 0.0032° at 1 kHz — already far below what the gyro's own noise
   contributes over the same second.
4. **Move the printing to the second core** on RP2040/RP2350 (see
   `examples/GPS_INS_Localization` for the pattern).

---

## 6. Calibration, in order

Do these once, write the numbers into your sketch, and repeat only if you change
the mounting or the surrounding metal.

### Where calibration lives

**All accelerometer and gyroscope offsets and scales are stored in the driver,
in the sensor's own axes**, and applied before any mount remap:

```
accel_out = (accel_raw - accOffset) * accScale
gyro_out  =  gyro_raw  - gyrOffset
```

So a value you read back with `getAccelOffset()` / `getGyroOffset()` and save to
EEPROM can be handed straight back to the matching setter at **any**
`setMountOrientation()` setting. The magnetometer's hard/soft iron are likewise
applied in sensor axes, before the remap.

### Step 1 — Gyroscope bias (every power-up, or store it)

```cpp
Serial.println("Keep the board perfectly still...");
sensor.autoCalibrateGyro(1000);
```

Takes a second or two. The board must not move — not even a table vibration.
To avoid repeating it, read the result back and store it:

```cpp
float gx, gy, gz;
sensor.imu.getGyroOffset(gx, gy, gz);     // sensor axes - safe to store
// ... later, after a restart:
sensor.imu.setGyroOffset(gx, gy, gz);
```

You can also let the filter keep learning the residual drift while it runs:

```cpp
fusion.setDynamicGyroBias(true, 0.0001f);
fusion.setMaxGyroBias(5.0f);              // bound, so a bad fix cannot run away
```

### Step 2 — Accelerometer (once per board)

Two methods write to the same storage, so the later one refines the earlier one
rather than fighting it:

```cpp
sensor.autoCalibrateAccel();              // 6-point sphere fit: offset AND scale
fusion.calibrateAccelStatic();            // 1-point: refines the offset only
```

Run `examples/IMUcalibration`, follow the prompts through all six orientations,
and copy the printed values into your sketch:

```cpp
sensor.imu.setAccelOffset(0.0234, -0.0112, 0.0517);
sensor.imu.setAccelScale (1.0012,  0.9988, 1.0023);
```

### Step 3 — Magnetometer (once per *installation*)

This one depends on everything magnetic near the sensor — motors, battery,
steel screws — so it must be redone whenever the surroundings change.

1. Flash `examples/MotionCal`.
2. Run the [MotionCal](https://www.pjrc.com/store/prop_shield.html) desktop tool.
3. Rotate the board slowly through every orientation until the sphere fills in.
4. Copy out the hard-iron offsets and the soft-iron matrix:

```cpp
fusion.setMagHardIron(-46.02, -0.85, -46.00);
float softIron[3][3] = {
  { 0.965,  0.008, -0.002},
  { 0.008,  0.981,  0.139},
  {-0.002,  0.139,  1.077}
};
fusion.setMagSoftIron(softIron);
fusion.setDeclination(5.6);               // your location, degrees east
```

Find your declination at [NOAA](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml).
Skipping it leaves yaw referenced to *magnetic* north.

### Step 4 — Initial orientation

```cpp
fusion.initOrientation();                 // board still, roughly level
```

Seeds the quaternion from gravity and the magnetic field so the filter starts
converged instead of spending its first seconds settling.

---

## 7. Reading the outputs

### Orientation

```cpp
float roll, pitch, yaw;
fusion.getOrientationDegrees(roll, pitch, yaw);
```

Roll and pitch are ±180° / ±90°; yaw is 0–360°.

> **Near vertical**, roll and yaw stop being independently meaningful — at
> exactly 90° of pitch only their *sum* is defined. This is a property of Euler
> angles, not of the filter: the internal quaternion stays correct throughout,
> and the library clamps the conversion so no NaN escapes. If your application
> spends time pointing straight up or down, work from the quaternion (`fusion.q`)
> rather than from Euler angles.

### Acceleration

```cpp
float ax, ay, az;
fusion.getLinearAcceleration(ax, ay, az, ACCEL_UNIT_MS2);        // body frame, gravity removed
fusion.getGlobalLinearAcceleration(ax, ay, az, ACCEL_UNIT_MS2);  // world frame, gravity removed
fusion.getGlobalAcceleration(ax, ay, az, ACCEL_UNIT_MS2);        // world frame, gravity included
```

Use `setLocalGravity()` if you want m/s² referenced to your local *g* rather than
the 9.80665 standard.

The accelerometer values these use are the **mean of every packet** the FIFO
delivered since the last call, not just the last one. That both keeps all the
data and low-passes vibration.

### Cross-checking the heading

```cpp
float fused = /* yaw from getOrientationDegrees */;
float magOnly = fusion.getMagHeadingDegrees();
```

`getMagHeadingDegrees()` is a tilt-compensated magnetometer-only heading that
never touches the gyro. If the two disagree by more than a few degrees and stay
apart, suspect magnetic interference or a stale magnetometer calibration.

### Barometer

```cpp
float pa   = sensor.readPressure();        // pascals
float degC = sensor.readTemperature();
float alt  = sensor.readAltitude(101325);  // metres; reference in PASCALS
```

Absolute altitude is only as good as the sea-level reference you pass. *Relative*
altitude — the change since you started — is far more accurate; record the
pressure at startup and use that as the reference.

---

## 8. Choosing a fusion engine

| | `SimpleTriFusion` | `AdvancedTriFusion` |
|---|---|---|
| Sensors used | gyro (+ optional gravity assist) | gyro + accel + magnetometer |
| Absolute heading | ✗ drifts | ✓ magnetometer-referenced |
| Roll/pitch drift | grows without bound | corrected against gravity |
| Cost | very low | moderate |
| Runs on an Uno | comfortably | yes, at a low ODR |

```cpp
SimpleTriFusion   fusion(&sensor.imu, &sensor.mag);
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);
```

`SimpleTriFusion` is the right choice when you only care about *change* over
short intervals. For anything that must stay correct over minutes, use
`AdvancedTriFusion`. Its gyro-only path still runs at the full ODR; the
accelerometer and magnetometer corrections are applied at a much lower rate,
which is why it stays cheap.

---

## 9. Tuning the AHRS

The defaults are sensible. Change them only against a symptom.

| Call | Default | Raise it when | Lower it when |
|---|---|---|---|
| `setMaxGains(accel, mag)` | 0.1, 0.1 | drift corrects too slowly | output is jumpy under vibration |
| `setAccelGaussian(ref, sigma)` | 1.0, 0.05 | your platform vibrates a lot | you want stricter rejection |
| `setMagGaussian(ref, sigma)` | 50.88, 3.5 | local field differs from 50.88 µT | you want stricter rejection |
| `setMagTiltSigma(deg)` | 15 | you operate at steep angles | heading wanders when tilted |
| `setMagCheckInterval(ms)` | 5 | you want less I2C traffic | you want faster heading pull-in |
| `setYawKi(ki)` | 0.005 | yaw drifts back slowly | yaw oscillates |

**How the gaussian gains work.** The accelerometer is trusted in proportion to
how close its magnitude is to 1 g, and the magnetometer to how close its
magnitude is to the expected field strength. Under linear acceleration, or next
to a motor, the measured magnitude departs from the reference and that sensor's
influence falls away on its own — no thresholds, no hysteresis.

`setMagTiltSigma()` additionally reduces the magnetometer's weight as the board
tilts, because tilt compensation gets less reliable the closer you are to
vertical. The library also applies its own hard cut above 80° of tilt, so
widening sigma cannot let an indeterminate heading through.

---

## 10. Troubleshooting

### `beginAll()` returns false

Check in this order:

1. **Power.** The BMP580 in particular is fussy about a clean 3V3.
2. **Addresses.** Scan the bus with an I2C scanner. Expect `0x68` or `0x69`
   (IMU), `0x0C` (magnetometer), `0x46` or `0x47` (barometer).
3. **Pull-ups.** I2C needs them — 4.7 kΩ is the usual choice. Many breakout
   boards have them; a bare module may not.
4. **The right mode.** `MODE_HYBRID` with a wrong CS pin fails silently in a way
   that looks like a dead sensor.

### `!! FIFO OVERFLOW`

**What it means.** The hardware FIFO filled up before the library could drain
it. When that happens the sensor discards samples, and discarded samples are
rotation that can never be integrated — a permanent attitude error, not a
glitch that washes out.

**First, check whether you are actually losing anything.** Ask the sensor — it
keeps the tally itself:

```cpp
Serial.println(sensor.imu.getLostPacketCount());   // packets the CHIP discarded
```

That is a real quantity, measured in hardware. `getFIFOOverflowCount()` is not:
it counts *events* — refills that found the FIFO full. One stalled loop is a
single event but may cost hundreds of packets, while a FIFO that merely sits
full produces an event per refill and costs nothing. **A high event count next
to a lost count of zero means your data is intact.**

The independent cross-check needs no extra registers:

```cpp
Serial.print(fusion.getActualFusionHz());   // samples integrated per second
Serial.print(" / ");
Serial.println(sensor.imu.getODRHz());      // samples produced per second
```

If those two match, nothing is being lost, whatever the event counter says.

**If the rates genuinely differ,** your loop is not keeping up. Work through
§4 and §5: measure how long your slowest iteration takes, compare it against the
deadline table, and either shorten the loop or lower the ODR. In nearly every
case the culprit is a blocking `Serial.print()`.

### Yaw drifts, roll and pitch are fine

The magnetometer is not correcting. Either:

- its calibration is stale — redo §6 step 3 *in situ*, with the real wiring and
  battery in place;
- something magnetic is too close — motors, speakers, steel, unshielded current;
- the board spends its time near vertical, where the library deliberately backs
  the magnetometer off (see §7).

### Orientation is noisy under vibration

Lower `setMaxGains()`, or tighten `setAccelGaussian()`'s sigma so the filter
distrusts the accelerometer sooner. Mechanical isolation beats both.

### Output becomes NaN and never recovers

This should not happen — the library guards the paths that used to allow it. If
it does, it means a sensor returned all zeros (a bus fault, or a sensor that
went to sleep). Check wiring and power integrity, and please open an issue with
the configuration.

### Everything is slower than expected

- `getODRHz()` may be reporting less than you asked for; see §4.
- I2C defaults to 100 kHz on many cores. `sensor.bmp.setI2CSpeed(400000)`
  retunes the whole shared bus.
- Check whether `Serial` is the bottleneck before blaming the sensor.

---

## 11. Platform notes

| Platform | FPU | Recommended configuration |
|---|---|---|
| **RP2350** (Pico 2, XIAO RP2350) | yes | `MODE_HYBRID`, HiRes FIFO, 1–4 kHz |
| **RP2040** (Pico) | no | `MODE_HYBRID`, HiRes FIFO, ≤ 2 kHz |
| **ESP32 / S3** | yes | `MODE_HYBRID`, HiRes FIFO, 1–2 kHz |
| **ESP32-C3 / S2** | no | `MODE_I2C` or SPI, 16-bit FIFO, ≤ 1 kHz |
| **SAMD51, STM32F4+, nRF52840, Teensy** | yes | `MODE_HYBRID`, HiRes FIFO, 1–4 kHz |
| **SAMD21** | no | 16-bit FIFO, ≤ 500 Hz |
| **Arduino Uno / Nano / Mega** | no | `MODE_I2C`, 16-bit FIFO, ≤ 200 Hz |

The library detects the FPU at compile time and picks its reciprocal square root
accordingly — a single `VSQRT` instruction where one exists, the bit-trick with
two Newton–Raphson steps where it does not. Nothing to configure.

### On AVR specifically

- `FIFO_20BIT_HIRES` falls back to `FIFO_16BIT` automatically; the 20-bit path
  needs 32-bit arithmetic the part cannot do cheaply.
- The FIFO burst buffer is 160 B rather than 2560 B, because an Uno has 2 KB of
  RAM in total.
- `AdvancedTriFusion` fits, but keep the ODR at or below 200 Hz.

### Forcing double precision

```cpp
#define FORCE_FUSION_DOUBLE
#include <TriSense.h>
```

Rarely worth it. Float already resolves far below the sensor's own noise floor,
and on any MCU without a double-precision FPU this costs several times the
runtime for no measurable gain.

---

## Further reading

- [README](../README.md) — feature overview and full API reference
- `examples/` — eight worked sketches, from raw reads to GPS/INS fusion
- [ICM-42688-P datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- [AK09918C datasheet](https://www.akm.com/global/en/products/electronic-compass/ak09918c/)
- [BMP580 datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp580/)
