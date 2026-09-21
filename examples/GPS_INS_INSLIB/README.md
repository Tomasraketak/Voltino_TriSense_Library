# GPS_INS_INSLIB — TriSense + L76K through a real 15-state ESKF

This example does the same job as [`GPS_INS_Localization`](../GPS_INS_Localization)
— fuse the TriSense IMU, magnetometer and barometer with a Quectel L76K into one
navigation solution — but hands the estimation to
[**INSLIB**](https://github.com/jnz/INSLIB), a portable C navigation-filter
library by Jan Zwiener, instead of the hand-built cascade the other example
uses.

The Voltino TriSense library is **not modified**. TriSense drives the sensors,
calibrates them and hands over physical units; everything that bridges to INSLIB
lives in this sketch.

---

## Licensing — read this first

| | |
|---|---|
| Voltino TriSense | MIT |
| **INSLIB** | **AGPL-3.0** |
| KFCore (INSLIB's Kalman backend) | BSD-3-Clause |

No INSLIB source is committed to this repository: `fetch_inslib.sh` downloads it
when you build. The firmware you end up flashing is still a combined work, and
the AGPL covers it — including the network-use clause. If that does not suit
your product, either take a commercial licence from INSLIB's author or use
`GPS_INS_Localization`, which is MIT all the way down.

---

## Setup

```sh
cd examples/GPS_INS_INSLIB
./fetch_inslib.sh
```

That clones INSLIB and KFCore, flattens them into `./inslib/`, and generates the
translation-unit wrappers in `./src/`. Nothing it writes is tracked by git — run
it again to update. Pin a version if you want a reproducible build:

```sh
INSLIB_REF=v1.1.0 ./fetch_inslib.sh
```

Then open `GPS_INS_INSLIB.ino` in the Arduino IDE and build it like any other
sketch.

**Requirements**

- Raspberry Pi Pico 2 (RP2350), **ARM** architecture — not RISC-V, the Hazard3
  cores have no FPU. The sketch refuses to compile otherwise.
- Earle Philhower [arduino-pico](https://github.com/earlephilhower/arduino-pico)
  core.
- TinyGPSPlus (Library Manager).
- `git` on your PATH, for the fetch script.

**Wiring**

```
TriSense   ICM-42688-P   SPI, CS = GP17
           AK09918C      I2C
           BMP580        I2C
L76K       module RX  <- GP0  (Pico TX)
           module TX  -> GP1  (Pico RX)
```

Built size: **143 KB of flash (3%)** and **61 KB of RAM (11%)** on a Pico 2, of
which the filter struct alone is 41 KB.

---

## Why bother, when `GPS_INS_Localization` already works

The cascade in the other example is the right trade for most vehicles and costs
roughly a tenth of the arithmetic. Its own header is honest about the one thing
it cannot do: it treats attitude error and velocity error as independent, so
heading can only ever come from the magnetometer.

A 15-state error-state Kalman filter carries them in one covariance. In practice
that buys:

- **Heading observable from motion.** Accelerate in a straight line and the
  filter works out which way it is pointing, with no magnetometer involved. Near
  steel or motors, that beats any amount of magnetometer calibration.
- **Gyroscope bias as an estimated state**, following temperature, rather than
  one number measured at boot and then outrun by it.
- **GNSS latency compensation.** An L76K fix is 100–200 ms old by the time the
  sentence has finished arriving. INSLIB anchors the correction that far back in
  its own state history instead of applying it to the present, which is what
  stops position lagging under acceleration.
- **Graceful degradation with a name.** `FULL` / `COASTING` / `ATTITUDE_ONLY`,
  plus a parallel AHRS and a barometric vertical channel that keep running when
  the position solution cannot.
- **Outlier rejection and automatic zero-velocity updates**, which is where most
  of the accelerometer-bias observability actually comes from.

The cost is arithmetic and about 6.8 KB of stack, both budgeted below.

---

## How INSLIB is built without touching it

INSLIB and KFCore size their working arrays from `-D` flags, and two of them
decide whether the filter fits on an RP2350 at all. The Arduino build offers no
way to pass `-D` to a sketch, and arduino-pico does not support `build_opt.h`
either. So:

- The INSLIB sources land in **`./inslib/`**, which Arduino copies into the build
  but does **not** compile — only the sketch root and `./src/` are compiled.
  (Verified against arduino-cli 1.5.2, not assumed.)
- Each one is pulled into its own one-line translation unit under **`./src/`**
  that includes `src/inslib_config.h` first:

  ```c
  #include "inslib_config.h"
  #include "../inslib/ins.c"
  ```

That is exactly what `-D` would have done, without patching a line of INSLIB.
One wrapper per source file, not a single unity build: INSLIB's files define
same-named static helpers (`time_diff_sec`, `qsquare`, …) that collide if
concatenated.

Both trees are flattened into one directory because every `#include` in INSLIB
and KFCore is an unqualified quoted name (`"ins.h"`, `"kalman_udu.h"`), so once
the files sit side by side each resolves relative to its own includer — no `-I`
needed, which is just as well, because a sketch cannot add one.

---

## The stack budget, which is the whole design constraint

arduino-pico gives a core **8192 bytes** of stack. Measured with INSLIB's own
call-graph stack analysis, against the exact toolchain and flags arduino-pico
uses for a Pico 2 (its arm-none-eabi-gcc 16.1.0, cortex-m33,
`armv8-m.main+fp+dsp`, `-Os`):

| configuration | `nav_suite_update()` needs | fits in 8192 B? |
|---|---:|---|
| `INS_UNKNOWNS_MAX=15`, KFCore 15/21 — **what this example uses** | 6776 B | yes, ~1.4 KB spare |
| `INS_UNKNOWNS_MAX=18`, KFCore 18/24 | 8528 B | no |
| `INS_UNKNOWNS_MAX=15`, KFCore's own defaults (32/32) | 12880 B | no, by 50% |

`-O2` moves the first figure by 24 bytes, so the optimization level in the Tools
menu does not change the conclusion.

That third row is the one worth staring at. KFCore's defaults are not
pathological — they are sized for a desktop — and on a Cortex-M the consequence
is not a crash but silent corruption of whatever sits below the stack. Hence
`src/inslib_config.h`, and hence two things in the sketch you should not remove:

```c
bool core1_separate_stack = true;   // 8 KB per core instead of 4 KB per core
```

arduino-pico splits its 8 KB stack in half the moment a sketch defines `loop1()`.
This flag gives core 1 a stack of its own and leaves core 0 the full 8 KB.

And the telemetry line reports the real margin, every second, from
`rp2040.getFreeStack()` sampled on core 0 immediately before the filter runs:

```
| stack 7784 B free, needs 6800
```

If that ever drops near the second number, the sketch says so loudly and once.

**15 states is not a compromise.** It is the full INS state vector: position,
velocity, attitude, accelerometer bias, gyroscope bias. The only thing dropped
is INSLIB's optional 3-state magnetometer hard-iron estimator, and the TriSense
magnetometer is calibrated offline with the `MotionCal` example anyway — a fit
over a full sphere beats one estimated from whatever rotations the vehicle
happened to perform.

---

## What runs where

```
core 0   SPI + I2C + the filter, never touches USB serial
         ├── drain the ICM-42688-P FIFO at 1 kHz, convert to FRD / SI
         ├── poll AK09918C at 50 Hz, BMP580 at 50 Hz
         └── 200 Hz: build one ins_measurements_t, nav_suite_update()

core 1   L76K UART + all printing (its own 8 KB stack)
         ├── PCAS configuration, TinyGPSPlus parsing
         └── 1 Hz telemetry, serial commands
```

A blocking print on the filter core is the classic cause of IMU FIFO overflow,
and a dropped FIFO packet is rotation that can never be integrated back. Core 0
only ever uses `mutex_try_enter()`, so the real-time core never blocks; a missed
handoff is retried on the next epoch 5 ms later.

The IMU runs at 1 kHz and the filter at 200 Hz. Feeding the ESKF every IMU sample
would spend the cycles inside `nav_suite_update()`'s fixed overhead rather than
on estimation. Integrating the mean of a 5-sample batch once over `dt` is the
same first-order strapdown step as integrating each sample over `dt/5`, so the
decimation is free rather than approximate at these rates.

All navigation maths is single-precision. The Cortex-M33 has a single-precision
FPU and no double-precision one, so every `double` is a software call — INSLIB is
built for exactly that, and `double` appears only in geodetic coordinates, a
handful of times per fix. If you add code here, keep the f-suffixed libm calls:
`sqrt()` instead of `sqrtf()` silently promotes to double and costs about 20×.

---

## Frames, which is the most common way to get confident nonsense

INSLIB's body frame is **FRD** (x forward, y right, z down) and its navigation
frame is **NED**. A TriSense lying flat, component side up, reads +1 g on Z at
rest, so its frame is right-handed Z-up. Mapping one to the other is one relabel
and two sign flips, and it lives in three macros near the top of the sketch:

```c
#define IMU_TO_FRD_F(x, y, z)  ( (x))
#define IMU_TO_FRD_R(x, y, z)  (-(y))
#define IMU_TO_FRD_D(x, y, z)  (-(z))
```

If your board is mounted rotated or upside down, fix it here rather than
compensating downstream, and check both of these afterwards:

- at rest and level, the FRD accelerometer must read `(0, 0, -9.81)`;
- a yaw to the **right** must produce a **positive** gyro Z.

The accelerometer, gyroscope and magnetometer must all arrive in the *same*
frame. Mixing them produces a solution that looks plausible and is wrong.

---

## Things you must set before trusting the output

1. **`MAG_HARD_IRON` / `MAG_SOFT_IRON`** — run the `MotionCal` example first.
   These are in the magnetometer's own axes, exactly as you would hand them to
   `TriSenseFusion::setMagCalibration()`; the sketch applies them and only then
   flips into FRD, which is the order that matters.
2. **Magnetic declination is deliberately absent.** INSLIB carries the World
   Magnetic Model and derives declination from your first fix, so the
   magnetometer references true north by itself. That also arms its
   field-strength gate against magnetic disturbance.
3. **`IMU_TO_FRD_*`** — see above.
4. **`GPS_UERE_M` and the entry gate next to it.** See below. This is the one
   that will silently stop the filter from ever starting.

### The GNSS entry gate

INSLIB refuses to enter a 3D solution until the aiding has been better than
`gnss_start_max_*` continuously for `gnss_init_dwell_sec`. Its defaults are 2 m
horizontal and 3 m vertical — *tighter than what this sketch's sigma model
reports for a perfect fix* (HDOP 1.0 × 2.2 m UERE = 2.2 m). Left alone, the two
settings contradict each other and the filter consumes fixes forever without
starting, in complete silence, because `src/inslib_config.h` turns INSLIB's
logging off.

So the gate is stated explicitly in `startFilter()` and kept consistent with the
sigma model. If you move to an RTK receiver, lower `GPS_UERE_M` and tighten
`GPS_GATE_*` **together**.

### What the L76K actually gives the filter

| NMEA | INSLIB measurement |
|---|---|
| GGA position + HDOP | `gnss_pos`, lat/lon/ellipsoid height with a diagonal NED covariance from HDOP × UERE |
| GGA field 11 (geoid separation) | turns the reported MSL altitude into the ellipsoid height INSLIB works in |
| RMC ground speed | `speed`, a scalar \|v\| measurement |
| RMC course over ground | `yaw`, **off by default** — see below |

The scalar-speed route is the honest way to use an NMEA receiver's velocity. RMC
reports speed and course over ground with no vertical component and no per-axis
accuracy, so offering it as a 3D NED velocity would mean inventing a vertical
velocity *and* a covariance for it — and a fabricated vertical velocity sigma is
exactly what the entry gate checks. A scalar speed constrains what is actually
measured.

`GPS_USE_COURSE_YAW` fuses course over ground as an absolute heading. That is
correct **only** for a vehicle that cannot move sideways — a car, a boat, a
fixed-wing — and wrong for a multicopter, a pedestrian, or anything at low speed.
Off by default; the magnetometer and the ESKF's own motion-driven heading
observability cover the general case.

---

## Reading the telemetry

```
[FULL] RPY -0.3/1.2/184.7 deg (+-0.4/1.9) | NED 12.4 -3.1 -0.8 m | v 1.42 -0.11 0.02 m/s | 50.0812345 14.4201234 h 274.3 m | sats 11 hdop 0.9
       bias a 0.012 -0.004 0.031 m/s2, g -0.021 0.008 0.114 dps | 200 Hz | filter 118/964 us | stack 7784 B free, needs 6800
```

| field | meaning |
|---|---|
| `[FULL]` | `nav_suite_get_mode()`: `FULL` (fresh position aiding), `COAST` (dead reckoning), `ATT` (attitude only), `INIT` |
| `warmup` | `ins_is_ready()` is still false — the filter has not levelled and gated in yet |
| `(+-a/b)` | 1-sigma on roll/pitch and on yaw, in degrees. Watch yaw shrink as you start moving |
| `NED` | position from the origin the first gated fix anchored |
| `bias a / g` | the estimated IMU biases. Gyro bias drifting with temperature is normal and is the point |
| `ZUPT` | INSLIB's own standstill detector is re-estimating biases |
| `DR 12s` | seconds of dead reckoning without absolute aiding |
| `filter x/y us` | last and worst `nav_suite_update()` duration. At 200 Hz the budget is 5000 µs |
| `stack N B free` | core 0 stack available when the filter ran, against what it needs |
| `FIFO full xN lost M` | `M`, the sensor's own count of discarded packets, is the one that matters |

Serial commands: `c` toggles CSV output, `n` dumps raw NMEA, `h` prints help.

---

## Tuning notes

**Sensor noise** (`ACC_PSD` / `GYR_PSD`) is left at INSLIB's conservative
consumer-MEMS defaults. The ICM-42688-P datasheet gives 70 µg/√Hz and
0.0028 dps/√Hz, but those are the sensor alone on a bench — on a vehicle,
vibration and mounting dominate them by an order of magnitude, and a filter told
the bench figure trusts the IMU far more than it should. Measure your own with
an Allan variance run (INSLIB ships `python/allan_variance.py`) before setting
them.

**`GPS_LATENCY_MS`** defaults to 150 ms, a reasonable starting point for an L76K
at 5 Hz over 115200 baud. It is worth measuring for your own module — INSLIB has
`tools/inslib_clock_error.py` for exactly this — because a guessed value biases
position during dynamic motion.

**`opt.kalman_update_dt_sec`** (50 Hz) is the single biggest lever on CPU cost.
The strapdown keeps running at the full 200 Hz epoch rate; only the covariance
prediction is throttled, and covariance changes far more slowly than state.

**`opt.max_deadreckoning_sec`** is raised to 120 s so a long tunnel keeps the
solution alive rather than throwing the position away. The mode says `COAST`
throughout and the growing position sigma says how much to believe it.

**Turning INSLIB's logging back on** is genuinely useful while bringing a new
vehicle up — "GNSS fusable but below the 3D entry gate" explains a filter that
never starts. Set `LOG_LEVEL` to 3 (WARN) in `src/inslib_config.h`, install a
sink with `log_set_sink()` that queues to the telemetry core, and do **not**
leave `printf` as the sink: it blocks, and it drags ~2 KB of stack into the
filter's worst-case path.

---

## Further reading

- INSLIB: <https://github.com/jnz/INSLIB> — `tutorial/c_tutorial.md` and
  `src/ins.h`, which is the reference for every option used here.
- `docs/GUIDE.md` in this repository, for the TriSense side.
- `examples/GPS_INS_Localization` — the MIT-licensed cascade, and its header's
  explanation of when a cascade beats an ESKF.
