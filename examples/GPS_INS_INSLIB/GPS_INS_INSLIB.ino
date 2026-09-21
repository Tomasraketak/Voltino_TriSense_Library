/*
 * Example: GPS_INS_INSLIB.ino
 *
 * The same job as examples/GPS_INS_Localization - full 3D localization from
 * TriSense + a Quectel L76K - handed to a real 15-state error-state Kalman
 * filter instead of the cascade the other example builds by hand:
 *
 *      INSLIB   https://github.com/jnz/INSLIB   (Jan Zwiener)
 *
 * TriSense does what it is good at (drive the sensors, calibrate them, hand
 * over physical units) and INSLIB does the estimation. The library itself is
 * untouched: everything below is sketch code.
 *
 * Outputs orientation, world-frame velocity, position in local NED and in
 * WGS84, the estimated IMU biases, and a solution mode that says out loud
 * whether it currently has GNSS, is coasting on inertia, or is down to
 * attitude only.
 *
 * ---------------------------------------------------------------------------
 * FIRST: INSLIB IS AGPL-3.0, THIS LIBRARY IS MIT
 * ---------------------------------------------------------------------------
 * Nothing in this repository is AGPL - ./fetch_inslib.sh downloads INSLIB when
 * you build. But the firmware you flash is a combined work and the AGPL covers
 * it, including the network-use clause. For a closed product either take a
 * commercial licence from INSLIB's author or use GPS_INS_Localization, which is
 * MIT throughout. KFCore, the Kalman backend INSLIB uses, is BSD-3-Clause.
 *
 * ---------------------------------------------------------------------------
 * HARDWARE
 * ---------------------------------------------------------------------------
 *   Raspberry Pi Pico 2 (RP2350) + Voltino TriSense + Quectel L76K
 *     TriSense : ICM-42688-P on SPI (CS = GP17), AK09918C + BMP580 on I2C
 *     L76K     : module RX <- GP0 (Pico TX),  module TX -> GP1 (Pico RX)
 *
 * SOFTWARE
 *   - Earle Philhower arduino-pico core (setup1()/loop1(), the Pico SDK mutex
 *     API, time_us_64(), rp2040.getFreeStack()).
 *   - TinyGPSPlus by Mikal Hart (Library Manager: "TinyGPSPlus").
 *   - INSLIB + KFCore, fetched by ./fetch_inslib.sh. Run it once before you
 *     compile; see README.md.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS AND NOT THE CASCADE
 * ---------------------------------------------------------------------------
 * GPS_INS_Localization runs a complementary filter for attitude and three
 * small per-axis Kalman filters for position. That is the right trade for most
 * vehicles and it is a tenth of the arithmetic. It has one blind spot, called
 * out in its own header: attitude error and velocity error are treated as
 * independent, so a long GNSS outage under real dynamics degrades in a way it
 * cannot recover from, and heading can only come from the magnetometer.
 *
 * A 15-state ESKF estimates position, velocity, attitude and both IMU biases in
 * one covariance, so those correlations are what it works with. Concretely,
 * this example gives you:
 *
 *   - heading that becomes observable from motion alone. Accelerate in a
 *     straight line and the filter works out which way it is pointing, with no
 *     magnetometer involved. Near steel or motors that matters more than any
 *     amount of magnetometer calibration.
 *   - gyroscope bias as an estimated state, not a fixed number from boot-time
 *     calibration. It follows temperature instead of being outrun by it.
 *   - GNSS latency compensation: the fix is 100-200 ms old by the time the
 *     NMEA sentence finishes arriving, and INSLIB anchors the correction where
 *     it belongs in its own state history (GPS_LATENCY_MS below).
 *   - graceful degradation with a name: FULL / COASTING / ATTITUDE_ONLY, plus a
 *     parallel AHRS and barometric vertical channel that keep running when the
 *     position solution cannot.
 *   - outlier rejection and automatic zero-velocity updates, which is where
 *     most of the accelerometer-bias observability actually comes from.
 *
 * The cost is arithmetic and roughly 7 KB of stack. Both are budgeted below.
 *
 * ---------------------------------------------------------------------------
 * RP2350: WHAT THE TWO CORES AND THE FPU ARE ACTUALLY DOING
 * ---------------------------------------------------------------------------
 *  - core 0 owns SPI, I2C and the filter, and never touches USB serial. core 1
 *    owns the L76K UART and all printing. A blocking print on the filter core
 *    is the classic cause of IMU FIFO overflow, and a dropped FIFO packet is
 *    rotation that can never be integrated back.
 *  - `core1_separate_stack = true` below is not optional. arduino-pico splits
 *    its 8 KB stack into 4 KB per core as soon as a sketch defines loop1();
 *    that flag gives core 1 its own 8 KB and leaves core 0 the full 8 KB.
 *    nav_suite_update() needs 6776 bytes of that - measured against this exact
 *    toolchain and flags, see src/inslib_config.h. On a 4 KB stack this sketch
 *    would not overflow loudly: it would quietly write through the bottom of
 *    core 0's stack and into core 1's. The telemetry line reports the real
 *    margin every second.
 *  - The Cortex-M33 has a single-precision FPU and no double-precision one, so
 *    every `double` is a software call. INSLIB is built for exactly that: the
 *    filter is float throughout and doubles appear only in geodetic
 *    coordinates, a handful of times per GNSS fix. Keep it that way in any code
 *    you add here - note the f-suffixed libm calls below, since sqrt() instead
 *    of sqrtf() silently promotes to double and costs about 20x.
 *  - The IMU runs at 1 kHz and the filter at 200 Hz. Feeding the ESKF every IMU
 *    sample would spend the cycles inside nav_suite_update()'s fixed overhead
 *    instead of on estimation; averaging each 5-sample batch into one epoch is
 *    the standard trade and loses nothing a ground or air vehicle will notice.
 *
 * ---------------------------------------------------------------------------
 * BEFORE YOU TRUST IT: four things you must set
 * ---------------------------------------------------------------------------
 *  1. MAG_HARD_IRON / MAG_SOFT_IRON - run the MotionCal example first.
 *  2. MAGNETIC_DECLINATION is NOT here on purpose. INSLIB carries the World
 *     Magnetic Model and works out declination from your first fix, so the
 *     magnetometer references true north by itself.
 *  3. IMU_TO_FRD_* - INSLIB works in a front-right-down body frame and the
 *     TriSense sits Z-up, so the sketch flips two axes. If your board is
 *     mounted any other way, this is the place to fix it, and getting it wrong
 *     is the single most effective way to produce confident nonsense.
 *  4. GPS_UERE_M and the entry gate next to it. The numbers a receiver deserves
 *     and the accuracy INSLIB demands before it will enter a 3D solution have
 *     to be consistent, or the filter will consume fixes forever and never
 *     start. Read the note there before changing either.
 */

// PICO_RP2350 comes from the core's own platform definitions. ARDUINO_ARCH_* is
// no use here: the arduino-pico platform is called "rp2040" whatever chip it is
// building for, so every board in it, Pico 2 included, gets
// -DARDUINO_ARCH_RP2040.
#if !defined(PICO_RP2350)
  #error "GPS_INS_INSLIB targets the RP2350 (Raspberry Pi Pico 2) on the Earle Philhower arduino-pico core. The RP2040 has no FPU and cannot give one core the 8 KB of stack this filter needs - use examples/GPS_INS_Localization there."
#endif
#if !defined(__ARM_FP)
  #error "Select the ARM (Cortex-M33) architecture for this board, not RISC-V: the Hazard3 cores have no FPU, and INSLIB's float arithmetic would run in software."
#endif

#include <Arduino.h>
#include <TriSense.h>
#include <TinyGPSPlus.h>
#include <pico/mutex.h>
#include <pico/time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>   // atof, for the GGA geoid-separation field

// The build configuration for the vendored INSLIB sources. It MUST come before
// the INSLIB headers: INS_UNKNOWNS_MAX changes the layout of ins_t, and a
// translation unit that disagrees about it would silently read the filter's
// state at the wrong offsets.
#include "src/inslib_config.h"
#include "inslib/nav_suite.h"

// ===========================================================================
// CONFIGURATION
// ===========================================================================

// ---- Wiring -----------------------------------------------------------------
#define IMU_CS_PIN            17
#define IMU_SPI_HZ            10000000UL
#define GPS_PIN_TX            0        // Pico TX  -> L76K RX
#define GPS_PIN_RX            1        // Pico RX  <- L76K TX

// ---- Rates ------------------------------------------------------------------
#define IMU_ODR               ODR_1KHZ // IMU sample rate
#define NAV_RATE_HZ           200      // INSLIB epochs per second
#define MAG_RATE_HZ           50       // magnetometer polling
#define BARO_RATE_HZ          50       // barometer polling, matched to its ODR
#define TELEMETRY_HZ          1        // serial output rate

// ---- Magnetometer calibration -----------------------------------------------
// From the MotionCal example, in the magnetometer's OWN axes, exactly as you
// would hand them to TriSenseFusion::setMagCalibration(). The sketch applies
// them here and hands INSLIB an already-calibrated vector, so INSLIB's own
// opt.mag_fixed_bias / opt.mag_misalignment stay at zero.
static const float MAG_HARD_IRON[3] = { -46.02f, -0.85f, -46.00f };
static const float MAG_SOFT_IRON[3][3] = {
  {  1.000f,  0.000f,  0.000f },
  {  0.000f,  1.000f,  0.000f },
  {  0.000f,  0.000f,  1.000f }
};

// ---- Body frame -------------------------------------------------------------
// INSLIB's body frame is FRD: x forward, y right, z down. Every sensor must
// arrive in that one frame - mixing frames between the accelerometer, the
// gyroscope and the magnetometer produces a solution that looks plausible and
// is wrong.
//
// A TriSense lying flat with the component side up reads +1 g on Z at rest, so
// its frame is right-handed Z-up. Mapping that to FRD is one relabel and two
// sign flips: forward = +X, right = -Y, down = -Z. Sanity check after any
// change: at rest and level the FRD accelerometer must read (0, 0, -9.81), and
// a yaw to the RIGHT must give a POSITIVE gyro Z.
//
// If the board is mounted rotated or upside down, change these three macros
// rather than trying to compensate downstream.
#define IMU_TO_FRD_F(x, y, z)  ( (x))
#define IMU_TO_FRD_R(x, y, z)  (-(y))
#define IMU_TO_FRD_D(x, y, z)  (-(z))

// ---- Sensor noise -----------------------------------------------------------
// Left at INSLIB's own conservative consumer-MEMS defaults (a 0 in Qll_diag
// selects them). The datasheet figures for the ICM-42688-P are an accelerometer
// noise density of 70 ug/sqrt(Hz) (PSD 4.7e-7 (m/s^2)^2/Hz) and a gyroscope
// noise density of 0.0028 dps/sqrt(Hz) (PSD 2.4e-9 (rad/s)^2/Hz) - but those
// are the sensor alone on a bench. On a vehicle, vibration and mounting
// dominate them by an order of magnitude, and a filter told the bench figure
// trusts the IMU far more than it should. Measure your own with an Allan
// variance run (INSLIB ships python/allan_variance.py) before setting these.
#define ACC_PSD               0.0f     // (m/s^2)^2/Hz per axis, 0 -> INSLIB default
#define GYR_PSD               0.0f     // (rad/s)^2/Hz per axis, 0 -> INSLIB default

// Barometric altitude noise, 1-sigma. Drives the vertical channel.
#define BARO_ALT_SIGMA_M      0.6f

// ---- GNSS -------------------------------------------------------------------
#define GPS_FAST_BAUD         115200UL
#define GPS_BOOT_BAUD         9600UL
#define GPS_RATE_HZ           5
#define GPS_MIN_SATS          4        // below this the fix is not offered
#define GPS_MAX_HDOP          6.0f     // above this the fix is not offered

// How old a fix is by the time the sentence has finished arriving: receiver
// processing and filtering, plus the UART transmission itself. INSLIB anchors
// the correction that far back in its state history instead of applying it to
// the present, which is what keeps position from lagging during acceleration.
// Worth measuring for your own module (INSLIB has tools/inslib_clock_error.py);
// 150 ms is a reasonable starting point for an L76K at 5 Hz over 115200 baud.
#define GPS_LATENCY_MS        150

// Horizontal position 1-sigma per unit HDOP. 2.2 m is a fair figure for a
// single-band consumer receiver with a decent antenna and open sky.
#define GPS_UERE_M            2.2f
#define GPS_SIGMA_FLOOR_M     1.5f     // no fix is better than this
// Vertical error is worse than horizontal for the same constellation geometry,
// because every satellite is above the horizon. 1.9 is the usual rule of thumb
// where VDOP is not reported; GGA does not carry it.
#define GPS_V_OVER_H          1.9f

// The 3D entry gate, and the trap it exists to spring.
//
// INSLIB refuses to enter a 3D solution until the aiding has been better than
// gnss_start_max_* continuously for gnss_init_dwell_sec. Its defaults are 2 m
// horizontal and 3 m vertical - tighter than what the sigma model above reports
// for a perfect fix (HDOP 1.0 * 2.2 m = 2.2 m). Left alone, the two settings
// contradict each other and the filter consumes fixes forever without ever
// starting, in complete silence, because src/inslib_config.h turns logging off.
//
// So the gate is stated here, consistently with the sigma model: enter on
// anything at HDOP <= ~2.7, give up the 3D solution only when fixes stay far
// worse than that. Tighten both if you move to an RTK receiver and lower
// GPS_UERE_M with them.
#define GPS_GATE_START_H_M    6.0f
#define GPS_GATE_START_V_M    12.0f
#define GPS_GATE_STOP_H_M     15.0f
#define GPS_GATE_STOP_V_M     30.0f

// Ground speed from RMC, fused as a scalar |v| measurement. This is the honest
// way to use an NMEA receiver's velocity: RMC reports speed over ground and
// course over ground, with no vertical component and no per-axis accuracy, so
// offering it as a 3D NED velocity would mean inventing a vertical velocity and
// a covariance for it. A scalar speed constrains what it actually measures.
#define GPS_USE_SPEED         1
#define GPS_SPEED_SIGMA_MPS   0.3f

// Course over ground as an absolute heading. Correct ONLY for a vehicle that
// cannot move sideways - a car, a boat, a fixed-wing. Wrong for a multicopter,
// a pedestrian or anything that can crab, and wrong for everything at low
// speed. Off by default; the magnetometer and the motion-driven heading
// observability of the ESKF cover the general case.
#define GPS_USE_COURSE_YAW    0
#define GPS_COURSE_MIN_MPS    3.0f
#define GPS_COURSE_SIGMA_DEG  8.0f

// Decimal year handed to the World Magnetic Model, which is what turns the
// magnetometer's magnetic north into true north. Declination moves slowly, so
// being a year out costs a fraction of a degree in most places - but update it
// if this firmware is still flying in a few years' time.
#define WMM_EPOCH_YEAR        2026.0f

// Geoid separation fallback [m] for receivers that leave GGA field 11 empty.
// GGA altitude is above mean sea level; INSLIB works in ellipsoid height, and
// h_ellipsoid = h_msl + geoid_separation. Central Europe is about +44 m.
#define GEOID_SEP_FALLBACK_M  44.0f

// ---- Stack watchdog ---------------------------------------------------------
// Measured worst-case need of nav_suite_update() on this target, from INSLIB's
// own call-graph stack analysis. See src/inslib_config.h for the measurement.
#define INSLIB_WORST_CASE_STACK 6800
#define STACK_WARN_MARGIN       512

// ===========================================================================
// SHARED STATE
// ===========================================================================

// One GNSS epoch, core 1 -> core 0.
struct GpsFix {
  uint32_t seq;          // increments on every new fix; 0 = none yet
  double   lat;          // deg
  double   lon;          // deg
  float    altEllip;     // m above the WGS84 ellipsoid
  float    hdop;
  float    speedMps;     // ground speed
  float    courseDeg;    // course over ground
  bool     haveSpeed;
  bool     haveCourse;
  uint8_t  sats;
  uint32_t stampMs;      // millis() when the sentence completed
};

// The navigation solution, core 0 -> core 1.
struct NavOut {
  float    roll, pitch, yaw;       // deg
  float    q[4];                   // Hamilton, q[0] = w, body -> NED
  float    vN, vE, vD;             // m/s
  float    pN, pE, pD;             // m from the NED origin
  double   lat, lon;               // deg
  float    hEllip;                 // m above the WGS84 ellipsoid
  float    hLocal;                 // m above the NED origin, positive up
  float    baroV;                  // m/s from the vertical channel, positive up
  float    accBias[3];             // m/s^2, body FRD
  float    gyrBias[3];             // deg/s, body FRD
  float    sigmaRollPitch;         // deg, 1-sigma
  float    sigmaYaw;               // deg, 1-sigma
  uint8_t  mode;                   // nav_suite_mode_t
  bool     insReady;
  bool     haveAbsolute;           // lat/lon/hEllip are real, not a guess
  bool     zupt;
  int      deadReckonMs;
  uint32_t epochHz;                // measured INSLIB epoch rate
  uint32_t fifoOverflows;
  uint32_t fifoLostPackets;
  uint32_t droppedEpochs;          // epochs skipped because no IMU data arrived
  int      stackFreeAtNav;         // bytes of core 0 stack left when the filter ran
  uint32_t navMaxUs;               // longest nav_suite_update() seen
  uint32_t navLastUs;
};

static mutex_t g_gpsMtx;
static mutex_t g_navMtx;
static GpsFix  g_gpsShared;
static NavOut  g_navShared;

static volatile bool g_navReady     = false;   // core 0 finished booting
static volatile bool g_sensorFail   = false;   // core 0 could not start the TriSense
static volatile bool g_initFail     = false;   // nav_suite_init() refused the config
static volatile bool g_csvOutput    = false;
static volatile bool g_rawNmeaDebug = false;

// arduino-pico splits its 8 KB stack in half the moment a sketch defines
// loop1(). This gives core 1 a stack of its own so core 0 keeps all 8 KB for
// the filter. Read the RP2350 note in the header before removing it.
bool core1_separate_stack = true;

// ===========================================================================
// CORE 0 - SENSORS AND THE FILTER
// ===========================================================================

static TriSense sensor;

// The filter. 41 KB of struct even at 15 states (8% of the RP2350's RAM), so it
// lives in .bss and never on a stack. INSLIB never calls malloc.
static nav_suite_t suite;

// The per-epoch measurement bundle. Also file scope, also deliberately: it is
// exactly 440 bytes, and core 0's stack has about 1.4 KB to spare once
// nav_suite_update() has taken its share.
static ins_measurements_t meas;

static const float DEG2RADF = 0.017453292519943295f;
static const float RAD2DEGF = 57.29577951308232f;
static const float G_MPS2   = 9.80665f;   // the g the ICM-42688-P scale factors imply

// IMU batch accumulated between epochs, already in FRD and SI units.
static float    accSum[3] = { 0.0f, 0.0f, 0.0f };
static float    gyrSum[3] = { 0.0f, 0.0f, 0.0f };
static uint32_t imuCount  = 0;

static uint64_t lastEpochUs   = 0;
static uint32_t lastMagUs     = 0;
static uint32_t lastBaroUs    = 0;
static uint32_t consumedFixSeq = 0;
static bool     magModelSet   = false;
static uint32_t droppedEpochs = 0;
static uint64_t lastStarvedUs = 0;

// Epoch-rate measurement.
static uint32_t epochCount    = 0;
static uint32_t lastRateMs    = 0;
static uint32_t epochHz       = 0;

static uint32_t navMaxUs      = 0;
static uint32_t navLastUs     = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Hard iron then soft iron, in the magnetometer's own axes, then into FRD. The
// order is not interchangeable: a MotionCal fit is derived from raw sensor axes,
// so applying it after an axis flip feeds each offset to the wrong axis. This
// mirrors TriSenseFusion::applyMagCalibration() exactly.
static void magToFRD(float rx, float ry, float rz, float out[3]) {
  const float hx = rx - MAG_HARD_IRON[0];
  const float hy = ry - MAG_HARD_IRON[1];
  const float hz = rz - MAG_HARD_IRON[2];

  const float cx = MAG_SOFT_IRON[0][0] * hx + MAG_SOFT_IRON[0][1] * hy + MAG_SOFT_IRON[0][2] * hz;
  const float cy = MAG_SOFT_IRON[1][0] * hx + MAG_SOFT_IRON[1][1] * hy + MAG_SOFT_IRON[1][2] * hz;
  const float cz = MAG_SOFT_IRON[2][0] * hx + MAG_SOFT_IRON[2][1] * hy + MAG_SOFT_IRON[2][2] * hz;

  out[0] = IMU_TO_FRD_F(cx, cy, cz);
  out[1] = IMU_TO_FRD_R(cx, cy, cz);
  out[2] = IMU_TO_FRD_D(cx, cy, cz);
}

// Fill a diagonal 3x3 NED covariance, column-major. INSLIB takes the full
// matrix; a receiver that only reports DOP has nothing to say about the
// off-diagonal terms, so they stay zero.
static void diagCov(float Q[9], float varN, float varE, float varD) {
  memset(Q, 0, 9 * sizeof(float));
  Q[0] = varN;
  Q[4] = varE;
  Q[8] = varD;
}

static bool takeGpsFix(GpsFix &out) {
  bool got = false;
  if (mutex_try_enter(&g_gpsMtx, NULL)) {
    if (g_gpsShared.seq != 0 && g_gpsShared.seq != consumedFixSeq) {
      out = g_gpsShared;
      consumedFixSeq = g_gpsShared.seq;
      got = true;
    }
    mutex_exit(&g_gpsMtx);
  }
  return got;
}

// ---------------------------------------------------------------------------
// One INSLIB epoch
// ---------------------------------------------------------------------------

static void publishNav(int stackFree) {
  NavOut n;
  memset(&n, 0, sizeof(n));

  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  if (nav_suite_get_rpy(&suite, &roll, &pitch, &yaw)) {
    n.roll  = roll  * RAD2DEGF;
    n.pitch = pitch * RAD2DEGF;
    n.yaw   = yaw   * RAD2DEGF;
    if (n.yaw < 0.0f) n.yaw += 360.0f;   // report a compass-style 0..360 heading
  }

  // Identity first: if the filter has no attitude yet the getter leaves the
  // array alone, and an all-zero quaternion is not a rotation.
  n.q[0] = 1.0f;
  ins_get_quaternion(&suite.ins, n.q);

  float vel[3], pos[3];
  if (ins_get_velocity_ned(&suite.ins, vel)) {
    n.vN = vel[0]; n.vE = vel[1]; n.vD = vel[2];
  }
  if (ins_get_position_local(&suite.ins, pos)) {
    n.pN = pos[0]; n.pE = pos[1]; n.pD = pos[2];
  }

  double llh[3];
  n.haveAbsolute = ins_get_latlonh(&suite.ins, llh);
  if (n.haveAbsolute) {
    n.lat    = llh[0] * (double)RAD2DEGF;
    n.lon    = llh[1] * (double)RAD2DEGF;
    n.hEllip = (float)llh[2];
  }

  // The local height arbitrates between INS and the barometric channel, so it
  // stays continuous when GNSS drops. The absolute height above does not.
  nav_suite_get_height(&suite, &n.hLocal);
  nav_suite_get_baro_alt(&suite, NULL, &n.baroV);

  float bias[3];
  if (ins_get_bias_acc(&suite.ins, bias)) {
    n.accBias[0] = bias[0]; n.accBias[1] = bias[1]; n.accBias[2] = bias[2];
  }
  if (ins_get_bias_gyr(&suite.ins, bias)) {
    n.gyrBias[0] = bias[0] * RAD2DEGF;
    n.gyrBias[1] = bias[1] * RAD2DEGF;
    n.gyrBias[2] = bias[2] * RAD2DEGF;
  }

  float sr = 0.0f, sp = 0.0f, sy = 0.0f;
  if (ins_get_rpy_stddev(&suite.ins, &sr, &sp, &sy)) {
    n.sigmaRollPitch = 0.5f * (sr + sp) * RAD2DEGF;
    n.sigmaYaw       = sy * RAD2DEGF;
  }

  n.mode            = (uint8_t)nav_suite_get_mode(&suite);
  n.insReady        = ins_is_ready(&suite.ins);
  n.zupt            = nav_suite_get_zaru_active(&suite);
  n.deadReckonMs    = ins_deadreckoning_ms(&suite.ins);
  n.epochHz         = epochHz;
  n.fifoOverflows   = sensor.imu.getFIFOOverflowCount();
  n.fifoLostPackets = sensor.imu.getLostPacketCount();
  n.droppedEpochs   = droppedEpochs;
  n.stackFreeAtNav  = stackFree;
  n.navMaxUs        = navMaxUs;
  n.navLastUs       = navLastUs;

  if (mutex_try_enter(&g_navMtx, NULL)) {
    g_navShared = n;
    mutex_exit(&g_navMtx);
  }
}

static void navStep(uint64_t nowUs, float dt) {
  memset(&meas, 0, sizeof(meas));
  meas.timestamp = (ins_time_us_t)nowUs;

  // --- IMU ------------------------------------------------------------------
  // The mean of the batch over the batch's own elapsed time. Integrating the
  // mean once over dt is the same first-order strapdown step as integrating
  // each sample over dt/N, which is what makes the 1 kHz -> 200 Hz decimation
  // free rather than approximate at these rates.
  const float inv = 1.0f / (float)imuCount;
  meas.strapdown_dt_sec = dt;

  meas.acc.is_valid = true;
  meas.acc.data[0]  = accSum[0] * inv;
  meas.acc.data[1]  = accSum[1] * inv;
  meas.acc.data[2]  = accSum[2] * inv;
  meas.acc.Qll_diag[0] = meas.acc.Qll_diag[1] = meas.acc.Qll_diag[2] = ACC_PSD;

  meas.gyr.is_valid = true;
  meas.gyr.data[0]  = gyrSum[0] * inv;
  meas.gyr.data[1]  = gyrSum[1] * inv;
  meas.gyr.data[2]  = gyrSum[2] * inv;
  meas.gyr.Qll_diag[0] = meas.gyr.Qll_diag[1] = meas.gyr.Qll_diag[2] = GYR_PSD;

  accSum[0] = accSum[1] = accSum[2] = 0.0f;
  gyrSum[0] = gyrSum[1] = gyrSum[2] = 0.0f;
  imuCount = 0;

  const uint32_t nowUs32 = (uint32_t)nowUs;

  // --- Magnetometer ---------------------------------------------------------
  // Flagged only on epochs carrying a genuinely new sample. A latched is_valid
  // would re-fuse one reading every epoch and make the filter overconfident
  // about its heading. INSLIB throttles the fusion itself
  // (opt.magnetometer_min_delay_ms) - the magnetometer is a long-term anchor
  // against yaw drift, not a per-epoch heading sensor.
  if ((uint32_t)(nowUs32 - lastMagUs) >= (1000000UL / MAG_RATE_HZ)) {
    lastMagUs = nowUs32;
    if (sensor.mag.readData() && !sensor.mag.overflow) {
      float m[3];
      magToFRD(sensor.mag.x, sensor.mag.y, sensor.mag.z, m);
      meas.mag.is_valid = true;
      meas.mag.data[0]  = m[0];
      meas.mag.data[1]  = m[1];
      meas.mag.data[2]  = m[2];   // uT, matching the WMM's units
    }
  }

  // --- Barometer ------------------------------------------------------------
  // Raw static pressure, not an altitude: INSLIB owns the conversion and the
  // weather offset. Same "new sample only" rule as the magnetometer, which is
  // why the polling rate matches the sensor's ODR.
  if ((uint32_t)(nowUs32 - lastBaroUs) >= (1000000UL / BARO_RATE_HZ)) {
    lastBaroUs = nowUs32;
    const float pa = sensor.bmp.readPressure();
    if (pa > 30000.0f && pa < 125000.0f) {      // the BMP580's own range
      meas.baro.is_valid   = true;
      meas.baro.pressure_pa = pa;
      meas.baro.stddev_m    = BARO_ALT_SIGMA_M;
    }
  }

  // --- GNSS -----------------------------------------------------------------
  GpsFix fix;
  if (takeGpsFix(fix)) {
    float sigmaH = GPS_UERE_M * fix.hdop;
    if (sigmaH < GPS_SIGMA_FLOOR_M) sigmaH = GPS_SIGMA_FLOOR_M;
    const float sigmaV = sigmaH * GPS_V_OVER_H;

    meas.gnss_pos.is_valid = true;
    meas.gnss_pos.llh[0]   = fix.lat * (double)DEG2RADF;
    meas.gnss_pos.llh[1]   = fix.lon * (double)DEG2RADF;
    meas.gnss_pos.llh[2]   = (double)fix.altEllip;
    diagCov(meas.gnss_pos.Qll_ned, sigmaH * sigmaH, sigmaH * sigmaH, sigmaV * sigmaV);

    // How old the fix is, counted from when the sentence finished arriving.
    // INSLIB anchors the residual that far back in its state history.
    const uint32_t ageMs = (uint32_t)(millis() - fix.stampMs);
    int delayMs = (int)GPS_LATENCY_MS + (int)(ageMs > 400 ? 400 : ageMs);
    meas.gnss_delay_ms = delayMs;

#if GPS_USE_SPEED
    if (fix.haveSpeed) {
      meas.speed.is_valid  = true;
      meas.speed.speed_mps = fix.speedMps;
      meas.speed.stddev_mps = GPS_SPEED_SIGMA_MPS;
      meas.speed_delay_ms  = delayMs;
    }
#endif

#if GPS_USE_COURSE_YAW
    if (fix.haveCourse && fix.haveSpeed && fix.speedMps >= GPS_COURSE_MIN_MPS) {
      meas.yaw.is_valid   = true;
      meas.yaw.yaw_rad    = fix.courseDeg * DEG2RADF;   // NED yaw == course over ground
      meas.yaw.stddev_rad = GPS_COURSE_SIGMA_DEG * DEG2RADF;
      meas.yaw_delay_ms   = delayMs;
    }
#endif

    // Declination and field strength from the built-in World Magnetic Model,
    // once we know where on Earth we are. This is what makes the magnetometer
    // reference TRUE north, and it also arms INSLIB's field-strength gate
    // against magnetic disturbances.
    //
    // Both filters need telling. nav_suite does not propagate the position to
    // its sub-filters, so without the second call the magnetometer-aided AHRS -
    // the one that supplies attitude in ATTITUDE_ONLY mode - would keep
    // referencing MAGNETIC north while the INS referenced true north, and the
    // reported heading would step by the local declination whenever the
    // solution degraded.
    if (!magModelSet) {
      ins_set_magnetic_model_from_position(&suite.ins,
                                           meas.gnss_pos.llh[0],
                                           meas.gnss_pos.llh[1],
                                           WMM_EPOCH_YEAR);
      ahrs_set_position(&suite.ahrs,
                        (float)meas.gnss_pos.llh[0],
                        (float)meas.gnss_pos.llh[1],
                        WMM_EPOCH_YEAR);
      magModelSet = true;
    }
  }

  // --- The filter -----------------------------------------------------------
  const int stackFree = rp2040.getFreeStack();
  const uint32_t t0 = micros();
  nav_suite_update(&suite, &meas);
  navLastUs = micros() - t0;
  if (navLastUs > navMaxUs) navMaxUs = navLastUs;

  publishNav(stackFree);
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

static bool startFilter() {
  memset(&suite, 0, sizeof(suite));

  ins_init_t init;
  memset(&init, 0, sizeof(init));
  // No start position: auto_init anchors the NED origin on the first fix that
  // passes the entry gate. An all-zero llh is a legal (if unlikely) place to
  // start and INSLIB treats it as the provisional anchor it is.
  init.pos_init_stddev_m   = 10.0f;
  init.vel_init_stddev_mps = 1.0f;
  init.rpy_init_stddev_rad[0] = 5.0f * DEG2RADF;   // roll  - leveled from gravity
  init.rpy_init_stddev_rad[1] = 5.0f * DEG2RADF;   // pitch - same
  init.rpy_init_stddev_rad[2] = 180.0f * DEG2RADF; // yaw   - genuinely unknown at boot

  ins_options_t opt;
  memset(&opt, 0, sizeof(opt));

  // Bootstrap position, level and time from the measurement stream instead of
  // demanding they be supplied. Everything left at 0 takes INSLIB's default.
  opt.auto_init = true;

  // Covariance prediction at 50 Hz while the strapdown keeps running at the
  // full 200 Hz epoch rate. The covariance changes far more slowly than the
  // state, and this is the single biggest lever on CPU cost.
  opt.kalman_update_dt_sec = 0.02f;

  // See the GPS_GATE_* note above: these have to agree with the sigma model,
  // or the filter consumes fixes forever without ever entering 3D.
  opt.gnss_start_max_horizontal_pos_stddev_m = GPS_GATE_START_H_M;
  opt.gnss_start_max_vertical_pos_stddev_m   = GPS_GATE_START_V_M;
  opt.gnss_stop_max_horizontal_pos_stddev_m  = GPS_GATE_STOP_H_M;
  opt.gnss_stop_max_vertical_pos_stddev_m    = GPS_GATE_STOP_V_M;
  // And the same consistency for fusion itself, not just for entering 3D.
  opt.gnss_max_horizontal_pos_stddev_m = GPS_GATE_STOP_H_M;
  opt.gnss_max_vertical_pos_stddev_m   = GPS_GATE_STOP_V_M;

  // Keep the solution alive through a long tunnel rather than throwing the
  // position away at the default coasting limit. The mode reported in telemetry
  // says COASTING throughout, and the growing position sigma says how much to
  // believe it.
  opt.max_deadreckoning_sec = 120.0f;

  // The magnetometer is calibrated in the sketch (magToFRD), so INSLIB's own
  // hard-iron/soft-iron correction stays at identity. Its field-strength
  // disturbance gate is left ON - it is what rejects a sample taken next to a
  // motor - and needs the WMM, which arrives with the first fix.
  opt.mag_field_tolerance = 0.30f;

  return nav_suite_init(&suite, &init, &opt) == 0;
}

void setup() {
  // Core 0 owns SPI, I2C and the filter. It deliberately never prints: core 1
  // does all the talking.
  if (!sensor.beginAll(MODE_HYBRID, IMU_CS_PIN, IMU_SPI_HZ)) {
    g_sensorFail = true;
    while (1) { delay(1000); }
  }

  sensor.imu.setODR(IMU_ODR);
  sensor.imu.setFIFOMode(FIFO_16BIT);

  // 400 kHz I2C rather than the 100 kHz default, so the magnetometer and
  // barometer reads stay a rounding error on core 0's budget.
  sensor.bmp.setI2CSpeed(400000);

  // Barometer at 50 Hz, sampled at 50 Hz, IIR off. Sampling AT the ODR means
  // nothing can alias, so the hardware filter would only add group delay and -
  // worse for a Kalman filter - correlate consecutive samples, which makes it
  // count each one as independent evidence it is not. OSR x8 buys ~4 cm of
  // noise per sample, which 50 Hz can afford. Config registers only latch
  // reliably in standby, so bracket the writes.
  sensor.bmp.setPowerMode(BMP580_MODE_STANDBY);
  delay(5);
  sensor.bmp.setOversampling(BMP580_OSR_x8, BMP580_OSR_x2);
  sensor.bmp.setODR(BMP580_ODR_50p1Hz);
  sensor.bmp.setIIRFilter(BMP580_IIR_OFF, BMP580_IIR_OFF);
  sensor.bmp.setPowerMode(BMP580_MODE_NORMAL);
  delay(20);

  // One boot-time gyro bias estimate, in sensor axes, applied by the driver.
  // INSLIB estimates the bias continuously from there, so this only has to be
  // close enough to keep the initial alignment honest - it is not the thing
  // that has to hold over temperature.
  sensor.autoCalibrateGyro(1000);

  if (!startFilter()) {
    g_initFail = true;
    while (1) { delay(1000); }
  }

  sensor.imu.flushFIFO();
  lastEpochUs = time_us_64();
  lastMagUs   = (uint32_t)lastEpochUs;
  lastBaroUs  = lastMagUs;
  lastRateMs  = millis();
  g_navReady  = true;
}

void loop() {
  // Drain the IMU FIFO as fast as it fills, converting to FRD and SI on the
  // way in. Everything else in this loop is rate-limited so that this gets the
  // cycles it needs.
  float ax, ay, az, gx, gy, gz;
  while (sensor.imu.readFIFO(ax, ay, az, gx, gy, gz)) {
    // g -> m/s^2 and dps -> rad/s, then sensor axes -> body FRD.
    const float axm = ax * G_MPS2, aym = ay * G_MPS2, azm = az * G_MPS2;
    const float gxr = gx * DEG2RADF, gyr_ = gy * DEG2RADF, gzr = gz * DEG2RADF;

    accSum[0] += IMU_TO_FRD_F(axm, aym, azm);
    accSum[1] += IMU_TO_FRD_R(axm, aym, azm);
    accSum[2] += IMU_TO_FRD_D(axm, aym, azm);

    gyrSum[0] += IMU_TO_FRD_F(gxr, gyr_, gzr);
    gyrSum[1] += IMU_TO_FRD_R(gxr, gyr_, gzr);
    gyrSum[2] += IMU_TO_FRD_D(gxr, gyr_, gzr);

    imuCount++;
    if (imuCount >= 64) break;   // never let one drain starve the epoch clock
  }

  const uint64_t nowUs = time_us_64();
  if ((uint64_t)(nowUs - lastEpochUs) < (uint64_t)(1000000UL / NAV_RATE_HZ)) return;

  // No IMU data means no epoch: INSLIB's strapdown is what carries the state
  // between corrections, and an epoch without it would advance the clock
  // without advancing the solution. lastEpochUs deliberately stays put, so dt
  // still measures the real elapsed time once samples resume - which is why the
  // counter is rate-limited rather than incremented per pass, or a single
  // starved second would report thousands of lost epochs instead of 200.
  if (imuCount == 0) {
    if ((uint64_t)(nowUs - lastStarvedUs) >= (uint64_t)(1000000UL / NAV_RATE_HZ)) {
      lastStarvedUs = nowUs;
      droppedEpochs++;
    }
    return;
  }

  float dt = (float)(nowUs - lastEpochUs) * 1e-6f;
  lastEpochUs = nowUs;
  if (dt > 0.25f) dt = 0.25f;   // a stalled loop must not inject a wild step

  navStep(nowUs, dt);

  epochCount++;
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastRateMs) >= 1000) {
    epochHz    = epochCount;
    epochCount = 0;
    lastRateMs = nowMs;
  }
}

// ===========================================================================
// CORE 1 - GNSS PARSING AND TELEMETRY
// ===========================================================================

static TinyGPSPlus gps;
// GGA field 11 is the geoid separation, which TinyGPSPlus does not expose and
// which is the difference between the altitude the receiver prints and the
// ellipsoid height INSLIB works in. Both talkers, because an L76K tracking more
// than one constellation emits GNGGA rather than GPGGA.
static TinyGPSCustom geoidGP(gps, "GPGGA", 11);
static TinyGPSCustom geoidGN(gps, "GNGGA", 11);

static uint32_t gpsBaud        = GPS_BOOT_BAUD;
static uint32_t lastPublishMs  = 0;
static uint32_t lastPrintMs    = 0;
static uint32_t fixSeq         = 0;
static bool     sensorFailShown = false;
static bool     initFailShown   = false;
static bool     stackWarned     = false;

// L76K proprietary commands (PCAS). Checksums are computed in gpsSend().
#define PCAS_BAUD_115200   "PCAS01,5"
#define PCAS_RATE_1HZ      "PCAS02,1000"
#define PCAS_RATE_5HZ      "PCAS02,200"
// GGA + RMC only. GSV alone can be four sentences per epoch; at 5 Hz that is
// most of the UART budget spent on data nothing here parses.
#define PCAS_NMEA_MINIMAL  "PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0,,,,0"

static void gpsSend(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; ++p) cs ^= (uint8_t)*p;
  char tail[5];
  snprintf(tail, sizeof(tail), "*%02X", cs);
  Serial1.print('$');
  Serial1.print(body);
  Serial1.print(tail);
  Serial1.print("\r\n");
  Serial1.flush();
}

// True if valid NMEA arrives at the currently open baud rate within `ms`.
static bool gpsListening(uint32_t ms) {
  const uint32_t base = gps.passedChecksum();
  const uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    while (Serial1.available()) {
      gps.encode((char)Serial1.read());
      if ((uint32_t)(gps.passedChecksum() - base) >= 2) return true;
    }
  }
  return false;
}

static void gpsOpen(uint32_t baud) {
  Serial1.end();
  Serial1.begin(baud);
  gpsBaud = baud;
  delay(50);
}

static void gpsConfigure() {
  Serial1.setTX(GPS_PIN_TX);
  Serial1.setRX(GPS_PIN_RX);

  // The module keeps its baud rate across a Pico reset, so try the fast rate
  // first and only then fall back to the 9600 factory default.
  gpsOpen(GPS_FAST_BAUD);
  if (!gpsListening(1500)) {
    gpsOpen(GPS_BOOT_BAUD);
    if (!gpsListening(1500)) {
      Serial.println(F("[GPS] No NMEA at 9600 or 115200 - check TX/RX wiring and antenna."));
      return;
    }
    gpsSend(PCAS_BAUD_115200);
    delay(200);
    gpsOpen(GPS_FAST_BAUD);
    if (!gpsListening(1500)) {
      gpsOpen(GPS_BOOT_BAUD);
      gpsSend(PCAS_RATE_1HZ);
      gpsSend(PCAS_NMEA_MINIMAL);
      return;
    }
  }

  gpsSend(PCAS_RATE_5HZ);
  delay(50);
  gpsSend(PCAS_NMEA_MINIMAL);
  Serial.print(F("[GPS] L76K at "));
  Serial.print(gpsBaud);
  Serial.print(F(" baud, "));
  Serial.print(GPS_RATE_HZ);
  Serial.println(F(" Hz, GGA+RMC only."));
}

static void publishFix() {
  if (!gps.location.isValid() || !gps.location.isUpdated()) return;
  if (gps.satellites.isValid() && gps.satellites.value() < GPS_MIN_SATS) return;
  if (!gps.hdop.isValid()) return;

  const float hdop = (float)gps.hdop.hdop();
  if (!(hdop > 0.0f) || hdop > GPS_MAX_HDOP) return;
  if (!gps.altitude.isValid()) return;

  // MSL altitude plus geoid separation is the ellipsoid height INSLIB wants.
  float geoid = GEOID_SEP_FALLBACK_M;
  const char *g = geoidGN.isValid() ? geoidGN.value() : (geoidGP.isValid() ? geoidGP.value() : NULL);
  if (g != NULL && g[0] != '\0') geoid = (float)atof(g);

  GpsFix f;
  memset(&f, 0, sizeof(f));
  f.lat        = gps.location.lat();
  f.lon        = gps.location.lng();
  f.altEllip   = (float)gps.altitude.meters() + geoid;
  f.hdop       = hdop;
  f.sats       = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  f.stampMs    = millis();
  f.haveSpeed  = gps.speed.isValid();
  f.speedMps   = f.haveSpeed ? (float)gps.speed.mps() : 0.0f;
  f.haveCourse = gps.course.isValid();
  f.courseDeg  = f.haveCourse ? (float)gps.course.deg() : 0.0f;
  f.seq        = ++fixSeq;

  // Core 1 may block here; core 0 never does.
  mutex_enter_blocking(&g_gpsMtx);
  g_gpsShared = f;
  mutex_exit(&g_gpsMtx);
}

static const char *modeName(uint8_t m) {
  switch ((nav_suite_mode_t)m) {
    case NAV_SUITE_MODE_FULL:          return "FULL";
    case NAV_SUITE_MODE_COASTING:      return "COAST";
    case NAV_SUITE_MODE_ATTITUDE_ONLY: return "ATT";
    default:                           return "INIT";
  }
}

static void printTelemetry(const NavOut &n, uint8_t sats, float hdop) {
  if (g_csvOutput) {
    // mode,ready,lat,lon,hEllip,hLocal,vN,vE,vD,roll,pitch,yaw,sigRP,sigYaw,sats,hdop
    Serial.print(n.mode);            Serial.print(',');
    Serial.print(n.insReady ? 1 : 0); Serial.print(',');
    Serial.print(n.lat, 8);          Serial.print(',');
    Serial.print(n.lon, 8);          Serial.print(',');
    Serial.print(n.hEllip, 2);       Serial.print(',');
    Serial.print(n.hLocal, 2);       Serial.print(',');
    Serial.print(n.vN, 3);           Serial.print(',');
    Serial.print(n.vE, 3);           Serial.print(',');
    Serial.print(n.vD, 3);           Serial.print(',');
    Serial.print(n.roll, 2);         Serial.print(',');
    Serial.print(n.pitch, 2);        Serial.print(',');
    Serial.print(n.yaw, 2);          Serial.print(',');
    Serial.print(n.sigmaRollPitch, 2); Serial.print(',');
    Serial.print(n.sigmaYaw, 2);     Serial.print(',');
    Serial.print(sats);              Serial.print(',');
    Serial.println(hdop, 2);
    return;
  }

  Serial.print(F("["));  Serial.print(modeName(n.mode));
  if (!n.insReady) Serial.print(F(" warmup"));
  Serial.print(F("] "));

  Serial.print(F("RPY "));
  Serial.print(n.roll, 1);  Serial.print('/');
  Serial.print(n.pitch, 1); Serial.print('/');
  Serial.print(n.yaw, 1);
  Serial.print(F(" deg (+-"));
  Serial.print(n.sigmaRollPitch, 1); Serial.print('/');
  Serial.print(n.sigmaYaw, 1);
  Serial.print(F(")"));

  Serial.print(F(" | NED "));
  Serial.print(n.pN, 1); Serial.print(' ');
  Serial.print(n.pE, 1); Serial.print(' ');
  Serial.print(n.pD, 1); Serial.print(F(" m"));

  Serial.print(F(" | v "));
  Serial.print(n.vN, 2); Serial.print(' ');
  Serial.print(n.vE, 2); Serial.print(' ');
  Serial.print(n.vD, 2); Serial.print(F(" m/s"));

  if (n.haveAbsolute) {
    Serial.print(F(" | "));
    Serial.print(n.lat, 7); Serial.print(' ');
    Serial.print(n.lon, 7);
    Serial.print(F(" h ")); Serial.print(n.hEllip, 1); Serial.print(F(" m"));
  }

  Serial.print(F(" | sats ")); Serial.print(sats);
  Serial.print(F(" hdop "));   Serial.print(hdop, 1);
  if (n.zupt) Serial.print(F(" ZUPT"));
  if (n.deadReckonMs > 0) {
    Serial.print(F(" DR "));
    Serial.print(n.deadReckonMs / 1000);
    Serial.print('s');
  }
  Serial.println();

  // Second line: the health numbers. Everything here is a thing that fails
  // quietly on an embedded target if nobody looks at it.
  Serial.print(F("        bias a "));
  Serial.print(n.accBias[0], 3); Serial.print(' ');
  Serial.print(n.accBias[1], 3); Serial.print(' ');
  Serial.print(n.accBias[2], 3); Serial.print(F(" m/s2, g "));
  Serial.print(n.gyrBias[0], 3); Serial.print(' ');
  Serial.print(n.gyrBias[1], 3); Serial.print(' ');
  Serial.print(n.gyrBias[2], 3); Serial.print(F(" dps"));

  Serial.print(F(" | ")); Serial.print(n.epochHz); Serial.print(F(" Hz"));
  Serial.print(F(" | filter ")); Serial.print(n.navLastUs);
  Serial.print(F("/")); Serial.print(n.navMaxUs); Serial.print(F(" us"));
  Serial.print(F(" | stack ")); Serial.print(n.stackFreeAtNav);
  Serial.print(F(" B free, needs ")); Serial.print(INSLIB_WORST_CASE_STACK);

  if (n.fifoLostPackets || n.fifoOverflows) {
    Serial.print(F(" | FIFO full x")); Serial.print(n.fifoOverflows);
    Serial.print(F(" lost ")); Serial.print(n.fifoLostPackets);
  }
  if (n.droppedEpochs) {
    Serial.print(F(" | no-IMU epochs ")); Serial.print(n.droppedEpochs);
  }
  Serial.println();

  // The one failure mode that produces no symptom until it produces every
  // symptom. Say it once, loudly.
  if (!stackWarned && n.stackFreeAtNav > 0 &&
      n.stackFreeAtNav < (INSLIB_WORST_CASE_STACK + STACK_WARN_MARGIN)) {
    stackWarned = true;
    Serial.println(F("!! STACK: core 0 has less headroom than the filter needs."));
    Serial.println(F("!! Check that `bool core1_separate_stack = true;` is still there,"));
    Serial.println(F("!! and that src/inslib_config.h still sets INS_UNKNOWNS_MAX to 15."));
  }
}

static void handleCommand() {
  while (Serial.available()) {
    const int c = Serial.read();
    switch (c) {
      case 'c': case 'C':
        g_csvOutput = !g_csvOutput;
        if (g_csvOutput) {
          Serial.println(F("mode,ready,lat,lon,hEllip,hLocal,vN,vE,vD,roll,pitch,yaw,sigRP,sigYaw,sats,hdop"));
        }
        break;
      case 'n': case 'N':
        g_rawNmeaDebug = !g_rawNmeaDebug;
        break;
      case 'h': case 'H':
        Serial.println(F("c = CSV on/off, n = raw NMEA on/off, h = this help"));
        break;
      default:
        break;
    }
  }
}

void setup1() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (uint32_t)(millis() - t0) < 3000) { delay(10); }

  Serial.println();
  Serial.println(F("Voltino TriSense + Quectel L76K + INSLIB (15-state ESKF)"));
  Serial.println(F("INSLIB is AGPL-3.0 - see the licence note in this sketch."));
  Serial.println(F("Press h for commands."));

  gpsConfigure();

  while (!g_navReady) {
    if (g_sensorFail && !sensorFailShown) {
      sensorFailShown = true;
      Serial.println(F("!! TriSense did not start. Check SPI/I2C wiring and the CS pin."));
    }
    if (g_initFail && !initFailShown) {
      initFailShown = true;
      Serial.println(F("!! nav_suite_init() refused the configuration. Check src/inslib_config.h."));
    }
    delay(100);
  }
  Serial.println(F("[NAV] filter running, waiting for a fix that passes the entry gate."));
}

void loop1() {
  // Feed the parser everything the UART has. This is the only place that may
  // block, and it is on the core that is allowed to.
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    if (g_rawNmeaDebug) Serial.write(c);
    gps.encode(c);
  }

  const uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - lastPublishMs) >= (1000 / GPS_RATE_HZ) / 2) {
    lastPublishMs = nowMs;
    publishFix();
  }

  handleCommand();

  if ((uint32_t)(nowMs - lastPrintMs) >= (1000 / TELEMETRY_HZ)) {
    lastPrintMs = nowMs;

    NavOut n;
    bool have = false;
    if (mutex_try_enter(&g_navMtx, NULL)) {
      n = g_navShared;
      have = true;
      mutex_exit(&g_navMtx);
    }
    if (have) {
      const uint8_t sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
      const float   hdop = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 99.0f;
      printTelemetry(n, sats, hdop);
    }
  }
}
