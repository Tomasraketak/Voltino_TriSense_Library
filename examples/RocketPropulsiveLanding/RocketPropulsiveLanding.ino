/*
 * Example: RocketPropulsiveLanding.ino  (Voltino TriSense v1.3.0)
 *
 * State estimator for an amateur propulsively-landed solid-motor rocket:
 * boost to 100-250 m, coast, descend, relight a solid motor and land on it.
 *
 * It is the rocket-specific sibling of GPS_INS_Localization.ino. Same cascade
 * (AdvancedTriFusion for attitude, a per-axis Kalman filter for translation),
 * but retuned around a flight profile that breaks most of the assumptions a
 * ground-vehicle navigator is built on.
 *
 * OUTPUTS
 *      attitude          roll / pitch / yaw, quaternion
 *      altitude AGL      metres above the pad
 *      climb rate        m/s, positive up            <- the number the burn needs
 *      velocity          N / E / Up
 *      position          N / E from the pad, plus lat/lon for recovery
 *      flight state      PAD / BOOST / COAST / DESCENT / LANDING_BURN / LANDED
 *      burn solution     ignition altitude for the landing motor
 *      flight log        ~40 s at 100 Hz in RAM, dumped as CSV after landing
 *
 * ===========================================================================
 * READ THIS FIRST
 * ===========================================================================
 * This sketch ESTIMATES state. It does not fly the rocket and it fires nothing.
 * onLandingBurnGo() is an empty hook. A flight computer that commands pyro needs
 * an arming switch, continuity checks, a watchdog and redundancy that are well
 * outside what an example sketch should pretend to provide - and it needs ground
 * testing before it ever sees a motor.
 *
 * Known limits, stated plainly:
 *   - The ICM-42688-P saturates at +/-16 g. If your landing burn pulls more, the
 *     accelerometer clips and velocity integration through the burn is garbage.
 *     The sketch counts saturated samples and reports them; if that count is
 *     non-zero, you need a separate high-g accelerometer, not better tuning.
 *   - Barometric altitude is corrupted by airspeed over the static ports and by
 *     motor plume. Handled below, but it is mitigation, not a cure.
 *   - GPS contributes essentially nothing between launch and landing. It sets
 *     the origin before flight and finds the rocket afterwards.
 *   - Attitude during boost is gyro-only (correctly - see below), so heading
 *     error grows with gyro bias. Calibrate on the pad and keep boost short.
 *
 * ===========================================================================
 * WHAT CHANGES VERSUS THE GROUND-VEHICLE VERSION, AND WHY
 * ===========================================================================
 *
 * 1. ZUPT IS NOW STATE-GATED, AND THAT IS A SAFETY FIX.
 *    The generic sketch declares a standstill when |accel| ~ 1 g and the gyro is
 *    quiet. A rocket descending at terminal velocity reads EXACTLY 1 g with a
 *    quiet gyro - drag balancing weight is what terminal velocity means. An
 *    ungated zero-velocity update would then pin the velocity estimate to zero
 *    while falling at 30 m/s, and the landing burn would be solved from it.
 *    ZUPT here is permitted only in PAD and LANDED.
 *
 * 2. THE TWO AXES HAVE COMPLETELY DIFFERENT SENSOR HIERARCHIES.
 *
 *    VERTICAL is baro-led. A differential barometer referenced to the pad is
 *    good to a few tenths of a metre and the Kalman filter differentiates it
 *    into climb rate. Attitude error barely touches this axis: at 5 g of thrust,
 *    1 degree of tilt error costs only 5*g*(1-cos 1deg) = 0.0075 m/s^2 of
 *    vertical acceleration. GPS altitude still contributes, though - see below.
 *
 *    HORIZONTAL is GPS-led, and this is not optional. The same 1 degree of
 *    attitude error costs 5*g*sin(1deg) = 0.87 m/s^2 LATERALLY - two orders of
 *    magnitude worse than its vertical cost, because it projects the whole
 *    thrust vector sideways. Worked for a 3.2 kg vehicle on a 270 Ns / 200 N /
 *    1.7 s motor (6.4 g peak, burnout ~68 m/s at ~58 m, apogee ~250 m at T+8.6s):
 *
 *        attitude error   lateral accel   drift by burnout   drift by apogee
 *            0.5 deg        0.43 m/s^2         0.7 m/s             16 m
 *            1.0 deg        0.87 m/s^2         1.5 m/s             32 m
 *            2.0 deg        1.73 m/s^2         2.9 m/s             64 m
 *
 *    Dead reckoning cannot hold horizontal position through that. GPS can: 8.6 s
 *    of ascent is ~43 fixes at 5 Hz, and each one is an absolute, non-drifting
 *    reference against which the drift above simply cannot accumulate. GPS is
 *    used through the ENTIRE flight here, de-weighted by phase rather than
 *    switched off.
 *
 * 3. GPS ALTITUDE IS ALMOST NOT TRUSTED AT ALL.
 *    Vertical stays baro + integrated accelerometer. GPS altitude enters with a
 *    15 m sigma floor - loose enough that it cannot move the solution, tight
 *    enough to catch a blocked static port or a dead sensor.
 *
 *    An earlier draft justified trusting it more, on the grounds that static-port
 *    error grows with v^2 while GPS has no airspeed term. That argument does not
 *    survive checking WHEN the error actually bites: the baro's dynamic error
 *    peaks at burnout, at 68 m/s, and falls to zero at apogee, because apogee is
 *    by definition where the vehicle is not moving. The single altitude number
 *    that matters most is measured at the exact moment the barometer is cleanest.
 *    Meanwhile GPS vertical error is 5-15 m, persistent, and does not care what
 *    the vehicle is doing. Feeding that into a 250 m flight buys noise.
 *
 * 4. GPS SPEED AND COURSE ARE THE HORIZONTAL VELOCITY CALIBRATION.
 *    This is where GPS earns its place. Ground speed and course over ground come
 *    from Doppler, not from differencing positions, so they are far more accurate
 *    than GPS position and completely immune to the INS drift they are correcting.
 *    They are what stop the lateral error in the table above from integrating.
 *    Course is meaningless when barely moving, so the measurement is gated on
 *    ground speed, and it is aged forward by the measured acceleration.
 *
 * 5. TRUST IS DRIVEN BY HDOP AND MEASURED ACCELERATION, NOT BY FLIGHT PHASE.
 *    HDOP is the dominant term and enters linearly - that is what a dilution of
 *    precision means. On top of it, sigma is inflated once measured specific
 *    force passes GPS_ACCEL_KNEE_G, because high g is when fix latency hurts most
 *    (150 ms at 40 m/s^2 is 6 m/s) and when the receiver's tracking loops are
 *    under the most stress.
 *
 *    Measured acceleration is a better trust signal than the flight state: it is
 *    continuous, it responds to what the vehicle is actually doing, and it does
 *    not depend on the state machine having called burnout correctly. It also
 *    gets the coast phase right for free - free fall reads ~0 g, which is benign
 *    for the receiver, so GPS is trusted fully there without a special case.
 *
 * 6. WHAT GPS STILL CANNOT DO HERE.
 *    NMEA carries no vertical velocity, so climb rate - the number the landing
 *    burn is solved from - comes entirely from baro + accelerometer. And lock can
 *    drop under boost vibration: nothing here depends on a fix arriving, every
 *    source is gated on quality, and outages are counted and reported afterwards.
 *
 * 7. BAROMETER TRUST IS SCHEDULED ON AIRSPEED AND FLIGHT PHASE.
 *    Flow over an imperfect static port reads low at speed, so apparent altitude
 *    reads high, and the error grows with v^2. Motor plume during boost and the
 *    landing burn pressurizes the base and is worse still. The measurement noise
 *    is therefore inflated as R = (sigma * phaseMultiplier)^2 + (k*v^2)^2 rather
 *    than the baro being trusted equally throughout.
 *
 * 8. THE ACCELEROMETER BIAS STATE IS FROZEN UNDER HIGH G.
 *    The bias state exists to absorb a constant offset; at 10-15 g what it would
 *    actually absorb is scale-factor error, which is proportional to acceleration
 *    and therefore wrong the moment the motor stops. Bias random walk is set to
 *    zero outside PAD/LANDED and the state is hard-clamped, exactly the way the
 *    library bounds its own learned gyro bias.
 *
 * 9. ATTITUDE CORRECTION SWITCHES ITSELF OFF, AND THAT IS CORRECT.
 *    AdvancedTriFusion gates the accelerometer with a Gaussian centred on 1 g
 *    (sigma 0.05). At 10 g of boost, and at ~0 g in coast, that gain is
 *    numerically zero, so attitude is pure gyro integration exactly when the
 *    accelerometer is not measuring gravity. This is the single most useful
 *    property of the library for rocketry and it needs no configuration. What
 *    this sketch adds is turning the magnetometer down during motor burns, where
 *    igniter current and the steel casing move the field.
 *
 * 10. FLIGHT STATE MACHINE.
 *    Process noise, ZUPT, bias learning, baro trust, GPS use and magnetometer
 *    trust are all scheduled from PAD/BOOST/COAST/DESCENT/LANDING_BURN/LANDED.
 *
 * 11. FLIGHT LOG IN RAM.
 *    Telemetry you did not record is telemetry you do not have. ~40 s at 100 Hz
 *    lands in a static buffer, including the seconds before launch, and is dumped
 *    as CSV after touchdown. The buffer freezes after landing so the flight
 *    cannot be overwritten while the rocket sits in a field.
 *
 * ===========================================================================
 * MOUNTING
 * ===========================================================================
 * Set ROCKET_MOUNT so that AFTER the library's axis remap, +Z points UP THE
 * ROCKET (out the nose). With the board lying flat across the airframe and its
 * Z axis along the body tube, that is ORIENTATION_Z_UP. Everything downstream -
 * launch detection, burnout detection, saturation checks - reads the axial
 * channel as the remapped Z, so getting this wrong breaks all of them.
 *
 * HARDWARE: Raspberry Pi Pico 2 (RP2350) + TriSense Pro (ICM on SPI CS17) +
 *           Quectel L76K on GP0/GP1. Core: Earle Philhower arduino-pico.
 * LIBRARY:  TinyGPSPlus by Mikal Hart.
 */

#if !defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_RP2350)
  #error "RocketPropulsiveLanding targets the RP2350/RP2040 on the Earle Philhower arduino-pico core."
#endif

#include <Arduino.h>
#include <TriSense.h>
#include <TinyGPSPlus.h>
#include <pico/mutex.h>
#include <math.h>

// ===========================================================================
// USER CONFIGURATION
// ===========================================================================

// ---- Wiring -----------------------------------------------------------------
#define IMU_CS_PIN            17
#define IMU_SPI_HZ            10000000UL
#define GPS_PIN_TX            0
#define GPS_PIN_RX            1

// ---- Vehicle ----------------------------------------------------------------
// After the remap, +Z must point up the rocket. See MOUNTING above.
#define ROCKET_MOUNT          ORIENTATION_Z_UP

// 4 kHz, and 20-bit FIFO. Solid motors are broadband vibration sources, and each
// navigation step publishes the MEAN of one batch of accelerometer samples, so a
// higher ODR directly buys a cleaner mean. 20-bit mode also pins the ranges at
// +/-16 g and +/-2000 dps, which is what a rocket needs anyway, and gives 16x the
// resolution during the near-zero-g coast where the numbers are small.
#define IMU_ODR               ODR_4KHZ

// ---- Site -------------------------------------------------------------------
#define MAGNETIC_DECLINATION  5.6f     // degrees East, positive

static float MAG_HARD_IRON[3] = { -46.02f, -0.85f, -46.00f };
static float MAG_SOFT_IRON[3][3] = {
  {  0.965f,  0.008f, -0.002f },
  {  0.008f,  0.981f,  0.139f },
  { -0.002f,  0.139f,  1.077f }
};

// On the TriSense Pro getOrientationDegrees() returns a clockwise compass
// heading. See GPS_INS_Localization.ino for the full note and the field check.
#define YAW_IS_COMPASS_HEADING 1

// ---- Flight detection -------------------------------------------------------
// Thresholds are on the AXIAL (remapped +Z) accelerometer channel unless stated.
#define LAUNCH_ACCEL_G        2.5f     // sustained axial g that means "lit"
#define LAUNCH_CONFIRM_MS     60       // ... for this long, to reject a knock
#define BURNOUT_ACCEL_G       0.5f     // axial g below which the motor is out
#define BURNOUT_CONFIRM_MS    100
#define MIN_BOOST_MS          250      // no burnout call before this
#define APOGEE_CONFIRM_MS     250      // climb rate negative for this long
#define MIN_APOGEE_AGL_M      20.0f    // below this, a negative rate is noise
#define BURN_DETECT_ACCEL_G   2.0f     // landing motor lit
#define BURN_CONFIRM_MS       60
#define LANDED_CONFIRM_MS     1500     // quiet, slow and low for this long
#define LANDED_MAX_AGL_M      15.0f
#define ACCEL_SATURATION_G    15.5f    // ICM-42688-P clips at 16 g

// ---- Landing burn solution --------------------------------------------------
// NET upward deceleration the landing motor produces: thrust/mass MINUS g, in
// m/s^2. Use AVERAGE thrust over the burn, not peak - the solve is open-loop and
// optimistic thrust is the failure mode that puts the rocket into the ground.
//
// Worked for 3.2 kg wet on a 270 Ns / 200 N / 1.7 s motor:
//     average thrust = 270/1.7                  = 158.8 N
//     propellant     = 270/(200*9.81)           = 0.14 kg, so dry mass 3.06 kg
//     net decel      = 158.8/3.06 - 9.81        = 42.1 m/s^2
// which stops the vehicle from 30 m/s in 10.7 m and from 50 m/s in 29.7 m.
//
// NOTE ON DESCENT SPEED: this motor carries 270 Ns, so it can remove at most
// 270/3.06 = 88 m/s of velocity with nothing left over. A 3.2 kg airframe falling
// ballistically reaches roughly 90 m/s, which needs the entire impulse and leaves
// no margin for error in ignition altitude. Plan the descent - drogue, airbrakes,
// whatever - so the burn starts below about 50 m/s.
#define LANDING_NET_DECEL_MS2 42.0f
#define IGNITER_DELAY_S       0.10f    // command to meaningful thrust

// ---- Rates ------------------------------------------------------------------
#define NAV_RATE_HZ           200      // Kalman predict rate
#define BARO_RATE_HZ          100      // barometer read rate (sensor runs 240 Hz)
#define LOG_RATE_HZ           100      // flight recorder rate
#define TELEMETRY_HZ          20

// ---- Flight recorder --------------------------------------------------------
// LOG_SECONDS * LOG_RATE_HZ * sizeof(LogRecord) bytes of SRAM. The default is
// about 144 KB out of the RP2350's 520 KB. The buffer is circular, so a long
// pad wait cannot overflow it, and it freezes after landing.
#define LOG_SECONDS           40
#define LOG_FREEZE_AFTER_S    10       // keep recording this long past touchdown

// ---- Filter tuning ----------------------------------------------------------
// Acceleration noise density, m/s^2/sqrt(Hz), scheduled by flight phase. What
// dominates is not sensor noise but residual attitude error projecting the
// thrust vector into the wrong axes: at 10 g, 1 degree of attitude error is
// 1.7 m/s^2 of phantom horizontal acceleration.
#define ACC_PSD_PAD           0.10f
#define ACC_PSD_BOOST         2.50f
#define ACC_PSD_COAST         0.50f
#define ACC_PSD_DESCENT       0.60f
#define ACC_PSD_BURN          2.50f
#define ACC_BIAS_RW           0.004f   // m/s^2/sqrt(s); zeroed outside PAD/LANDED
#define ACC_BIAS_MAX          1.0f     // hard clamp, m/s^2

// Barometer. R = (BARO_ALT_SIGMA * phase)^2 + (BARO_VEL_COEF * v^2)^2.
#define BARO_ALT_SIGMA        0.5f     // m, static-air noise
#define BARO_VEL_COEF         0.0015f  // m per (m/s)^2 of airspeed error
#define BARO_MULT_BOOST       12.0f    // plume and shock on the static ports
#define BARO_MULT_BURN        12.0f
#define BARO_MULT_COAST       2.0f

/*
 * GPS. Used throughout the flight, but with sharply different weight per axis:
 * horizontal position and velocity are the primary references, while altitude is
 * barely trusted at all. See points 3-5 in the header.
 *
 *      sigma_horizontal = GPS_UERE_M * HDOP * trust(accel)
 *      sigma_velocity   = GPS_VEL_SIGMA * HDOP * trust(accel)
 *      sigma_altitude   = max(the above * GPS_VDOP_FACTOR, GPS_ALT_SIGMA_MIN_M)
 */
#define GPS_UERE_M            2.5f     // m of horizontal error per unit HDOP
#define GPS_VEL_SIGMA         0.30f    // m/s at HDOP 1. Doppler-derived, hence small
#define GPS_VEL_MIN_MPS       2.0f     // below this, course over ground is noise
#define GPS_MIN_SATS          4
#define GPS_MAX_HDOP          6.0f     // hard reject above this
#define GPS_LATENCY_S         0.15f
#define GPS_OUTAGE_MS         800      // no usable fix for this long = an outage

// --- GPS altitude: deliberately almost ignored ------------------------------
// Vertical belongs to the barometer and the integrated accelerometer. The sigma
// floor is what makes this a blunder detector rather than a measurement: loose
// enough that it cannot pull the altitude solution around, tight enough that a
// blocked static port or a dead BMP580 still gets caught by the innovation gate.
// Set GPS_ALT_ENABLE to 0 to drop it entirely.
#define GPS_ALT_ENABLE        1
#define GPS_VDOP_FACTOR       4.0f     // vertical sigma relative to horizontal
#define GPS_ALT_SIGMA_MIN_M   15.0f    // ... but never tighter than this

/*
 * --- Dynamic trust: HDOP first, then measured acceleration ------------------
 *
 * HDOP is the dominant term and enters LINEARLY, because that is what a dilution
 * of precision is: it ranges 0.8 to the 6.0 reject threshold, so it can swing the
 * sigma by 7x on its own.
 *
 * On top of that, sigma is inflated once measured specific force passes the knee.
 * High g is when fix latency hurts most (150 ms at 40 m/s^2 is 6 m/s of velocity
 * error) and when the receiver's tracking loops are under the most stress.
 *
 * TUNE THE KNEE FOR YOUR VEHICLE. At 4.0 g a 3.2 kg airframe on a 200 N motor
 * (6.4 g peak, 5.1 g average) is de-weighted for most of the burn and trusted
 * fully everywhere else - including coast, which reads ~0 g in free fall and is
 * benign for the receiver. Raise the knee if your boost is gentler; lower it if
 * you see GPS fighting the filter under thrust.
 */
#define GPS_ACCEL_KNEE_G      4.0f     // no extra de-weighting below this
#define GPS_ACCEL_SLOPE       1.0f     // sigma multiplier added per g above it
#define GPS_ACCEL_MAX_MULT    6.0f     // cap on the multiplier

#define ZUPT_VEL_SIGMA        0.02f
#define ZUPT_ACC_TOL_G        0.04f
#define ZUPT_GYRO_TOL_DPS     2.0f
#define ZUPT_HOLD_MS          300

#define GATE_SIGMA            4.0f
#define GPS_MAX_REJECTS       12
#define BARO_TRACK_TAU_S      60.0f

// ===========================================================================
// L76K PCAS COMMANDS (checksums computed at runtime by gpsSend)
// ===========================================================================
#define PCAS_BAUD_115200   "PCAS01,5"
#define PCAS_RATE_1HZ      "PCAS02,1000"
// 5 Hz is the ceiling on the Seeed XIAO L76K carrier. Over an 8.6 s ascent that
// is ~43 fixes, which is ample: what matters is that each one is an absolute
// reference, not how densely they arrive. It does mean the inter-fix interval is
// 200 ms, so the latency compensation in applyGpsFix() is doing real work rather
// than being a rounding correction - do not remove it.
#define PCAS_RATE_5HZ      "PCAS02,200"
#define PCAS_NMEA_MINIMAL  "PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0,,,,0"
#define PCAS_NMEA_DEFAULT  "PCAS03,1,1,1,1,1,1,0,0,0,0,,,0,0,,,,0"

#define GPS_FAST_BAUD      115200UL
#define GPS_BOOT_BAUD      9600UL

// ===========================================================================
// TYPES  (every type used by a function signature must precede the first
// function, because the Arduino builder inserts generated prototypes there)
// ===========================================================================

enum FlightState : uint8_t {
  FS_INIT = 0,      // booting, calibrating
  FS_PAD,           // armed and stationary
  FS_BOOST,         // ascent motor burning
  FS_COAST,         // unpowered ascent
  FS_DESCENT,       // past apogee, unpowered
  FS_LANDING_BURN,  // landing motor burning
  FS_LANDED
};

struct GpsFix {
  double   lat, lon;
  float    altMSL;
  float    speedMps;
  float    courseDeg;
  float    hdop;
  uint8_t  sats;
  bool     haveAlt;
  bool     haveVel;
  uint32_t stampMs;
  uint32_t seq;
};

struct NavOut {
  float    roll, pitch, yaw;
  float    q[4];
  float    axialG;                // remapped +Z accelerometer, g
  float    aN, aE, aUp;           // m/s^2, gravity and estimated bias removed
  float    vN, vE, climbRate;     // m/s, climbRate positive up
  float    pN, pE, altAGL;        // m, altitude above the pad
  double   lat, lon;
  float    altMSL;
  float    sigmaAlt, sigmaClimb, sigmaPosH;
  float    biasUp;
  float    fusionHz;
  // Flight summary
  uint8_t  state;
  uint32_t stateMs;               // ms in the current state
  uint32_t flightMs;              // ms since launch detection
  float    maxAltAGL;
  float    maxSpeedUp;
  float    maxAxialG;
  float    burnAltAGL;            // solved ignition altitude for the landing motor
  float    burnMarginM;           // altAGL - burnAltAGL; <= 0 means light it now
  uint32_t accelSatCount;
  uint32_t gpsOutages;            // times the fix went stale for GPS_OUTAGE_MS
  uint32_t maxGpsGapMs;           // longest interval without a usable fix
  uint32_t fifoOverflows;
  uint8_t  sats;
  float    hdop;
  bool     gpsValid;
  bool     zupt;
  bool     logFrozen;
  uint32_t logCount;
};

// 36 bytes. Fixed point throughout - at 100 Hz for 40 s the difference between
// this and a float record is 60 KB of SRAM.
struct LogRecord {
  uint32_t tMs;
  int16_t  roll, pitch, yaw;      // 0.1 deg
  int16_t  ax, ay, az;            // 0.001 g   (body, after remap)
  int16_t  gx, gy, gz;            // 0.1 dps
  int16_t  altAGL;                // 0.1 m
  int16_t  climbRate;             // 0.1 m/s
  int16_t  vN, vE;                // 0.1 m/s
  int16_t  pN, pE;                // 0.1 m
  uint8_t  state;
  uint8_t  flags;                 // bit0 sat, bit1 gps, bit2 zupt, bit3 fifo
};

#define LOG_FLAG_ACCEL_SAT  0x01
#define LOG_FLAG_GPS_VALID  0x02
#define LOG_FLAG_ZUPT       0x04
#define LOG_FLAG_FIFO_OVF   0x08

/*
 * One navigation axis: x = [ position, velocity, acceleration bias ].
 * Identical structure to GPS_INS_Localization.ino - see that sketch's header for
 * why three 3-state filters are an exact factorization of one 9-state filter.
 */
struct AxisKF {
  float x[3];
  float P[3][3];
};

// ===========================================================================
// CROSS-CORE SHARED STATE
// ===========================================================================
auto_init_mutex(g_gpsMtx);
auto_init_mutex(g_navMtx);

static GpsFix        g_gpsShared;
static NavOut        g_navShared;
static volatile bool g_navReady     = false;
static volatile bool g_sensorFail   = false;
static volatile bool g_resetRequest = false;
static volatile bool g_dumpRequest  = false;
static volatile bool g_rawNmeaDebug = false;

// Flight recorder. Written by core 0, read by core 1 only after g_logFrozen.
#define LOG_CAPACITY  ((uint32_t)LOG_SECONDS * LOG_RATE_HZ)
static LogRecord     g_log[LOG_CAPACITY];
static volatile uint32_t g_logWrite  = 0;   // total records ever written
static volatile bool     g_logFrozen = false;

// ===========================================================================
// CORE 0 STATE
// ===========================================================================

TriSense          sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

static const float DEG2RAD = 0.017453292519943295f;
static const float RAD2DEG = 57.29577951308232f;
static const float GRAV_NOMINAL = 9.80665f;

static AxisKF kfN, kfE, kfD;      // NED internally; altitude AGL = -kfD.x[0]

static FlightState flightState   = FS_INIT;
static uint32_t    stateEnterMs  = 0;
static uint32_t    launchMs      = 0;
static uint32_t    landedMs      = 0;
static uint32_t    candidateMs   = 0;   // when the pending transition first held

static float    maxAltAGL     = 0.0f;
static float    maxSpeedUp    = 0.0f;
static float    maxAxialG     = 0.0f;
static uint32_t accelSatCount = 0;

// Local tangent plane, origin at the pad.
static bool   originSet       = false;
static double originLat       = 0.0, originLon = 0.0;
static float  metersPerRadLat = 6367000.0f;
static float  metersPerRadLon = 6367000.0f;

/*
 * VERTICAL DATUM. The Down axis is measured against the PAD, in the barometer's
 * own pressure-altitude scale:
 *
 *     padPressureAlt   pressure altitude at reset, so AGL = -kfD.x[0] exactly
 *     padAltMSL        the same point's altitude MSL, learned from GPS
 *
 * Keeping the D channel in the pressure scale makes the barometric measurement
 * a pure difference - weather offset and the 101325 Pa reference cancel - and it
 * works before GPS has a fix. padAltMSL exists only to convert GPS altitude into
 * that scale and to report MSL, and is trained on the ground where both sources
 * are static and honest.
 */
static float    padPressureAlt = 0.0f;
static float    padAltMSL      = 0.0f;
static bool     padAltMSLValid = false;
static float    lastBaroAlt    = 0.0f;
static uint32_t lastBaroUs     = 0;

// GPS health, reported after the flight - this is the data that tells you
// whether to trust GPS aiding on the next one.
static uint32_t gpsOutages     = 0;
static uint32_t maxGpsGapMs    = 0;
static uint32_t gpsLastGoodMs  = 0;
static bool     gpsInOutage    = false;

static float    accSum[3] = {0.0f, 0.0f, 0.0f};
static uint32_t accCount  = 0;

static uint32_t lastNavUs      = 0;
static uint32_t lastLogUs      = 0;
static uint32_t consumedFixSeq = 0;
static uint8_t  gpsRejects     = 0;
static uint32_t lastGpsUseMs   = 0;

static uint32_t quietSinceMs = 0;
static bool     zuptActive   = false;

// ===========================================================================
// CORE 1 STATE
// ===========================================================================

static TinyGPSPlus gps;
static uint32_t    gpsBaud       = GPS_BOOT_BAUD;
static uint32_t    lastPublishMs = 0;
static uint32_t    lastPrintMs   = 0;
static uint8_t     lastShownState = 0xFF;
static bool        summaryShown   = false;
static bool        sensorFailShown = false;

// ===========================================================================
// KALMAN FILTER
// ===========================================================================

static void kfReset(AxisKF &k, float p0, float v0, float sigP, float sigV, float sigB) {
  k.x[0] = p0; k.x[1] = v0; k.x[2] = 0.0f;
  for (uint8_t i = 0; i < 3; i++)
    for (uint8_t j = 0; j < 3; j++) k.P[i][j] = 0.0f;
  k.P[0][0] = sigP * sigP;
  k.P[1][1] = sigV * sigV;
  k.P[2][2] = sigB * sigB;
}

// P <- F P F^T + Q, F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]].
// biasRW is the bias random-walk density, passed in so the flight state machine
// can zero it under thrust (see FROZEN BIAS in the header).
static void kfPredict(AxisKF &k, float u, float dt, float accPsd, float biasRW) {
  const float h = 0.5f * dt * dt;
  const float a = u - k.x[2];

  k.x[0] += k.x[1] * dt + a * h;
  k.x[1] += a * dt;

  float A[3][3];
  for (uint8_t j = 0; j < 3; j++) {
    A[0][j] = k.P[0][j] + dt * k.P[1][j] - h * k.P[2][j];
    A[1][j] = k.P[1][j] - dt * k.P[2][j];
    A[2][j] = k.P[2][j];
  }
  for (uint8_t i = 0; i < 3; i++) {
    const float a0 = A[i][0], a1 = A[i][1], a2 = A[i][2];
    k.P[i][0] = a0;
    k.P[i][1] = dt * a0 + a1;
    k.P[i][2] = -h * a0 - dt * a1 + a2;
  }

  const float qa  = accPsd * accPsd;
  const float qb  = biasRW * biasRW;
  const float dt2 = dt * dt;
  k.P[0][0] += qa * dt2 * dt / 3.0f;
  k.P[0][1] += qa * dt2 * 0.5f;
  k.P[1][0] += qa * dt2 * 0.5f;
  k.P[1][1] += qa * dt;
  k.P[2][2] += qb * dt;
}

// Scalar update against a unit measurement row - no matrix inversion anywhere.
static bool kfCorrect(AxisKF &k, uint8_t idx, float z, float R, float gateSigma) {
  const float S = k.P[idx][idx] + R;
  if (!(S > 0.0f)) return false;
  const float y = z - k.x[idx];

  if (gateSigma > 0.0f && (y * y) > (gateSigma * gateSigma * S)) return false;

  const float invS = 1.0f / S;
  const float K0 = k.P[0][idx] * invS;
  const float K1 = k.P[1][idx] * invS;
  const float K2 = k.P[2][idx] * invS;

  k.x[0] += K0 * y;
  k.x[1] += K1 * y;
  k.x[2] += K2 * y;

  const float r0 = k.P[idx][0], r1 = k.P[idx][1], r2 = k.P[idx][2];
  k.P[0][0] -= K0 * r0; k.P[0][1] -= K0 * r1; k.P[0][2] -= K0 * r2;
  k.P[1][0] -= K1 * r0; k.P[1][1] -= K1 * r1; k.P[1][2] -= K1 * r2;
  k.P[2][0] -= K2 * r0; k.P[2][1] -= K2 * r1; k.P[2][2] -= K2 * r2;

  // Single precision loses symmetry over millions of updates, and an asymmetric
  // P eventually goes indefinite and takes the filter with it.
  k.P[0][1] = k.P[1][0] = 0.5f * (k.P[0][1] + k.P[1][0]);
  k.P[0][2] = k.P[2][0] = 0.5f * (k.P[0][2] + k.P[2][0]);
  k.P[1][2] = k.P[2][1] = 0.5f * (k.P[1][2] + k.P[2][1]);
  return true;
}

// Bounded bias, for the same reason the library bounds its learned gyro bias:
// one bad measurement burst must not wind the integrator somewhere it cannot
// come back from, and there is no honest bias information available in flight.
static void kfClampBias(AxisKF &k, float maxAbs) {
  if (k.x[2] >  maxAbs) k.x[2] =  maxAbs;
  if (k.x[2] < -maxAbs) k.x[2] = -maxAbs;
}

static float kfSigma(const AxisKF &k, uint8_t idx) {
  return k.P[idx][idx] > 0.0f ? sqrtf(k.P[idx][idx]) : 0.0f;
}

// ===========================================================================
// FLIGHT PHASE SCHEDULING
// ===========================================================================

static float phaseAccPsd(FlightState s) {
  switch (s) {
    case FS_BOOST:        return ACC_PSD_BOOST;
    case FS_COAST:        return ACC_PSD_COAST;
    case FS_DESCENT:      return ACC_PSD_DESCENT;
    case FS_LANDING_BURN: return ACC_PSD_BURN;
    default:              return ACC_PSD_PAD;
  }
}

// Bias may only be learned when the vehicle is at rest and the accelerometer is
// genuinely measuring gravity plus a constant offset.
static bool phaseLearnsBias(FlightState s) {
  return (s == FS_PAD) || (s == FS_LANDED) || (s == FS_INIT);
}

static bool phaseAllowsZupt(FlightState s) {
  // NOT during descent: a rocket at terminal velocity reads 1 g with a quiet
  // gyro, which is indistinguishable from sitting still by that test alone.
  return (s == FS_PAD) || (s == FS_LANDED);
}

/*
 * GPS sigma multiplier from measured specific force, in g.
 *
 * GPS is never switched off - it is the only non-drifting horizontal reference
 * the vehicle has, and switching it off is how the lateral drift in the header
 * table gets to accumulate unchallenged. It is only ever trusted less.
 *
 * Driving this from measured acceleration rather than from the flight state is
 * deliberate: it is continuous, it tracks what the vehicle is actually doing, and
 * it does not depend on the state machine having called burnout correctly. Free
 * fall reads ~0 g and is benign for the receiver, so coast gets full trust with
 * no special case.
 */
static float gpsAccelTrust(float accelG) {
  if (accelG <= GPS_ACCEL_KNEE_G) return 1.0f;
  const float m = 1.0f + GPS_ACCEL_SLOPE * (accelG - GPS_ACCEL_KNEE_G);
  return (m > GPS_ACCEL_MAX_MULT) ? (float)GPS_ACCEL_MAX_MULT : m;
}

static float phaseBaroMultiplier(FlightState s) {
  switch (s) {
    case FS_BOOST:        return BARO_MULT_BOOST;
    case FS_LANDING_BURN: return BARO_MULT_BURN;
    case FS_COAST:        return BARO_MULT_COAST;
    case FS_DESCENT:      return BARO_MULT_COAST;
    default:              return 1.0f;
  }
}

// ===========================================================================
// CORE 0 - NAVIGATION
// ===========================================================================

static float wrap180(float d) {
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static float localGravity(double latDeg, float altM) {
  const float s  = sinf((float)latDeg * DEG2RAD);
  const float s2 = s * s;
  const float g  = 9.7803253359f * (1.0f + 0.00193185265241f * s2) /
                   sqrtf(1.0f - 0.00669437999013f * s2);
  return g - 3.086e-6f * altM;
}

/*
 * Ignition altitude for a constant-thrust landing burn.
 *
 * Falling at v with the motor producing a NET upward deceleration a, the burn
 * needs v^2 / (2a) of altitude. Ignition is not instant, so during IGNITER_DELAY_S
 * the rocket keeps falling and keeps accelerating: it drops a further
 * v*td + g*td^2/2 and arrives at the start of thrust doing v + g*td.
 *
 * Returned altitude is AGL and does NOT include a safety margin. Add your own -
 * this is an open-loop solve from an estimated state, and both the estimate and
 * the motor's real thrust curve have error bars.
 */
static float burnAltitudeAGL(float descentSpeed) {
  if (descentSpeed <= 0.0f) return 0.0f;
  const float td = IGNITER_DELAY_S;
  const float vIg = descentSpeed + GRAV_NOMINAL * td;
  const float aNet = (LANDING_NET_DECEL_MS2 > 1.0f) ? LANDING_NET_DECEL_MS2 : 1.0f;
  return (vIg * vIg) / (2.0f * aNet) + descentSpeed * td + 0.5f * GRAV_NOMINAL * td * td;
}

/*
 * Hook for the landing burn command. DELIBERATELY EMPTY.
 *
 * If you wire pyro here, everything that actually makes an ignition circuit safe
 * still has to exist around it: a physical arming switch in the firing loop, a
 * continuity check, a state-machine interlock so it cannot fire before apogee has
 * been declared, an altitude floor, a maximum-fire-duration timeout, and a
 * ground test with the motor replaced by a lamp. None of that belongs in an
 * example sketch, and half of it is worse than none.
 */
static void onLandingBurnGo(float altAGL, float descentSpeed) {
  (void)altAGL;
  (void)descentSpeed;
}

static void setOrigin(double lat, double lon, float altMSL) {
  originLat = lat;
  originLon = lon;

  const double latRad = lat * (double)DEG2RAD;
  const double sinLat = sin(latRad);
  const double e2     = 6.69437999014e-3;
  const double w      = 1.0 - e2 * sinLat * sinLat;
  const double a      = 6378137.0;
  metersPerRadLon = (float)((a / sqrt(w)) * cos(latRad));
  metersPerRadLat = (float)((a * (1.0 - e2)) / (w * sqrt(w)));

  originSet = true;
  fusion.setLocalGravity(localGravity(lat, altMSL));
}

// Magnetometer and gyro-bias trust, scheduled per phase. Called on transitions
// only, so it costs nothing in the loop.
static void applyPhaseSensorTrust(FlightState s) {
  const bool burning = (s == FS_BOOST) || (s == FS_LANDING_BURN);

  // Igniter current and a steel motor casing both move the local field, and the
  // rocket is rotating fastest exactly when it is under thrust.
  fusion.setMaxGains(0.1f, burning ? 0.0f : 0.1f);

  // In-flight gyro drift learning is driven by the accel/mag corrections, which
  // are meaningless under thrust. The pad-learned bias is the one to keep.
  fusion.setDynamicGyroBias(!burning, 0.0001f);
}

static void enterState(FlightState s, uint32_t nowMs) {
  flightState  = s;
  stateEnterMs = nowMs;
  candidateMs  = 0;
  applyPhaseSensorTrust(s);
  if (s == FS_BOOST)  launchMs = nowMs;
  if (s == FS_LANDED) landedMs = nowMs;
}

static void resetNavigation() {
  kfReset(kfN, 0.0f, 0.0f, 50.0f, 1.0f, 0.3f);
  kfReset(kfE, 0.0f, 0.0f, 50.0f, 1.0f, 0.3f);
  kfReset(kfD, 0.0f, 0.0f,  3.0f, 0.5f, 0.3f);
  originSet      = false;
  padAltMSL      = 0.0f;
  padAltMSLValid = false;
  gpsRejects     = 0;
  lastGpsUseMs   = 0;
  gpsOutages     = 0;
  maxGpsGapMs    = 0;
  gpsLastGoodMs  = 0;
  gpsInOutage    = false;
  accSum[0] = accSum[1] = accSum[2] = 0.0f;
  accCount      = 0;
  quietSinceMs  = 0;
  zuptActive    = false;
  maxAltAGL     = 0.0f;
  maxSpeedUp    = 0.0f;
  maxAxialG     = 0.0f;
  accelSatCount = 0;
  launchMs      = 0;
  landedMs      = 0;

  // The pad is the datum: pressure altitude at reset is 0 m AGL, so the vertical
  // channel is live before GPS ever gets a fix.
  padPressureAlt = sensor.bmp.readAltitude();
  lastBaroAlt    = padPressureAlt;

  g_logWrite  = 0;
  g_logFrozen = false;

  enterState(FS_PAD, millis());
}

static bool takeGpsFix(GpsFix &out) {
  if (!mutex_try_enter(&g_gpsMtx, NULL)) return false;
  const bool fresh = (g_gpsShared.seq != 0) && (g_gpsShared.seq != consumedFixSeq);
  if (fresh) {
    out = g_gpsShared;
    consumedFixSeq = out.seq;
  }
  mutex_exit(&g_gpsMtx);
  return fresh;
}

// aN/aE are the current world-frame accelerations, used to age the GPS velocity
// forward the same way position is aged forward. accelG is the measured specific
// force magnitude in g, which drives the dynamic trust multiplier.
static void applyGpsFix(const GpsFix &fix, float aN, float aE, float accelG) {
  if (fix.sats < GPS_MIN_SATS || fix.hdop > GPS_MAX_HDOP) return;

  const uint32_t nowMs = millis();
  const float    hdop  = fix.hdop < 0.8f ? 0.8f : fix.hdop;

  if (!originSet) {
    setOrigin(fix.lat, fix.lon, fix.haveAlt ? fix.altMSL : padPressureAlt);
    if (fix.haveAlt) {
      padAltMSL      = fix.altMSL;
      padAltMSLValid = true;
    }
    lastGpsUseMs  = nowMs;
    gpsLastGoodMs = nowMs;
    return;
  }

  // HDOP (linear, dominant) x dynamic stress from measured acceleration.
  const float mult = gpsAccelTrust(accelG);

  // Total age of the measurement: the receiver's own processing lag plus however
  // long the fix has been sitting in the cross-core mailbox.
  float lag = GPS_LATENCY_S + (float)(uint32_t)(nowMs - fix.stampMs) * 0.001f;
  if (lag > 1.0f) lag = 1.0f;

  // --- Horizontal position: the primary horizontal reference -----------------
  const double dLat = fix.lat - originLat;
  const double dLon = fix.lon - originLon;
  float zN = (float)(dLat * (double)DEG2RAD) * metersPerRadLat;
  float zE = (float)(dLon * (double)DEG2RAD) * metersPerRadLon;
  zN += kfN.x[1] * lag;
  zE += kfE.x[1] * lag;

  const float sigP = GPS_UERE_M * hdop * mult;
  const float Rpos = sigP * sigP;

  const bool okN = kfCorrect(kfN, 0, zN, Rpos, GATE_SIGMA);
  const bool okE = kfCorrect(kfE, 0, zE, Rpos, GATE_SIGMA);

  if (okN && okE) {
    gpsRejects = 0;
  } else if (++gpsRejects >= GPS_MAX_REJECTS) {
    // Sustained rejection under boost means the INS drifted, not that the GPS
    // lied - drift is exactly what the lateral-error table predicts. Re-anchor.
    kfReset(kfN, zN, kfN.x[1], sigP, 3.0f, 0.3f);
    kfReset(kfE, zE, kfE.x[1], sigP, 3.0f, 0.3f);
    gpsRejects = 0;
  }

  // --- Horizontal velocity: the calibration that matters ---------------------
  // Ground speed and course over ground are Doppler-derived, not differenced
  // positions, so they are far more accurate than GPS position AND completely
  // independent of the INS drift they are correcting. This is what stops the
  // lateral error in the header table from integrating.
  //
  // Two caveats handled here: course is meaningless when barely moving, hence
  // the speed gate; and the measurement is 150+ ms old, hence aging it forward
  // by the acceleration the IMU measured in the meantime.
  if (fix.haveVel && fix.speedMps > GPS_VEL_MIN_MPS) {
    const float cr = fix.courseDeg * DEG2RAD;
    const float sv = GPS_VEL_SIGMA * hdop * mult;
    const float Rv = sv * sv;
    kfCorrect(kfN, 1, fix.speedMps * cosf(cr) + aN * lag, Rv, GATE_SIGMA);
    kfCorrect(kfE, 1, fix.speedMps * sinf(cr) + aE * lag, Rv, GATE_SIGMA);
  }

#if GPS_ALT_ENABLE
  // --- Altitude: a blunder detector, not a measurement -----------------------
  // The sigma floor is the whole point. Vertical belongs to the barometer and the
  // integrated accelerometer, which are better than GPS by an order of magnitude
  // over a 250 m flight - and the apogee number in particular is measured at the
  // one moment the barometer is cleanest, because apogee is where airspeed is
  // zero. What this correction is here for is a blocked static port or a dead
  // sensor, where the baro walks away and nothing else would notice.
  if (fix.haveAlt && padAltMSLValid) {
    float sigV = GPS_UERE_M * hdop * GPS_VDOP_FACTOR * mult;
    if (sigV < GPS_ALT_SIGMA_MIN_M) sigV = GPS_ALT_SIGMA_MIN_M;
    const float zD = -(fix.altMSL - padAltMSL) + kfD.x[1] * lag;
    kfCorrect(kfD, 0, zD, sigV * sigV, GATE_SIGMA);
  }
#endif

  // --- MSL datum -------------------------------------------------------------
  // Only trained on the ground. In flight both sources are degraded, and moving
  // the datum during the part of the flight that matters would be self-inflicted.
  if (fix.haveAlt && (flightState == FS_PAD || flightState == FS_LANDED)) {
    if (!padAltMSLValid) {
      padAltMSL      = fix.altMSL;
      padAltMSLValid = true;
    } else {
      const float dtFix = (float)(uint32_t)(nowMs - lastGpsUseMs) * 0.001f;
      float kGain = (dtFix > 0.0f && dtFix < 5.0f) ? (dtFix / BARO_TRACK_TAU_S) : 0.01f;
      if (kGain > 0.2f) kGain = 0.2f;
      // The pad does not move, so what is being tracked is the receiver's slow
      // vertical wander, not real altitude change.
      padAltMSL += kGain * ((fix.altMSL + kfD.x[0]) - padAltMSL);
    }
  }

  lastGpsUseMs  = nowMs;
  gpsLastGoodMs = nowMs;
}

// Flight state machine. `axialG` is the remapped +Z accelerometer channel, i.e.
// along the rocket; `climbRate` is the filtered vertical velocity, positive up.
static void updateFlightState(float axialG, float totalG, float gyroSum,
                              float altAGL, float climbRate, uint32_t nowMs) {
  const uint32_t inState = (uint32_t)(nowMs - stateEnterMs);

  switch (flightState) {
    case FS_INIT:
      break;

    case FS_PAD:
      // Sustained axial thrust. Confirming over LAUNCH_CONFIRM_MS rejects a
      // knock against the rail, someone bumping the airframe, or a single
      // vibration spike; a real motor holds its acceleration for a second or more.
      if (axialG > LAUNCH_ACCEL_G) {
        if (candidateMs == 0) candidateMs = nowMs ? nowMs : 1;
        if ((uint32_t)(nowMs - candidateMs) >= LAUNCH_CONFIRM_MS) enterState(FS_BOOST, nowMs);
      } else {
        candidateMs = 0;
      }
      break;

    case FS_BOOST:
      if (inState >= MIN_BOOST_MS && axialG < BURNOUT_ACCEL_G) {
        if (candidateMs == 0) candidateMs = nowMs ? nowMs : 1;
        if ((uint32_t)(nowMs - candidateMs) >= BURNOUT_CONFIRM_MS) enterState(FS_COAST, nowMs);
      } else {
        candidateMs = 0;
      }
      break;

    case FS_COAST:
      // Apogee. Taken from the FILTERED climb rate rather than from a raw baro
      // derivative: near apogee the vertical velocity is small and a differenced
      // barometer is almost pure noise there, which is precisely where a false
      // apogee call would be most expensive.
      if (climbRate < 0.0f && altAGL > MIN_APOGEE_AGL_M) {
        if (candidateMs == 0) candidateMs = nowMs ? nowMs : 1;
        if ((uint32_t)(nowMs - candidateMs) >= APOGEE_CONFIRM_MS) enterState(FS_DESCENT, nowMs);
      } else {
        candidateMs = 0;
      }
      break;

    case FS_DESCENT:
      if (axialG > BURN_DETECT_ACCEL_G) {
        if (candidateMs == 0) candidateMs = nowMs ? nowMs : 1;
        if ((uint32_t)(nowMs - candidateMs) >= BURN_CONFIRM_MS) {
          enterState(FS_LANDING_BURN, nowMs);
          break;
        }
      } else {
        candidateMs = 0;
      }
      // Fallback: a ballistic arrival still has to be called landed, or the log
      // never freezes and the recorder overwrites the flight.
      if (altAGL < LANDED_MAX_AGL_M && fabsf(climbRate) < 1.0f &&
          fabsf(totalG - 1.0f) < 0.15f && gyroSum < 20.0f) {
        enterState(FS_LANDED, nowMs);
      }
      break;

    case FS_LANDING_BURN:
    case FS_LANDED:
      if (flightState == FS_LANDING_BURN) {
        const bool quiet = fabsf(totalG - 1.0f) < 0.12f && gyroSum < 15.0f &&
                           fabsf(climbRate) < 1.0f && altAGL < LANDED_MAX_AGL_M;
        if (quiet) {
          if (candidateMs == 0) candidateMs = nowMs ? nowMs : 1;
          if ((uint32_t)(nowMs - candidateMs) >= LANDED_CONFIRM_MS) enterState(FS_LANDED, nowMs);
        } else {
          candidateMs = 0;
        }
      }
      break;
  }
}

static void publishNav(float roll, float pitch, float yaw, float axialG,
                       float aN, float aE, float aUp, uint32_t nowMs) {
  NavOut n;

  n.roll = roll; n.pitch = pitch; n.yaw = yaw;
  for (uint8_t i = 0; i < 4; i++) n.q[i] = (float)fusion.q[i];

  n.axialG = axialG;
  n.aN = aN - kfN.x[2];
  n.aE = aE - kfE.x[2];
  n.aUp = aUp + kfD.x[2];          // kfD is Down-positive, so its bias inverts
  n.vN = kfN.x[1];
  n.vE = kfE.x[1];
  n.climbRate = -kfD.x[1];
  n.pN = kfN.x[0];
  n.pE = kfE.x[0];
  n.altAGL = -kfD.x[0];

  if (originSet) {
    n.lat = originLat + (double)(kfN.x[0] / metersPerRadLat) * (double)RAD2DEG;
    n.lon = originLon + (double)(kfE.x[0] / metersPerRadLon) * (double)RAD2DEG;
  } else {
    n.lat = 0.0;
    n.lon = 0.0;
  }
  n.altMSL = (padAltMSLValid ? padAltMSL : padPressureAlt) + n.altAGL;

  n.sigmaAlt   = kfSigma(kfD, 0);
  n.sigmaClimb = kfSigma(kfD, 1);
  n.sigmaPosH  = 0.5f * (kfSigma(kfN, 0) + kfSigma(kfE, 0));
  n.biasUp     = -kfD.x[2];
  n.fusionHz   = fusion.getActualFusionHz();

  n.state      = (uint8_t)flightState;
  n.stateMs    = (uint32_t)(nowMs - stateEnterMs);
  n.flightMs   = launchMs ? (uint32_t)(nowMs - launchMs) : 0;
  n.maxAltAGL  = maxAltAGL;
  n.maxSpeedUp = maxSpeedUp;
  n.maxAxialG  = maxAxialG;

  const float descentSpeed = -n.climbRate;
  n.burnAltAGL  = burnAltitudeAGL(descentSpeed);
  n.burnMarginM = n.altAGL - n.burnAltAGL;

  n.accelSatCount = accelSatCount;
  n.gpsOutages    = gpsOutages;
  n.maxGpsGapMs   = maxGpsGapMs;
  n.fifoOverflows = sensor.imu.getFIFOOverflowCount();
  n.zupt          = zuptActive;
  n.logFrozen     = g_logFrozen;
  n.logCount      = g_logWrite < LOG_CAPACITY ? g_logWrite : LOG_CAPACITY;

  n.sats     = 0;
  n.hdop     = 99.0f;
  n.gpsValid = false;
  if (mutex_try_enter(&g_gpsMtx, NULL)) {
    if (g_gpsShared.seq != 0) {
      n.sats     = g_gpsShared.sats;
      n.hdop     = g_gpsShared.hdop;
      n.gpsValid = (uint32_t)(nowMs - g_gpsShared.stampMs) < 3000;
    }
    mutex_exit(&g_gpsMtx);
  }

  if (mutex_try_enter(&g_navMtx, NULL)) {
    g_navShared = n;
    mutex_exit(&g_navMtx);
  }
}

static int16_t clamp16(float v) {
  if (v >  32767.0f) return  32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)v;
}

static void logSample(float roll, float pitch, float yaw, bool accelSat,
                      bool gpsValid, bool fifoOvf, uint32_t nowMs) {
  if (g_logFrozen) return;

  LogRecord &r = g_log[g_logWrite % LOG_CAPACITY];
  r.tMs   = nowMs;
  r.roll  = clamp16(roll  * 10.0f);
  r.pitch = clamp16(pitch * 10.0f);
  r.yaw   = clamp16(yaw   * 10.0f);
  r.ax    = clamp16((float)fusion.lastAx * 1000.0f);
  r.ay    = clamp16((float)fusion.lastAy * 1000.0f);
  r.az    = clamp16((float)fusion.lastAz * 1000.0f);
  r.gx    = clamp16((float)fusion.lastGx * 10.0f);
  r.gy    = clamp16((float)fusion.lastGy * 10.0f);
  r.gz    = clamp16((float)fusion.lastGz * 10.0f);
  r.altAGL    = clamp16(-kfD.x[0] * 10.0f);
  r.climbRate = clamp16(-kfD.x[1] * 10.0f);
  r.vN    = clamp16(kfN.x[1] * 10.0f);
  r.vE    = clamp16(kfE.x[1] * 10.0f);
  r.pN    = clamp16(kfN.x[0] * 10.0f);
  r.pE    = clamp16(kfE.x[0] * 10.0f);
  r.state = (uint8_t)flightState;
  r.flags = (accelSat ? LOG_FLAG_ACCEL_SAT : 0) |
            (gpsValid ? LOG_FLAG_GPS_VALID : 0) |
            (zuptActive ? LOG_FLAG_ZUPT : 0) |
            (fifoOvf  ? LOG_FLAG_FIFO_OVF : 0);

  g_logWrite++;
}

static void navStep(float dt) {
  // --- 1. Mean body-frame linear acceleration since the last step ------------
  const float inv = 1.0f / (float)accCount;
  const float bx = accSum[0] * inv;
  const float by = accSum[1] * inv;
  const float bz = accSum[2] * inv;
  accSum[0] = accSum[1] = accSum[2] = 0.0f;
  accCount = 0;

  float roll, pitch, yaw;
  fusion.getOrientationDegrees(roll, pitch, yaw);

  // --- 2. Body -> level -> North/East/Up -------------------------------------
  // Roll and pitch come from gravity and are unambiguous; only the heading
  // rotation depends on the yaw convention. See GPS_INS_Localization.ino for why
  // this is done here instead of via getGlobalAcceleration().
  const float phi = roll * DEG2RAD, theta = pitch * DEG2RAD;
  const float cp = cosf(phi),   sp = sinf(phi);
  const float ct = cosf(theta), st = sinf(theta);

  const float aFwd  =  ct * bx + st * sp * by + st * cp * bz;
  const float aLeft =            cp * by      - sp * bz;
  const float aUp   = -st * bx + ct * sp * by + ct * cp * bz;

#if YAW_IS_COMPASS_HEADING
  const float psi = yaw * DEG2RAD;
#else
  const float psi = (360.0f - yaw) * DEG2RAD;
#endif
  const float cpsi = cosf(psi), spsi = sinf(psi);

  const float aN =  aFwd * cpsi + aLeft * spsi;
  const float aE =  aFwd * spsi - aLeft * cpsi;
  const float aD = -aUp;

  // --- 3. Phase-scheduled predict --------------------------------------------
  const float psd    = phaseAccPsd(flightState);
  const float biasRW = phaseLearnsBias(flightState) ? (float)ACC_BIAS_RW : 0.0f;

  kfPredict(kfN, aN, dt, psd, biasRW);
  kfPredict(kfE, aE, dt, psd, biasRW);
  kfPredict(kfD, aD, dt, psd, biasRW);

  // --- 4. Barometric altitude -------------------------------------------------
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastBaroUs) >= (1000000UL / BARO_RATE_HZ)) {
    lastBaroUs = nowUs;
    const float alt = sensor.bmp.readAltitude();
    if (!isnan(alt)) {
      lastBaroAlt = alt;

      // Airspeed- and phase-scheduled trust. The v^2 term models static-port
      // error, which is what makes a barometer read high on a fast rocket; the
      // phase multiplier covers motor plume over the base.
      const float speed = fabsf(kfD.x[1]);
      const float sigStatic = BARO_ALT_SIGMA * phaseBaroMultiplier(flightState);
      const float sigDyn    = BARO_VEL_COEF * speed * speed;
      const float Rbaro     = sigStatic * sigStatic + sigDyn * sigDyn;

      // Pure difference against the pad, in the barometer's own pressure-altitude
      // scale, so the sea-level reference and the day's weather cancel out.
      const float zD = padPressureAlt - alt;
      kfCorrect(kfD, 0, zD, Rbaro, GATE_SIGMA);
    }
  }

  // --- 5. GPS -----------------------------------------------------------------
  // Specific force magnitude, in g, drives the dynamic trust multiplier. Computed
  // here rather than in section 6 because applyGpsFix() needs it.
  const float accelMagG = sqrtf((float)(fusion.lastAx * fusion.lastAx +
                                        fusion.lastAy * fusion.lastAy +
                                        fusion.lastAz * fusion.lastAz));
  GpsFix fix;
  if (takeGpsFix(fix)) applyGpsFix(fix, aN, aE, accelMagG);

  // GPS health. An outage is a fact about the flight, not an error - log it and
  // report it afterwards so the next flight's tuning is based on evidence.
  {
    const uint32_t nowMsG = millis();
    const uint32_t gap = gpsLastGoodMs ? (uint32_t)(nowMsG - gpsLastGoodMs) : 0;
    if (gpsLastGoodMs && gap > maxGpsGapMs) maxGpsGapMs = gap;
    if (gpsLastGoodMs && gap > GPS_OUTAGE_MS) {
      if (!gpsInOutage) { gpsInOutage = true; gpsOutages++; }
    } else {
      gpsInOutage = false;
    }
  }

  // --- 6. Standstill / ZUPT ---------------------------------------------------
  const float amag = accelMagG;
  const float gsum = fabsf((float)fusion.lastGx) +
                     fabsf((float)fusion.lastGy) +
                     fabsf((float)fusion.lastGz);
  const float axialG = (float)fusion.lastAz;
  const uint32_t nowMs = millis();

  if (fabsf(amag - 1.0f) < ZUPT_ACC_TOL_G && gsum < ZUPT_GYRO_TOL_DPS) {
    if (quietSinceMs == 0) quietSinceMs = nowMs ? nowMs : 1;
  } else {
    quietSinceMs = 0;
  }

  // The phase gate is the important half of this condition - see ZUPT in the
  // header. Terminal-velocity descent passes the quiet test on its own.
  zuptActive = phaseAllowsZupt(flightState) && (quietSinceMs != 0) &&
               ((uint32_t)(nowMs - quietSinceMs) >= ZUPT_HOLD_MS);

  if (zuptActive) {
    const float Rz = ZUPT_VEL_SIGMA * ZUPT_VEL_SIGMA;
    kfCorrect(kfN, 1, 0.0f, Rz, 0.0f);
    kfCorrect(kfE, 1, 0.0f, Rz, 0.0f);
    kfCorrect(kfD, 1, 0.0f, Rz, 0.0f);
  }

  kfClampBias(kfN, ACC_BIAS_MAX);
  kfClampBias(kfE, ACC_BIAS_MAX);
  kfClampBias(kfD, ACC_BIAS_MAX);

  // --- 7. Flight state, peaks, saturation -------------------------------------
  const float altAGL    = -kfD.x[0];
  const float climbRate = -kfD.x[1];

  const bool accelSat = (fabsf(axialG) >= ACCEL_SATURATION_G) ||
                        (amag >= ACCEL_SATURATION_G);
  if (accelSat) accelSatCount++;

  if (altAGL > maxAltAGL) maxAltAGL = altAGL;
  if (climbRate > maxSpeedUp) maxSpeedUp = climbRate;
  if (axialG > maxAxialG) maxAxialG = axialG;

  updateFlightState(axialG, amag, gsum, altAGL, climbRate, nowMs);

  // Informational only - onLandingBurnGo() is an empty hook by design.
  if (flightState == FS_DESCENT && climbRate < -1.0f) {
    if (altAGL <= burnAltitudeAGL(-climbRate)) onLandingBurnGo(altAGL, -climbRate);
  }

  // Freeze the recorder a while after touchdown so the flight cannot be
  // overwritten by the rocket lying in a field waiting to be found.
  if (flightState == FS_LANDED && landedMs &&
      (uint32_t)(nowMs - landedMs) > (uint32_t)LOG_FREEZE_AFTER_S * 1000UL) {
    g_logFrozen = true;
  }

  // --- 8. Log and publish -----------------------------------------------------
  const bool fifoOvf = sensor.imu.fifoOverflowed();
  if ((uint32_t)(nowUs - lastLogUs) >= (1000000UL / LOG_RATE_HZ)) {
    lastLogUs = nowUs;
    logSample(roll, pitch, yaw, accelSat, lastGpsUseMs != 0, fifoOvf, nowMs);
  }

  publishNav(roll, pitch, yaw, axialG, aN, aE, aUp, nowMs);
}

void setup() {
  if (!sensor.beginAll(MODE_HYBRID, IMU_CS_PIN, IMU_SPI_HZ)) {
    g_sensorFail = true;
    while (1) { delay(1000); }
  }

  sensor.imu.setODR(IMU_ODR);
  // 20-bit high-resolution FIFO. Pins the ranges at +/-16 g and +/-2000 dps -
  // both required here - and gives 16x resolution in the near-zero-g coast.
  sensor.imu.setFIFOMode(FIFO_20BIT_HIRES);

  // 400 kHz I2C. The bus carries the magnetometer at 200 Hz and the barometer at
  // 100 Hz; at the 100 kHz default those two alone would eat a sixth of core 0.
  sensor.bmp.setI2CSpeed(400000);

  /*
   * Barometer at the driver's maximum rate. For a rocket this is bandwidth over
   * per-sample noise, which is the opposite of the choice GPS_INS_Localization
   * makes, and the reason is lag: altitude lag turns directly into landing-burn
   * error, whereas per-sample noise is what the Kalman filter exists to absorb.
   *
   *   ODR 240 Hz -> driver maximum, and 2.4x the 100 Hz read rate
   *   OSR x2     -> the combination beginAll() already proves works at 240 Hz.
   *                 Higher oversampling cannot sustain this ODR; the extra rate
   *                 buys back more than the lower OSR gives up.
   *   IIR 3      -> ~12.7 Hz cutoff, safely under the 50 Hz read-rate Nyquist so
   *                 broadband motor vibration cannot alias into the altitude
   *                 signal. Costs ~12.5 ms of group delay: 0.85 m at burnout
   *                 speed, 0.4 m at a 30 m/s burn entry.
   *
   * Config registers only latch reliably in standby, so bracket the writes.
   */
  sensor.bmp.setPowerMode(BMP580_MODE_STANDBY);
  delay(5);
  sensor.bmp.setOversampling(BMP580_OSR_x2, BMP580_OSR_x2);
  sensor.bmp.setODR(BMP580_ODR_240Hz);
  sensor.bmp.setIIRFilter(BMP580_IIR_3, BMP580_IIR_1);
  sensor.bmp.setPowerMode(BMP580_MODE_NORMAL);
  delay(20);

  // Long gyro calibration. This is the pad, there is time, and every bit of
  // residual gyro bias becomes attitude error during the boost and coast where
  // there is no accelerometer correction to pull it back.
  sensor.autoCalibrateGyro(2000);

  fusion.setMountOrientation(ROCKET_MOUNT);
  fusion.setMagCalibration(MAG_HARD_IRON, MAG_SOFT_IRON);
  fusion.setDeclination(MAGNETIC_DECLINATION);
  fusion.setMaxGyroBias(5.0f);
  fusion.setLocalGravity(GRAV_NOMINAL);

  sensor.imu.flushFIFO();
  fusion.initOrientation();

  resetNavigation();
  lastNavUs  = micros();
  lastBaroUs = lastNavUs;
  lastLogUs  = lastNavUs;
  g_navReady = true;
}

void loop() {
  if (fusion.update()) {
    float bx, by, bz;
    fusion.getLinearAcceleration(bx, by, bz, ACCEL_UNIT_MS2);
    accSum[0] += bx;
    accSum[1] += by;
    accSum[2] += bz;
    accCount++;
  }

  if (g_resetRequest) {
    g_resetRequest = false;
    resetNavigation();
    lastNavUs = micros();
  }

  const uint32_t nowUs = micros();
  if (accCount > 0 && (uint32_t)(nowUs - lastNavUs) >= (1000000UL / NAV_RATE_HZ)) {
    float dt = (float)(uint32_t)(nowUs - lastNavUs) * 1e-6f;
    lastNavUs = nowUs;
    if (dt > 0.25f) dt = 0.25f;
    navStep(dt);
  }
}

// ===========================================================================
// CORE 1 - GNSS, TELEMETRY, FLIGHT LOG DUMP
// ===========================================================================

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

  gpsOpen(GPS_FAST_BAUD);
  if (!gpsListening(1500)) {
    gpsOpen(GPS_BOOT_BAUD);
    if (!gpsListening(1500)) {
      Serial.println(F("[GPS] No NMEA - check wiring and antenna."));
      return;
    }
    gpsSend(PCAS_BAUD_115200);
    delay(200);
    gpsOpen(GPS_FAST_BAUD);
    if (!gpsListening(1500)) {
      gpsOpen(GPS_BOOT_BAUD);
      gpsSend(PCAS_RATE_1HZ);
      return;
    }
  }

  gpsSend(PCAS_RATE_5HZ);
  delay(100);
  gpsSend(PCAS_NMEA_MINIMAL);
  delay(150);
  if (!gpsListening(2000)) {
    gpsSend(PCAS_NMEA_DEFAULT);
    delay(150);
  }
}

static void publishFix() {
  GpsFix f;
  f.lat       = gps.location.lat();
  f.lon       = gps.location.lng();
  f.haveAlt   = gps.altitude.isValid();
  f.altMSL    = f.haveAlt ? (float)gps.altitude.meters() : 0.0f;
  f.hdop      = gps.hdop.isValid() ? ((float)gps.hdop.value() * 0.01f) : 99.0f;
  f.sats      = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  f.speedMps  = gps.speed.isValid() ? (float)gps.speed.mps() : 0.0f;
  f.courseDeg = gps.course.isValid() ? (float)gps.course.deg() : 0.0f;
  f.haveVel   = gps.speed.isValid() && gps.course.isValid() && (f.speedMps > 1.0f);
  f.stampMs   = millis();

  mutex_enter_blocking(&g_gpsMtx);
  f.seq = g_gpsShared.seq + 1;
  if (f.seq == 0) f.seq = 1;
  g_gpsShared = f;
  mutex_exit(&g_gpsMtx);

  lastPublishMs = f.stampMs;
}

static const char *stateName(uint8_t s) {
  switch ((FlightState)s) {
    case FS_INIT:         return "INIT";
    case FS_PAD:          return "PAD";
    case FS_BOOST:        return "BOOST";
    case FS_COAST:        return "COAST";
    case FS_DESCENT:      return "DESCENT";
    case FS_LANDING_BURN: return "LANDING_BURN";
    case FS_LANDED:       return "LANDED";
  }
  return "?";
}

// Dumps the recorder as CSV. Only safe once core 0 has frozen the buffer, which
// it does LOG_FREEZE_AFTER_S seconds after touchdown - otherwise core 0 would be
// writing records underneath the reader.
static void dumpLog() {
  if (!g_logFrozen) {
    Serial.println(F("[LOG] not frozen yet (freezes after landing). Use 'Z' to force."));
    return;
  }

  const uint32_t total = g_logWrite;
  const uint32_t count = (total < LOG_CAPACITY) ? total : LOG_CAPACITY;
  const uint32_t start = (total < LOG_CAPACITY) ? 0 : (total % LOG_CAPACITY);

  Serial.print(F("[LOG] "));
  Serial.print(count);
  Serial.println(F(" records"));
  Serial.println(F("t_ms,state,roll,pitch,yaw,ax_g,ay_g,az_g,gx,gy,gz,agl_m,climb_mps,vN,vE,pN,pE,flags"));

  for (uint32_t i = 0; i < count; i++) {
    const LogRecord &r = g_log[(start + i) % LOG_CAPACITY];
    Serial.print(r.tMs);                 Serial.print(',');
    Serial.print(stateName(r.state));    Serial.print(',');
    Serial.print(r.roll  * 0.1f, 1);     Serial.print(',');
    Serial.print(r.pitch * 0.1f, 1);     Serial.print(',');
    Serial.print(r.yaw   * 0.1f, 1);     Serial.print(',');
    Serial.print(r.ax * 0.001f, 3);      Serial.print(',');
    Serial.print(r.ay * 0.001f, 3);      Serial.print(',');
    Serial.print(r.az * 0.001f, 3);      Serial.print(',');
    Serial.print(r.gx * 0.1f, 1);        Serial.print(',');
    Serial.print(r.gy * 0.1f, 1);        Serial.print(',');
    Serial.print(r.gz * 0.1f, 1);        Serial.print(',');
    Serial.print(r.altAGL * 0.1f, 1);    Serial.print(',');
    Serial.print(r.climbRate * 0.1f, 1); Serial.print(',');
    Serial.print(r.vN * 0.1f, 1);        Serial.print(',');
    Serial.print(r.vE * 0.1f, 1);        Serial.print(',');
    Serial.print(r.pN * 0.1f, 1);        Serial.print(',');
    Serial.print(r.pE * 0.1f, 1);        Serial.print(',');
    Serial.println(r.flags);
  }
  Serial.println(F("[LOG] end"));
}

static void printSummary(const NavOut &n) {
  Serial.println(F("=============== FLIGHT SUMMARY ==============="));
  Serial.print(F("  apogee AGL     ")); Serial.print(n.maxAltAGL, 1);  Serial.println(F(" m"));
  Serial.print(F("  max climb rate ")); Serial.print(n.maxSpeedUp, 1); Serial.println(F(" m/s"));
  Serial.print(F("  max axial      ")); Serial.print(n.maxAxialG, 1);  Serial.println(F(" g"));
  Serial.print(F("  flight time    ")); Serial.print(n.flightMs / 1000.0f, 1); Serial.println(F(" s"));
  Serial.print(F("  landing site   "));
  Serial.print(n.lat, 7); Serial.print(' '); Serial.println(n.lon, 7);
  Serial.print(F("  drift from pad ")); Serial.print(sqrtf(n.pN * n.pN + n.pE * n.pE), 1);
  Serial.println(F(" m"));
  Serial.print(F("  GPS outages    ")); Serial.print(n.gpsOutages);
  Serial.print(F(", longest gap ")); Serial.print(n.maxGpsGapMs);
  Serial.println(F(" ms"));
  if (n.accelSatCount) {
    Serial.print(F("  !! ACCEL SATURATED on ")); Serial.print(n.accelSatCount);
    Serial.println(F(" samples - velocity through that phase is not trustworthy"));
  }
  if (n.fifoOverflows) {
    Serial.print(F("  !! IMU FIFO OVERFLOW x")); Serial.println(n.fifoOverflows);
  }
  Serial.println(F("  press 'p' to dump the flight log as CSV"));
  Serial.println(F("=============================================="));
}

static void handleCommand(char c) {
  switch (c) {
    case 'd':
      g_rawNmeaDebug = !g_rawNmeaDebug;
      break;
    case 'p':
      g_dumpRequest = true;
      break;
    case 'Z':
      g_logFrozen = true;
      Serial.println(F("[LOG] frozen by command"));
      break;
    case 'z':
      g_resetRequest = true;
      summaryShown = false;
      lastShownState = 0xFF;
      Serial.println(F("[CMD] filter and recorder reset - rocket must be still"));
      break;
    case 'h':
      Serial.println(F("[CMD] p=dump log  Z=freeze log  z=reset  d=raw NMEA"));
      break;
    default:
      break;
  }
}

static void printTelemetry(const NavOut &n) {
  if (n.state != lastShownState) {
    lastShownState = n.state;
    Serial.print(F("\n>>> "));
    Serial.print(stateName(n.state));
    Serial.print(F("   T+"));
    Serial.print(n.flightMs / 1000.0f, 2);
    Serial.print(F("s  AGL "));
    Serial.print(n.altAGL, 1);
    Serial.print(F(" m  climb "));
    Serial.print(n.climbRate, 1);
    Serial.println(F(" m/s"));
  }

  Serial.print(stateName(n.state));
  Serial.print(F(" AGL "));    Serial.print(n.altAGL, 1);
  Serial.print(F(" +/-"));     Serial.print(n.sigmaAlt, 1);
  Serial.print(F(" | climb ")); Serial.print(n.climbRate, 1);
  Serial.print(F(" +/-"));     Serial.print(n.sigmaClimb, 1);
  Serial.print(F(" | axial ")); Serial.print(n.axialG, 1); Serial.print(F("g"));
  Serial.print(F(" | RPY "));
  Serial.print(n.roll, 0);  Serial.print(' ');
  Serial.print(n.pitch, 0); Serial.print(' ');
  Serial.print(n.yaw, 0);
  Serial.print(F(" | drift "));
  Serial.print(sqrtf(n.pN * n.pN + n.pE * n.pE), 0); Serial.print(F("m"));

  if (n.state == FS_DESCENT) {
    Serial.print(F(" | BURN@")); Serial.print(n.burnAltAGL, 1);
    Serial.print(F("m margin ")); Serial.print(n.burnMarginM, 1); Serial.print(F("m"));
  }

  Serial.print(F(" | sats ")); Serial.print(n.sats);
  if (n.zupt) Serial.print(F(" ZUPT"));
  if (n.accelSatCount) Serial.print(F(" !SAT"));
  Serial.println();
}

void setup1() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (uint32_t)(millis() - t0) < 3000) delay(10);

  Serial.println(F("--- TriSense rocket navigator : propulsive landing ---"));
  Serial.println(F("Keys: p=dump log  Z=freeze log  z=reset  d=raw NMEA  h=help"));
  Serial.print(F("[LOG] capacity "));
  Serial.print(LOG_CAPACITY);
  Serial.print(F(" records, "));
  Serial.print((uint32_t)(LOG_CAPACITY * sizeof(LogRecord)) / 1024);
  Serial.println(F(" KB"));

  gpsConfigure();
  Serial.print(F("[GPS] UART @ ")); Serial.println(gpsBaud);
  Serial.println(F("[SYS] core 0 calibrating - keep the rocket still and vertical"));
}

void loop1() {
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    if (g_rawNmeaDebug) Serial.write(c);
    if (gps.encode(c)) {
      if (gps.date.isUpdated()) {
        (void)gps.date.value();
        if (gps.location.isValid()) publishFix();
      } else if (gps.location.isUpdated() && gps.location.isValid() &&
                 (uint32_t)(millis() - lastPublishMs) > 1500) {
        (void)gps.location.lat();
        publishFix();
      }
    }
  }

  while (Serial.available()) handleCommand((char)Serial.read());

  if (g_dumpRequest) {
    g_dumpRequest = false;
    dumpLog();
  }

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastPrintMs) >= (1000UL / TELEMETRY_HZ)) {
    lastPrintMs = nowMs;

    if (g_sensorFail) {
      if (!sensorFailShown) {
        sensorFailShown = true;
        Serial.println(F("[SYS] TriSense init FAILED - check SPI CS pin and I2C wiring."));
      }
      return;
    }
    if (!g_navReady) return;

    NavOut n;
    mutex_enter_blocking(&g_navMtx);
    n = g_navShared;
    mutex_exit(&g_navMtx);

    // Once down, stop the 20 Hz chatter and beacon the recovery position slowly.
    if (n.state == FS_LANDED) {
      if (!summaryShown) {
        summaryShown = true;
        printSummary(n);
      }
      if ((nowMs / 1000UL) % 2UL == 0UL) {
        Serial.print(F("[RECOVERY] "));
        Serial.print(n.lat, 7); Serial.print(' '); Serial.print(n.lon, 7);
        Serial.print(F("  sats ")); Serial.print(n.sats);
        Serial.print(F("  log ")); Serial.print(n.logFrozen ? F("frozen") : F("running"));
        Serial.println();
      }
      return;
    }

    printTelemetry(n);
  }
}
