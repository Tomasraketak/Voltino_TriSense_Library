/*
 * Example: GPS_INS_Localization.ino  (Voltino TriSense v1.3.0)
 *
 * Full 3D localization: fuses the TriSense IMU + magnetometer + barometer with a
 * Quectel L76K GNSS receiver into a single navigation solution that outputs
 *
 *      orientation   roll / pitch / yaw (deg) + quaternion
 *      acceleration  world-frame, gravity removed, accelerometer bias removed  [m/s^2]
 *      velocity      North / East / Down                                       [m/s]
 *      position      North / East / Down from a local origin, plus lat/lon/alt
 *
 * HARDWARE
 *   Raspberry Pi Pico 2 (RP2350) + Voltino TriSense Pro + Quectel L76K
 *     TriSense : ICM-42688-P on SPI (CS = GP17), AK09918C + BMP580 on I2C
 *     L76K     : module RX <- GP0 (Pico TX),  module TX -> GP1 (Pico RX)
 *
 * SOFTWARE
 *   - Earle Philhower arduino-pico core (this sketch uses setup1()/loop1(), the
 *     Pico SDK mutex API, and Serial1.setTX()/setRX()).
 *   - TinyGPSPlus by Mikal Hart (Library Manager: "TinyGPSPlus").
 *
 * ---------------------------------------------------------------------------
 * WHICH FILTER, AND WHY
 * ---------------------------------------------------------------------------
 * This sketch uses a CASCADED (loosely-coupled) estimator, not one monolithic
 * filter:
 *
 *   Stage 1 - attitude:  AdvancedTriFusion, the library's adaptive complementary
 *             filter. Runs at the IMU ODR, corrects the gyro with accelerometer
 *             and magnetometer using Gaussian confidence gating.
 *
 *   Stage 2 - navigation: a linear Kalman filter over
 *                 x = [ position, velocity, acceleration-bias ]
 *             in a local North-East-Down tangent frame, driven by the world-frame
 *             acceleration from stage 1 and corrected by GPS position, GPS
 *             velocity, barometric altitude and zero-velocity updates.
 *
 * Why not a single 15-state error-state Kalman filter (ESKF)? An ESKF is the
 * textbook answer and it is genuinely better in one specific regime: long GNSS
 * outages under high dynamics, where attitude error and velocity error are
 * strongly correlated and the filter has to recover heading from GPS. It also
 * costs a 15x15 covariance propagation (thousands of multiply-adds) on every
 * step. Here the magnetometer already observes heading directly and the
 * accelerometer already observes tilt, so that correlation buys very little -
 * while the cascade costs a small fraction of the arithmetic and is far easier
 * to tune and to debug. If you fly a fast fixed-wing with no usable
 * magnetometer, upgrade to an ESKF; for rovers, boats, cars, drones and
 * trackers this cascade is the better trade.
 *
 * Why the navigation stage is a real Kalman filter and not another
 * complementary filter: GPS accuracy varies by an order of magnitude with HDOP
 * and satellite count, and the accelerometer bias has to be *estimated*, not
 * assumed. A Kalman filter weights each measurement by its actual variance and
 * makes that bias observable; a fixed-gain complementary filter can do neither.
 *
 * Why three independent 3-state filters instead of one 9-state filter: because
 * attitude is supplied externally, the 9-state system matrix is exactly block
 * diagonal per axis, the process noise is diagonal, and every measurement
 * touches exactly one axis. A block-diagonal covariance therefore stays block
 * diagonal forever, so three 3x3 filters are mathematically IDENTICAL to one
 * 9x9 filter - at roughly a ninth of the arithmetic. An exact factorization,
 * not an approximation.
 *
 * ---------------------------------------------------------------------------
 * RP2350 OPTIMIZATION NOTES
 * ---------------------------------------------------------------------------
 *  - Both cores are used. Core 0 runs only the IMU/AHRS loop and the Kalman
 *    filter and never touches USB serial; core 1 parses NMEA and prints. A
 *    blocking print on the fusion core is the classic cause of IMU FIFO
 *    overflow, and overflowed packets are rotation that can never be recovered.
 *  - Shared state is exchanged through two small mutex-protected structs. Core 0
 *    only ever uses mutex_try_enter(), so the real-time core never blocks; a
 *    missed handoff is simply retried on the next iteration ~10 ms later.
 *  - All navigation math is single-precision. The Cortex-M33 has a
 *    single-precision FPU but no double-precision unit, so every `double`
 *    operation is a software call. Doubles appear ONLY in latitude/longitude
 *    handling (float would quantize position to ~0.4 m) and only a handful of
 *    times per GPS fix. Note the f-suffixed libm calls (sqrtf/sinf/cosf) - the
 *    unsuffixed versions would silently promote to double and cost ~20x.
 *  - The IMU runs at 2 kHz rather than the 8 kHz default. Attitude bandwidth
 *    above a few hundred Hz is worthless for navigation, and the saved cycles
 *    and SPI traffic become headroom.
 *  - The L76K is configured to emit only GGA and RMC. GSV alone can be four
 *    sentences per epoch; at 5 Hz that is most of the UART budget spent on data
 *    TinyGPSPlus does not even parse.
 *
 * ---------------------------------------------------------------------------
 * BEFORE YOU TRUST IT: three things you must set
 * ---------------------------------------------------------------------------
 *  1. Magnetometer calibration (MAG_HARD_IRON / MAG_SOFT_IRON below). Run the
 *     MotionCal example first. An uncalibrated magnetometer gives a heading
 *     error, and a heading error rotates your entire velocity vector.
 *  2. MAGNETIC_DECLINATION for your location (magnetic-declination.com).
 *  3. YAW_IS_COMPASS_HEADING - see the note next to it. Get this wrong and
 *     East/West is mirrored. The sketch measures it against GPS course and tells
 *     you if it disagrees, so just move in a straight line and read the log.
 */

#if !defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_RP2350)
  #error "GPS_INS_Localization targets the RP2350/RP2040 on the Earle Philhower arduino-pico core (it uses setup1()/loop1() and the Pico SDK mutex API)."
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
#define GPS_PIN_TX            0        // Pico TX  -> L76K RX
#define GPS_PIN_RX            1        // Pico RX  <- L76K TX

// ---- GNSS -------------------------------------------------------------------
#define GPS_FAST_BAUD         115200UL
#define GPS_BOOT_BAUD         9600UL
#define GPS_TRIM_SENTENCES    1        // 1 = ask the L76K for GGA+RMC only
#define GPS_MIN_SATS          4        // Below this the fix is not used
#define GPS_MAX_HDOP          6.0f     // Above this the fix is not used
#define GPS_COURSE_MIN_MPS    1.0f     // Course over ground is noise below this

// Age of a fix by the time it is applied: NMEA is emitted after the receiver has
// computed the solution, and the sentence itself takes time on the wire. The
// measurement is extrapolated forward by this much using the filter's own
// velocity before being applied. 0.10-0.25 s is typical; at 20 m/s an
// uncorrected 150 ms lag is a 3 m position error that the filter would otherwise
// have to interpret as real motion.
#define GPS_LATENCY_S         0.15f

// ---- Site -------------------------------------------------------------------
// Declination turns magnetic heading into true heading. GPS course is relative
// to TRUE north, so this has to be right or the two will never agree.
#define MAGNETIC_DECLINATION  5.6f     // degrees East, positive (Prague ~ +5.6)

// Magnetometer calibration - REPLACE with your own MotionCal output.
static float MAG_HARD_IRON[3] = { -46.02f, -0.85f, -46.00f };
static float MAG_SOFT_IRON[3][3] = {
  {  0.965f,  0.008f, -0.002f },
  {  0.008f,  0.981f,  0.139f },
  { -0.002f,  0.139f,  1.077f }
};

/*
 * YAW SENSE
 * ---------
 * This sketch rotates acceleration into North/East using the yaw from
 * getOrientationDegrees(), so it has to know whether that number is a CLOCKWISE
 * compass heading (0 = North, 90 = East, what a map bearing means) or a
 * COUNTER-CLOCKWISE bearing (0 = North, 90 = West).
 *
 * On the TriSense Pro it is a compass heading, which is why this defaults to 1.
 * The switch stays because the answer depends on how the AK09918C's axes sit
 * relative to the ICM-42688-P, and that is a board-layout property rather than
 * something the library source pins down. Confirm on your unit either way:
 *   a) Point the board's +X axis East. Compass heading -> yaw reads ~90.
 *      Counter-clockwise bearing -> yaw reads ~270.
 *   b) Run this sketch and move in a straight line above 2 m/s. It compares yaw
 *      against GPS course over ground and prints a one-time verdict. That check
 *      doubles as a magnetometer-calibration and declination sanity test, since
 *      a bad hard-iron fit shows up as heading that will not track course.
 *
 * If you ever set this to 0, also negate MAGNETIC_DECLINATION: the library adds
 * declination to that same yaw value, so a counter-clockwise yaw needs the
 * opposite sign to end up pointing at true north.
 *
 * NOTE - why this sketch does NOT use getGlobalAcceleration(): with a clockwise
 * compass yaw, the library builds its quaternion as Rz(+yaw), while a geographic
 * frame needs Rz(-yaw). The two agree for anything along body +X but come out
 * mirrored for the body +Y (left) component, so the library's "world" X/Y are not
 * a consistent North/East pair. This sketch therefore takes body-frame linear
 * acceleration - which is yaw-independent, since the gravity vector it removes is
 * a function of roll and pitch only - and does the heading rotation itself, below
 * in navStep(). Roll and pitch are unaffected either way.
 */
#define YAW_IS_COMPASS_HEADING 1

// ---- Rates ------------------------------------------------------------------
#define NAV_RATE_HZ           100      // Kalman predict rate
#define BARO_RATE_HZ          25       // Barometer read rate (I2C traffic)
#define TELEMETRY_HZ          10       // Serial output rate

/*
 * IMU output data rate.
 *
 * 2 kHz is not a compromise on attitude accuracy. First-order quaternion
 * integration loses about (w*dt)^3/12 rad per step, which at 2 kHz and a full
 * 2000 deg/s slew is 0.05 deg/s of drift - half a degree over a ten-second
 * aerobatic sequence, and utterly negligible in normal motion. The anti-alias
 * bandwidth (~ODR/2 = 1 kHz) is likewise far above anything a vehicle produces.
 *
 * The real reason to raise it is VIBRATION AVERAGING. Each nav step averages one
 * 10 ms batch of accelerometer samples: 20 samples at 2 kHz, 80 at 8 kHz, so
 * 8 kHz cuts the residual vibration in that mean by 2x. Cost is only ~2% of one
 * core at 8 kHz, so:
 *
 *    ODR_1KHZ / ODR_2KHZ   rovers, boats, cars, handheld trackers, fixed-wing
 *    ODR_4KHZ / ODR_8KHZ   multirotors and anything else with props or a motor
 *                          bolted to the same frame as the sensor
 *
 * If you see "IMU FIFO OVERFLOW" in the log, the loop is not draining fast
 * enough - lower this rather than living with the dropped packets.
 */
#define IMU_ODR               ODR_2KHZ

// ---- Filter tuning ----------------------------------------------------------
// Process noise. ACC_PSD is the acceleration noise DENSITY driving velocity, in
// m/s^2 per sqrt(Hz). The ICM-42688-P itself is around 0.0007; the values below
// are two orders larger on purpose, because what really perturbs the velocity
// estimate is residual attitude error (1 deg of tilt error injects 0.17 m/s^2 of
// phantom horizontal acceleration) and vibration, not sensor noise. Raise it if
// the filter lags real motion; lower it if the output is jittery between fixes.
#define ACC_PSD_HORZ          0.25f    // m/s^2/sqrt(Hz)
#define ACC_PSD_VERT          0.35f    // vertical also absorbs baro noise
#define ACC_BIAS_RW           0.004f   // m/s^2/sqrt(s), bias random walk

// Measurement noise.
#define GPS_UERE_M            2.2f     // sigma_position = GPS_UERE_M * HDOP   [m]
#define GPS_VEL_SIGMA         0.25f    // sigma_velocity at HDOP 1           [m/s]
// BARO_ALT_SIGMA is dominated by the ATMOSPHERE, not by the BMP580: with the
// oversampling set in setup() the sensor contributes ~4 cm, while wind gusts,
// prop wash and opening a car door contribute decimetres. Raise it if altitude
// looks twitchy on your platform; lower it only inside a shielded static port.
#define BARO_ALT_SIGMA        0.6f     // short-term barometric altitude noise [m]
#define ZUPT_VEL_SIGMA        0.02f    // how hard a detected standstill pins v [m/s]

// Innovation gate, in sigma. A measurement further than this from the prediction
// is rejected as an outlier (multipath, a jumped fix). GPS_MAX_REJECTS
// consecutive rejections mean the filter, not the GPS, is wrong - so it gets
// re-anchored on the GPS fix.
#define GATE_SIGMA            4.0f
#define GPS_MAX_REJECTS       12

// The barometer holds altitude beautifully over minutes but drifts with weather
// over hours; GPS altitude is noisy but unbiased. The offset between the two is
// tracked with this time constant, so short-term altitude comes from the baro and
// the long-term datum comes from GPS.
#define BARO_TRACK_TAU_S      90.0f

// Standstill detection for zero-velocity updates. ZUPTs are what make the
// accelerometer bias observable - without them a stationary vehicle slowly
// "drives away" between GPS fixes.
#define ZUPT_ACC_TOL_G        0.04f    // |accel| within this of 1 g
#define ZUPT_GYRO_TOL_DPS     2.0f     // sum |gyro| below this
#define ZUPT_HOLD_MS          300      // must stay quiet this long

// ===========================================================================
// L76K PCAS COMMAND SET
// ===========================================================================
// Bodies only - the NMEA checksum is computed and appended at runtime by
// gpsSend(), so there are no hand-calculated "*19" constants to get wrong.
#define PCAS_BAUD_9600     "PCAS01,1"
#define PCAS_BAUD_115200   "PCAS01,5"
#define PCAS_RATE_1HZ      "PCAS02,1000"
#define PCAS_RATE_5HZ      "PCAS02,200"
#define PCAS_RATE_10HZ     "PCAS02,100"
// PCAS03 field order: GGA,GLL,GSA,GSV,RMC,VTG,ZDA,ANT,DHV,LPS,res,res,UTC,GST,res,res,res,TIM
#define PCAS_NMEA_MINIMAL  "PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0,,,,0"
#define PCAS_NMEA_DEFAULT  "PCAS03,1,1,1,1,1,1,0,0,0,0,,,0,0,,,,0"

// ===========================================================================
// TYPES  (all defined before the first function - see note below)
// ===========================================================================
// The Arduino builder auto-generates prototypes for the free functions in a
// .ino and inserts them just above the first one, so every type a prototype
// mentions has to be declared before that point. That is also why AxisKF is a
// plain struct operated on by free functions rather than one with methods.

struct GpsFix {
  double   lat, lon;      // deg
  float    altMSL;        // m above mean sea level
  float    speedMps;
  float    courseDeg;     // true, clockwise from North
  float    hdop;
  uint8_t  sats;
  bool     haveAlt;
  bool     haveVel;
  uint32_t stampMs;       // millis() when the sentence completed
  uint32_t seq;           // increments on every published fix
};

struct NavOut {
  float    roll, pitch, yaw;      // deg
  float    q[4];                  // w, x, y, z  (body -> fusion world frame)
  float    aN, aE, aD;            // m/s^2, gravity and estimated bias removed
  float    vN, vE, vD;            // m/s
  float    pN, pE, pD;            // m from local origin
  double   lat, lon;              // deg
  float    altMSL;                // m
  float    sigmaPosH, sigmaPosV, sigmaVelH;   // 1-sigma, m and m/s
  float    biasN, biasE, biasD;   // learned accel bias, m/s^2
  float    fusionHz;
  float    baroOffset;            // baro-to-GPS altitude offset, m
  uint8_t  sats;
  float    hdop;
  uint32_t fixAgeMs;
  uint32_t fifoOverflows;
  bool     originSet;
  bool     gpsAiding;
  bool     zupt;
  int8_t   headingHint;           // 0 = untested, +1 = setting OK, -1 = flip it
};

/*
 * One navigation axis: x = [ position, velocity, acceleration bias ].
 *
 * Continuous model, with `u` the measured world-frame acceleration:
 *      p' = v
 *      v' = (u - b) + w_a          w_a ~ acceleration noise, density ACC_PSD
 *      b' = w_b                    w_b ~ bias random walk, density ACC_BIAS_RW
 *
 * The bias state absorbs everything that looks like a constant acceleration
 * offset in the navigation frame: real accelerometer bias, an error in local
 * gravity, and - most importantly - a small residual tilt error, which projects
 * gravity onto the horizontal axes. That is why estimating it matters far more
 * than the datasheet bias figure would suggest.
 */
struct AxisKF {
  float x[3];
  float P[3][3];
};

// ===========================================================================
// CROSS-CORE SHARED STATE
// ===========================================================================
// auto_init_mutex() initializes at image start-up, which matters because
// arduino-pico launches core 1 before core 0 reaches setup().
auto_init_mutex(g_gpsMtx);
auto_init_mutex(g_navMtx);

static GpsFix        g_gpsShared;                 // written by core 1
static NavOut        g_navShared;                 // written by core 0
static volatile bool g_navReady      = false;     // core 0 finished booting
static volatile bool g_sensorFail    = false;     // core 0 could not start the TriSense
static volatile bool g_resetRequest  = false;     // core 1 -> core 0
static volatile bool g_rawNmeaDebug  = false;
static volatile bool g_csvOutput     = false;

// ===========================================================================
// CORE 0 STATE - INERTIAL NAVIGATION
// ===========================================================================

TriSense          sensor;
AdvancedTriFusion fusion(&sensor.imu, &sensor.mag);

static const float DEG2RAD = 0.017453292519943295f;
static const float RAD2DEG = 57.29577951308232f;

static AxisKF kfN, kfE, kfD;

// Local tangent plane origin.
static bool   originSet       = false;
static double originLat       = 0.0, originLon = 0.0;
static float  originAlt       = 0.0f;        // m MSL, the datum for the Down axis
static float  metersPerRadLat = 6367000.0f;  // meridian radius of curvature
static float  metersPerRadLon = 6367000.0f;  // prime-vertical radius * cos(lat)

// Barometer.
static float    baroOffset    = 0.0f;        // baroAlt + baroOffset ~ altitude MSL
static float    lastBaroAlt   = 0.0f;
static uint32_t lastBaroUs    = 0;

// Body acceleration accumulated between navigation steps.
static float    accSum[3]     = {0.0f, 0.0f, 0.0f};
static uint32_t accCount      = 0;

static uint32_t lastNavUs      = 0;
static uint32_t consumedFixSeq = 0;
static uint8_t  gpsRejects     = 0;
static uint32_t lastGpsUseMs   = 0;

// Standstill detection.
static uint32_t quietSinceMs  = 0;
static bool     zuptActive    = false;

// Heading-sense diagnostic.
static float    headingErrCW   = 0.0f;
static float    headingErrCCW  = 0.0f;
static uint16_t headingSamples = 0;
static int8_t   headingHint    = 0;

// ===========================================================================
// CORE 1 STATE - GNSS AND TELEMETRY
// ===========================================================================

static TinyGPSPlus gps;
static uint32_t    gpsBaud           = GPS_BOOT_BAUD;
static uint32_t    lastPublishMs     = 0;
static uint32_t    lastPrintMs       = 0;
static bool        headingHintShown  = false;
static bool        sensorFailShown   = false;

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

// P <- F P F^T + Q, with F = [[1, dt, -dt^2/2], [0, 1, -dt], [0, 0, 1]].
// Written out rather than looped: the zeros in F make a generic 3x3 product
// mostly multiplications by zero.
static void kfPredict(AxisKF &k, float u, float dt, float accPsd) {
  const float h = 0.5f * dt * dt;
  const float a = u - k.x[2];

  k.x[0] += k.x[1] * dt + a * h;
  k.x[1] += a * dt;
  // k.x[2] is constant across a predict step; the random walk enters through Q.

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

  // Continuous white-noise-acceleration discretization.
  const float qa  = accPsd * accPsd;
  const float qb  = (float)ACC_BIAS_RW * (float)ACC_BIAS_RW;
  const float dt2 = dt * dt;
  k.P[0][0] += qa * dt2 * dt / 3.0f;
  k.P[0][1] += qa * dt2 * 0.5f;
  k.P[1][0] += qa * dt2 * 0.5f;
  k.P[1][1] += qa * dt;
  k.P[2][2] += qb * dt;
}

// Scalar update of a single state (position or velocity). H is a unit row, so
// the whole Kalman update collapses to a handful of operations - there is no
// matrix inversion anywhere in this sketch.
// gateSigma <= 0 disables outlier rejection. Returns false if the measurement
// was gated out.
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

  // Force symmetry. In single precision the short-form covariance update
  // accumulates asymmetry over millions of steps, and an asymmetric P
  // eventually goes indefinite and takes the filter with it.
  k.P[0][1] = k.P[1][0] = 0.5f * (k.P[0][1] + k.P[1][0]);
  k.P[0][2] = k.P[2][0] = 0.5f * (k.P[0][2] + k.P[2][0]);
  k.P[1][2] = k.P[2][1] = 0.5f * (k.P[1][2] + k.P[2][1]);
  return true;
}

static float kfSigma(const AxisKF &k, uint8_t idx) {
  return k.P[idx][idx] > 0.0f ? sqrtf(k.P[idx][idx]) : 0.0f;
}

// ===========================================================================
// CORE 0 - NAVIGATION
// ===========================================================================

static float wrap180(float d) {
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// WGS84 gravity (Somigliana) with a free-air correction. Feeding the fusion the
// real local value instead of 9.80665 removes an offset of up to 0.03 m/s^2 that
// the filter's bias state would otherwise have to chase.
static float localGravity(double latDeg, float altM) {
  const float s  = sinf((float)latDeg * DEG2RAD);
  const float s2 = s * s;
  const float g  = 9.7803253359f * (1.0f + 0.00193185265241f * s2) /
                   sqrtf(1.0f - 0.00669437999013f * s2);
  return g - 3.086e-6f * altM;
}

static void setOrigin(double lat, double lon, float altMSL) {
  originLat = lat;
  originLon = lon;
  originAlt = altMSL;

  // Radii of curvature at the origin. Over a few tens of kilometres a tangent
  // plane built from these is sub-metre accurate, and it costs two transcendental
  // calls once instead of a full geodetic projection on every fix.
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

static void resetNavigation() {
  kfReset(kfN, 0.0f, 0.0f, 100.0f, 2.0f, 0.5f);
  kfReset(kfE, 0.0f, 0.0f, 100.0f, 2.0f, 0.5f);
  kfReset(kfD, 0.0f, 0.0f,  10.0f, 1.0f, 0.5f);
  originSet      = false;
  baroOffset     = 0.0f;
  gpsRejects     = 0;
  lastGpsUseMs   = 0;
  headingErrCW   = 0.0f;
  headingErrCCW  = 0.0f;
  headingSamples = 0;
  headingHint    = 0;
  accSum[0] = accSum[1] = accSum[2] = 0.0f;
  accCount  = 0;
  quietSinceMs = 0;
  zuptActive   = false;

  // The vertical channel works from boot with no GPS at all: the pressure
  // altitude at power-up becomes the datum, re-anchored on the first fix.
  originAlt   = sensor.bmp.readAltitude();
  lastBaroAlt = originAlt;
}

// Pull the freshest GPS fix, if core 1 published one we have not used yet.
// Never blocks: a missed handoff is retried on the next navigation step.
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

static void applyGpsFix(const GpsFix &fix, float yawDeg) {
  if (fix.sats < GPS_MIN_SATS || fix.hdop > GPS_MAX_HDOP) return;

  const uint32_t nowMs = millis();
  const float    hdop  = fix.hdop < 0.8f ? 0.8f : fix.hdop;

  if (!originSet) {
    const float alt0 = fix.haveAlt ? fix.altMSL : lastBaroAlt;
    setOrigin(fix.lat, fix.lon, alt0);
    kfReset(kfN, 0.0f, 0.0f, GPS_UERE_M * hdop, 2.0f, 0.5f);
    kfReset(kfE, 0.0f, 0.0f, GPS_UERE_M * hdop, 2.0f, 0.5f);
    kfReset(kfD, 0.0f, 0.0f, 5.0f, 1.0f, 0.5f);
    baroOffset   = alt0 - lastBaroAlt;
    lastGpsUseMs = nowMs;
    return;
  }

  // Geodetic -> local tangent plane. The subtraction has to happen in double:
  // at 50 deg latitude a float holds ~4e-6 deg, i.e. 0.4 m of quantization.
  const double dLat = fix.lat - originLat;
  const double dLon = fix.lon - originLon;
  float zN = (float)(dLat * (double)DEG2RAD) * metersPerRadLat;
  float zE = (float)(dLon * (double)DEG2RAD) * metersPerRadLon;

  // Latency compensation: the fix describes where we were, so move it forward to
  // where we are now using the current velocity estimate.
  float lag = GPS_LATENCY_S + (float)(uint32_t)(nowMs - fix.stampMs) * 0.001f;
  if (lag > 1.0f) lag = 1.0f;
  zN += kfN.x[1] * lag;
  zE += kfE.x[1] * lag;

  const float sigP = GPS_UERE_M * hdop;
  const float Rpos = sigP * sigP;

  const bool okN = kfCorrect(kfN, 0, zN, Rpos, GATE_SIGMA);
  const bool okE = kfCorrect(kfE, 0, zE, Rpos, GATE_SIGMA);

  if (okN && okE) {
    gpsRejects = 0;
  } else if (++gpsRejects >= GPS_MAX_REJECTS) {
    // Repeated rejections mean the filter drifted away from reality, not that
    // the GPS is lying. Re-anchor on it and re-open the covariance.
    kfReset(kfN, zN, kfN.x[1], sigP, 2.0f, 0.5f);
    kfReset(kfE, zE, kfE.x[1], sigP, 2.0f, 0.5f);
    gpsRejects = 0;
  }

  if (fix.haveVel) {
    const float cr = fix.courseDeg * DEG2RAD;
    const float sv = GPS_VEL_SIGMA * hdop;
    const float Rv = sv * sv;
    kfCorrect(kfN, 1, fix.speedMps * cosf(cr), Rv, GATE_SIGMA);
    kfCorrect(kfE, 1, fix.speedMps * sinf(cr), Rv, GATE_SIGMA);

    // Heading-sense diagnostic. Under straight-line motion a vehicle's heading
    // and its course over ground agree; whichever reading of yaw tracks GPS
    // course is the correct one.
    if (fix.speedMps > 2.0f && headingSamples < 60) {
      headingErrCW  += fabsf(wrap180(yawDeg - fix.courseDeg));
      headingErrCCW += fabsf(wrap180((360.0f - yawDeg) - fix.courseDeg));
      if (++headingSamples >= 60) {
        const bool cwWins = (headingErrCW < headingErrCCW);
#if YAW_IS_COMPASS_HEADING
        headingHint = cwWins ? 1 : -1;
#else
        headingHint = cwWins ? -1 : 1;
#endif
      }
    }
  }

  // Train the baro-to-GPS altitude offset. Short-term altitude still comes from
  // the barometer; GPS only moves the datum, slowly.
  if (fix.haveAlt) {
    const float dtFix = (float)(uint32_t)(nowMs - lastGpsUseMs) * 0.001f;
    float kGain = (dtFix > 0.0f && dtFix < 5.0f) ? (dtFix / BARO_TRACK_TAU_S) : 0.01f;
    if (kGain > 0.2f) kGain = 0.2f;
    baroOffset += kGain * (fix.altMSL - (lastBaroAlt + baroOffset));
  }

  lastGpsUseMs = nowMs;
}

static void publishNav(float roll, float pitch, float yaw,
                       float aN, float aE, float aD, uint32_t nowMs) {
  NavOut n;

  n.roll = roll; n.pitch = pitch; n.yaw = yaw;
  for (uint8_t i = 0; i < 4; i++) n.q[i] = (float)fusion.q[i];

  n.aN = aN - kfN.x[2];  n.aE = aE - kfE.x[2];  n.aD = aD - kfD.x[2];
  n.vN = kfN.x[1];       n.vE = kfE.x[1];       n.vD = kfD.x[1];
  n.pN = kfN.x[0];       n.pE = kfE.x[0];       n.pD = kfD.x[0];
  n.biasN = kfN.x[2];    n.biasE = kfE.x[2];    n.biasD = kfD.x[2];

  if (originSet) {
    n.lat = originLat + (double)(kfN.x[0] / metersPerRadLat) * (double)RAD2DEG;
    n.lon = originLon + (double)(kfE.x[0] / metersPerRadLon) * (double)RAD2DEG;
  } else {
    n.lat = 0.0;
    n.lon = 0.0;
  }
  n.altMSL = originAlt - kfD.x[0];

  n.sigmaPosH = 0.5f * (kfSigma(kfN, 0) + kfSigma(kfE, 0));
  n.sigmaPosV = kfSigma(kfD, 0);
  n.sigmaVelH = 0.5f * (kfSigma(kfN, 1) + kfSigma(kfE, 1));

  n.fusionHz      = fusion.getActualFusionHz();
  n.baroOffset    = baroOffset;
  n.fifoOverflows = sensor.imu.getFIFOOverflowCount();
  n.originSet     = originSet;
  n.gpsAiding     = originSet && ((uint32_t)(nowMs - lastGpsUseMs) < 3000);
  n.zupt          = zuptActive;
  n.headingHint   = headingHint;

  n.sats     = 0;
  n.hdop     = 99.0f;
  n.fixAgeMs = 0xFFFFFFFFUL;
  if (mutex_try_enter(&g_gpsMtx, NULL)) {
    if (g_gpsShared.seq != 0) {
      n.sats     = g_gpsShared.sats;
      n.hdop     = g_gpsShared.hdop;
      n.fixAgeMs = (uint32_t)(nowMs - g_gpsShared.stampMs);
    }
    mutex_exit(&g_gpsMtx);
  }

  if (mutex_try_enter(&g_navMtx, NULL)) {
    g_navShared = n;
    mutex_exit(&g_navMtx);
  }
}

static void navStep(float dt) {
  // --- 1. Mean body-frame linear acceleration since the last step ------------
  // Each fusion.update() that returned true contributed the mean of one FIFO
  // batch. Batches are near-uniform in size when the loop runs freely, so an
  // unweighted mean of batch means is within noise of a true sample mean.
  const float inv = 1.0f / (float)accCount;
  const float bx = accSum[0] * inv;
  const float by = accSum[1] * inv;
  const float bz = accSum[2] * inv;
  accSum[0] = accSum[1] = accSum[2] = 0.0f;
  accCount = 0;

  float roll, pitch, yaw;
  fusion.getOrientationDegrees(roll, pitch, yaw);

  // --- 2. Body -> level -> North/East/Down -----------------------------------
  // Roll and pitch come straight from gravity and are unambiguous. Only the
  // final heading rotation depends on the yaw convention, which is exactly why
  // it is isolated here behind YAW_IS_COMPASS_HEADING instead of being buried
  // inside a quaternion rotation.
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

  // Body +Y sits 90 deg to the LEFT of +X in a right-handed Z-up frame, which is
  // where the signs come from: North = fwd*cos + left*sin, East = fwd*sin - left*cos.
  const float aN =  aFwd * cpsi + aLeft * spsi;
  const float aE =  aFwd * spsi - aLeft * cpsi;
  const float aD = -aUp;

  // --- 3. Predict -------------------------------------------------------------
  kfPredict(kfN, aN, dt, ACC_PSD_HORZ);
  kfPredict(kfE, aE, dt, ACC_PSD_HORZ);
  kfPredict(kfD, aD, dt, ACC_PSD_VERT);

  // --- 4. Barometric altitude -------------------------------------------------
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastBaroUs) >= (1000000UL / BARO_RATE_HZ)) {
    lastBaroUs = nowUs;
    const float alt = sensor.bmp.readAltitude();
    if (!isnan(alt)) {
      lastBaroAlt = alt;
      const float zD = originAlt - (alt + baroOffset);
      kfCorrect(kfD, 0, zD, BARO_ALT_SIGMA * BARO_ALT_SIGMA, GATE_SIGMA);
    }
  }

  // --- 5. GPS -----------------------------------------------------------------
  GpsFix fix;
  if (takeGpsFix(fix)) applyGpsFix(fix, yaw);

  // --- 6. Zero-velocity update ------------------------------------------------
  // Cheap, and the single most effective thing in this filter: standing still is
  // the only condition under which the accelerometer bias is directly visible.
  const float amag = sqrtf((float)(fusion.lastAx * fusion.lastAx +
                                   fusion.lastAy * fusion.lastAy +
                                   fusion.lastAz * fusion.lastAz));
  const float gmag = fabsf((float)fusion.lastGx) +
                     fabsf((float)fusion.lastGy) +
                     fabsf((float)fusion.lastGz);
  const uint32_t nowMs = millis();

  if (fabsf(amag - 1.0f) < ZUPT_ACC_TOL_G && gmag < ZUPT_GYRO_TOL_DPS) {
    if (quietSinceMs == 0) quietSinceMs = nowMs ? nowMs : 1;
  } else {
    quietSinceMs = 0;
  }
  zuptActive = (quietSinceMs != 0) &&
               ((uint32_t)(nowMs - quietSinceMs) >= ZUPT_HOLD_MS);

  if (zuptActive) {
    const float Rz = ZUPT_VEL_SIGMA * ZUPT_VEL_SIGMA;
    kfCorrect(kfN, 1, 0.0f, Rz, 0.0f);
    kfCorrect(kfE, 1, 0.0f, Rz, 0.0f);
    kfCorrect(kfD, 1, 0.0f, Rz, 0.0f);
  }

  // --- 7. Publish -------------------------------------------------------------
  publishNav(roll, pitch, yaw, aN, aE, aD, nowMs);
}

void setup() {
  // Core 0 owns SPI, I2C and the fusion loop. It deliberately never prints:
  // core 1 does all the talking.
  if (!sensor.beginAll(MODE_HYBRID, IMU_CS_PIN, IMU_SPI_HZ)) {
    g_sensorFail = true;
    while (1) { delay(1000); }
  }

  sensor.imu.setODR(IMU_ODR);
  sensor.imu.setFIFOMode(FIFO_16BIT);

  // Barometer tuned for navigation rather than weather logging. Configuration
  // registers only latch reliably in standby, so bracket the writes.
  //
  //   OSR x8 on pressure  -> roughly 4 cm RMS of sensor noise, vs ~11 cm at x2
  //   ODR 50 Hz           -> 2x the 25 Hz read rate, so no aliasing
  //   IIR coefficient 1   -> light anti-alias filter, ~20 ms of group delay
  //
  // Deliberately NOT filtered harder than that: the Kalman filter is the
  // smoother here, and it does that job better when handed a low-latency
  // measurement with an honestly stated sigma than a pre-smoothed, laggy one.
  sensor.bmp.setPowerMode(BMP580_MODE_STANDBY);
  delay(5);
  sensor.bmp.setOversampling(BMP580_OSR_x8, BMP580_OSR_x1);
  sensor.bmp.setODR(BMP580_ODR_50p1Hz);
  sensor.bmp.setIIRFilter(BMP580_IIR_1, BMP580_IIR_OFF);
  sensor.bmp.setPowerMode(BMP580_MODE_NORMAL);
  delay(20);

  sensor.autoCalibrateGyro(1000);

  fusion.setMountOrientation(ORIENTATION_Z_UP);
  fusion.setMagCalibration(MAG_HARD_IRON, MAG_SOFT_IRON);
  fusion.setDeclination(MAGNETIC_DECLINATION);
  fusion.setDynamicGyroBias(true, 0.0001f);
  fusion.setMaxGyroBias(5.0f);
  fusion.setLocalGravity(9.80665f);   // replaced with the real value on first fix

  sensor.imu.flushFIFO();
  fusion.initOrientation();

  resetNavigation();
  lastNavUs  = micros();
  lastBaroUs = lastNavUs;
  g_navReady = true;
}

void loop() {
  // Drain the IMU FIFO as fast as it fills. Everything else in this loop is
  // rate-limited so that this call gets the cycles it needs.
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

  // Predict at a fixed rate rather than per IMU sample: at 2 kHz that would be
  // 20x the arithmetic for no accuracy gain, since GPS and baro arrive far
  // slower than 100 Hz. dt is measured, not assumed, so jitter is harmless.
  const uint32_t nowUs = micros();
  if (accCount > 0 && (uint32_t)(nowUs - lastNavUs) >= (1000000UL / NAV_RATE_HZ)) {
    float dt = (float)(uint32_t)(nowUs - lastNavUs) * 1e-6f;
    lastNavUs = nowUs;
    if (dt > 0.25f) dt = 0.25f;   // a stalled loop must not inject a wild step
    navStep(dt);
  }
}

// ===========================================================================
// CORE 1 - GNSS PARSING AND TELEMETRY
// ===========================================================================

// Sends "$<body>*<checksum>\r\n". Computing the checksum here means new PCAS
// commands can be added without hand-calculating one.
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
      // The baud switch did not take. Stay at 9600, and drop to 1 Hz so the
      // slower link is not saturated.
      gpsOpen(GPS_BOOT_BAUD);
      gpsSend(PCAS_RATE_1HZ);
      return;
    }
  }

  gpsSend(PCAS_RATE_5HZ);
  delay(100);

#if GPS_TRIM_SENTENCES
  // GGA carries position, altitude, satellites and HDOP; RMC carries speed,
  // course, date and time. Together they are everything TinyGPSPlus parses, so
  // GSA/GSV/GLL/VTG are pure UART load.
  gpsSend(PCAS_NMEA_MINIMAL);
  delay(150);
  if (!gpsListening(2000)) {
    // The firmware did not accept that field layout and has gone quiet. Put the
    // default sentence set back rather than leave the receiver mute.
    gpsSend(PCAS_NMEA_DEFAULT);
    delay(150);
  }
#endif
}

static void publishFix() {
  GpsFix f;
  f.lat       = gps.location.lat();
  f.lon       = gps.location.lng();
  f.haveAlt   = gps.altitude.isValid();
  f.altMSL    = f.haveAlt ? (float)gps.altitude.meters() : 0.0f;
  // value() is hundredths of HDOP in every TinyGPSPlus release; hdop() is not.
  f.hdop      = gps.hdop.isValid() ? ((float)gps.hdop.value() * 0.01f) : 99.0f;
  f.sats      = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  f.speedMps  = gps.speed.isValid() ? (float)gps.speed.mps() : 0.0f;
  f.courseDeg = gps.course.isValid() ? (float)gps.course.deg() : 0.0f;
  f.haveVel   = gps.speed.isValid() && gps.course.isValid() &&
                (f.speedMps > GPS_COURSE_MIN_MPS);
  f.stampMs   = millis();

  mutex_enter_blocking(&g_gpsMtx);
  f.seq = g_gpsShared.seq + 1;
  if (f.seq == 0) f.seq = 1;         // 0 means "never had a fix"
  g_gpsShared = f;
  mutex_exit(&g_gpsMtx);

  lastPublishMs = f.stampMs;
}

static void handleCommand(char c) {
  switch (c) {
    case 'd':
      g_rawNmeaDebug = !g_rawNmeaDebug;
      Serial.println(g_rawNmeaDebug ? F("[CMD] raw NMEA ON") : F("[CMD] raw NMEA OFF"));
      break;
    case 'v':
      g_csvOutput = !g_csvOutput;
      if (g_csvOutput) {
        Serial.println(F("ms,roll,pitch,yaw,aN,aE,aD,vN,vE,vD,pN,pE,pD,lat,lon,alt,sigPos,sats,hdop,mode"));
      }
      break;
    case 'z':
      g_resetRequest = true;
      Serial.println(F("[CMD] navigation filter reset requested"));
      break;
    case 'r':
      gpsSend(PCAS_RATE_10HZ);
      Serial.println(F("[CMD] GNSS -> 10 Hz"));
      break;
    case 's':
      gpsSend(PCAS_RATE_1HZ);
      Serial.println(F("[CMD] GNSS -> 1 Hz"));
      break;
    case 'h':
      Serial.println(F("[CMD] d=raw NMEA  v=CSV  z=reset filter  r=10Hz  s=1Hz"));
      break;
    default:
      break;
  }
}

static const char *navModeName(const NavOut &n) {
  if (n.gpsAiding) return n.zupt ? "GPS+ZUPT" : "GPS";
  return n.zupt ? "INS+ZUPT" : "INS";
}

static void printTelemetry(const NavOut &n) {
  if (g_csvOutput) {
    Serial.print(millis());       Serial.print(',');
    Serial.print(n.roll, 2);      Serial.print(',');
    Serial.print(n.pitch, 2);     Serial.print(',');
    Serial.print(n.yaw, 2);       Serial.print(',');
    Serial.print(n.aN, 3);        Serial.print(',');
    Serial.print(n.aE, 3);        Serial.print(',');
    Serial.print(n.aD, 3);        Serial.print(',');
    Serial.print(n.vN, 3);        Serial.print(',');
    Serial.print(n.vE, 3);        Serial.print(',');
    Serial.print(n.vD, 3);        Serial.print(',');
    Serial.print(n.pN, 2);        Serial.print(',');
    Serial.print(n.pE, 2);        Serial.print(',');
    Serial.print(n.pD, 2);        Serial.print(',');
    Serial.print(n.lat, 7);       Serial.print(',');
    Serial.print(n.lon, 7);       Serial.print(',');
    Serial.print(n.altMSL, 2);    Serial.print(',');
    Serial.print(n.sigmaPosH, 2); Serial.print(',');
    Serial.print(n.sats);         Serial.print(',');
    Serial.print(n.hdop, 1);      Serial.print(',');
    Serial.println(navModeName(n));
    return;
  }

  Serial.print(F("RPY "));
  Serial.print(n.roll, 1);  Serial.print(' ');
  Serial.print(n.pitch, 1); Serial.print(' ');
  Serial.print(n.yaw, 1);

  Serial.print(F(" | a[NED] "));
  Serial.print(n.aN, 2); Serial.print(' ');
  Serial.print(n.aE, 2); Serial.print(' ');
  Serial.print(n.aD, 2);

  Serial.print(F(" | v[NED] "));
  Serial.print(n.vN, 2); Serial.print(' ');
  Serial.print(n.vE, 2); Serial.print(' ');
  Serial.print(n.vD, 2);

  Serial.print(F(" | p[NED] "));
  Serial.print(n.pN, 1); Serial.print(' ');
  Serial.print(n.pE, 1); Serial.print(' ');
  Serial.print(n.pD, 1);

  if (n.originSet) {
    Serial.print(F(" | "));
    Serial.print(n.lat, 7); Serial.print(' ');
    Serial.print(n.lon, 7); Serial.print(' ');
    Serial.print(n.altMSL, 1); Serial.print(F("m"));
  } else {
    Serial.print(F(" | no fix yet"));
  }

  Serial.print(F(" | +/-"));  Serial.print(n.sigmaPosH, 1); Serial.print(F("m"));
  Serial.print(F(" sats "));  Serial.print(n.sats);
  Serial.print(F(" hdop "));  Serial.print(n.hdop, 1);
  if (n.fixAgeMs != 0xFFFFFFFFUL) {
    Serial.print(F(" age ")); Serial.print(n.fixAgeMs); Serial.print(F("ms"));
  }
  Serial.print(F(" | "));     Serial.print(navModeName(n));
  Serial.print(' ');          Serial.print(n.fusionHz, 0); Serial.print(F("Hz"));

  if (n.fifoOverflows) {
    Serial.print(F("  !! IMU FIFO OVERFLOW x"));
    Serial.print(n.fifoOverflows);
  }
  Serial.println();

  if (n.headingHint != 0 && !headingHintShown) {
    headingHintShown = true;
    if (n.headingHint > 0) {
      Serial.println(F("[CHECK] Yaw tracks GPS course - YAW_IS_COMPASS_HEADING is correct."));
    } else {
      Serial.println(F("[CHECK] Yaw disagrees with GPS course by a mirror flip."));
      Serial.println(F("[CHECK] Invert YAW_IS_COMPASS_HEADING, negate MAGNETIC_DECLINATION, reflash."));
    }
  }
}

void setup1() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (uint32_t)(millis() - t0) < 3000) delay(10);

  Serial.println(F("--- Voltino TriSense + Quectel L76K : GPS/INS localization ---"));
  Serial.println(F("Keys: d=raw NMEA  v=CSV  z=reset filter  r=10Hz  s=1Hz  h=help"));

  gpsConfigure();
  Serial.print(F("[GPS] UART @ "));
  Serial.println(gpsBaud);
  Serial.println(F("[SYS] core 0 is calibrating the gyro - keep the board still..."));
}

void loop1() {
  // 1. GNSS bytes.
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    if (g_rawNmeaDebug) Serial.write(c);
    if (gps.encode(c)) {
      // RMC closes the NMEA epoch, so at that point position (GGA), altitude,
      // satellites, HDOP, speed and course all belong to the same solution.
      // Publishing on every sentence instead would hand the filter the same fix
      // two or more times and make it over-confident.
      if (gps.date.isUpdated()) {
        (void)gps.date.value();                 // consume the updated flag
        if (gps.location.isValid()) publishFix();
      } else if (gps.location.isUpdated() && gps.location.isValid() &&
                 (uint32_t)(millis() - lastPublishMs) > 1500) {
        (void)gps.location.lat();               // fallback if RMC is disabled
        publishFix();
      }
    }
  }

  // 2. Host commands.
  while (Serial.available()) handleCommand((char)Serial.read());

  // 3. Telemetry.
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
    printTelemetry(n);
  }
}
