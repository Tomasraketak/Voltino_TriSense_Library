/*
 * inslib_config.h - build configuration for the vendored INSLIB/KFCore sources.
 *
 * Every generated src/inslib_tu_*.c includes this before the INSLIB source it
 * compiles, and GPS_INS_INSLIB.ino includes it before the INSLIB headers. That
 * is essential, not cosmetic: INS_UNKNOWNS_MAX changes the layout of ins_t, so
 * every translation unit that sees ins.h must agree on it. If you add a file
 * that touches INSLIB, include this first.
 *
 * These are the knobs that decide whether the filter fits on an RP2350 at all.
 */

#ifndef INSLIB_CONFIG_H
#define INSLIB_CONFIG_H

/* ---------------------------------------------------------------------------
 * State vector size - the one that decides the stack budget
 * ---------------------------------------------------------------------------
 * INSLIB dimensions its covariance and history arrays at compile time from
 * INS_UNKNOWNS_MAX, and KFCore dimensions the scratch matrices inside
 * kalman_udu_predict() from KALMAN_MAX_STATE_SIZE / KALMAN_MAX_NOISE_SIZE. All
 * of that lands on the stack of whoever calls nav_suite_update(). Measured with
 * INSLIB's own call-graph stack analysis (`make stack`) against the exact
 * toolchain and flags arduino-pico uses for a Pico 2 - its own arm-none-eabi-gcc
 * 16.1.0, cortex-m33, armv8-m.main+fp+dsp, -Os:
 *
 *     INS_UNKNOWNS_MAX=15, KALMAN 15/21  ->  nav_suite_update() needs  6776 B
 *     INS_UNKNOWNS_MAX=18, KALMAN 18/24  ->  nav_suite_update() needs  8528 B
 *     INS_UNKNOWNS_MAX=15, KFCore 32/32  ->  nav_suite_update() needs 12880 B
 *
 * (-O2 moves the first figure by 24 bytes, so whichever optimization level you
 * pick in the Tools menu, the conclusion is the same.)
 *
 * A core on arduino-pico has 8192 bytes of stack. So 15 states fits with about
 * 1.4 KB to spare, 18 states does not fit at all, and leaving KFCore at its
 * defaults overflows inside the first Kalman prediction - which on a Cortex-M
 * is not a crash but silent corruption of whatever sits below the stack.
 *
 * That last line is the one worth staring at: KFCore's defaults are not
 * pathological, they are simply sized for a desktop. Nothing warns you.
 *
 * 15 is the full INS state vector: position, velocity, attitude, accelerometer
 * bias, gyroscope bias. The only thing it gives up is the optional 3-state
 * magnetometer hard-iron estimator (ins_options_t.estimate_mag_bias), which
 * needs 18. That is no loss here: the TriSense magnetometer is calibrated
 * offline with the MotionCal example and the result is handed to the filter as
 * opt.mag_fixed_bias / opt.mag_misalignment, which is the better answer anyway
 * (a fixed bias measured over a full sphere beats one estimated from whatever
 * rotations the vehicle happened to perform).
 *
 * Raising this to 18 will compile and then misbehave at run time. If you need
 * the mag-bias states, move the filter off the Arduino core's stack first.
 */
#define INS_UNKNOWNS_MAX      15
#define KALMAN_MAX_STATE_SIZE 15
/* INS_NOISE_COLS_MAX == 12 + INS_UNKNOWNS_MAX - 6 == 21. ins.c turns a wrong
 * value here into a _Static_assert failure rather than an overflowing write. */
#define KALMAN_MAX_NOISE_SIZE 21

/* ---------------------------------------------------------------------------
 * Logging off
 * ---------------------------------------------------------------------------
 * INSLIB's default log sink is printf(). On the navigation core that is a
 * blocking call that can take longer than the epoch it was logging about, and
 * it drags ~2 KB of printf stack into the worst-case path of
 * nav_suite_correct_step(). At LOG_LEVEL 0 (LOG_LEVEL_NONE) every log call
 * disappears at the preprocessor, so there is nothing left to cost anything.
 *
 * The literal 0 rather than LOG_LEVEL_NONE is deliberate - the symbolic name is
 * defined inside log.h, i.e. after this header has been read.
 *
 * Turning logging back on is genuinely useful while bringing a new vehicle up
 * ("GNSS fusable but below the 3D entry gate" explains a filter that never
 * starts). Set LOG_LEVEL to 3 (WARN), install a sink with log_set_sink() that
 * queues to the telemetry core, and do NOT leave printf as the sink.
 */
#define LOG_LEVEL 0

/* ---------------------------------------------------------------------------
 * assert() off
 * ---------------------------------------------------------------------------
 * The release setting for an embedded build, and the one the stack figures
 * above were measured with. The size checks that actually matter here are
 * _Static_assert in ins.c and fire at compile time regardless.
 */
#ifndef NDEBUG
#define NDEBUG 1
#endif

#endif /* INSLIB_CONFIG_H */
