/*
 * calib.c
 *
 * See calib.h for the module-level design summary. Implementation
 * notes specific to this file:
 *
 * SPIN's gyro-vs-kinematics comparison: both angle estimates are
 * integrated every tick -- including through the post-target coast
 * phase, not just while actively commanding -- so a leg's cached
 * totals reflect the SAME physical rotation for both sources (a
 * spinning mass doesn't stop the instant power is cut; if only the
 * gyro kept integrating through that coast, the two totals would
 * diverge by the coast amount and corrupt the calibration ratio).
 * The kinematics-implied angle is measured independently here (raw
 * encoder delta counts -> radians -> KiwiKinematics_WheelDeltaToBodyDelta),
 * NOT read from odometry.c, keeping the modules decoupled -- odometry
 * already does the same conversion internally for its own pose
 * estimate, but exposing that would couple calib.c to odometry.c's
 * internals for no benefit.
 *
 * ROLL has no such two-source-divergence concern: Odometry_GetPosition
 * is live and directly encoder-driven every control tick regardless of
 * cal state (main.c calls Odometry_Update unconditionally), so simply
 * comparing current position against the position cached at test start
 * is sufficient -- no coast/settle staging needed.
 */
#include "calib.h"
#include "odometry.h"
#include "kiwi_kinematics.h"
#include "motor_controller.h"
#include "encoder.h"
#include "comm.h"
#include "waypoint_nav.h"      /* NAV_RPM_AT_100PCT -- shared physical wheel-RPM ceiling */
#include "stm32f1xx_hal.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265359f
#endif
#define CAL_RAD2DEG  ((float)(180.0 / M_PI))

extern KiwiKinematics_t   g_kin;
extern Odometry_t         g_odom;
extern Encoder_t          enc1;
extern Encoder_t          enc2;
extern Encoder_t          enc3;
extern MotorController_t  ctrl1;
extern MotorController_t  ctrl2;
extern MotorController_t  ctrl3;

/* Wheel RPM at 100% cal power -- same physical ceiling as waypoint
 * patrol, per the plan's explicit "reuse NAV_RPM_AT_100PCT" call. A
 * single #define alias rather than a duplicated magic number, so the
 * two can never silently drift apart. */
#define CAL_RPM_AT_100PCT        NAV_RPM_AT_100PCT

#define CAL_POWER_PCT_MIN        10.0f
#define CAL_POWER_PCT_MAX        100.0f

/* Sanity floors -- reject a pathologically small target (e.g. a
 * malformed/zero payload) rather than let it produce a near-instant,
 * meaningless "leg" and a divide-by-near-zero in the Apply formulas. */
#define CAL_SPIN_MIN_DEG         10.0f
#define CAL_ROLL_MIN_DIST_M      0.1f

/* Coast-settle confirmation, same pattern/period as waypoint_nav.c's
 * NAV_SETTLE_TICKS (10 ms ticks -> 50 ms of confirmed-zero wheel
 * deltas). Kept as calib.c's own constant rather than reusing Nav's
 * directly -- same reasoning as the plan's independent-measurement
 * choice above: decoupled modules, coincidentally the same tuning. */
#define CAL_SETTLE_TICKS         5u

#define CAL_STATUS_PERIOD_MS     200u   /* 5 Hz in-progress stream, matches NAV_STATUS_PERIOD_MS */

/* Per-test safety timeout, scaled to the requested target rather than
 * a flat constant: expected duration at the derived rate, x3 margin,
 * with a floor covering PID ramp-up on tiny targets. Guards against a
 * runaway/stuck test (e.g. badly wrong existing gyro_scale making a
 * SPIN leg never see its gyro target) -- not needed for correctness
 * of a normal run, purely a safety backstop. */
#define CAL_TIMEOUT_SAFETY_FACTOR 3.0f
#define CAL_TIMEOUT_FLOOR_MS      3000u

/* ---- Module state ------------------------------------------------- */
static CalMode_t s_mode   = CAL_IDLE;
static bool      s_active = false;
static float     s_power_pct = CAL_POWER_PCT_MAX;

static uint32_t  s_start_tick_ms = 0;
static uint32_t  s_timeout_ms    = CAL_TIMEOUT_FLOOR_MS;
static uint32_t  s_last_status_ms = 0;

/* ---- SPIN ----------------------------------------------------------- */
static uint8_t   s_spin_leg       = 0;      /* 0 = CCW (leg 1), 1 = CW (leg 2) */
static bool      s_spin_coasting  = false;
static bool      s_leg_timed_out  = false;
static float     s_spin_deg_ccw_target = 0.0f;
static float     s_spin_deg_cw_target  = 0.0f;
static float     s_gyro_accum_deg = 0.0f;
static float     s_kin_accum_deg  = 0.0f;
static int32_t   s_prev_enc[3]    = { 0, 0, 0 };
static uint8_t   s_settle_ticks   = 0;

static float     s_g1_gyro_deg = 0.0f, s_g1_kin_deg = 0.0f;
static float     s_g2_gyro_deg = 0.0f, s_g2_kin_deg = 0.0f;
static bool      s_spin_result_valid = false;

/* ---- ROLL ------------------------------------------------------------ */
static float     s_roll_dist_target_m = 0.0f;
static float     s_roll_x0 = 0.0f, s_roll_y0 = 0.0f;
static float     s_dist_estimated_m   = 0.0f;
static bool      s_roll_result_valid  = false;

/* ---- Small helpers ---------------------------------------------------- */

/* Scales a 0-100 cal power percentage into a body omega (rad/s) such
 * that the FASTEST of the 3 wheels reaches exactly CAL_RPM_AT_100PCT
 * at 100% -- identical structure to waypoint_nav.c's
 * nav_pct_to_speed_mps, computed live from g_kin so it stays correct
 * for the real mounting geometry. */
static float cal_pct_to_omega_radps(float pct)
{
    float w1, w2, w3;
    KiwiKinematics_InverseKinematics(&g_kin, 0.0f, 0.0f, 1.0f, &w1, &w2, &w3);
    float max_abs = fmaxf(fabsf(w1), fmaxf(fabsf(w2), fabsf(w3)));
    if (max_abs < 1e-6f) return 0.0f;
    float omega_at_100 = CAL_RPM_AT_100PCT / max_abs;
    return (pct / 100.0f) * omega_at_100;
}

/* Same idea for a pure +X body speed. */
static float cal_pct_to_vx_mps(float pct)
{
    float w1, w2, w3;
    KiwiKinematics_InverseKinematics(&g_kin, 1.0f, 0.0f, 0.0f, &w1, &w2, &w3);
    float max_abs = fmaxf(fabsf(w1), fmaxf(fabsf(w2), fabsf(w3)));
    if (max_abs < 1e-6f) return 0.0f;
    float vx_at_100 = CAL_RPM_AT_100PCT / max_abs;
    return (pct / 100.0f) * vx_at_100;
}

/* Coasts (free-wheel), not an active brake -- see motor.c's MOTOR_STOP
 * vs MOTOR_BRAKE and waypoint_nav.c's identical nav_brake(). The SPIN
 * coast phase specifically relies on this NOT being an active brake,
 * so the true mechanical coast rotation is what both angle estimates
 * measure. */
static void cal_drive_stop(void)
{
    MotorController_Stop(&ctrl1);
    MotorController_Stop(&ctrl2);
    MotorController_Stop(&ctrl3);
}

static void cal_send_status(uint8_t type, uint8_t leg, uint8_t done,
                             uint8_t reserved, float val1, float val2)
{
    uint8_t buf[14];
    buf[0] = type;
    buf[1] = leg;
    buf[2] = done;
    buf[3] = reserved;
    memcpy(buf + 4, &val1, 4);
    memcpy(buf + 8, &val2, 4);
    Comm_SendPacket(CMD_CAL_STATUS, buf, sizeof(buf));
    s_last_status_ms = HAL_GetTick();
}

/* expected_duration = target / rate, x3 safety margin, floor for
 * ramp-up. rate <= 0 (degenerate kinematics) falls back to the floor
 * alone rather than a divide-by-zero. */
static uint32_t cal_compute_timeout_ms(float target_magnitude, float rate_magnitude)
{
    if (rate_magnitude < 1e-6f) return CAL_TIMEOUT_FLOOR_MS;
    float expected_s = target_magnitude / rate_magnitude;
    uint32_t ms = (uint32_t)(expected_s * 1000.0f * CAL_TIMEOUT_SAFETY_FACTOR);
    return (ms < CAL_TIMEOUT_FLOOR_MS) ? CAL_TIMEOUT_FLOOR_MS : ms;
}

static float cal_roll_distance_now(void)
{
    float x, y, th;
    Odometry_GetPosition(&g_odom, &x, &y, &th);
    (void)th;
    float dx = x - s_roll_x0;
    float dy = y - s_roll_y0;
    return sqrtf(dx * dx + dy * dy);
}

/* ---- Per-mode tick handlers -------------------------------------------- */

static void cal_update_spin(float dtheta_gyro_rad)
{
    /* Always integrate both estimates -- see file header comment on
     * why this must run through the coast phase too. */
    s_gyro_accum_deg += dtheta_gyro_rad * CAL_RAD2DEG;

    int32_t c1 = Encoder_GetPosition(&enc1);
    int32_t c2 = Encoder_GetPosition(&enc2);
    int32_t c3 = Encoder_GetPosition(&enc3);
    float w1 = ((float)(c1 - s_prev_enc[0]) / ENCODER_PPR) * 2.0f * (float)M_PI;
    float w2 = ((float)(c2 - s_prev_enc[1]) / ENCODER_PPR) * 2.0f * (float)M_PI;
    float w3 = ((float)(c3 - s_prev_enc[2]) / ENCODER_PPR) * 2.0f * (float)M_PI;
    s_prev_enc[0] = c1; s_prev_enc[1] = c2; s_prev_enc[2] = c3;

    float dx, dy, dtheta_kin;
    KiwiKinematics_WheelDeltaToBodyDelta(&g_kin, w1, w2, w3, &dx, &dy, &dtheta_kin);
    s_kin_accum_deg += dtheta_kin * CAL_RAD2DEG;

    float target_deg = (s_spin_leg == 0u) ? s_spin_deg_ccw_target : s_spin_deg_cw_target;

    if (!s_spin_coasting) {
        bool target_hit = fabsf(s_gyro_accum_deg) >= target_deg;
        bool timed_out  = (HAL_GetTick() - s_start_tick_ms) > s_timeout_ms;

        if (target_hit || timed_out) {
            /* Target hit (or safety timeout): stop commanding and
             * coast -- same two-stage pattern waypoint_nav.c's
             * DRIVING->SETTLING transition uses. */
            cal_drive_stop();
            s_spin_coasting = true;
            s_settle_ticks  = 0;
            s_leg_timed_out = timed_out;
        } else {
            float omega_sign = (s_spin_leg == 0u) ? 1.0f : -1.0f;
            float omega = omega_sign * cal_pct_to_omega_radps(s_power_pct);
            float wo1, wo2, wo3;
            KiwiKinematics_InverseKinematics(&g_kin, 0.0f, 0.0f, omega, &wo1, &wo2, &wo3);
            MotorController_SetSpeed(&ctrl1, wo1);
            MotorController_SetSpeed(&ctrl2, wo2);
            MotorController_SetSpeed(&ctrl3, wo3);
        }
        return;
    }

    /* Coasting: wait for the wheels to actually stop before trusting
     * either accumulated total as final (same
     * Odometry_GetLastDeltas-zero check waypoint_nav.c's SETTLING
     * uses; Odometry_Update already refreshes these every control
     * tick in main.c regardless of cal state). */
    int32_t d1, d2, d3;
    Odometry_GetLastDeltas(&g_odom, &d1, &d2, &d3);
    s_settle_ticks = (d1 == 0 && d2 == 0 && d3 == 0) ? (uint8_t)(s_settle_ticks + 1) : 0u;

    if (s_settle_ticks < CAL_SETTLE_TICKS) return;

    /* Cache as MAGNITUDES, not signed accumulators. s_gyro_accum_deg
     * carries the natural CCW-positive sign (positive on leg 1,
     * negative on leg 2) -- summing the two SIGNED totals would
     * nearly cancel to ~0 for a well-executed symmetric test, instead
     * of the "total rotation across both legs" the Apply-handler
     * formulas in main.c need. Same reasoning applies to the
     * kinematics-implied angle (its sign tracks the gyro's by
     * construction). */
    if (s_spin_leg == 0u) { s_g1_gyro_deg = fabsf(s_gyro_accum_deg); s_g1_kin_deg = fabsf(s_kin_accum_deg); }
    else                  { s_g2_gyro_deg = fabsf(s_gyro_accum_deg); s_g2_kin_deg = fabsf(s_kin_accum_deg); }

    cal_send_status(0u, s_spin_leg, 1u, s_leg_timed_out ? 1u : 0u,
                     fabsf(s_gyro_accum_deg), fabsf(s_kin_accum_deg));

    if (s_spin_leg == 0u) {
        /* Leg 1 (CCW) done -> start leg 2 (CW), fresh accumulators,
         * fresh timeout scaled to leg 2's own target. */
        s_spin_leg       = 1u;
        s_spin_coasting  = false;
        s_gyro_accum_deg = 0.0f;
        s_kin_accum_deg  = 0.0f;
        s_settle_ticks   = 0u;
        s_leg_timed_out  = false;
        s_start_tick_ms  = HAL_GetTick();
        s_timeout_ms     = cal_compute_timeout_ms(
                               s_spin_deg_cw_target,
                               cal_pct_to_omega_radps(s_power_pct) * CAL_RAD2DEG);
        s_prev_enc[0] = Encoder_GetPosition(&enc1);
        s_prev_enc[1] = Encoder_GetPosition(&enc2);
        s_prev_enc[2] = Encoder_GetPosition(&enc3);
    } else {
        /* Both legs done -> full SPIN test complete. */
        s_spin_result_valid = true;
        s_active = false;
        s_mode   = CAL_IDLE;
    }
}

static void cal_update_roll(void)
{
    float dist = cal_roll_distance_now();
    bool reached   = dist >= s_roll_dist_target_m;
    bool timed_out = (HAL_GetTick() - s_start_tick_ms) > s_timeout_ms;

    if (reached || timed_out) {
        cal_drive_stop();
        s_dist_estimated_m  = dist;
        s_roll_result_valid = true;
        s_active = false;
        s_mode   = CAL_IDLE;
        cal_send_status(1u, 0u, 1u, timed_out ? 1u : 0u, s_dist_estimated_m, 0.0f);
        return;
    }

    float vx = cal_pct_to_vx_mps(s_power_pct);
    float w1, w2, w3;
    KiwiKinematics_InverseKinematics(&g_kin, vx, 0.0f, 0.0f, &w1, &w2, &w3);
    MotorController_SetSpeed(&ctrl1, w1);
    MotorController_SetSpeed(&ctrl2, w2);
    MotorController_SetSpeed(&ctrl3, w3);
}

/* ---- Public API -------------------------------------------------------- */

void Cal_Init(void)
{
    s_mode   = CAL_IDLE;
    s_active = false;
    s_power_pct = CAL_POWER_PCT_MAX;

    s_start_tick_ms  = 0;
    s_timeout_ms     = CAL_TIMEOUT_FLOOR_MS;
    s_last_status_ms = 0;

    s_spin_leg = 0u;
    s_spin_coasting = false;
    s_leg_timed_out = false;
    s_spin_deg_ccw_target = 0.0f;
    s_spin_deg_cw_target  = 0.0f;
    s_gyro_accum_deg = 0.0f;
    s_kin_accum_deg  = 0.0f;
    s_prev_enc[0] = s_prev_enc[1] = s_prev_enc[2] = 0;
    s_settle_ticks = 0u;
    s_g1_gyro_deg = s_g1_kin_deg = 0.0f;
    s_g2_gyro_deg = s_g2_kin_deg = 0.0f;
    s_spin_result_valid = false;

    s_roll_dist_target_m = 0.0f;
    s_roll_x0 = s_roll_y0 = 0.0f;
    s_dist_estimated_m  = 0.0f;
    s_roll_result_valid = false;
}

void Cal_StartSpin(float deg_ccw_target, float deg_cw_target, float power_pct)
{
    if (s_active) return;

    float ccw = fabsf(deg_ccw_target);
    float cw  = fabsf(deg_cw_target);
    if (ccw < CAL_SPIN_MIN_DEG) ccw = CAL_SPIN_MIN_DEG;
    if (cw  < CAL_SPIN_MIN_DEG) cw  = CAL_SPIN_MIN_DEG;

    float pct = power_pct;
    if (pct < CAL_POWER_PCT_MIN) pct = CAL_POWER_PCT_MIN;
    if (pct > CAL_POWER_PCT_MAX) pct = CAL_POWER_PCT_MAX;

    s_mode = CAL_SPIN;
    s_spin_deg_ccw_target = ccw;
    s_spin_deg_cw_target  = cw;
    s_power_pct = pct;

    s_spin_leg       = 0u;      /* leg 1: CCW */
    s_spin_coasting  = false;
    s_gyro_accum_deg = 0.0f;
    s_kin_accum_deg  = 0.0f;
    s_settle_ticks   = 0u;
    s_leg_timed_out  = false;
    s_spin_result_valid = false;   /* new test -- invalidate any stale cached result */

    s_prev_enc[0] = Encoder_GetPosition(&enc1);
    s_prev_enc[1] = Encoder_GetPosition(&enc2);
    s_prev_enc[2] = Encoder_GetPosition(&enc3);

    s_start_tick_ms = HAL_GetTick();
    s_timeout_ms    = cal_compute_timeout_ms(ccw, cal_pct_to_omega_radps(pct) * CAL_RAD2DEG);
    s_last_status_ms = s_start_tick_ms;

    s_active = true;
}

void Cal_StartRoll(float dist_m, float power_pct)
{
    if (s_active) return;

    float dist = fabsf(dist_m);
    if (dist < CAL_ROLL_MIN_DIST_M) dist = CAL_ROLL_MIN_DIST_M;

    float pct = power_pct;
    if (pct < CAL_POWER_PCT_MIN) pct = CAL_POWER_PCT_MIN;
    if (pct > CAL_POWER_PCT_MAX) pct = CAL_POWER_PCT_MAX;

    s_mode = CAL_ROLL;
    s_roll_dist_target_m = dist;
    s_power_pct = pct;
    s_roll_result_valid = false;   /* new test -- invalidate any stale cached result */

    float th;
    Odometry_GetPosition(&g_odom, &s_roll_x0, &s_roll_y0, &th);
    (void)th;

    s_start_tick_ms = HAL_GetTick();
    s_timeout_ms    = cal_compute_timeout_ms(dist, cal_pct_to_vx_mps(pct));
    s_last_status_ms = s_start_tick_ms;

    s_active = true;
}

void Cal_Abort(void)
{
    if (!s_active) return;

    uint8_t type = (s_mode == CAL_SPIN) ? 0u : 1u;
    uint8_t leg  = (s_mode == CAL_SPIN) ? s_spin_leg : 0u;

    cal_drive_stop();
    s_active = false;
    s_mode   = CAL_IDLE;

    /* reserved=1 -> aborted, not a clean target/timeout completion.
     * Does NOT touch s_spin_result_valid/s_roll_result_valid -- a
     * previously COMPLETED result (if any) stays available to Apply;
     * only starting a new test invalidates it (see Cal_StartSpin/Roll). */
    cal_send_status(type, leg, 1u, 1u, 0.0f, 0.0f);
}

bool Cal_IsActive(void) { return s_active; }

OdomMotionMode_t Cal_GetMotionMode(void)
{
    if (!s_active) return ODOM_IDLE;
    return (s_mode == CAL_SPIN) ? ODOM_ROTATING : ODOM_STRAIGHT;
}

void Cal_Update(float dt, float dtheta_gyro_rad)
{
    if (!s_active) return;

    if (s_mode == CAL_SPIN)      cal_update_spin(dtheta_gyro_rad);
    else if (s_mode == CAL_ROLL) cal_update_roll();

    /* Rate-limited in-progress stream. Skipped when the mode-specific
     * update above just finished the test (s_active already false) --
     * that path already sent its own done=1 status. */
    if (s_active && (HAL_GetTick() - s_last_status_ms) >= CAL_STATUS_PERIOD_MS) {
        if (s_mode == CAL_SPIN) {
            cal_send_status(0u, s_spin_leg, 0u, 0u, fabsf(s_gyro_accum_deg), fabsf(s_kin_accum_deg));
        } else if (s_mode == CAL_ROLL) {
            cal_send_status(1u, 0u, 0u, 0u, cal_roll_distance_now(), 0.0f);
        }
    }

    /* Cal owns the inner wheel-speed PID loop while active -- the
     * manual-drive branch in main.c that normally calls this is
     * skipped whenever Cal_IsActive(), exactly like Nav_Update. */
    MotorController_Update(&ctrl1, dt);
    MotorController_Update(&ctrl2, dt);
    MotorController_Update(&ctrl3, dt);
}

bool Cal_GetSpinResult(float *g1_gyro_deg, float *g1_kin_deg,
                        float *g2_gyro_deg, float *g2_kin_deg)
{
    if (!s_spin_result_valid) return false;
    if (g1_gyro_deg) *g1_gyro_deg = s_g1_gyro_deg;
    if (g1_kin_deg)  *g1_kin_deg  = s_g1_kin_deg;
    if (g2_gyro_deg) *g2_gyro_deg = s_g2_gyro_deg;
    if (g2_kin_deg)  *g2_kin_deg  = s_g2_kin_deg;
    return true;
}

bool Cal_GetRollResult(float *dist_estimated_m)
{
    if (!s_roll_result_valid) return false;
    if (dist_estimated_m) *dist_estimated_m = s_dist_estimated_m;
    return true;
}

void Cal_ClearSpinResult(void) { s_spin_result_valid = false; }
void Cal_ClearRollResult(void) { s_roll_result_valid = false; }
