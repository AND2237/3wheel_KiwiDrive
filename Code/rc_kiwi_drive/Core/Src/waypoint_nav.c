/*
 * waypoint_nav.c
 *
 *  Created on: Aug 28, 2026
 *      Author: Alireza
 */
#include "waypoint_nav.h"
#include "odometry.h"
#include "motor_controller.h"
#include "kiwi_kinematics.h"
#include "comm.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265359f
#endif
#define NAV_DEG2RAD  ((float)(M_PI / 180.0))
#define NAV_RAD2DEG  ((float)(180.0 / M_PI))

extern Odometry_t         g_odom;
extern KiwiKinematics_t   g_kin;
extern MotorController_t  ctrl1;
extern MotorController_t  ctrl2;
extern MotorController_t  ctrl3;

typedef struct { float x, y; } NavWp_t;

/* ---- Module state ---------------------------------------------------- */
static NavWp_t    s_wps[NAV_MAX_WAYPOINTS];
static uint8_t    s_wp_count       = 0;
static NavState_t s_state          = NAV_STATE_IDLE;
static bool       s_active         = false;
static uint8_t    s_target         = 0;
static uint8_t    s_speed_pct      = 95;
static uint8_t    s_fault          = NAV_FAULT_NONE;
static float      s_dist_to_target = 0.0f;

static uint8_t    s_settle_ticks   = 0;
static uint16_t   s_settling_total_ticks = 0;

/* encoder-disconnect supervision, one counter per wheel */
static uint16_t   s_enc_zero[3] = { 0, 0, 0 };

static uint32_t   s_last_status_ms = 0;

/* ==== TEMPORARY DEBUG (remove after diagnosis) ========================= */
#if NAV_DEBUG_ENABLED

static float nav_wrap_rad(float a);
static void  nav_target_geometry(float px, float py, float *dist, float *bearing);

static uint32_t s_last_debug_ms  = 0;

static struct {
    uint8_t state;
    uint8_t target;
    uint8_t wp_count;
    uint8_t flags;
    uint8_t fault;
    uint8_t settle_ticks;
    int16_t d1, d2, d3;
    float   dist;
    float   rate_dps;
    float   theta_deg;
    float   bearing_deg;
    float   err_deg;
    float   cmd1_rpm, cmd2_rpm, cmd3_rpm;
} s_dbg;

static void nav_refresh_debug(float dist, float bearing, float theta,
                              float rate_dps, float err_deg)
{
    int32_t d1, d2, d3;
    Odometry_GetLastDeltas(&g_odom, &d1, &d2, &d3);
    s_dbg.state        = (uint8_t)s_state;
    s_dbg.target       = s_target;
    s_dbg.wp_count     = s_wp_count;
    s_dbg.flags        = (uint8_t)((s_active?2u:0u));
    s_dbg.fault        = s_fault;
    s_dbg.settle_ticks = s_settle_ticks;
    s_dbg.d1           = (int16_t)d1;
    s_dbg.d2           = (int16_t)d2;
    s_dbg.d3           = (int16_t)d3;
    s_dbg.dist         = dist;
    s_dbg.rate_dps     = rate_dps;
    s_dbg.theta_deg    = theta * NAV_RAD2DEG;
    s_dbg.bearing_deg  = bearing * NAV_RAD2DEG;
    s_dbg.err_deg      = err_deg;
    s_dbg.cmd1_rpm     = ctrl1.target_rpm;
    s_dbg.cmd2_rpm     = ctrl2.target_rpm;
    s_dbg.cmd3_rpm     = ctrl3.target_rpm;
}

static void nav_send_debug(void)
{
    uint8_t buf[NAV_DEBUG_PKT_BYTES];
    buf[0] = s_dbg.state;
    buf[1] = s_dbg.target;
    buf[2] = s_dbg.wp_count;
    buf[3] = s_dbg.flags;
    buf[4] = s_dbg.fault;
    buf[5] = s_dbg.settle_ticks;
    memcpy(buf +  6, &s_dbg.d1,           2);
    memcpy(buf +  8, &s_dbg.d2,           2);
    memcpy(buf + 10, &s_dbg.d3,           2);
    memcpy(buf + 12, &s_dbg.dist,         4);
    memcpy(buf + 16, &s_dbg.rate_dps,     4);
    memcpy(buf + 20, &s_dbg.theta_deg,    4);
    memcpy(buf + 24, &s_dbg.bearing_deg,  4);
    memcpy(buf + 28, &s_dbg.err_deg,      4);
    memcpy(buf + 32, &s_dbg.cmd1_rpm,     4);
    memcpy(buf + 36, &s_dbg.cmd2_rpm,     4);
    memcpy(buf + 40, &s_dbg.cmd3_rpm,     4);
    Comm_SendPacket(CMD_NAV_DEBUG, buf, sizeof(buf));
}

/* Rate-limited debug stream. Works in ALL modes:
 *  - when nav is ACTIVE, Nav_Update() already refreshed s_dbg this tick,
 *    so we only send;
 *  - when nav is IDLE (free-drive / survey), we refresh s_dbg here with
 *    live odometry values so the stream stays meaningful. */
void Nav_DebugTick(void)
{
#if NAV_DEBUG_ENABLED
    uint32_t now = HAL_GetTick();
    if ((now - s_last_debug_ms) < NAV_DEBUG_PERIOD_MS) return;
    float dt_dbg = (float)(now - s_last_debug_ms) * 0.001f;
    s_last_debug_ms = now;

    float px, py, theta;
    Odometry_GetPosition(&g_odom, &px, &py, &theta);
    static float prev_th = 0.0f;
    float dth = nav_wrap_rad(theta - prev_th); prev_th = theta;
    float rate = (dt_dbg > 1e-3f) ? (fabsf(dth)/dt_dbg)*NAV_RAD2DEG : 0.0f;
    float dist, bearing;
    nav_target_geometry(px, py, &dist, &bearing);
    nav_refresh_debug(dist, bearing, theta, rate, nav_wrap_rad(bearing - theta)*NAV_RAD2DEG);
    nav_send_debug();
#else
    (void)0;
#endif
}

#else  /* NAV_DEBUG_ENABLED == 0 */
void Nav_DebugTick(void) { }
#endif
/* ==== end TEMPORARY DEBUG ============================================= */

/* ---- Small helpers ---------------------------------------------------- */
static float nav_wrap_rad(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* Scales a 0-100 "speed" percentage into a body speed (m/s) such that
 * the FASTEST of the 3 wheels reaches exactly NAV_RPM_AT_100PCT at
 * 100%. Computed from the live kinematics (g_kin) rather than a
 * hardcoded ratio, so it stays correct for whatever the real mounting
 * geometry turns out to be, not just an assumed-perfect 120-degree
 * layout. */
static float nav_pct_to_speed_mps(float pct)
{
    float w1, w2, w3;
    KiwiKinematics_InverseKinematics(&g_kin, 1.0f, 0.0f, 0.0f, &w1, &w2, &w3);
    float max_abs = fmaxf(fabsf(w1), fmaxf(fabsf(w2), fabsf(w3)));
    if (max_abs < 1e-6f) return 0.0f;
    float speed_at_100 = NAV_RPM_AT_100PCT / max_abs;
    return (pct / 100.0f) * speed_at_100;
}

static void nav_drive(float vx_mps, float vy_mps)
{
    float w1, w2, w3;
    KiwiKinematics_InverseKinematics(&g_kin, vx_mps, vy_mps, 0, &w1, &w2, &w3);
    MotorController_SetSpeed(&ctrl1, w1);
    MotorController_SetSpeed(&ctrl2, w2);
    MotorController_SetSpeed(&ctrl3, w3);
}

/* Coasts (free-wheel, not an active electrical brake -- see motor.c's
 * MOTOR_STOP vs MOTOR_BRAKE) rather than actively braking, so SETTLING's
 * "wait for the wheels to actually stop" check reflects the true
 * mechanical coast distance rather than a brake-induced lurch masking
 * it. */
static void nav_brake(void)
{
    MotorController_Stop(&ctrl1);
    MotorController_Stop(&ctrl2);
    MotorController_Stop(&ctrl3);
}

static void nav_target_geometry(float px, float py, float *dist, float *bearing)
{
    float dx = s_wps[s_target].x - px;
    float dy = s_wps[s_target].y - py;
    *dist    = sqrtf(dx * dx + dy * dy);
    *bearing = atan2f(dy, dx);
}

/* ---- Status ----------------------------------------------------------- */
void Nav_SendStatus(void)
{
    /* Layout (8 bytes):
     *   [0] u8  NavState_t (0 idle / 1 driving / 2 settling)
     *   [1] u8  current target index
     *   [2] u8  waypoint count
     *   [3] u8  fault code (NAV_FAULT_*)
     *   [4..7] f32 distance to current target (m)
     */
    uint8_t buf[8];
    buf[0] = (uint8_t)s_state;
    buf[1] = s_target;
    buf[2] = s_wp_count;
    buf[3] = s_fault;
    memcpy(buf + 4, &s_dist_to_target, 4);
    Comm_SendPacket(CMD_NAV_STATUS, buf, sizeof(buf));
    s_last_status_ms = HAL_GetTick();
}

/* ---- Public API -------------------------------------------------------- */
void Nav_Init(void)
{
    memset(s_wps, 0, sizeof(s_wps));
    s_wp_count = 0;
    s_state    = NAV_STATE_IDLE;
    s_active   = false;
    s_target   = 0;
    s_fault    = NAV_FAULT_NONE;
}

bool Nav_AddWaypoint(float x, float y)
{
    if (s_active || s_wp_count >= NAV_MAX_WAYPOINTS) return false;
    s_wps[s_wp_count].x = x;
    s_wps[s_wp_count].y = y;
    s_wp_count++;
    return true;
}

void Nav_ClearWaypoints(void)
{
    if (s_active) return;
    s_wp_count = 0;
    s_fault    = NAV_FAULT_NONE;
}

uint8_t Nav_GetWaypointCount(void) { return s_wp_count; }

bool Nav_Start(uint8_t speed_pct)
{
    if (s_active || s_wp_count == 0) return false;
    if (speed_pct < 10)  speed_pct = 10;
    if (speed_pct > 100) speed_pct = 100;

    s_speed_pct    = speed_pct;
    s_target       = 0;
    s_state        = NAV_STATE_DRIVING;
    s_settle_ticks = 0;
    s_settling_total_ticks = 0;
    s_enc_zero[0] = s_enc_zero[1] = s_enc_zero[2] = 0;
    s_fault        = NAV_FAULT_NONE;

    /* Heading is held fixed at whatever it is right now for the whole
     * patrol -- see this file's header comment for why. */
    float x, y, th;
    Odometry_GetPosition(&g_odom, &x, &y, &th);

    s_active = true;
    return true;
}

void Nav_Stop(void)
{
    if (!s_active) return;
    nav_brake();
    s_active = false;
    s_state  = NAV_STATE_IDLE;
}

void Nav_AbortFailsafe(void)
{
    if (!s_active) return;
    nav_brake();
    s_active = false;
    s_state  = NAV_STATE_IDLE;
    Nav_SendStatus();
}

bool       Nav_IsActive(void)          { return s_active; }
NavState_t Nav_GetState(void)          { return s_state; }
uint8_t    Nav_GetTargetIndex(void)    { return s_target; }
float      Nav_GetDistanceToTarget(void){ return s_dist_to_target; }
uint8_t    Nav_GetFaultCode(void)      { return s_fault; }

OdomMotionMode_t Nav_GetMotionMode(void)
{
    /* STRAIGHT vs ROTATING no longer changes Odometry_Update's
     * behavior (see odometry.h) -- only IDLE-vs-not matters. */
    return (s_state == NAV_STATE_IDLE) ? ODOM_IDLE : ODOM_STRAIGHT;
}

/* ---- Main control step (10 ms) ---------------------------------------- */
void Nav_Update(float dt)
{
    if (!s_active) return;

    float px, py, theta;
    Odometry_GetPosition(&g_odom, &px, &py, &theta);

    float dist, bearing;
    nav_target_geometry(px, py, &dist, &bearing);
    s_dist_to_target = dist;

    static float s_prev_theta = 0.0f;
    float dtheta_tick = nav_wrap_rad(theta - s_prev_theta);
    s_prev_theta = theta;
    float rate_dps = (dt > 1e-6f) ? (fabsf(dtheta_tick) / dt) * NAV_RAD2DEG : 0.0f;

    switch (s_state) {

    /* ------------------------------------------------------- DRIVING */
    case NAV_STATE_DRIVING: {
        if (dist <= NAV_ARRIVE_DIST_M) {
            s_state        = NAV_STATE_SETTLING;   /* power cut; coast in */
            s_settle_ticks = 0;
            s_settling_total_ticks = 0;
            nav_brake();
            break;
        }

        float cruise = (float)s_speed_pct;
        /* linear decel ramp down to the floor */
        float spd_pct = dist / NAV_APPROACH_RAMP_M * cruise;
        if (spd_pct < NAV_APPROACH_FLOOR_PCT) spd_pct = NAV_APPROACH_FLOOR_PCT;
        if (spd_pct > cruise)                 spd_pct = cruise;

        float speed_mps = nav_pct_to_speed_mps(spd_pct);
        /* World-frame velocity (magnitude=speed_mps, direction=bearing)
         * rotated into the current body frame -- the robot can drive
         * straight at the target from ANY heading, no facing required. */
        float vx = speed_mps * cosf(bearing - theta);
        float vy = speed_mps * sinf(bearing - theta);

        nav_drive(vx, vy);
        break;
    }

    /* -------------------------------------------------------- SETTLING */
    case NAV_STATE_SETTLING: {
        float spd1 = fabsf(MotorController_GetCurrentSpeed(&ctrl1));
        float spd2 = fabsf(MotorController_GetCurrentSpeed(&ctrl2));
        float spd3 = fabsf(MotorController_GetCurrentSpeed(&ctrl3));
        if (spd1 < NAV_BRAKE_ENGAGE_RPM && spd2 < NAV_BRAKE_ENGAGE_RPM && spd3 < NAV_BRAKE_ENGAGE_RPM)
            nav_brake();

        int32_t d1, d2, d3;
        Odometry_GetLastDeltas(&g_odom, &d1, &d2, &d3);
        if (d1 == 0 && d2 == 0 && d3 == 0) s_settle_ticks++;
        else                               s_settle_ticks = 0;

        s_settling_total_ticks++;

        if (s_settle_ticks >= NAV_SETTLE_TICKS || s_settling_total_ticks >= NAV_SETTLE_TIMEOUT_TICKS) {
            s_target       = (uint8_t)((s_target + 1) % s_wp_count);
            s_state        = NAV_STATE_DRIVING;
            s_settle_ticks = 0;
        }
        break;
    }

    default:
        break;
    }
#if NAV_DEBUG_ENABLED
    nav_refresh_debug(dist, bearing, theta, rate_dps,
                      nav_wrap_rad(bearing - theta) * NAV_RAD2DEG);
#endif

    /* ---- encoder disconnect supervision (only while commanding) ----- */
    {
        int32_t d[3];
        Odometry_GetLastDeltas(&g_odom, &d[0], &d[1], &d[2]);
        MotorController_t *ctrls[3] = { &ctrl1, &ctrl2, &ctrl3 };
        bool disconnected = false;

        for (int i = 0; i < 3; i++) {
            float cmd = fabsf(ctrls[i]->target_rpm);
            if (cmd > NAV_ENC_CHECK_MIN_RPM) {
                s_enc_zero[i] = (d[i] == 0) ? (uint16_t)(s_enc_zero[i] + 1) : 0;
            } else {
                s_enc_zero[i] = 0;
            }
            if (s_enc_zero[i] >= NAV_ENC_DISCONNECT_TICKS) disconnected = true;
        }

        if (disconnected) {
            s_fault  = NAV_FAULT_ENCODER_LOST;
            nav_brake();
            s_active = false;
            s_state  = NAV_STATE_IDLE;
            Nav_SendStatus();
#if NAV_DEBUG_ENABLED
            nav_send_debug();
#endif
            return;
        }
    }

    /* ---- inner wheel-speed loops (nav owns them while active) ------- */
    MotorController_Update(&ctrl1, dt);
    MotorController_Update(&ctrl2, dt);
    MotorController_Update(&ctrl3, dt);

    /* ---- periodic status -------------------------------------------- */
    if ((HAL_GetTick() - s_last_status_ms) >= NAV_STATUS_PERIOD_MS) {
        Nav_SendStatus();
    }
}
