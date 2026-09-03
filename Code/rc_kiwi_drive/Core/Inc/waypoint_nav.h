/*
 * waypoint_nav.h
 *
 * ---------------------------------------------------------------------
 * Autonomous waypoint patrol
 * ---------------------------------------------------------------------
 * Drives the robot through a user-defined list of waypoints in order,
 * loops back to the first one and repeats until explicitly stopped.
 *
 * CONTROL STRATEGY -- substantially simpler than the old skid-steer
 * version, because a holonomic drive does not need to face a waypoint
 * before it can move toward it:
 *   DRIVING  : the body-frame velocity needed to head straight for the
 *              target (world-frame bearing rotated into the current
 *              body frame) is sent through KiwiKinematics_InverseKinematics
 *              every tick, speed ramping down near NAV_APPROACH_RAMP_M
 *              to NAV_APPROACH_FLOOR_PCT so the robot creeps into the
 *              arrival radius. omega is always commanded as 0 -- the
 *              robot's heading is simply whatever it happens to be and
 *              is never actively held or corrected during patrol. This
 *              is a deliberate choice, not an oversight: the point of a
 *              holonomic drive is that translation and orientation are
 *              independent, so there is nothing to gain by coupling
 *              them here, and odometry's gyro-tracked heading is used
 *              every tick to rotate world-frame bearing into the body
 *              frame regardless of what that heading drifts to.
 *   SETTLING : power cut after the arrival radius is reached; wait for
 *              the wheels to actually stop (coast) so odometry keeps
 *              integrating the coast displacement, then advance to the
 *              next waypoint.
 *
 *   This replaces the old ALIGNING -> APPROACHING -> SETTLING sequence.
 *   ALIGNING -- a turn-in-place-then-coast-to-heading maneuver with an
 *   adaptively-learned "lead angle" -- existed purely because skid-
 *   steer cannot translate without first facing the target; a kiwi
 *   drive has no such restriction, so there is no separate alignment
 *   phase, no lead-angle learning, and no mid-leg "realign" trigger
 *   (that trigger existed specifically because skid-steer's yaw
 *   authority is scrub-limited while translating at speed -- a holonomic
 *   drive's omega axis is independent of Vx/Vy and has no such limit).
 *
 *   There is deliberately NO timeout (per the original requirement):
 *   as long as all components are connected the controller keeps
 *   trying until the waypoint is reached.
 *
 * SAFETY / EXCLUSIVITY:
 *   - While active, manual CMD_SET_SPEED / CMD_SET_MOTION_MODE are
 *     ignored by main.c (exclusive autonomous mode).
 *   - Fail-safes handled by main.c: comm loss or IMU fault ->
 *     Nav_AbortFailsafe() -> full stop.
 *   - Encoder DISCONNECT detection (per requirement: abort only when an
 *     encoder reports nothing at all): while a wheel is commanded above
 *     NAV_ENC_CHECK_MIN_RPM, its per-tick odometry delta is watched;
 *     NAV_ENC_DISCONNECT_TICKS consecutive zero deltas -> abort with
 *     NAV_FAULT_ENCODER_LOST for THAT wheel. NOTE: a hard mechanical
 *     stall of all 3 wheels at once produces the same signature (no
 *     current sensing exists to tell them apart) -- documented,
 *     accepted, same as the old 2-wheel version.
 *
 * MOTOR PATH:
 *   Drives ctrl1/ctrl2/ctrl3 directly via KiwiKinematics_InverseKinematics
 *   -- there is no separate L/R-sync loop to bypass any more (see
 *   kiwi_kinematics.h): all 3 wheels' targets come from one shared
 *   command, so they are synchronized by construction.
 *
 * TUNING: every constant below is PROVISIONAL -- re-tune on the real
 * floor/carpet (same convention as imu.h).
 * ---------------------------------------------------------------------
 */
#ifndef WAYPOINT_NAV_H
#define WAYPOINT_NAV_H

#include <stdint.h>
#include <stdbool.h>
#include "odometry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_MAX_WAYPOINTS           16u     /* per requirement */

#define NAV_ARRIVE_DIST_M           0.15f
/* Deceleration: below this distance the commanded speed ramps linearly
 * from cruise down to the floor. */
#define NAV_APPROACH_RAMP_M         0.8f
/* Minimum approach power (%). PROVISIONAL -- raise if the robot stalls
 * at this power on the actual surface (PID holds wheel RPM closed-loop,
 * so modest floors normally work). */
#define NAV_APPROACH_FLOOR_PCT      30.0f

#define NAV_SETTLE_TICKS            5u
#define NAV_SETTLE_TIMEOUT_TICKS    200u   /* 2.0 s backstop -- advance to the
                                             * next waypoint anyway if a clean
                                             * zero-delta settle window never
                                             * arrives (incline creep, floor
                                             * vibration), rather than hanging
                                             * the whole patrol on one waypoint */
#define NAV_BRAKE_ENGAGE_RPM        25.0f

/* ---- Encoder disconnect supervision --------------------------------- */
#define NAV_ENC_CHECK_MIN_RPM       30.0f   /* supervise only above this command */
#define NAV_ENC_DISCONNECT_TICKS    100u    /* 1.0 s of zero counts -> disconnect */

/* ---- Status reporting ----------------------------------------------- */
#define NAV_STATUS_PERIOD_MS        200u    /* 5 Hz while active */

/* Wheel RPM at 100% command -- must match the ESP maxRPM scale. */
#define NAV_RPM_AT_100PCT           236.0f

#define NAV_FAULT_NONE              0u
#define NAV_FAULT_ENCODER_LOST      1u

#define NAV_DEBUG_ENABLED        1u
#define NAV_DEBUG_PERIOD_MS      100u
#define NAV_DEBUG_PKT_BYTES      44u

typedef enum {
    NAV_STATE_IDLE     = 0,
    NAV_STATE_DRIVING  = 1,
    NAV_STATE_SETTLING = 2
} NavState_t;

/* One-time module init (static state is zero-initialised anyway; kept
 * for symmetry with the other modules and future use). */
void     Nav_Init(void);

/* Waypoint list management. Add is rejected while active or full.
 * Clear is rejected while active. Coordinates are in the CURRENT
 * odometry frame (shared with survey). */
bool     Nav_AddWaypoint(float x, float y);
void     Nav_ClearWaypoints(void);
uint8_t  Nav_GetWaypointCount(void);

/* Start patrol from waypoint 0 at the given speed (10..100 %).
 * Returns false when active or when the list is empty. */
bool     Nav_Start(uint8_t speed_pct);

/* Operator stop: full stop, back to IDLE. Safe to call when idle. */
void     Nav_Stop(void);

/* Fail-safe abort used by main.c on comm loss / IMU fault: full stop. */
void     Nav_AbortFailsafe(void);

/* Call exactly once per 10 ms control tick while active; drives the
 * motor controllers and updates the internal state machine. No-op when
 * inactive. Also owns the 5 Hz CMD_NAV_STATUS stream and the encoder
 * disconnect supervision. */
void     Nav_Update(float dt);

bool       Nav_IsActive(void);
NavState_t Nav_GetState(void);
/* Motion mode the odometry integrator must use while nav is active. */
OdomMotionMode_t Nav_GetMotionMode(void);
uint8_t    Nav_GetTargetIndex(void);
float      Nav_GetDistanceToTarget(void);
uint8_t    Nav_GetFaultCode(void);   /* cleared by the next Nav_Start */

/* Sends one CMD_NAV_STATUS packet now (state, target, count, fault,
 * distance). Called by main.c after NAV_* commands and by the module
 * itself on fault abort and at NAV_STATUS_PERIOD_MS while active. */
void     Nav_SendStatus(void);

void Nav_DebugTick(void);

#ifdef __cplusplus
}
#endif

#endif /* WAYPOINT_NAV_H */
