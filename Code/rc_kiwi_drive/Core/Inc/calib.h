/*
 * calib.h
 *
 * ---------------------------------------------------------------------
 * Field calibration -- 3-wheel holonomic, no L/R concept anywhere
 * ---------------------------------------------------------------------
 * Two automated procedures, each isolating exactly one physical
 * quantity NVS_Params_t already exists to hold:
 *
 *   SPIN : pure omega command, two legs (CCW then CW) to average out
 *          directional asymmetry. Compares the gyro-integrated angle
 *          (trustworthy reference) against the kinematics-implied
 *          angle (from wheel encoders through the SAME kinematics
 *          model the whole drivetrain already trusts) for the same
 *          physical rotation -> corrects gyro_scale AND robot_radius
 *          from that one test.
 *   ROLL : pure +X body-frame Vx command, straight-line distance ->
 *          corrects wheel_radius against an operator-measured ground
 *          truth distance.
 *
 * Recommended order: ROLL before SPIN -- the robot_radius formula
 * (see main.c's CMD_CAL_APPLY_SPIN handler) assumes wheel_radius is
 * already reasonably accurate.
 *
 * Actuation always goes through KiwiKinematics_InverseKinematics into
 * MotorController_SetSpeed on all 3 controllers -- never a wheel
 * controller touched directly -- consistent with every other motion
 * source in this codebase (manual drive, waypoint_nav.c).
 *
 * Same public-API shape as the pre-migration module (Cal_Init /
 * Cal_IsActive / Cal_Update / Cal_GetMotionMode / Cal_Abort) so
 * main.c's calling convention is unchanged. Cal_GetSpinResult /
 * Cal_GetRollResult are additions beyond that original shape: the
 * Apply-handler formulas in main.c (CMD_CAL_APPLY_SPIN/ROLL) need to
 * read back the cached per-test measurements this module produces,
 * and those are calib.c's own private static state -- an accessor is
 * the only way main.c can reach them without calib.c exposing its
 * internals directly.
 * ---------------------------------------------------------------------
 */
#ifndef CALIB_H
#define CALIB_H

#include <stdbool.h>
#include "odometry.h"   /* OdomMotionMode_t */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAL_IDLE = 0,
    CAL_SPIN = 1,
    CAL_ROLL = 2
} CalMode_t;

void Cal_Init(void);

/* deg_ccw_target/deg_cw_target: magnitude in degrees for each leg
 * (sign is implied by the leg, not the argument). power_pct: 10..100,
 * clamped -- same convention as Nav_Start's speed_pct. */
void Cal_StartSpin(float deg_ccw_target, float deg_cw_target, float power_pct);

/* dist_m: magnitude in meters. power_pct: 10..100, clamped. */
void Cal_StartRoll(float dist_m, float power_pct);

void Cal_Abort(void);

/* Call exactly once per 10 ms control tick while active; drives the
 * motor controllers and owns their inner PID loop (MotorController_Update)
 * while it does, exactly like Nav_Update. No-op when inactive.
 * dtheta_gyro_rad: this tick's gyro-measured heading change (same
 * value main.c already computes from IMU_UpdateGyroZ for odometry). */
void Cal_Update(float dt, float dtheta_gyro_rad);

bool Cal_IsActive(void);

/* Motion mode the odometry integrator must use while cal is active
 * (see Nav_GetMotionMode's identical role). */
OdomMotionMode_t Cal_GetMotionMode(void);

/* Reads back the last COMPLETED spin test's cached per-leg results.
 * Returns false (outputs untouched) if no spin test has completed
 * since the last Cal_StartSpin call -- guards CMD_CAL_APPLY_SPIN
 * against acting on stale or never-populated data. An aborted
 * (CMD_CAL_STOP) run does NOT overwrite a previously completed
 * result -- only starting a NEW test invalidates the old one. */
bool Cal_GetSpinResult(float *g1_gyro_deg, float *g1_kin_deg,
                        float *g2_gyro_deg, float *g2_kin_deg);

/* Reads back the last COMPLETED roll test's cached result. Same
 * validity contract as Cal_GetSpinResult. */
bool Cal_GetRollResult(float *dist_estimated_m);

/* Call exactly once, immediately after a successful CMD_CAL_APPLY_SPIN
 * / CMD_CAL_APPLY_ROLL actually applies the correction (i.e. right
 * after the Get* call that fed it). Without this, re-sending the same
 * Apply command would apply the SAME cached test result a second time
 * on top of the already-corrected value -- compounding the
 * correction rather than being a harmless no-op/repeat. Does not
 * affect Cal_IsActive()/Cal_GetMotionMode() or any in-progress test. */
void Cal_ClearSpinResult(void);
void Cal_ClearRollResult(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_H */
