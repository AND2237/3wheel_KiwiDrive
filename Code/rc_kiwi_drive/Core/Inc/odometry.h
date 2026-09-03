/*
 * odometry.h
 *
 *  Created on: Aug 28, 2026
 *      Author: Alireza
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include "kiwi_kinematics.h"

/* Kept for interface compatibility with calib.c/waypoint_nav.c's
 * existing Cal_GetMotionMode()/Nav_GetMotionMode() (their own rewrite
 * for the kiwi drive is a separate step). Internally, odometry now
 * treats STRAIGHT and ROTATING identically: holonomic kinematics
 * computes combined translation+rotation in one unified formula and
 * no longer needs the distinction skid-steer required (see
 * Odometry_Update). Only ODOM_IDLE is still functionally distinct. */
typedef enum {
    ODOM_IDLE     = 0,
    ODOM_ROTATING = 1,
    ODOM_STRAIGHT = 2
} OdomMotionMode_t;

typedef struct {
    float   x;
    float   y;
    float   theta;

    KiwiKinematics_t kin;      /* Owned (copied at Init/SetParams), not
                                   borrowed -- odom does not depend on
                                   the caller's KiwiKinematics_t
                                   outliving the call. */

    float   wheel_scale[3];    /* Multiplicative trims around
                                   kin.wheel_radius_m, one per wheel,
                                   default 1.0 (no-op). Same role the
                                   old 2-wheel wheel_scale_left/right
                                   played: corrects a persistent
                                   per-wheel effective-rolling-radius
                                   mismatch (manufacturing tolerance,
                                   tire compression) -- whatever the
                                   physical cause, this is what
                                   actually corrects the ODOMETRY,
                                   kept separate from the shared,
                                   ideal kinematic model every wheel is
                                   commanded against. See
                                   Odometry_SetWheelScale(). */

    int32_t prev_count[3];
    int32_t last_delta[3];

    float   heading_disagreement_signed_deg;
    float   heading_disagreement_abs_integral_deg;
} Odometry_t;

/* kin_params is COPIED into odom's own embedded KiwiKinematics_t (not
 * stored by pointer) -- odom does not need kin_params to outlive this
 * call. count1/2/3_init: each wheel's current raw encoder position
 * (e.g. Encoder_GetPosition()), so the first Update() call afterward
 * sees a correct small delta instead of a spurious jump from 0. */
void Odometry_Init(Odometry_t *odom, const KiwiKinematics_t *kin_params,
                    int32_t count1_init, int32_t count2_init, int32_t count3_init);

/* Re-applies kin_params (e.g. after loading calibrated geometry from
 * flash) without resetting pose. */
void Odometry_SetParams(Odometry_t *odom, const KiwiKinematics_t *kin_params);

/* scale1/2/3: multiplicative trims around kin.wheel_radius_m, expected
 * near 1.0. Any one outside (0.3, 3.0) is rejected (that wheel's scale
 * left unchanged) as implausible. */
void Odometry_SetWheelScale(Odometry_t *odom, float scale1, float scale2, float scale3);

void Odometry_ResetPose(Odometry_t *odom,
                         int32_t count1_init, int32_t count2_init, int32_t count3_init);

/* count1/2/3: each wheel's current RAW cumulative encoder position
 * (e.g. Encoder_GetPosition() -- NOT the filtered speed from
 * Encoder_GetSpeed()/MotorController_GetCurrentSpeed(), so position
 * tracking carries no filter lag; see kiwi_kinematics.h).
 *
 * dtheta_gyro_rad: gyro-measured heading change this tick. Heading
 * stays gyro-primary, same as the old skid-steer odometry -- a
 * dedicated rate gyro remains the more reliable SHORT-TERM heading
 * source regardless of drivetrain (a kinematics-derived heading rate
 * is only as good as the robot_radius calibration behind it, and
 * error there compounds over time; the gyro doesn't share that
 * failure mode). Translation, however, now comes directly from the
 * unified 3-wheel kinematics: no more STRAIGHT-mode
 * min(|d_left|,|d_right|) scrub-rejection heuristic and no more
 * ROTATING-mode "rotation lurch" compensation, because both were
 * specifically working around skid-steer wheel scrub that a
 * non-slipping omniwheel doesn't have -- removing them is a genuine
 * simplification, not just a port.
 *
 * mode: see OdomMotionMode_t -- only ODOM_IDLE is still distinct
 * (freezes x/y/theta; delta-tracking and the disagreement diagnostic
 * below still run regardless of mode, matching the old behavior). */
void Odometry_Update(Odometry_t *odom,
                      int32_t count1, int32_t count2, int32_t count3,
                      float dtheta_gyro_rad, OdomMotionMode_t mode);

void Odometry_GetPosition(const Odometry_t *odom, float *x, float *y, float *theta_rad);

/* Cumulative disagreement between the kinematics' own implied
 * rotation (3rd output of KiwiKinematics_WheelDeltaToBodyDelta) and
 * the gyro's measured rotation -- a health/slip diagnostic, not part
 * of the pose estimate itself. Generalizes the old 2-wheel
 * (ds_r-ds_l)/track_width version: dtheta_encoder now comes directly
 * off the kinematics matrix instead of being hand-derived from just 2
 * wheels. */
void Odometry_GetHeadingDisagreement(const Odometry_t *odom, float *signed_deg, float *abs_integral_deg);

void Odometry_GetLastDeltas(const Odometry_t *odom, int32_t *d1, int32_t *d2, int32_t *d3);

float Odometry_GetDisplacement(const Odometry_t *odom);

#endif
