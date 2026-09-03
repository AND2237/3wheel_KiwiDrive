/*
 * odometry.c
 *
 *  Created on: Aug 28, 2026
 *      Author: Alireza
 */

#include "odometry.h"
#include "encoder.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265359f
#endif

#define RAD_TO_DEG (180.0f / M_PI)

static float NormalizeAngleRad(float a)
{
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

static void ResetInternal(Odometry_t *odom, int32_t c1, int32_t c2, int32_t c3)
{
    odom->x     = 0.0f;
    odom->y     = 0.0f;
    odom->theta = 0.0f;

    odom->prev_count[0] = c1;
    odom->prev_count[1] = c2;
    odom->prev_count[2] = c3;

    odom->last_delta[0] = 0;
    odom->last_delta[1] = 0;
    odom->last_delta[2] = 0;

    odom->heading_disagreement_signed_deg       = 0.0f;
    odom->heading_disagreement_abs_integral_deg = 0.0f;
}

void Odometry_Init(Odometry_t *odom, const KiwiKinematics_t *kin_params,
                    int32_t count1_init, int32_t count2_init, int32_t count3_init)
{
    odom->kin = *kin_params;
    odom->wheel_scale[0] = 1.0f;
    odom->wheel_scale[1] = 1.0f;
    odom->wheel_scale[2] = 1.0f;
    ResetInternal(odom, count1_init, count2_init, count3_init);
}

void Odometry_SetParams(Odometry_t *odom, const KiwiKinematics_t *kin_params)
{
    odom->kin = *kin_params;
}

void Odometry_SetWheelScale(Odometry_t *odom, float scale1, float scale2, float scale3)
{
    if (scale1 > 0.3f && scale1 < 3.0f) odom->wheel_scale[0] = scale1;
    if (scale2 > 0.3f && scale2 < 3.0f) odom->wheel_scale[1] = scale2;
    if (scale3 > 0.3f && scale3 < 3.0f) odom->wheel_scale[2] = scale3;
}

void Odometry_ResetPose(Odometry_t *odom,
                         int32_t count1_init, int32_t count2_init, int32_t count3_init)
{
    ResetInternal(odom, count1_init, count2_init, count3_init);
}

void Odometry_Update(Odometry_t *odom,
                      int32_t count1, int32_t count2, int32_t count3,
                      float dtheta_gyro_rad, OdomMotionMode_t mode)
{
    int32_t d1 = count1 - odom->prev_count[0];
    int32_t d2 = count2 - odom->prev_count[1];
    int32_t d3 = count3 - odom->prev_count[2];
    odom->prev_count[0] = count1;
    odom->prev_count[1] = count2;
    odom->prev_count[2] = count3;
    odom->last_delta[0] = d1;
    odom->last_delta[1] = d2;
    odom->last_delta[2] = d3;

    /* Raw (unfiltered) per-wheel angular displacement this tick, in
     * radians, with each wheel's own calibrated scale trim folded in
     * (see wheel_scale comment in odometry.h). */
    float w1 = ((float)d1 / ENCODER_PPR) * 2.0f * M_PI * odom->wheel_scale[0];
    float w2 = ((float)d2 / ENCODER_PPR) * 2.0f * M_PI * odom->wheel_scale[1];
    float w3 = ((float)d3 / ENCODER_PPR) * 2.0f * M_PI * odom->wheel_scale[2];

    float dx_body, dy_body, dtheta_encoder;
    KiwiKinematics_WheelDeltaToBodyDelta(&odom->kin, w1, w2, w3,
                                          &dx_body, &dy_body, &dtheta_encoder);

    float disagreement_rad = dtheta_encoder - dtheta_gyro_rad;
    odom->heading_disagreement_signed_deg       += disagreement_rad * RAD_TO_DEG;
    odom->heading_disagreement_abs_integral_deg += fabsf(disagreement_rad) * RAD_TO_DEG;

    if (mode != ODOM_IDLE) {
        /* Gyro-primary heading (see Odometry_Update's doc comment in
         * odometry.h), body-frame translation from the unified
         * kinematics rotated into the world frame at this tick's
         * MIDPOINT heading -- the same first-order midpoint
         * integration trick the old code used for its scalar forward
         * distance, now applied to a full 2D (dx,dy) vector, since a
         * holonomic drive can genuinely move sideways in the body
         * frame (dy_body != 0 is a real, meaningful case here, unlike
         * skid-steer). */
        float theta_before = odom->theta;
        float theta_mid     = theta_before + dtheta_gyro_rad * 0.5f;
        float cos_m = cosf(theta_mid);
        float sin_m = sinf(theta_mid);

        odom->x += dx_body * cos_m - dy_body * sin_m;
        odom->y += dx_body * sin_m + dy_body * cos_m;
        odom->theta = NormalizeAngleRad(theta_before + dtheta_gyro_rad);
    }
}

void Odometry_GetPosition(const Odometry_t *odom, float *x, float *y, float *theta_rad)
{
    if (x)         *x         = odom->x;
    if (y)         *y         = odom->y;
    if (theta_rad) *theta_rad = odom->theta;
}

void Odometry_GetHeadingDisagreement(const Odometry_t *odom, float *signed_deg, float *abs_integral_deg)
{
    if (signed_deg)       *signed_deg       = odom->heading_disagreement_signed_deg;
    if (abs_integral_deg) *abs_integral_deg = odom->heading_disagreement_abs_integral_deg;
}

void Odometry_GetLastDeltas(const Odometry_t *odom, int32_t *d1, int32_t *d2, int32_t *d3)
{
    if (d1) *d1 = odom->last_delta[0];
    if (d2) *d2 = odom->last_delta[1];
    if (d3) *d3 = odom->last_delta[2];
}

float Odometry_GetDisplacement(const Odometry_t *odom)
{
    return sqrtf(odom->x * odom->x + odom->y * odom->y);
}
