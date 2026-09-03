/*
 * kiwi_kinematics.c
 *
 * Author: Alireza
 */
#include "kiwi_kinematics.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265359f
#endif

#define RAD_PER_SEC_TO_RPM  (60.0f / (2.0f * (float)M_PI))
#define RPM_TO_RAD_PER_SEC  ((2.0f * (float)M_PI) / 60.0f)

/* General 3x3 inverse via the cofactor/adjugate method. Not specific
 * to the kiwi geometry -- kept generic (rather than a closed-form
 * shortcut only valid for exactly-120-degree spacing) so Init still
 * works correctly for any real-world mounting-angle error. */
static bool Invert3x3(const float A[3][3], float out[3][3])
{
    float a = A[0][0], b = A[0][1], c = A[0][2];
    float d = A[1][0], e = A[1][1], f = A[1][2];
    float g = A[2][0], h = A[2][1], i = A[2][2];

    float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (fabsf(det) < 1e-9f) {
        return false;
    }
    float inv_det = 1.0f / det;

    out[0][0] =  (e * i - f * h) * inv_det;
    out[0][1] = -(b * i - c * h) * inv_det;
    out[0][2] =  (b * f - c * e) * inv_det;
    out[1][0] = -(d * i - f * g) * inv_det;
    out[1][1] =  (a * i - c * g) * inv_det;
    out[1][2] = -(a * f - c * d) * inv_det;
    out[2][0] =  (d * h - e * g) * inv_det;
    out[2][1] = -(a * h - b * g) * inv_det;
    out[2][2] =  (a * e - b * d) * inv_det;
    return true;
}

bool KiwiKinematics_Init(KiwiKinematics_t *kin,
                          float wheel_radius_m, float robot_radius_m,
                          float mount_angle1_rad, float mount_angle2_rad,
                          float mount_angle3_rad)
{
    memset(kin, 0, sizeof(*kin));

    kin->wheel_radius_m = wheel_radius_m;
    kin->robot_radius_m = robot_radius_m;
    kin->mount_angle_rad[0] = mount_angle1_rad;
    kin->mount_angle_rad[1] = mount_angle2_rad;
    kin->mount_angle_rad[2] = mount_angle3_rad;

    /* Inverse-kinematics matrix M: w_i (rad/s) = (1/r) * M * [Vx,Vy,omega].
     * row_i = [-sin(beta_i), cos(beta_i), R], derived from projecting
     * the rigid-body contact-point velocity at each wheel onto that
     * wheel's tangential driven axis (beta_i + 90 degrees) -- the only
     * direction an omniwheel resists/drives, since its rollers let it
     * slip freely perpendicular to that. See module header for the
     * frame convention. */
    float M[3][3];
    for (int i = 0; i < 3; i++) {
        M[i][0] = -sinf(kin->mount_angle_rad[i]);
        M[i][1] =  cosf(kin->mount_angle_rad[i]);
        M[i][2] =  robot_radius_m;
    }

    float Minv[3][3];
    if (!Invert3x3(M, Minv)) {
        /* Degenerate geometry (e.g. two wheels at the same mounting
         * angle) -- kin stays zeroed/invalid rather than risk
         * dividing by ~0 on every subsequent call. */
        return false;
    }

    /* Forward kinematics: [Vx,Vy,omega] = r * Minv * [w1,w2,w3]. Fold
     * the wheel-radius scale into the cached matrix now so the
     * per-tick call is one 3x3 multiply with no extra scalar. */
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            kin->fwd[r][c] = wheel_radius_m * Minv[r][c];
        }
    }

    kin->valid = true;
    return true;
}

void KiwiKinematics_InverseKinematics(const KiwiKinematics_t *kin,
                                       float vx_mps, float vy_mps, float omega_radps,
                                       float *w1_rpm, float *w2_rpm, float *w3_rpm)
{
    if (!kin->valid) {
        *w1_rpm = 0.0f; *w2_rpm = 0.0f; *w3_rpm = 0.0f;
        return;
    }

    float inv_r = 1.0f / kin->wheel_radius_m;
    float *out[3] = { w1_rpm, w2_rpm, w3_rpm };

    for (int i = 0; i < 3; i++) {
        float beta = kin->mount_angle_rad[i];
        float w_radps = (-sinf(beta) * vx_mps + cosf(beta) * vy_mps
                          + kin->robot_radius_m * omega_radps) * inv_r;
        *out[i] = w_radps * RAD_PER_SEC_TO_RPM;
    }
}

/* Shared by both forward-direction public functions below: the
 * cached matrix is a linear map, so the identical multiply serves a
 * velocity-in/velocity-out call and a displacement-in/displacement-out
 * call alike -- only the physical units of what's passed in differ. */
static void ForwardCore(const KiwiKinematics_t *kin,
                         float w1, float w2, float w3,
                         float *out_x, float *out_y, float *out_theta)
{
    if (!kin->valid) {
        *out_x = 0.0f; *out_y = 0.0f; *out_theta = 0.0f;
        return;
    }
    *out_x     = kin->fwd[0][0] * w1 + kin->fwd[0][1] * w2 + kin->fwd[0][2] * w3;
    *out_y     = kin->fwd[1][0] * w1 + kin->fwd[1][1] * w2 + kin->fwd[1][2] * w3;
    *out_theta = kin->fwd[2][0] * w1 + kin->fwd[2][1] * w2 + kin->fwd[2][2] * w3;
}

void KiwiKinematics_ForwardVelocity(const KiwiKinematics_t *kin,
                                     float w1_rpm, float w2_rpm, float w3_rpm,
                                     float *vx_mps, float *vy_mps, float *omega_radps)
{
    ForwardCore(kin,
                w1_rpm * RPM_TO_RAD_PER_SEC,
                w2_rpm * RPM_TO_RAD_PER_SEC,
                w3_rpm * RPM_TO_RAD_PER_SEC,
                vx_mps, vy_mps, omega_radps);
}

void KiwiKinematics_WheelDeltaToBodyDelta(const KiwiKinematics_t *kin,
                                           float w1_delta_rad, float w2_delta_rad,
                                           float w3_delta_rad,
                                           float *dx_m, float *dy_m, float *dtheta_rad)
{
    ForwardCore(kin, w1_delta_rad, w2_delta_rad, w3_delta_rad, dx_m, dy_m, dtheta_rad);
}
