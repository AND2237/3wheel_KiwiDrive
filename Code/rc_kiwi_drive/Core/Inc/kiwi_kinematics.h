/*
 * kiwi_kinematics.h
 *
 * Author: Alireza
 *
 * Holonomic kinematics for a 3-omniwheel "kiwi drive". Replaces
 * sync_drive.c's differential-drive L/R-sync PI loop: with kiwi
 * kinematics, all 3 wheels' target speeds are derived from ONE shared
 * body-frame command via the matrix below, so they are synchronized
 * by construction and need no separate corrective loop.
 *
 * Body frame convention (matches odometry.c):
 *   +X = forward, +Y = left, theta/omega positive = counter-clockwise,
 *   all angles in radians.
 *
 * Wheel mounting-angle convention: mount_angle_i is the body-frame
 * angle (radians, CCW from +X) of wheel i's MOUNTING POSITION on the
 * chassis -- not its rolling direction. This module assumes the
 * standard kiwi/omni layout, where each wheel's rolling axis is
 * tangent to the chassis circle at its mounting point (perpendicular
 * to the radius line from the robot's center to that wheel) -- the
 * layout used by essentially every 3-omniwheel "kiwi drive" chassis,
 * wheels spaced around the chassis with axles pointing radially
 * outward. The 3 angles are passed in rather than hardcoded to 120
 * degrees apart so they can match your actual physical mounting
 * exactly (nominally 120 degrees apart for a symmetric kiwi drive,
 * but the math below does not require exact symmetry).
 *
 * Units at the module boundary: linear velocity/position in meters
 * (and m/s), angles in radians (and rad/s), wheel speed in RPM
 * (matching MotorController_t's target_rpm/current_rpm) except where
 * a function is explicitly documented otherwise.
 */
#ifndef KIWI_KINEMATICS_H
#define KIWI_KINEMATICS_H

#include <stdbool.h>

typedef struct {
    float wheel_radius_m;
    float robot_radius_m;
    float mount_angle_rad[3];

    float fwd[3][3];

    bool valid;
} KiwiKinematics_t;

bool KiwiKinematics_Init(KiwiKinematics_t *kin,
                          float wheel_radius_m, float robot_radius_m,
                          float mount_angle1_rad, float mount_angle2_rad,
                          float mount_angle3_rad);

void KiwiKinematics_InverseKinematics(const KiwiKinematics_t *kin,
                                       float vx_mps, float vy_mps, float omega_radps,
                                       float *w1_rpm, float *w2_rpm, float *w3_rpm);

void KiwiKinematics_ForwardVelocity(const KiwiKinematics_t *kin,
                                     float w1_rpm, float w2_rpm, float w3_rpm,
                                     float *vx_mps, float *vy_mps, float *omega_radps);

void KiwiKinematics_WheelDeltaToBodyDelta(const KiwiKinematics_t *kin,
                                           float w1_delta_rad, float w2_delta_rad,
                                           float w3_delta_rad,
                                           float *dx_m, float *dy_m, float *dtheta_rad);

#endif /* KIWI_KINEMATICS_H */
