#ifndef NVS_H
#define NVS_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float gyro_scale;
    float wheel_radius;    /* was wheel_diameter -- kiwi_kinematics.h
                               uses radius throughout */
    float robot_radius;    /* was track_width -- center-to-wheel-contact
                               distance, replaces the L/R track width
                               concept that no longer applies */
    float wheel_scale1;    /* was wheel_scale_left/_right -- one
                               multiplicative trim per wheel now (3,
                               not 2). Mounting angles are NOT stored
                               here: they're a fixed design-time chassis
                               property, not something a runtime
                               calibration procedure adjusts -- see the
                               KIWI_MOUNT_ANGLE*_DEG defines in main.c,
                               same treatment ENCODER_PPR already gets
                               in encoder.h. */
    float wheel_scale2;
    float wheel_scale3;
} NVS_Params_t;

void NVS_GetDefaults(NVS_Params_t *out);

bool NVS_Load(NVS_Params_t *out);

bool NVS_Save(const NVS_Params_t *p);
#endif