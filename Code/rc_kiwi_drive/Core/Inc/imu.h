/*
 * imu.h
 *
 * MPU6050 gyro-Z driver for heading estimation.
 */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#define IMU_EPS_SPEED_RPM              2.0f

#define IMU_GYRO_STATIONARY_RANGE_DPS  1.5f

#define IMU_BIAS_CALIBRATION_SAMPLES   100u

#define IMU_MAX_BIAS_JUMP_DPS          2.0f

void  IMU_Init(void);

void  IMU_CalibrationTick(void);

bool  IMU_IsCalibrated(void);

float IMU_UpdateGyroZ(float dt);

float IMU_GetBiasRaw(void);


float IMU_GetLastRawDps(void);

bool  IMU_SetScaleCorrection(float factor);

float IMU_GetScaleCorrection(void);

void  IMU_ResetDiagnostics(void);

bool  IMU_IsFaulted(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
