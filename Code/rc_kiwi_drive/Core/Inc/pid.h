#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float setpoint;
    float integral;
    float prevError;
    float prevInput;

    float outputMin;
    float outputMax;

    float integralMax;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float outMin, float outMax);
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);
void PID_SetSetpoint(PID_t *pid, float setpoint);
float PID_Compute(PID_t *pid, float input, float dt);
void PID_Reset(PID_t *pid);

#endif
