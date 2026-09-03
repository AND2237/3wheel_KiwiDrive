/*
 * pid.c
 *
 * Author: Alireza
 */

#include "pid.h"

static float compute_integral_max(float ki, float output_max)
{
    return (ki > 1e-6f) ? (output_max / ki) : output_max;
}


void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float outMin, float outMax)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->setpoint  = 0.0f;
    pid->integral  = 0.0f;
    pid->prevError = 0.0f;
    pid->prevInput = 0.0f;

    pid->outputMin = outMin;
    pid->outputMax = outMax;

    pid->integralMax = compute_integral_max(ki, outMax);
}


void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->integralMax = compute_integral_max(ki, pid->outputMax);
}


void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}


float PID_Compute(PID_t *pid, float input, float dt)
{
    if (dt < 1e-6f) return 0.0f;

    float error = pid->setpoint - input;

    float pTerm = pid->Kp * error;

    float iTerm_prev  = pid->Ki * pid->integral;
    float raw_output  = pTerm + iTerm_prev;

    float sat_output  = raw_output;
    if (sat_output >  pid->outputMax) sat_output =  pid->outputMax;
    if (sat_output <  pid->outputMin) sat_output =  pid->outputMin;

    float Kb = (pid->Kp > 1e-6f) ? (pid->Ki / pid->Kp) : 0.0f;
    pid->integral += (error + Kb * (sat_output - raw_output)) * dt;

    if (pid->integral >  pid->integralMax) pid->integral =  pid->integralMax;
    if (pid->integral < -pid->integralMax) pid->integral = -pid->integralMax;

    float iTerm = pid->Ki * pid->integral;

    float dTerm = 0.0f;
    if (pid->Kd > 1e-6f && dt > 1e-4f) {
        float dInput = (input - pid->prevInput) / dt;
        dTerm = -pid->Kd * dInput;
    }

    pid->prevInput = input;
    pid->prevError = error;

    float output = pTerm + iTerm + dTerm;
    if (output >  pid->outputMax) output =  pid->outputMax;
    if (output <  pid->outputMin) output =  pid->outputMin;

    return output;
}


void PID_Reset(PID_t *pid)
{
    pid->integral  = 0.0f;
    pid->prevError = 0.0f;
    pid->prevInput = 0.0f;
}
