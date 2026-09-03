/*
 * motor_controller.c
 *
 * Author: Alireza
 */

#include "motor_controller.h"
#include <math.h>

#define DEFAULT_KP   0.6f
#define DEFAULT_KI   1.5f
#define DEFAULT_KD   0.0f

#define MAX_DUTY_PERCENT  90u

#define PID_OUT_MIN  (-(float)MAX_DUTY_PERCENT)
#define PID_OUT_MAX  ((float)MAX_DUTY_PERCENT)


void MotorController_Init(MotorController_t *ctrl,
                          Motor_t *motor, Encoder_t *enc)
{
    ctrl->motor       = motor;
    ctrl->encoder     = enc;
    ctrl->target_rpm  = 0.0f;
    ctrl->current_rpm = 0.0f;
    ctrl->output_duty = 0;
    ctrl->enabled     = false;

    PID_Init(&ctrl->pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
             PID_OUT_MIN, PID_OUT_MAX);

    Motor_Init(ctrl->motor);

    Encoder_Reset(ctrl->encoder);

    Motor_SetDutyDirect(ctrl->motor, MOTOR_STOP, 0);
}


void MotorController_SetSpeed(MotorController_t *ctrl, float rpm)
{
    if (rpm == 0.0f) {
        ctrl->target_rpm = 0.0f;
        MotorController_Stop(ctrl);
        return;
    }

    if ((ctrl->target_rpm > 0.0f && rpm < 0.0f) ||
        (ctrl->target_rpm < 0.0f && rpm > 0.0f)) {
        PID_Reset(&ctrl->pid);
    }

    ctrl->target_rpm = rpm;
    ctrl->enabled    = true;
    PID_SetSetpoint(&ctrl->pid, rpm);
}


void MotorController_Stop(MotorController_t *ctrl)
{
    ctrl->enabled     = false;
    ctrl->target_rpm  = 0.0f;
    ctrl->output_duty = 0;

    Motor_SetDutyDirect(ctrl->motor, MOTOR_STOP, 0);

    PID_Reset(&ctrl->pid);
}

void MotorController_Brake(MotorController_t *ctrl)
{
    ctrl->enabled     = false;
    ctrl->target_rpm  = 0.0f;
    ctrl->output_duty = 0;
    Motor_SetDutyDirect(ctrl->motor, MOTOR_BRAKE, 100.0f);
    PID_Reset(&ctrl->pid);
}

void MotorController_Update(MotorController_t *ctrl, float dt)
{
    Encoder_Update(ctrl->encoder, dt);
    ctrl->current_rpm = Encoder_GetSpeed(ctrl->encoder);

    if (!ctrl->enabled || dt < 1e-6f) return;

    float pid_out = PID_Compute(&ctrl->pid, ctrl->current_rpm, dt);

    if (pid_out >  PID_OUT_MAX) pid_out =  PID_OUT_MAX;
    if (pid_out <  PID_OUT_MIN) pid_out =  PID_OUT_MIN;

    MotorDirection_t dir;
    float            duty;

    if (ctrl->target_rpm >= 0.0f) {
        if (pid_out >= 0.0f) {
            dir  = MOTOR_FORWARD;
            duty = pid_out;
        } else {
            dir  = MOTOR_STOP;
            duty = 0.0f;
        }
    } else {
        if (pid_out <= 0.0f) {
            dir  = MOTOR_BACKWARD;
            duty = -pid_out;
        } else {
            dir  = MOTOR_STOP;
            duty = 0.0f;
        }
    }

    if (duty > (float)MAX_DUTY_PERCENT) duty = (float)MAX_DUTY_PERCENT;

    ctrl->output_duty = (uint8_t)(duty + 0.5f);

    Motor_SetDutyDirect(ctrl->motor, dir, duty);
}


float MotorController_GetCurrentSpeed(const MotorController_t *ctrl)
{
    return ctrl->current_rpm;
}

uint8_t MotorController_GetDuty(const MotorController_t *ctrl)
{
    return ctrl->output_duty;
}
