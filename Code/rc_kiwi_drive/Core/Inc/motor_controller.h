/*
 * motor_controller.h
 *
 *  Created on: Aug 28, 2026
 *      Author: Alireza
 */

#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include <stdbool.h>

typedef struct {
		Motor_t   *motor;
		Encoder_t *encoder;
		PID_t      pid;

		float      target_rpm;
		float      current_rpm;
		uint8_t    output_duty;

		bool       enabled;
}MotorController_t;

void  MotorController_Init        (MotorController_t *ctrl, Motor_t *motor, Encoder_t *enc);
void  MotorController_SetSpeed    (MotorController_t *ctrl, float rpm);
void  MotorController_Stop        (MotorController_t *ctrl);
void  MotorController_Brake     (MotorController_t *ctrl);
void  MotorController_Update      (MotorController_t *ctrl, float dt);
float MotorController_GetCurrentSpeed(const MotorController_t *ctrl);
uint8_t MotorController_GetDuty   (const MotorController_t *ctrl);

#endif /* INC_MOTOR_CONTROLLER_H_ */
