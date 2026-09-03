#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "bsp_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
	MOTOR_BRAKE
} MotorDirection_t;

typedef struct
{
    GPIO_TypeDef *IN1_Port;
    uint16_t      IN1_Pin;

    GPIO_TypeDef *IN2_Port;
    uint16_t      IN2_Pin;

    PWM_Channel_t pwmChannel;
} Motor_t;

void Motor_Init(Motor_t *motor);
void Motor_SetDutyDirect(Motor_t *motor, MotorDirection_t dir, float duty);

#ifdef __cplusplus
}
#endif

#endif
