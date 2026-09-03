/*
 * motor.c
 *
 *      Author: Alireza
 */
#include "stm32f1xx_hal.h"
#include "motor.h"

void Motor_Init(Motor_t *motor)
{
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);

    BSP_PWM_Start(motor->pwmChannel);
    BSP_PWM_SetDuty(motor->pwmChannel, 0);
}

void Motor_SetDutyDirect(Motor_t *motor, MotorDirection_t dir, float duty)
{
    if (duty > 100.0f)
        duty = 100.0f;
    if (duty < 0.0f)
        duty = 0.0f;

    switch (dir)
    {
    case MOTOR_FORWARD:
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
        break;

    case MOTOR_BACKWARD:
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
        break;

    case MOTOR_BRAKE:
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
        break;

    case MOTOR_STOP:
    default:
        duty = 0.0f;
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
        break;
    }

    BSP_PWM_SetDuty(motor->pwmChannel, duty);
}
