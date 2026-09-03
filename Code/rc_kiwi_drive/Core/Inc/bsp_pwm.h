/*
 * bsp_pwm.h
 *
 *      Author: Alireza
 */

#ifndef BSP_PWM_H_
#define BSP_PWM_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;

typedef enum {
	PWM_CH1 = 0,
	PWM_CH2,
	PWM_CH3,
}PWM_Channel_t;

void BSP_PWM_Start(PWM_Channel_t ch);
void BSP_PWM_SetDuty(PWM_Channel_t ch, float duty_percent);

#endif /* INC_BSP_PWM_H_ */
