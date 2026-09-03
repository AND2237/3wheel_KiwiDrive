/*
 * bsp_pwm.c
 *
 *  Created on: Aug 26, 2026
 *      Author: Alireza
 */
#include "bsp_pwm.h"

#define PWM_TIMER htim1
#define PWM_MAX_DUTY 1799

static uint32_t PWM_ChannelMap[] =
{
		TIM_CHANNEL_1,
		TIM_CHANNEL_2,
		TIM_CHANNEL_3,
};

void BSP_PWM_Start(PWM_Channel_t ch)
{
	HAL_TIM_PWM_Start(&PWM_TIMER, PWM_ChannelMap[ch]);
}

void BSP_PWM_SetDuty(PWM_Channel_t ch, float duty_percent)
{
	uint16_t ccr;

	if(duty_percent <= 0.0f)
	{
		ccr = 0;
	}
	else if(duty_percent >= 100.0f)
	{
		ccr = PWM_MAX_DUTY + 1;
	}
	else
	{
		float ccr_f = (duty_percent * (float)(PWM_MAX_DUTY + 1)) / 100.0f;
		ccr = (uint16_t)(ccr_f + 0.5f);
	}

	__HAL_TIM_SET_COMPARE(&PWM_TIMER, PWM_ChannelMap[ch], ccr);
	
}

