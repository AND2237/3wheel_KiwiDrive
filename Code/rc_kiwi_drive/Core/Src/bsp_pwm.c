/*
 * bsp_pwm.c
 *
 *  Created on: Aug 26, 2026
 *      Author: Alireza
 */
#include "bsp_pwm.h"

#define PWM_TIMER htim1

/* NOTE: the max compare value is intentionally NOT a hardcoded literal.
 * It used to be (#define PWM_MAX_DUTY 1799), matching TIM1.Period when
 * TIM1 was originally configured with Period=1799. When TIM1.Period was
 * later changed to 899 (commit 1f24a3b), this file was not updated, so
 * every duty command was scaled against an ARR value 2x too large:
 * CCR = duty% * 1800/100, while the real ARR was only 899. That silently
 * doubled the effective duty for every motor and hard-saturated the
 * output to 100% for any PID-commanded duty >= 50%, regardless of the
 * value the PID actually asked for. Reading the live ARR register here
 * makes this self-consistent with whatever TIM1.Period CubeMX generates,
 * so this class of bug cannot reoccur silently after a future .ioc edit. */
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
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&PWM_TIMER);
	uint16_t ccr;

	if(duty_percent <= 0.0f)
	{
		ccr = 0;
	}
	else if(duty_percent >= 100.0f)
	{
		ccr = (uint16_t)(arr + 1u);
	}
	else
	{
		float ccr_f = (duty_percent * (float)(arr + 1u)) / 100.0f;
		ccr = (uint16_t)(ccr_f + 0.5f);
	}

	__HAL_TIM_SET_COMPARE(&PWM_TIMER, PWM_ChannelMap[ch], ccr);

}

