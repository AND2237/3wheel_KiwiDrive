#include "stm32f1xx_hal.h"
#include "encoder.h"

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim)
{
    enc->htim       = htim;
    enc->prevCount  = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    enc->totalCount = 0;
    enc->speed      = 0.0f;
    enc->speedRaw   = 0.0f;
    enc->dir_sign   = +1;

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void Encoder_Update(Encoder_t *enc, float dt)
{
    uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);

    int32_t delta = (int16_t)(current - enc->prevCount);
    enc->prevCount   = current;
    enc->totalCount += (int32_t)enc->dir_sign * delta;

    if (dt > 0.001f)
    {
        float raw = ((float)(delta * enc->dir_sign) / ENCODER_PPR) * (60.0f / dt);
        enc->speedRaw = raw;

        enc->speed = ENCODER_SPEED_ALPHA * raw
                   + (1.0f - ENCODER_SPEED_ALPHA) * enc->speed;
    }
    else
    {
        enc->speedRaw = 0.0f;
        enc->speed    = 0.0f;
    }
}

void Encoder_Reset(Encoder_t *enc)
{
    __HAL_TIM_SET_COUNTER(enc->htim, 0);
    enc->prevCount  = 0;
    enc->totalCount = 0;
    enc->speed      = 0.0f;
    enc->speedRaw   = 0.0f;
}

float Encoder_GetSpeed(Encoder_t *enc)
{
    return enc->speed;
}

int32_t Encoder_GetPosition(Encoder_t *enc)
{
    return enc->totalCount;
}
