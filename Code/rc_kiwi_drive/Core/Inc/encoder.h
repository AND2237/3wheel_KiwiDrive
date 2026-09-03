#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f1xx_hal.h"
#include "stdint.h"

#define ENCODER_PPR        1768.0f

#define ENCODER_SPEED_ALPHA  0.2f


typedef struct {
    TIM_HandleTypeDef *htim;

    uint16_t  prevCount;
    int32_t   totalCount;

    float     speed;
    float     speedRaw;

    int8_t    dir_sign;
} Encoder_t;


void  Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim);
void  Encoder_Update(Encoder_t *enc, float dt);
void  Encoder_Reset(Encoder_t *enc);

float Encoder_GetSpeed(Encoder_t *enc);
int32_t Encoder_GetPosition(Encoder_t *enc);

#endif
