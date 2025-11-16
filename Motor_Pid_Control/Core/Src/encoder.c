#include "encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;
int32_t encoder_overflow = 0;
float Encoder_GetAngle(void)
{
    int32_t total_count = Encoder_GetCount();
    float angle = (float)total_count*360.0f / TOTAL_PULSES;
    while(angle < 0) angle += 360.0f;
    while(angle >= 360.0f) angle -= 360.0f;

    return angle;
}

int32_t Encoder_GetCount(void)
{
    return encoder_overflow + (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
}

void Encoder_ClearCount(void)
{
    encoder_overflow = 0;
    __HAL_TIM_SET_COUNTER(&htim3,0);
}
