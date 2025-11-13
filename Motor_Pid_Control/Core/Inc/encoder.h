#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

#define ENCODER_PPR 500//编码器每转脉冲数
#define GEAR_RATIO  34 //减速比
#define TOTAL_PULSES (ENCODER_PPR * GEAR_RATIO * 4)//总脉冲数   

float Encoder_GetAngle(void);
int32_t Encoder_GetCount(void);
void Encoder_ClearCount(void);

#endif // __ENCODER_H
