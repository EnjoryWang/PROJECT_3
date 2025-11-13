#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include "main.h"

typedef struct{
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    float prev_error; // 上一次误差
    float integral;       // 积分值
    float integral_max; //最大积分值
    float output_max, output_min; // 输出限幅
    uint32_t last_time; // 上次计算时间
} PID_Controller;

void PID_Init(void);
float PID_Calculate(float setpoint,float feedback);
void PID_Reset(void);

extern PID_Controller angle_pid; // 角度控制PID

#endif // __PID_CONTROLLER_H
