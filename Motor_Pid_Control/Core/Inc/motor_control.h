#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include"main.h"

typedef struct{
    TIM_HandleTypeDef *TIMx; // PWM定时器句柄
    uint32_t PWM_Channel;        // PWM通道
    GPIO_TypeDef* DIR_Port_1, *DIR_Port_2;   // 方向控制GPIO端口
    uint16_t DIR_Pin_1, DIR_Pin_2;            // 方向控制GPIO引脚
    int16_t current_speed; // 当前速度
    uint8_t direction;    // 方向：0-正转，1-反转
} Motor_TypeDef;

void Motor_Init(void); // 电机初始化
void Motor_SetSpeed(int16_t speed);// 设置电机速度
void Motor_Stop(void); // 停止电机

extern Motor_TypeDef Motor1; // 电机1句柄

#endif // __MOTOR_CONTROL_H
