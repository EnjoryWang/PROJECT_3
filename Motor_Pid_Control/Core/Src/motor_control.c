#include"motor_control.h"

Motor_TypeDef Motor1;
extern TIM_HandleTypeDef htim2;

void Motor_Init(void)
{
   Motor1.TIMx = &htim2; // 使用定时器2
    Motor1.PWM_Channel = TIM_CHANNEL_1; // 使用通道1
    Motor1.DIR_Port = GPIOA; // 方向控制端口
    Motor1.DIR_Pin = GPIO_PIN_1; // 方向控制引脚
    Motor1.current_speed = 0;
    Motor1.direction = 0;
}

void Motor_SetSpeed(int16_t speed)
{
  uint16_t pwm_value = 0;
  if(speed > 1000) speed = 1000;
  if(speed < -1000) speed = -1000;// 限制速度范围在-1000到1000
  
  if(speed >= 0)
  {
    HAL_GPIO_WritePin(Motor1.DIR_Port, Motor1.DIR_Pin, GPIO_PIN_RESET);
    Motor1.direction = 0;
    pwm_value = speed;
  }
  else{
    HAL_GPIO_WritePin(Motor1.DIR_Port,Motor1.DIR_Pin,GPIO_PIN_SET);
    Motor1.direction = 1;
    pwm_value = -speed;
  }  //设置方向
  __HAL_TIM_SET_COMPARE(Motor1.TIMx,Motor1.PWM_Channel,pwm_value);
  Motor1.current_speed = speed; //装载pwm
}

 void Motor_Stop(void)
  {
    Motor_SetSpeed(0);
  }
