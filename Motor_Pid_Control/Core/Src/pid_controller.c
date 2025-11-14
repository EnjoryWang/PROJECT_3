#include "pid_controller.h"

PID_Controller angle_pid;

void PID_Init(void)
{
    angle_pid.Kp = 15.0f;
    angle_pid.Ki = 0.5f;
    angle_pid.Kd = 0.1f;
    angle_pid.output_max = 1000.0f;
    angle_pid.output_min = -1000.0f;
    angle_pid.integral_max = 1000.0f;
    PID_Reset();
}

float PID_Calculate(float setpoint,float feedback)
{
    extern uint32_t system_time;
    static uint32_t last_calc_time = 0;
    float dt,error,derivative,output;
    if(last_calc_time ==0)
    {
        dt = 0.01f;
    }
    else{
        dt = (float)(system_time - last_calc_time) / 1000.0f;
        if(dt <= 0)
        dt = 0.01f;
    }
    last_calc_time = system_time; //计算时间间隔（s）
    error = setpoint - feedback; //计算误差
    output = angle_pid.Kp*error; //比例
    angle_pid.integral += error*dt;
    if(angle_pid.integral > angle_pid.integral_max)
      angle_pid.integral = angle_pid.integral_max;
    if(angle_pid.integral < -angle_pid.integral_max)
      angle_pid.integral = -angle_pid.integral_max;
    output += angle_pid.Kd*angle_pid.integral; //积分项（带限幅）
    derivative = (error - angle_pid.prev_error)/dt;
    output += angle_pid.Kd*derivative;
    angle_pid.prev_error = error; //微分项

    if(output > angle_pid.output_max)
     output = angle_pid.output_max;
    if(output < angle_pid.output_min)
     output = angle_pid.output_min; //输出限幅

    return output;

}

void PID_Reset(void)
{
    angle_pid.integral = 0.0f;
    angle_pid.prev_error = 0.0f;
    angle_pid.last_time = 0;
}
