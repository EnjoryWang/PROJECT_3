#include "serial_interface.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// 外部声明
extern PID_Controller angle_pid;
extern float Encoder_GetAngle(void);
extern void Encoder_ClearCount(void);
extern void PID_Reset(void);
extern void Motor_Stop(void);
extern uint8_t vofa_enabled;

void Process_Serial_Command(char* cmd)
{
    float temp_val;
    
    // 移除命令前后的空白字符
    char *trimmed_cmd = cmd;
    while(*trimmed_cmd == ' ') trimmed_cmd++;
    
    if(sscanf(trimmed_cmd, "set kp %f", &temp_val) == 1) {
        angle_pid.Kp = temp_val;
        printf("Kp set to: %.3f\r\n", temp_val);
    }
    else if(sscanf(trimmed_cmd, "set ki %f", &temp_val) == 1) {
        angle_pid.Ki = temp_val;
        printf("Ki set to: %.3f\r\n", temp_val);
    }
    else if(sscanf(trimmed_cmd, "set kd %f", &temp_val) == 1) {
        angle_pid.Kd = temp_val;
        printf("Kd set to: %.3f\r\n", temp_val);
    }
    else if(strcmp(trimmed_cmd, "pid show") == 0) {
        printf("Current PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", 
               angle_pid.Kp, angle_pid.Ki, angle_pid.Kd);
    }
    else if(strcmp(trimmed_cmd, "pid reset") == 0) {
        PID_Reset();
        printf("PID reset done\r\n");
    }
    else if(strcmp(trimmed_cmd, "angle") == 0) {
        extern float Encoder_GetAngle(void);
        float angle = Encoder_GetAngle();
        printf("Current angle: %.2f°\r\n", angle);
    }
    else if(strcmp(trimmed_cmd, "zero") == 0) {
        extern void Encoder_ClearCount(void);
        Encoder_ClearCount();
        printf("Encoder zero position set\r\n");
    }
    else if(strcmp(trimmed_cmd, "help") == 0) {
        printf("=== Motor PID Control Commands ===\r\n");
        printf("set kp <value>    - Set proportional gain\r\n");
        printf("set ki <value>    - Set integral gain\r\n");
        printf("set kd <value>    - Set derivative gain\r\n");
        printf("pid show          - Show current PID values\r\n");
        printf("pid reset         - Reset PID integral and error\r\n");
        printf("angle             - Read current angle\r\n");
        printf("zero              - Set current position as zero\r\n");
        printf("help              - Show this help\r\n");
		}
    else if (strcmp(cmd, "vofa on") == 0) {
    vofa_enabled = 1;
    printf("VOFA+ data streaming ENABLED\r\n");
    }
    else if (strcmp(cmd, "vofa off") == 0) {
    vofa_enabled = 0;
    printf("VOFA+ data streaming DISABLED\r\n");
    }
    
    else {
        printf("Unknown command: %s\r\n", trimmed_cmd);
        printf("Type 'help' for available commands.\r\n");
    }
}
