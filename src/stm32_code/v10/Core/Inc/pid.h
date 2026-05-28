#ifndef __pid_H
#define __pid_H
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdbool.h>
enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,//PID相关枚举类型
};

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
float PID_Compute(PID_t *pid, float setpoint, float measured_value, bool position);


#endif
		

		
