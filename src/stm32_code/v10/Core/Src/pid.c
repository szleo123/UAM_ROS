/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include "main.h"
#define LOC_DEAD_ZONE 0.0f 
#define SPD_DEAD_ZONE 0.0f

void PID_Init(PID_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0;
    pid->integral = 0;
}

float find_error(float setpoint, float measured_value, bool position) {
	if (position) {
			const int N = 8192;
			const int HALF = N / 2 ; 
			float d = setpoint - measured_value; 
			d = fmod(fmod((d + HALF), N) + N, N) - HALF; 
			if (d == -HALF) d = HALF; 
			return d; 
	}
	else {
	return setpoint - measured_value; 
	}
}
float PID_Compute(PID_t *pid, float setpoint, float measured_value, bool position) {
    float error = find_error(setpoint, measured_value, position);
		if (position && error >= -LOC_DEAD_ZONE && error <= LOC_DEAD_ZONE){
				error = 0; 
				pid->prev_error = error; 
		}
		if (!position && error >= -SPD_DEAD_ZONE && error <= SPD_DEAD_ZONE){
				error = 0; 
				pid->prev_error = error; 
		}
		//pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error; 
		float res = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
		
		return res;
}


