#ifndef _CUBEMARS_MOTOR_CTRL_H_
#define _CUBEMARS_MOTOR_CTRL_H_

#include "cubemars_motor_drv.h"

// 声明全局电机数组，供外部调用
extern cubemars_motor_t cm_motor[4];

// 函数声明
void cubemars_motor_init(void);
void cubemars_rx_handler_can1(uint32_t rx_id, uint8_t *rx_data);
void cubemars_rx_handler_can2(uint32_t rx_id, uint8_t *rx_data);

#endif