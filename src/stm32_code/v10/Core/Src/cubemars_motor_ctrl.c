#include "cubemars_motor_ctrl.h"
#include <string.h>

// 定义电机数组实例
cubemars_motor_t cm_motor[4];

/**
 * @brief  初始化CubeMars电机参数
 */
void cubemars_motor_init(void)
{
    memset(cm_motor, 0, sizeof(cm_motor));
		
    // --- Motor 0 (AKE80-8) ---
    cm_motor[0].id = 0x67; 
    cm_motor[0].ctrl.mode = CM_MIT_MODE; //CM_MIT_MODE or CM_POS_SPD_MODE
    cm_motor[0].ctrl.pos_set = 0.0f;
    cm_motor[0].ctrl.vel_set = 0.0f; // should set 0 when in mit mode
    cm_motor[0].ctrl.acc_set = 0.0f; // no affect when in mit mode
	
	  //for real
//		cm_motor[0].ctrl.kp_set = 320;
//		cm_motor[0].ctrl.kd_set = 1;
//		cm_motor[0].ctrl.tor_set = 3.8;
    
		// FOR TEST
		cm_motor[0].ctrl.kp_set = 50;
		cm_motor[0].ctrl.kd_set = 0.6;
		cm_motor[0].ctrl.tor_set = 0;
	
    // 【关键修改】使用新变量名初始化
    cm_motor[0].mit_para.p_limit = 12.56f;
    cm_motor[0].mit_para.v_limit = 40.0f;
    cm_motor[0].mit_para.t_limit = 15.0f;
    cm_motor[0].mit_para.kp_limit = 500.0f;
    cm_motor[0].mit_para.kd_limit = 5.0f;
}

/**
 * @brief  CAN1 接收处理回调
 * @note   在 bsp_fdcan.c/fdcan1_rx_callback 中调用
 */
void cubemars_rx_handler_can1(uint32_t rx_id, uint8_t *rx_data)
{
    // CubeMars电机反馈ID通常是: 驱动器ID (低8位)
    // 根据手册 4.3.1，CAN上传报文ID通常就是电机ID (如果是单ID模式)
    // 或者需要屏蔽高位。这里简单取低8位匹配
    uint8_t motor_id = rx_id & 0xFF;

    if (motor_id == cm_motor[0].id) {
        cm_motor_fb_decode(&cm_motor[0], rx_data);
    } 
    else if (motor_id == cm_motor[1].id) {
        cm_motor_fb_decode(&cm_motor[1], rx_data);
    }
		// 【新增】处理第3个电机
    else if (motor_id == cm_motor[2].id) {
        cm_motor_fb_decode(&cm_motor[2], rx_data);
    }
}

/**
 * @brief  CAN2 接收处理回调
 */
void cubemars_rx_handler_can2(uint32_t rx_id, uint8_t *rx_data)
{
    uint8_t motor_id = rx_id & 0xFF;

    if (motor_id == cm_motor[0].id) {
        cm_motor_fb_decode(&cm_motor[0], rx_data);
    } 
    else if (motor_id == cm_motor[1].id) {
        cm_motor_fb_decode(&cm_motor[1], rx_data);
    }
		// 【新增】处理第3个电机
    else if (motor_id == cm_motor[2].id) {
        cm_motor_fb_decode(&cm_motor[2], rx_data);
    }
}