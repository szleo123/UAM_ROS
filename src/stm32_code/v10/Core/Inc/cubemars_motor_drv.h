#ifndef _CUBEMARS_MOTOR_DRV_H_
#define _CUBEMARS_MOTOR_DRV_H_

#include "main.h"
#include "bsp_fdcan.h" // 确保这里包含了 FDCAN 相关定义

// 控制模式定义 (参考手册 V3.1.0)
typedef enum {
    CM_DUTY_MODE    = 0,    // 占空比模式
    CM_CUR_MODE     = 1,    // 电流环模式
    CM_BRAKE_MODE   = 2,    // 电流刹车模式
    CM_SPD_MODE     = 3,    // 速度环模式
    CM_POS_MODE     = 4,    // 位置环模式
    CM_ORIGIN_MODE  = 5,    // 设置原点模式
    CM_POS_SPD_MODE = 6,    // 位置速度环模式
    CM_MIT_MODE     = 8     // 力控模式 (MIT)
} cm_ctrl_mode_e;

// 电机参数结构体
typedef struct {
    uint8_t  id;            // 电机ID (例如 0x01, 0x08)
    
    struct {
        cm_ctrl_mode_e mode; // 当前控制模式
        float pos_set;       // 目标位置 (deg)
        float vel_set;       // 目标速度 (ERPM 或 rad/s)
        float tor_set;       // 目标力矩 (Nm) 或 电流 (A)
        float cur_set;       // 目标电流 (A)
        float kp_set;        // MIT模式 KP
        float kd_set;        // MIT模式 KD
        float acc_set;       // 目标加速度 (ERPM/s, 仅Pos-Spd模式用)
    } ctrl;

    struct {
        float pos;           // 反馈位置 (deg)
        float vel;           // 反馈速度 (ERPM)
        float cur;           // 反馈电流 (A)
        int8_t temp;         // 电机温度
        int8_t error;        // 错误码
    } fb; // feedback

    // MIT模式专用限制参数 (根据电机型号不同而不同)
    struct {
        float p_limit;       // 原 P_MAX
        float v_limit;       // 原 V_MAX
        float t_limit;       // 原 T_MAX
        float kp_limit;      // 原 KP_MAX (解决报错)
        float kd_limit;      // 原 KD_MAX (解决报错)
    } mit_para;

} cubemars_motor_t;

// 函数声明
void cm_motor_ctrl_send(hcan_t* hcan, cubemars_motor_t *motor);
void cm_motor_fb_decode(cubemars_motor_t *motor, uint8_t *rx_data);
void cm_motor_probe(hcan_t* hcan, cubemars_motor_t *motor);
void cm_motor_probe_safe(hcan_t* hcan, cubemars_motor_t *motor);
void cm_motor_set_absolute_zero(hcan_t* hcan, cubemars_motor_t *motor, uint8_t save);

#endif



