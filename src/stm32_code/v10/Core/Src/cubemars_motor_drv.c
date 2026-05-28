#include "cubemars_motor_drv.h"
#include <string.h>
#include <math.h>

// 辅助函数：大端模式写入int32 (CubeMars协议要求高位在前)
static void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 24);
    buffer[(*index)++] = (uint8_t)(number >> 16);
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

// 辅助函数：大端模式写入int16
static void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

// MIT模式用的浮点转Uint辅助函数
static int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
    float span = x_max - x_min;
    float offset = x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  位置速度环模式打包发送
 * @note   对应手册 4.1.7 章节
 */
static void cm_send_pos_spd(hcan_t* hcan, uint8_t id, float pos, float spd, float acc)
{
    uint8_t buffer[8];
    int32_t index = 0;
    // ID = 6 << 8 | Motor_ID
    uint32_t can_id = (CM_POS_SPD_MODE << 8) | id;

    // Pos: int32, *10000.0
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &index);
    // Spd: int16, /10.0
    buffer_append_int16(buffer, (int16_t)(spd / 10.0f), &index);
    // Acc: int16, /10.0
    buffer_append_int16(buffer, (int16_t)(acc / 10.0f), &index);

    // 使用支持扩展帧的V2发送函数
    fdcanx_send_data_v2(hcan, can_id, FDCAN_EXTENDED_ID, buffer, 8);
}

/**
 * @brief  MIT (运控) 模式打包发送
 * @note   对应手册 4.2 章节
 */
static void cm_send_mit(hcan_t* hcan, cubemars_motor_t *motor)
{
    uint8_t data[8];
    uint16_t p_int, v_int, kp_int, kd_int, t_int;
    uint32_t can_id = (CM_MIT_MODE << 8) | motor->id; 

    // 使用新变量名 p_limit, v_limit 等
    p_int = float_to_uint(motor->ctrl.pos_set, -motor->mit_para.p_limit, motor->mit_para.p_limit, 16);
    v_int = float_to_uint(motor->ctrl.vel_set, -motor->mit_para.v_limit, motor->mit_para.v_limit, 12);
    kp_int = float_to_uint(motor->ctrl.kp_set, 0, motor->mit_para.kp_limit, 12);
    kd_int = float_to_uint(motor->ctrl.kd_set, 0, motor->mit_para.kd_limit, 12);
    t_int = float_to_uint(motor->ctrl.tor_set, -motor->mit_para.t_limit, motor->mit_para.t_limit, 12);

    data[0] = (kp_int >> 4);
    data[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8);
    data[2] = kd_int & 0xFF;
    data[3] = (p_int >> 8);
    data[4] = p_int & 0xFF;
    data[5] = (v_int >> 4);
    data[6] = ((v_int & 0xF) << 4) | (t_int >> 8);
    data[7] = t_int & 0xFF;

    fdcanx_send_data_v2(hcan, can_id, FDCAN_EXTENDED_ID, data, 8);
}

/**
 * @brief  通用单值发送 (适用于速度、位置、电流模式)
 * @note   手册 4.1.2 - 4.1.5
 */
static void cm_send_single_value(hcan_t* hcan, uint8_t id, uint8_t mode, int32_t val)
{
    uint8_t buffer[8] = {0}; // 初始化清零
    int32_t index = 0;
    uint32_t can_id = (mode << 8) | id;
    
    // 大多数单值模式只需要前4个字节
    buffer_append_int32(buffer, val, &index); 
    
    fdcanx_send_data_v2(hcan, can_id, FDCAN_EXTENDED_ID, buffer, 8); // 始终发送8字节DLC
}

/**
 * @brief  主发送控制逻辑
 */
void cm_motor_ctrl_send(hcan_t* hcan, cubemars_motor_t *motor)
{
    switch(motor->ctrl.mode)
    {
        case CM_POS_SPD_MODE:
            // 6: 位置速度环
            cm_send_pos_spd(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.acc_set);
            break;

        case CM_MIT_MODE:
            // 8: MIT模式
            cm_send_mit(hcan, motor);
            break;

        case CM_SPD_MODE:
            // 3: 速度模式 (RPM)
            cm_send_single_value(hcan, motor->id, CM_SPD_MODE, (int32_t)motor->ctrl.vel_set); 
            break;

        case CM_POS_MODE:
            // 4: 位置模式 (deg * 10000)
            cm_send_single_value(hcan, motor->id, CM_POS_MODE, (int32_t)(motor->ctrl.pos_set * 10000.0f)); 
            break;
            
        case CM_CUR_MODE:
            // 1: 电流模式 (A * 1000)
            cm_send_single_value(hcan, motor->id, CM_CUR_MODE, (int32_t)(motor->ctrl.cur_set * 1000.0f));
            break;

        default:
            break;
    }
}

/**
 * @brief  解析反馈数据
 * @note   Manual 4.3.1
 */
void cm_motor_fb_decode(cubemars_motor_t *motor, uint8_t *rx_data)
{
    // Data[0-1]: Pos (int16) 0.1 deg (大端)
    int16_t pos_int = (rx_data[0] << 8) | rx_data[1];
    // Data[2-3]: Spd (int16) 10 RPM (大端)
    int16_t spd_int = (rx_data[2] << 8) | rx_data[3];
    // Data[4-5]: Cur (int16) 0.01 A (大端)
    int16_t cur_int = (rx_data[4] << 8) | rx_data[5];

    motor->fb.pos = (float)pos_int * 0.1f;
    motor->fb.vel = (float)spd_int * 10.0f; // 手册说明比例为10:1 (32000 -> 320000rpm)
    motor->fb.cur = (float)cur_int * 0.01f;
    motor->fb.temp = (int8_t)rx_data[6];
    motor->fb.error = (int8_t)rx_data[7];
}

/**
 * @brief  CubeMars 上电安全探测函数 (获取初始位置但不使电机转动)
 * @note   发送 0% 的占空比指令 (Mode 0)
 */
void cm_motor_probe(hcan_t* hcan, cubemars_motor_t *motor)
{
    uint8_t buffer[8] = {0}; 
    
    // 控制模式 1：电流环模式 (Mode 1)
    // 依据手册：Control mode ID = 1，改成0，其他不变就是0占空比模式
    uint32_t can_id = (1 << 8) | motor->id; 
    
    // 发送 0A 电流 (4字节整数 0)
    // 物理效果：电机产生 0 N·m 扭矩，绝对不会转动
    buffer[0] = 0; 
    buffer[1] = 0; 
    buffer[2] = 0; 
    buffer[3] = 0;
    
    // 使用扩展帧 (FDCAN_EXTENDED_ID) 强制电机回传一帧反馈
    fdcanx_send_data_v2(hcan, can_id, FDCAN_EXTENDED_ID, buffer, 8);
}

void cm_motor_set_absolute_zero(hcan_t* hcan, cubemars_motor_t *motor, uint8_t save)
{
    uint8_t buffer[8] = {0};

    // cmd = 5 → SET_ORIGIN_HERE
    uint32_t can_id = (5 << 8) | motor->id;

    // 0 = 临时原点
    // 1 = 永久原点（写Flash）
    buffer[0] = save;

    fdcanx_send_data_v2(hcan, can_id, FDCAN_EXTENDED_ID, buffer, 1);
}

