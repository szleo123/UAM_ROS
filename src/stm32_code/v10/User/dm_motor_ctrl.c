#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "string.h"
#include "stdbool.h"
// 引入 CubeMars 的控制头文件
#include "cubemars_motor_ctrl.h"

motor_t motor[num];


/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310电机初始化函数
* @param:      	void
* @retval:     	void
* @details:    	初始化1个DM4310型号的电机，设置默认参数和控制模式。
*               设置ID、控制模式和命令模式等信息。
************************************************************************
**/
void dm_motor_init(void)
{
	// 初始化Motor1和Motor2的电机结构
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
	memset(&motor[Motor4], 0, sizeof(motor[Motor4]));
	memset(&motor[Motor5], 0, sizeof(motor[Motor5]));
	memset(&motor[Motor6], 0, sizeof(motor[Motor6]));

	// 设置Motor1的电机信息，DM4340
	motor[Motor1].id = 0x01;
	motor[Motor1].mst_id = 0x11;	// 实际没有用上，只做标识作用
	motor[Motor1].tmp.read_flag = 1;
	motor[Motor1].ctrl.mode 	= mit_mode;
	motor[Motor1].ctrl.vel_set 	= 0.0f;
	motor[Motor1].ctrl.pos_set 	= 0.0f;
	motor[Motor1].ctrl.tor_set 	= 5.0f;
	motor[Motor1].ctrl.cur_set 	= 0.02f;
	motor[Motor1].ctrl.kp_set 	= 360.0f;
	motor[Motor1].ctrl.kd_set 	= 2.0f;
	motor[Motor1].tmp.PMAX		= 12.5f;
	motor[Motor1].tmp.VMAX		= 20.0f;
	motor[Motor1].tmp.TMAX		= 28.0f;
	
	// 测试Motor1的电机信息，DM4310
//	motor[Motor1].id = 0x01;
//	motor[Motor1].mst_id = 0x11;	// 实际没有用上，只做标识作用
//	motor[Motor1].tmp.read_flag = 1;
//	motor[Motor1].ctrl.mode 	= mit_mode;
//	motor[Motor1].ctrl.vel_set 	= 0.0f;
//	motor[Motor1].ctrl.pos_set 	= 0.0f;
//	motor[Motor1].ctrl.tor_set 	= 0.8f;
//	motor[Motor1].ctrl.cur_set 	= 0.02f;
//	motor[Motor1].ctrl.kp_set 	= 40.0f;
//	motor[Motor1].ctrl.kd_set 	= 0.3f;
//	motor[Motor1].tmp.PMAX		= 12.5f;
//	motor[Motor1].tmp.VMAX		= 50.0f;
//	motor[Motor1].tmp.TMAX		= 5.0f;
    
    // 设置Motor2的电机信息,DM10010
//	motor[Motor2].id = 0x02;
//	motor[Motor2].mst_id = 0x12;
//	motor[Motor2].tmp.read_flag = 1;
//	motor[Motor2].ctrl.mode 	= mit_mode;
//	motor[Motor2].ctrl.vel_set 	= 0.0f;
//	motor[Motor2].ctrl.pos_set 	= 1.6f; 
//	motor[Motor2].ctrl.tor_set 	= 20.0f;
//	motor[Motor2].ctrl.cur_set 	= 0.02f;
//	motor[Motor2].ctrl.kp_set 	= 480.0f;
//	motor[Motor2].ctrl.kd_set 	= 2.0f;
//	motor[Motor2].tmp.PMAX		= 12.5f;
//	motor[Motor2].tmp.VMAX		= 20.0f;
//	motor[Motor2].tmp.TMAX		= 200.0f;
  // 测试Motor2的电机信息，DM4310
	motor[Motor2].id = 0x02;
	motor[Motor2].mst_id = 0x12;	
	motor[Motor2].tmp.read_flag = 1;
	motor[Motor2].ctrl.mode 	= mit_mode;
	motor[Motor2].ctrl.vel_set 	= 0.0f;
	motor[Motor2].ctrl.pos_set 	= 0.0f; 
	motor[Motor2].ctrl.tor_set 	= 0.8f;
	motor[Motor2].ctrl.cur_set 	= 0.02f;
	motor[Motor2].ctrl.kp_set 	= 40.0f;
	motor[Motor2].ctrl.kd_set 	= 0.3f;
	motor[Motor2].tmp.PMAX		= 12.5f;
	motor[Motor2].tmp.VMAX		= 50.0f;
	motor[Motor2].tmp.TMAX		= 5.0f;
    // 设置Motor3的电机信息，DM4340
//	motor[Motor3].id = 0x03;
//	motor[Motor3].mst_id = 0x13;	// 实际没有用上，只做标识作用
//	motor[Motor3].tmp.read_flag = 1;
//	motor[Motor3].ctrl.mode 	= mit_mode;
//	motor[Motor3].ctrl.vel_set 	= 0.0f;
//	motor[Motor3].ctrl.pos_set 	= -3.14f;
//	motor[Motor3].ctrl.tor_set 	= 8.0f;
//	motor[Motor3].ctrl.cur_set 	= 0.02f;
//	motor[Motor3].ctrl.kp_set 	= 480.0f;
//	motor[Motor3].ctrl.kd_set 	= 1.0f;
//	motor[Motor3].tmp.PMAX		= 3.14159f;
//	motor[Motor3].tmp.VMAX		= 20.0f;
//	motor[Motor3].tmp.TMAX		= 28.0f;
	
	  // 设置Motor4的电机信息，DM4310
	motor[Motor4].id = 0x04;
	motor[Motor4].mst_id = 0x14;	// 实际没有用上，只做标识作用
	motor[Motor4].tmp.read_flag = 1;
	motor[Motor4].ctrl.mode 	= mit_mode;
	motor[Motor4].ctrl.vel_set 	= 0.0f;
	motor[Motor4].ctrl.pos_set 	= 0.0f;
	motor[Motor4].ctrl.tor_set 	= 1.0f;
	motor[Motor4].ctrl.cur_set 	= 0.02f;
	motor[Motor4].ctrl.kp_set 	= 100.0f;
	motor[Motor4].ctrl.kd_set 	= 0.5f;
	motor[Motor4].tmp.PMAX		= 12.5f;
	motor[Motor4].tmp.VMAX		= 30.0f;
	motor[Motor4].tmp.TMAX		= 10.0f;
	
	  // 设置Motor5的电机信息，DM3507
	motor[Motor5].id = 0x05;
	motor[Motor5].mst_id = 0x15;	// 实际没有用上，只做标识作用
	motor[Motor5].tmp.read_flag = 1;
	motor[Motor5].ctrl.mode 	= mit_mode;
	motor[Motor5].ctrl.vel_set 	= 0.0f;
	motor[Motor5].ctrl.pos_set 	= 1.57f;
	motor[Motor5].ctrl.tor_set 	= 0.8f;
	motor[Motor5].ctrl.cur_set 	= 0.02f;
	motor[Motor5].ctrl.kp_set 	= 60.0f;
	motor[Motor5].ctrl.kd_set 	= 0.3f;
	motor[Motor5].tmp.PMAX		= 12.566f;
	motor[Motor5].tmp.VMAX		= 50.0f;
	motor[Motor5].tmp.TMAX		= 5.0f;
	
//	// 测试空载4310用
//	motor[Motor5].id = 0x05;
//	motor[Motor5].mst_id = 0x15;	// 实际没有用上，只做标识作用
//	motor[Motor5].tmp.read_flag = 1;
//	motor[Motor5].ctrl.mode 	= mit_mode;
//	motor[Motor5].ctrl.vel_set 	= 0.0f;
//	motor[Motor5].ctrl.pos_set 	= 0.0f;
//	motor[Motor5].ctrl.tor_set 	= 1.0f;
//	motor[Motor5].ctrl.cur_set 	= 0.02f;
//	motor[Motor5].ctrl.kp_set 	= 80.0f;
//	motor[Motor5].ctrl.kd_set 	= 1.0f;
//	motor[Motor5].tmp.PMAX		= 12.5f;
//	motor[Motor5].tmp.VMAX		= 30.0f;
//	motor[Motor5].tmp.TMAX		= 10.0f;
	
}
/**
************************************************************************
* @brief:      	read_all_motor_data: 读取电机的所有寄存器的数据信息
* @param:      	motor_t：电机参数结构体
* @retval:     	void
* @details:    	逐次发送读取命令
************************************************************************
**/
void read_all_motor_data(motor_t *motor)
{
    switch (motor->tmp.read_flag)
    {
		case 1:  read_motor_data(motor->id, RID_UV_VALUE);  break; // UV_Value
		case 2:  read_motor_data(motor->id, RID_KT_VALUE);  break; // KT_Value
		case 3:  read_motor_data(motor->id, RID_OT_VALUE);  break; // OT_Value
		case 4:  read_motor_data(motor->id, RID_OC_VALUE);  break; // OC_Value
		case 5:  read_motor_data(motor->id, RID_ACC);       break; // ACC
		case 6:  read_motor_data(motor->id, RID_DEC);       break; // DEC
		case 7:  read_motor_data(motor->id, RID_MAX_SPD);   break; // MAX_SPD
		case 8:  read_motor_data(motor->id, RID_MST_ID);    break; // MST_ID 
		case 9:  read_motor_data(motor->id, RID_ESC_ID);    break; // ESC_ID
		case 10: read_motor_data(motor->id, RID_TIMEOUT);   break; // TIMEOUT 
		case 11: read_motor_data(motor->id, RID_CMODE);     break; // CTRL_MODE 
		case 12: read_motor_data(motor->id, RID_DAMP);      break; // Damp 
		case 13: read_motor_data(motor->id, RID_INERTIA);   break; // Inertia
		case 14: read_motor_data(motor->id, RID_HW_VER);    break; // Rsv1 
		case 15: read_motor_data(motor->id, RID_SW_VER);    break; // sw_ver 
		case 16: read_motor_data(motor->id, RID_SN);        break; // Rsv2 
		case 17: read_motor_data(motor->id, RID_NPP);       break; // NPP 
		case 18: read_motor_data(motor->id, RID_RS);        break; // Rs 
		case 19: read_motor_data(motor->id, RID_LS);        break; // Ls 
		case 20: read_motor_data(motor->id, RID_FLUX);      break; // Flux 
		case 21: read_motor_data(motor->id, RID_GR);        break; // Gr 
		case 22: read_motor_data(motor->id, RID_PMAX);      break; // PMAX 
		case 23: read_motor_data(motor->id, RID_VMAX);      break; // VMAX 
		case 24: read_motor_data(motor->id, RID_TMAX);      break; // TMAX 
		case 25: read_motor_data(motor->id, RID_I_BW);      break; // I_BW 
		case 26: read_motor_data(motor->id, RID_KP_ASR);    break; // KP_ASR 
		case 27: read_motor_data(motor->id, RID_KI_ASR);    break; // KI_ASR 
		case 28: read_motor_data(motor->id, RID_KP_APR);    break; // KP_APR 
		case 29: read_motor_data(motor->id, RID_KI_APR);    break; // KI_APR 
		case 30: read_motor_data(motor->id, RID_OV_VALUE);  break; // OV_Value 
		case 31: read_motor_data(motor->id, RID_GREF);      break; // GREF 
		case 32: read_motor_data(motor->id, RID_DETA);      break; // Deta 
		case 33: read_motor_data(motor->id, RID_V_BW);      break; // V_BW 
		case 34: read_motor_data(motor->id, RID_IQ_CL);     break; // IQ_c1 
		case 35: read_motor_data(motor->id, RID_VL_CL);     break; // VL_c1 
		case 36: read_motor_data(motor->id, RID_CAN_BR);    break; // can_br 
		case 37: read_motor_data(motor->id, RID_SUB_VER);   break; // sub_ver 
		case 38: read_motor_data(motor->id, RID_U_OFF);     break; // u_off 
		case 39: read_motor_data(motor->id, RID_V_OFF);     break; // v_off 
		case 40: read_motor_data(motor->id, RID_K1);        break; // k1 
		case 41: read_motor_data(motor->id, RID_K2);        break; // k2 
		case 42: read_motor_data(motor->id, RID_M_OFF);     break; // m_off 
		case 43: read_motor_data(motor->id, RID_DIR);       break; // dir 
		case 44: read_motor_data(motor->id, RID_P_M);       break; // pm 
		case 45: read_motor_data(motor->id, RID_X_OUT);     break; // xout 
    }
}
/**
************************************************************************
* @brief:      	receive_motor_data: 接收电机返回的数据信息
* @param:      	motor_t：电机参数结构体
* @param:      	data：接收的数据
* @retval:     	void
* @details:    	逐次接收电机回传的参数信息
************************************************************************
**/
void receive_motor_data(motor_t *motor, uint8_t *data)
{
	if(motor->tmp.read_flag == 0)
		return ;
	
	float_type_u y;
	
	if(data[2] == 0x33)
	{
		uint16_t rid_value = data[3];
		y.b_val[0] = data[4];
		y.b_val[1] = data[5];
		y.b_val[2] = data[6];
		y.b_val[3] = data[7];
		
		switch (rid_value) 
		{
			case RID_UV_VALUE: motor->tmp.UV_Value = y.f_val; motor->tmp.read_flag =  2; break;
			case RID_KT_VALUE: motor->tmp.KT_Value = y.f_val; motor->tmp.read_flag =  3; break;
			case RID_OT_VALUE: motor->tmp.OT_Value = y.f_val; motor->tmp.read_flag =  4; break;
			case RID_OC_VALUE: motor->tmp.OC_Value = y.f_val; motor->tmp.read_flag =  5; break;
			case RID_ACC:      motor->tmp.ACC      = y.f_val; motor->tmp.read_flag =  6; break;
			case RID_DEC:      motor->tmp.DEC      = y.f_val; motor->tmp.read_flag =  7; break;
			case RID_MAX_SPD:  motor->tmp.MAX_SPD  = y.f_val; motor->tmp.read_flag =  8; break;
			case RID_MST_ID:   motor->tmp.MST_ID   = y.u_val; motor->tmp.read_flag =  9; break;
			case RID_ESC_ID:   motor->tmp.ESC_ID   = y.u_val; motor->tmp.read_flag = 10; break;
			case RID_TIMEOUT:  motor->tmp.TIMEOUT  = y.u_val; motor->tmp.read_flag = 11; break;
			case RID_CMODE:    motor->tmp.cmode    = y.u_val; motor->tmp.read_flag = 12; break;
			case RID_DAMP:     motor->tmp.Damp     = y.f_val; motor->tmp.read_flag = 13; break;
			case RID_INERTIA:  motor->tmp.Inertia  = y.f_val; motor->tmp.read_flag = 14; break;
			case RID_HW_VER:   motor->tmp.hw_ver   = y.u_val; motor->tmp.read_flag = 15; break;
			case RID_SW_VER:   motor->tmp.sw_ver   = y.u_val; motor->tmp.read_flag = 16; break;
			case RID_SN:       motor->tmp.SN       = y.u_val; motor->tmp.read_flag = 17; break;
			case RID_NPP:      motor->tmp.NPP      = y.u_val; motor->tmp.read_flag = 18; break;
			case RID_RS:       motor->tmp.Rs       = y.f_val; motor->tmp.read_flag = 19; break;
			case RID_LS:       motor->tmp.Ls       = y.f_val; motor->tmp.read_flag = 20; break;
			case RID_FLUX:     motor->tmp.Flux     = y.f_val; motor->tmp.read_flag = 21; break;
			case RID_GR:       motor->tmp.Gr       = y.f_val; motor->tmp.read_flag = 22; break;
			case RID_PMAX:     motor->tmp.PMAX     = y.f_val; motor->tmp.read_flag = 23; break;
			case RID_VMAX:     motor->tmp.VMAX     = y.f_val; motor->tmp.read_flag = 24; break;
			case RID_TMAX:     motor->tmp.TMAX     = y.f_val; motor->tmp.read_flag = 25; break;
			case RID_I_BW:     motor->tmp.I_BW     = y.f_val; motor->tmp.read_flag = 26; break;
			case RID_KP_ASR:   motor->tmp.KP_ASR   = y.f_val; motor->tmp.read_flag = 27; break;
			case RID_KI_ASR:   motor->tmp.KI_ASR   = y.f_val; motor->tmp.read_flag = 28; break;
			case RID_KP_APR:   motor->tmp.KP_APR   = y.f_val; motor->tmp.read_flag = 29; break;
			case RID_KI_APR:   motor->tmp.KI_APR   = y.f_val; motor->tmp.read_flag = 30; break;
			case RID_OV_VALUE: motor->tmp.OV_Value = y.f_val; motor->tmp.read_flag = 31; break;
			case RID_GREF:     motor->tmp.GREF     = y.f_val; motor->tmp.read_flag = 32; break;
			case RID_DETA:     motor->tmp.Deta     = y.f_val; motor->tmp.read_flag = 33; break;
			case RID_V_BW:     motor->tmp.V_BW     = y.f_val; motor->tmp.read_flag = 34; break;
			case RID_IQ_CL:    motor->tmp.IQ_cl    = y.f_val; motor->tmp.read_flag = 35; break;
			case RID_VL_CL:    motor->tmp.VL_cl    = y.f_val; motor->tmp.read_flag = 36; break;
			case RID_CAN_BR:   motor->tmp.can_br   = y.u_val; motor->tmp.read_flag = 37; break;
			case RID_SUB_VER:  motor->tmp.sub_ver  = y.u_val; motor->tmp.read_flag = 38; break;
			case RID_U_OFF:    motor->tmp.u_off    = y.f_val; motor->tmp.read_flag = 39; break;
			case RID_V_OFF:    motor->tmp.v_off    = y.f_val; motor->tmp.read_flag = 40; break;
			case RID_K1:       motor->tmp.k1       = y.f_val; motor->tmp.read_flag = 41; break;
			case RID_K2:       motor->tmp.k2       = y.f_val; motor->tmp.read_flag = 42; break;
			case RID_M_OFF:    motor->tmp.m_off    = y.f_val; motor->tmp.read_flag = 43; break;
			case RID_DIR:      motor->tmp.dir      = y.f_val; motor->tmp.read_flag = 44; break;
			case RID_P_M:      motor->tmp.p_m      = y.f_val; motor->tmp.read_flag = 45; break;
			case RID_X_OUT:    motor->tmp.x_out    = y.f_val; motor->tmp.read_flag = 0 ; break;
		}
	}
}

/**
************************************************************************
* @brief:      	fdcan1_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
int64_t rec_t[5]={0};
void fdcan1_rx_callback(void)
{
	uint16_t rec_id_16;
	uint8_t rx_data[8] = {0};
	// 注意：请确保 bsp_fdcan.c 中的接收函数能处理扩展ID，或者直接读取寄存器
    // 这里假设 fdcanx_receive 内部已经能把扩展ID传出来
    // 如果 fdcanx_receive 第2个参数是 uint16_t*，你可能需要强转或者修改该函数签名
    // 临时方案：如果你的底层只支持16位ID指针，先读出来再处理
	fdcanx_receive(&hfdcan1, &rec_id_16, rx_data);
	// 严谨做法：建议修改 fdcanx_receive 的参数 rec_id 为 uint32_t* 以支持扩展帧
  // 如果暂时改不了，且CubeMars反馈ID低16位不冲突，也可以先用 rec_id_16
	switch (rec_id_16)
	{
 		case 0x11: dm_motor_fbdata(&motor[Motor1], rx_data); receive_motor_data(&motor[Motor1], rx_data);rec_t[0]++; 
		if(first_moto_cb)
		{
			init_flag_1 = true;
		}
		break;
		case 0x12: dm_motor_fbdata(&motor[Motor2], rx_data); receive_motor_data(&motor[Motor2], rx_data);rec_t[1]++; 
		if(first_moto_cb)
		{
			init_flag_2 = true;
		}
		break;
	}
}

void fdcan2_rx_callback(void)
{
	uint16_t rec_id_16;
	uint32_t rec_id_32; // 新增：用于存储扩展帧ID
	uint8_t rx_data[8] = {0};
	fdcanx_receive(&hfdcan2, &rec_id_16, rx_data);
	rec_id_32 = rec_id_16; // 同样获取完整的扩展ID低16位
	switch (rec_id_16)
	{
 		case 0x14: dm_motor_fbdata(&motor[Motor4], rx_data); receive_motor_data(&motor[Motor4], rx_data);rec_t[3]++; 
		if(first_moto_cb)
		{
			init_flag_4 = true;
		}
		break;
		case 0x15: dm_motor_fbdata(&motor[Motor5], rx_data); receive_motor_data(&motor[Motor5], rx_data);rec_t[4]++;
		if(first_moto_cb)
		{
			init_flag_5 = true;
		}
		break;
//		case 0x16: dm_motor_fbdata(&motor[Motor6], rx_data); receive_motor_data(&motor[Motor6], rx_data);rec_t[1]++; break;
		case 0x201: case 0x202: case 0x203: case 0x204:
                case 0x205: case 0x206: case 0x207: case 0x208:
                    get_moto_measure(&moto_dji2006, rx_data);  // 解析大疆反馈：:contentReference[oaicite:13]{index=13}
                    // e.g. copy to a global struct, set a flag                    
								break;
								
		// ==========================================================
		// 【CubeMars (AK) 电机专属通道】
		// 0x2967 = 0x29(状态反馈特征) << 8 | 0x67(电机ID)
		// 只有电机完全清醒并发送出 2967 时，才会被这里捕获！
		// ==========================================================
		case 0x2967:
				// 1. 交给专门的 handler 去解析位置、速度、电流
				cubemars_rx_handler_can2(rec_id_32, rx_data);
				
				// 2. 此时 fb.pos 已经绝对更新成了真实的截断位置(如 -128.0)
				// 放心大胆地置位标志，通知 main 函数可以去锁定位置了！
//				if(first_moto_cb)
//				{
//						init_flag_3 = true; 
//				}
				break;
		// --- 2. CubeMars (AK) 电机处理 (新增逻辑) ---
		// AK电机反馈ID通常包含驱动器ID
		// 建议放到 default 里或者通过 ID 范围判断
		default:
		// 尝试解析为 CubeMars 电机数据
		// 注意：传入 rec_id_16 即可，因为反馈ID通常就是电机ID (如 0x08)
//		cubemars_rx_handler_can2(rec_id_32, rx_data);
//		// 之前在 cubemars_motor_ctrl.c 里，我们写过一个 can1 的 handler
//		// 如果你的代码里有 cubemars_rx_handler_can2 就用 can2 的
//		// 如果没有，直接用 can1 的也完全没关系，它只是个解析函数而已
//		// 【极其重要】必须在这里告诉系统：收到反馈了！
//		if(first_moto_cb)
//		{
//				init_flag_3 = true; 
//		}
//		break;
	}
}


