/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include "delay.h"
#include <stdio.h> /* printf */
#include "pid.h"
#include "stdbool.h"
#include <math.h>
#include "cubemars_motor_ctrl.h" // 引入CubeMars控制头文件
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ====== Constants ====== */
#define ENC_MOD 8192 // 13-bit encoder modulus
#define ENC_HALF 4096
#define TICKS_TO_RAD (2.0f * (float)M_PI / (float)ENC_MOD)
#define RAD_TO_TICKS ((float)ENC_MOD / (2.0f * (float)M_PI))

/* ====== Helpers ====== */

// Shortest signed difference in encoder ticks in [-4096, 4095]
static inline int32_t wrap_diff_ticks(int32_t tgt, int32_t meas)
{
  int32_t d = tgt - meas;
  while (d > ENC_HALF) d -= ENC_MOD;
  while (d < -ENC_HALF) d += ENC_MOD;
  return d;
}

// Clamp utility
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float rpm_to_rad_s(float rpm) {
    return rpm * (2.0f * (float)M_PI / 60.0f);
}

/* ====== Derivative (first-order low-pass filtered) ======
Given current error e and previous filtered derivative d_filt,
compute new filtered derivative to reduce noise.
alpha in (0,1): smaller => more smoothing.
*/
typedef struct {
  float prev_err;
  float d_filt;
  float alpha; // e.g., 0.1f .. 0.3f (tune)
} DerivLPF;

static inline void derivlpf_init(DerivLPF* d, float alpha)
{
  d->prev_err = 0.0f;
  d->d_filt = 0.0f;
  d->alpha = alpha;
}

static inline float derivlpf_update(DerivLPF* d, float err, float dt)
{
  float raw = (err - d->prev_err) / (dt > 1e-6f ? dt : 1e-6f);
  d->d_filt = d->alpha * raw + (1.0f - d->alpha) * d->d_filt;
  d->prev_err = err;
  return d->d_filt;
}

/* ====== PD block ====== */
typedef struct {
  float kp;
  float kd;
  DerivLPF d;
} PD;




static inline void pd_init(PD* c, float kp, float kd, float d_alpha)
{
  c->kp = kp; c->kd = kd;
  derivlpf_init(&c->d, d_alpha);
}

static inline float pd_update(PD* c, float err, float dt)
{
  float de = derivlpf_update(&c->d, err, dt);
  return c->kp * err + c->kd * de;
}

static inline float pd_update1(PD* c, float err, float dt)
{
  float de = derivlpf_update(&c->d, err, dt);
  return c->kp * err + c->kd * de;
}

/* ====== Double-Loop PD Controller ====== */
typedef struct {
  // Position loop (outputs target speed)
  PD pos_pd;
  float w_ref_limit; // |target speed| limit (rad/s)
  // Speed loop (outputs current)
  PD    vel_pd;
  float current_limit;     // |current| limit, e.g. 10000
  // State for speed estimation
  uint16_t prev_pos_ticks;
  int      has_prev;       // 0 until first update
} DJI2006_PD_Controller;

static inline void dji2006_pd_init(DJI2006_PD_Controller* ctl,
float kp_pos, float kd_pos, float pos_d_alpha,
float kp_vel, float kd_vel, float vel_d_alpha,
float w_ref_limit, float current_limit)
{
  pd_init(&ctl->pos_pd, kp_pos, kd_pos, pos_d_alpha);
  pd_init(&ctl->vel_pd, kp_vel, kd_vel, vel_d_alpha);
  ctl->w_ref_limit = w_ref_limit;
  ctl->current_limit = current_limit;
  ctl->prev_pos_ticks = 0;
  ctl->has_prev = 0;
}

/* ====== Main update ======
Inputs:
target_pos_ticks ∈ [0,8191]
meas_pos_ticks ∈ [0,8191]
dt (s), controller timestep
Output:
commanded current in [-10000, 10000]
*/


float temp_rf;
float temp_ms;


static inline float dji2006_pd_update_with_rpm(DJI2006_PD_Controller* ctl,
                                               int32_t target_pos_ticks,
                                               int32_t meas_total_ang,
                                               float meas_rpm,
                                               float dt)
{
    // --- Outer loop: position error (radians, shortest wrap path)
    int32_t e_ticks = target_pos_ticks-meas_total_ang;
    float   e_pos   = (float)e_ticks * TICKS_TO_RAD;  // rad

    // Position PD -> target speed (rad/s)
    float w_ref = pd_update1(&ctl->pos_pd, e_pos, dt);
//		w_ref = 50.0f;
    w_ref = clampf(w_ref, -ctl->w_ref_limit, ctl->w_ref_limit);

    // --- Inner loop: use ESC-measured speed in RPM
    float w_meas = rpm_to_rad_s(meas_rpm);            // rad/s

//		temp_ms = w_meas;
//		temp_rf = w_ref;
	
    float e_w   = w_ref - w_meas;                     // rad/s error
    float i_cmd = pd_update(&ctl->vel_pd, e_w, dt);   // current command

    // Clamp to ESC current limits ([-10000, 10000])
    i_cmd = clampf(i_cmd, -ctl->current_limit, ctl->current_limit);
    return i_cmd;
}

/* ====== 系统状态机与握手协议定义 ====== */
typedef enum {
    STATE_WAIT_ROS_INIT = 0,  // 等待 ROS2 下发初始化指令
    STATE_WAKE_DAMIAO,        // 唤醒达妙电机并锁定当前位置
    STATE_WAIT_ROS_MOVE,      // [新增] 等待ROS2接管大臂并移动到下垂点
    STATE_WAIT_SETTLE,        // 收到ROS2到位信号，等待重力沉降
    STATE_CALIBRATE_ZERO,     // AKE物理置零
    STATE_NORMAL_RUN          // 初始化彻底完成，全权交给ROS2
} SystemState_t;

volatile SystemState_t system_state = STATE_WAIT_ROS_INIT;

// 小臂 Offset (根据你的测量值填写，单位：度)
// 定义：物理限位位置在 URDF 坐标系下的角度
// 假设：下垂触碰限位时，URDF里是 -150度
#define FOREARM_OFFSET (-100.0f) 


// 串口握手帧的特殊帧头 
#define SYS_CMD_HEADER  0xEE
// 发送给 ROS2 的事件码
#define EVENT_DAMIAO_READY 0x01 // 告诉ROS2：达妙已就绪
#define EVENT_ALL_READY    0x02 // 告诉ROS2：寻零完成，全解除封印
// 接收来自 ROS2 的事件码
#define CMD_REACHED_DROP_POSE 0x01 // ROS2告诉单片机：大臂到位了
#define CMD_START_DAMIAO_INIT 0x10 // ROS2告诉单片机：开始唤醒达妙电机

volatile bool ros_start_damiao_init = false; // ROS2初始化启动标志位
volatile bool ros_reached_drop_pose = false; // ROS2到位标志位


// 修改 init_flag_3 为 false，它现在代表寻零是否完成
bool init_flag_3 = false; 

// 增加一个计时器，用于状态切换
volatile uint32_t state_timer = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rxbuf_uart7[1];
uint8_t rxcnt_uart7 = 0;
uint8_t rxdata_uart7[14];
// 全在0 rad： ff 00 00 00 00 00 00 00 00 00 00 00 00 ff
// 全在1 rad： ff e8 03 e8 03 e8 03 e8 03 e8 03 e8 03 81
volatile bool uart10_tx_ready = true;
uint8_t txdata_uart10[14];
volatile bool uart10_has_pending_event = false;
uint8_t uart10_pending_event_code = 0;

// 电机位置变量
float motor1_pos = 0.0f;
float motor2_pos = 0.0f;
float motor3_pos = 0.0f;
float motor4_pos = 0.0f;
float motor5_pos = 0.0f;
float motor6_pos = 0.0f;

// dji
DJI2006_PD_Controller ctrl; 
int32_t target_position = 0; 

// init_flag
bool init_flag_1 = true; // 初始时，应该让所有init_flag为false,若设置true，则初始化成功检测程序会跳过该电机;
bool init_flag_2 = true;

bool init_flag_4 = true; 
bool init_flag_5 = true;

bool first_moto_cb = true; // 初始时，应该为true


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 大疆电机pid计算 begin*/

void setup_controller(void)
{
  // Tune these gains for your rig. Start conservative.
//  float kp_pos = 14.0f; // [A/(rad·s)] indirectly via inner loop; start low
//  float kd_pos = 0.0f; // [A/rad] effect via filtered derivative
	
	float kp_pos = 44.0f; // [A/(rad·s)] indirectly via inner loop; start low
  float kd_pos = 0.0f; // [A/rad] effect via filtered derivative
	
  float kp_vel = 50.0f; // [current / (rad/s)]
  float kd_vel = 0.0f; // [current / (rad/s^2)]
  float pos_d_alpha = 1.0f;
  float vel_d_alpha = 0.1f;

  float w_ref_limit   = 144.0f;     // rad/s (~764 rpm) cap for safety
  float current_limit = 10000.0f;  // ESC range

  dji2006_pd_init(&ctrl,
      kp_pos, kd_pos, pos_d_alpha,
      kp_vel, kd_vel, vel_d_alpha,
      w_ref_limit, current_limit);

}

// Suppose your CAN parser gives you: int16_t speed_rpm;  // signed rpm from ESC
float control_step_rpm(int32_t target_ticks,
                       float dt)
{
    // NOTE: ensure your rpm sign convention matches your current sign
    //       (positive rpm increases encoder ticks in positive direction).
    int32_t meas_ticks = moto_dji2006.total_angle; 
    int16_t speed = moto_dji2006.speed_rpm; 
    return dji2006_pd_update_with_rpm(&ctrl, target_ticks, meas_ticks,
                                      (float)speed, dt);
}


/*大疆电机pid计算 end*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart10) {
        uart10_tx_ready = true;
    }
}


// [新增] 专门给ROS2发送状态事件的函数
void send_system_event_to_ros(uint8_t event_code) {
    uart10_pending_event_code = event_code;
    uart10_has_pending_event = true;
}


// Called from TIM4 ISR (quick, non-blocking)
static inline void pack_and_kick_uart10(void) {
    if (!uart10_tx_ready) return;

    if (uart10_has_pending_event) {
        uint8_t sum = 0;
        memset(txdata_uart10, 0, sizeof(txdata_uart10));
        txdata_uart10[0] = SYS_CMD_HEADER;
        txdata_uart10[1] = uart10_pending_event_code;
        for (int i = 0; i < 13; ++i) {
            sum += txdata_uart10[i];
        }
        txdata_uart10[13] = sum;
        uart10_has_pending_event = false;
        uart10_tx_ready = false;
        HAL_UART_Transmit_DMA(&huart10, txdata_uart10, 14);
        return;
    }

    int16_t ang[6] = {0};
    uint8_t sum = 0; 

    // Use *measured* values here:
    ang[0] = (int16_t)(motor[Motor1].para.pos * 1000.0f);
    ang[1] = (int16_t)(motor[Motor2].para.pos * 1000.0f);
    // 注意单位，cm_motor[0].fb.pos 在驱动里通常是“度”，如果这里需要一致性请自行换算
    // 发给 ROS2 的值 = 电机物理角度 + FOREARM_OFFSET
		// ================== 【核心欺骗逻辑】 ==================
    float urdf_angle_deg = 0.0f;
    if (init_flag_3 == false) {
        // 寻零未完成，向ROS2发送安全的恒定下垂角度，防止MoveIt报警
        urdf_angle_deg = FOREARM_OFFSET;
    } else {
        // 寻零完成，发送叠加了 Offset 的真实物理角度
        urdf_angle_deg = cm_motor[0].fb.pos + FOREARM_OFFSET;
    }
    ang[2] = (int16_t)( (urdf_angle_deg * 3.1415926f / 180.0f) * 1000.0f );
    // ======================================================
    ang[3] = (int16_t)(motor[Motor4].para.pos * 1000.0f);
    ang[4] = (int16_t)(motor[Motor5].para.pos * 1000.0f);
					
    ang[5] = (int16_t)( (moto_dji2006.total_angle * 3.14 *1000 ) /(8191*36)); 

    txdata_uart10[0] = 0xAA; sum += txdata_uart10[0];
    for (int i = 0; i < 6; ++i) {
        uint16_t u = (uint16_t)ang[i];
        txdata_uart10[2*i + 1] = (uint8_t)(u & 0xFF);
        txdata_uart10[2*i + 2] = (uint8_t)((u >> 8) & 0xFF); 
        sum += txdata_uart10[2*i + 1];
        sum += txdata_uart10[2*i + 2];
    }
    txdata_uart10[13] = sum; 

    uart10_tx_ready = false;
    HAL_UART_Transmit_DMA(&huart10, txdata_uart10, 14);
}


 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3) {
        // 1. 达妙电机无脑下发控制帧 (此时目标位置全靠 ROS2 更新)
        dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);
        dm_motor_ctrl_send(&hfdcan1, &motor[Motor2]); 
        dm_motor_ctrl_send(&hfdcan2, &motor[Motor4]);
        dm_motor_ctrl_send(&hfdcan2, &motor[Motor5]);
        
        // 2. AKE 电机保护接管逻辑
        if (init_flag_3 == true) {
            // 彻底寻零完毕，允许对 CubeMars 进行位置闭环强控
            cm_motor_ctrl_send(&hfdcan2, &cm_motor[0]);
        } else {
            // 在ROS2抬大臂和重力下垂的期间，保持发送 0A 探针阻尼
            cm_motor_probe(&hfdcan2, &cm_motor[0]);
        }
        
        // 3. 大疆电机逻辑保持不变
        float i_send = control_step_rpm(target_position, 0.01);
        set_moto_current(&hfdcan2, 0x1ff, (int16_t)i_send);
    }
	if (htim->Instance == TIM4) {
		 pack_and_kick_uart10();  // fast & non-blocking
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
			rxdata_uart7[rxcnt_uart7] = rxbuf_uart7[0];
			rxcnt_uart7++;
			if(rxcnt_uart7 == 1)
			{
				// 找到原来的 if(rxdata_uart7[0]==0xff)，替换为：
				if(rxdata_uart7[0] == 0xFF || rxdata_uart7[0] == SYS_CMD_HEADER)
				{
					// 重新启动接收中断
					HAL_UART_Receive_IT(&huart7, rxbuf_uart7, 1);
				}
				else
				{
					rxcnt_uart7 = 0;
					HAL_UART_Receive_IT(&huart7, rxbuf_uart7, 1);
				}
			}
			else if (rxcnt_uart7 > 1)
			{
				if(rxcnt_uart7 < 14)
				{
					HAL_UART_Receive_IT(&huart7, rxbuf_uart7, 1);
				}
				else if (rxcnt_uart7 == 14)
				{
					rxcnt_uart7 = 0;
					// 解析
					uint8_t checksum_calculated = 0;
					for (int i = 0; i < 13; i++) // 帧头 + 12个数据字节
					{
							checksum_calculated += rxdata_uart7[i];
					}
					if (checksum_calculated == rxdata_uart7[13])
					{
							if (rxdata_uart7[0] == SYS_CMD_HEADER) 
							{
								// 【新增】处理系统级指令帧
								if (rxdata_uart7[1] == CMD_START_DAMIAO_INIT) {
									ros_start_damiao_init = true; // ROS2告诉主循环开始唤醒达妙电机
								}
								else if (rxdata_uart7[1] == CMD_REACHED_DROP_POSE) {
									ros_reached_drop_pose = true; // ROS2告诉主循环大臂到位了
								}
							}
						else if (rxdata_uart7[0] == 0xFF)
						{
							// ================= 正常的轨迹解析开始 =================
							// 解析电机1位置（第2和第3字节）
							int16_t motor1_raw = (rxdata_uart7[2] << 8) | rxdata_uart7[1]; 
							motor1_pos = (float)motor1_raw / 1000.0f;      
				
							// 解析电机2位置（第4和第5字节）
							int16_t motor2_raw = (rxdata_uart7[4] << 8) | rxdata_uart7[3];
							motor2_pos = (float)motor2_raw / 1000.0f;
							
							// 解析电机3位置（第6和第7字节）
							int16_t motor3_raw = (rxdata_uart7[6] << 8) | rxdata_uart7[5];
							motor3_pos = (float)motor3_raw / 1000.0f;
							
							// 解析电机4位置（第8和第9字节）
							int16_t motor4_raw = (rxdata_uart7[8] << 8) | rxdata_uart7[7];
							motor4_pos = (float)motor4_raw / 1000.0f;
							
							// 解析电机5位置（第10和第11字节）
							int16_t motor5_raw = (rxdata_uart7[10] << 8) | rxdata_uart7[9];
							motor5_pos = (float)motor5_raw / 1000.0f;
							
							// 解析电机6位置（第12和第13字节）
							int16_t motor6_raw = (rxdata_uart7[12] << 8) | rxdata_uart7[11];
							motor6_pos = (float)motor6_raw / 1000.0f;
							motor6_pos = (motor6_pos/3.14f)*8191*36; // 转换为编码器的值再×36(总减速比)
							
							// ================= 赋值电机位置 =================
							motor[Motor1].ctrl.pos_set = motor1_pos;
							motor[Motor2].ctrl.pos_set = motor2_pos; // 此时ROS2正在慢慢改变大臂角度
							
							// 【核心修改】：只有AKE寻零完成后，才允许ROS2修改小臂的 target
							if (init_flag_3 == true) {
								float ros_target_deg = motor3_pos * (180.0f / 3.1415926f);
								cm_motor[0].ctrl.pos_set = ros_target_deg - FOREARM_OFFSET;
							}
							
							motor[Motor4].ctrl.pos_set = motor4_pos;
							motor[Motor5].ctrl.pos_set = motor5_pos;
							target_position = (int32_t)motor6_pos;
						}
					}
					else
					{
						//校验失败
					}
					// 校验完不管成功与否都要开启下一帧接收
					HAL_UART_Receive_IT(&huart7, rxbuf_uart7, 1);
				}
			}
			else
			{
				
			}
    }
		
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_FDCAN2_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  /* USER CODE BEGIN 2 */
	power1(1);
  power2(1);
  power3(1);
	HAL_Delay(3000);
	bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
  bsp_fdcan_set_baud(&hfdcan2, CAN_CLASS, CAN_BR_1M);
	bsp_can_init();
		dm_motor_init();
		cubemars_motor_init(); // 初始化CubeMars电机参数
		setup_controller(); 
		HAL_UART_Receive_IT(&huart7, rxbuf_uart7, 1);
	  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
	    switch (system_state)
	    {
	        case STATE_WAIT_ROS_INIT:
	            if (ros_start_damiao_init) {
	                ros_start_damiao_init = false;
	                system_state = STATE_WAKE_DAMIAO;
	            }
	            HAL_Delay(50);
	            break;

	        case STATE_WAKE_DAMIAO:
	            dm_motor_enable(&hfdcan1, &motor[Motor1]);
	            dm_motor_enable(&hfdcan1, &motor[Motor2]);
            dm_motor_enable(&hfdcan2, &motor[Motor4]);
            dm_motor_enable(&hfdcan2, &motor[Motor5]);
            
            if (init_flag_1 && init_flag_2 && init_flag_4 && init_flag_5) {
                HAL_Delay(100);
                // 锁住当前位置
                motor[Motor1].ctrl.pos_set = motor[Motor1].para.pos;
                motor[Motor2].ctrl.pos_set = motor[Motor2].para.pos;
                motor[Motor4].ctrl.pos_set = motor[Motor4].para.pos;
                motor[Motor5].ctrl.pos_set = motor[Motor5].para.pos;
	                
	                HAL_TIM_Base_Start_IT(&htim4); 
	                HAL_TIM_Base_Start_IT(&htim3); 
                
                // 告诉 ROS2: 达妙好了，开始发大臂轨迹吧
                send_system_event_to_ros(EVENT_DAMIAO_READY);
                system_state = STATE_WAIT_ROS_MOVE;
            }
            HAL_Delay(500);
            break;

        case STATE_WAIT_ROS_MOVE:
            // 死等 ROS2 发来 0xEE 到位信号
            if (ros_reached_drop_pose) {
                state_timer = HAL_GetTick();
                system_state = STATE_WAIT_SETTLE;
            }
            break;

        case STATE_WAIT_SETTLE:
            // ROS2 已经把大臂停住了，死等 2 秒让小臂彻底靠稳限位
            if (HAL_GetTick() - state_timer > 2000) {
                system_state = STATE_CALIBRATE_ZERO;
            }
            break;

        case STATE_CALIBRATE_ZERO:
            // 物理置零
            cm_motor_set_absolute_zero(&hfdcan2, &cm_motor[0], 0); 
            HAL_Delay(500); 
            
            cm_motor[0].ctrl.pos_set = 0.0f; 
            init_flag_3 = true; 
            
            // 告诉 ROS2: 机械臂全部就绪，解锁小臂控制权限
            send_system_event_to_ros(EVENT_ALL_READY);
            system_state = STATE_NORMAL_RUN;
            break;

        case STATE_NORMAL_RUN:
            // 此时已经是 ROS2 的天下，单片机只需要在中断里跑就行了
            break;
    }
    /* USER CODE END 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart10, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
