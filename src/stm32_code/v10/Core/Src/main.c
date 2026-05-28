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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
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
#include "cubemars_motor_ctrl.h" // CubeMars header file
#include "ros_protocol.h"

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
    w_ref = clampf(w_ref, -ctl->w_ref_limit, ctl->w_ref_limit);

    // --- Inner loop: use ESC-measured speed in RPM
    float w_meas = rpm_to_rad_s(meas_rpm);            // rad/s
    
    float e_w   = w_ref - w_meas;                     // rad/s error
    float i_cmd = pd_update(&ctl->vel_pd, e_w, dt);   // current command

    // Clamp to ESC current limits ([-10000, 10000])
    i_cmd = clampf(i_cmd, -ctl->current_limit, ctl->current_limit);
    return i_cmd;
}

/* ====== System State Machine & Handshake Protocol Definition ====== */
typedef enum {
    STATE_WAKE_DAMIAO = 0,    // Wake up Damiao motors and lock current position
    STATE_WAIT_HOST_CONNECT,  // Wait for ROS2 to knock
    STATE_WAIT_ROS_MOVE,      // Wait for ROS2 to take over the upper arm and move to the drop point
    STATE_WAIT_SETTLE,        // Received ROS2 in-position signal, wait for gravity settling
    STATE_CALIBRATE_ZERO,     // AKE physical zeroing
    STATE_NORMAL_RUN          // Initialization completely finished, full control handed over to ROS2
} SystemState_t;

volatile SystemState_t system_state = STATE_WAKE_DAMIAO;

// Forearm Offset (fill in according to your measurement, unit: radians)
// Definition: The angle of the physical limit position in the URDF coordinate system
// Assumption: When hanging down and touching the limit, it is -150 degrees in URDF
#define FOREARM_OFFSET (0.0f)  // rad

// Special frame header for serial handshake frame
#define SYS_CMD_HEADER  0xEE
// Event codes sent to ROS2
#define EVENT_DAMIAO_READY 0x01 // Tell ROS2: Damiao is ready
#define EVENT_ALL_READY    0x02 // Tell ROS2: Homing completed, all locks released
// Event codes received from ROS2
#define CMD_REACHED_DROP_POSE 0x01 // ROS2 tells MCU: Upper arm is in position
// Receive from ROS2: Host connection knock
#define CMD_HOST_CONNECT 0x10

// Modify init_flag_3 to false, it now represents whether homing is completed
bool init_flag_3 = false; 

// Add a timer for state switching
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

// Motor position variables
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
// Initially, all init_flags should be false. If set to true, the initialization success detection routine will skip the motor;
bool init_flag_1 = true; 
bool init_flag_2 = false;

bool init_flag_4 = true; 
bool init_flag_5 = true;

// Initially, it should be true
bool first_moto_cb = true; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* DJI motor PID calculation begin */
void setup_controller(void)
{
  // Tune these gains for your rig. Start conservative.
  float kp_pos = 44.0f; // [A/(rad·s)] indirectly via inner loop; start low
  float kd_pos = 0.0f;  // [A/rad] effect via filtered derivative
    
  float kp_vel = 50.0f; // [current / (rad/s)]
  float kd_vel = 0.0f;  // [current / (rad/s^2)]
  float pos_d_alpha = 1.0f;
  float vel_d_alpha = 0.1f;

  float w_ref_limit   = 144.0f;     // rad/s (~764 rpm) cap for safety
  float current_limit = 10000.0f;   // ESC range

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
/* DJI motor PID calculation end */

 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    if (htim->Instance == TIM3) {
        
        // 1. Update all motor targets dynamically via USB ROS2 matrix
        Protocol_Execute_Router();

        // 2. Broadcast DM Motors CAN frames
        dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);
        dm_motor_ctrl_send(&hfdcan1, &motor[Motor2]); 
        dm_motor_ctrl_send(&hfdcan2, &motor[Motor4]);
        dm_motor_ctrl_send(&hfdcan2, &motor[Motor5]);
        
        // 3. Broadcast CubeMars CAN frame (with initialization safety lock)
        if (init_flag_3 == true) {
            cm_motor_ctrl_send(&hfdcan2, &cm_motor[0]);
        } else {
            cm_motor_probe(&hfdcan2, &cm_motor[0]);
        }
        
        // 4. Broadcast DJI 2006 CAN frame
        float i_send = control_step_rpm(target_position, 0.01f);
        set_moto_current(&hfdcan2, 0x1ff, (int16_t)i_send);
    }
    if (htim->Instance == TIM4) {
         Protocol_Send_Feedback();
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
    power1(1);
    power2(1);
    power3(1);
    HAL_Delay(3000);
    bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
    bsp_fdcan_set_baud(&hfdcan2, CAN_CLASS, CAN_BR_1M);
    bsp_can_init();
    dm_motor_init();
    
    // Initialize CubeMars motor parameters, when in pos_spd mode use degree, while in mit mode use rad
    cubemars_motor_init(); 
    setup_controller(); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch (system_state)
    {
        case STATE_WAKE_DAMIAO:
            dm_motor_enable(&hfdcan1, &motor[Motor1]);
            dm_motor_enable(&hfdcan1, &motor[Motor2]);
            dm_motor_enable(&hfdcan2, &motor[Motor4]);
            dm_motor_enable(&hfdcan2, &motor[Motor5]);
            
            if (init_flag_1 && init_flag_2 && init_flag_4 && init_flag_5) {
                HAL_Delay(100);
                
                // Lock current positions
                motor[Motor1].ctrl.pos_set = motor[Motor1].para.pos;
                motor[Motor2].ctrl.pos_set = motor[Motor2].para.pos;
                motor[Motor4].ctrl.pos_set = motor[Motor4].para.pos;
                motor[Motor5].ctrl.pos_set = motor[Motor5].para.pos;
                
                // Start high-speed interrupt engines
                HAL_TIM_Base_Start_IT(&htim4);
                HAL_TIM_Base_Start_IT(&htim3);
                
                system_state = STATE_WAIT_HOST_CONNECT;
            }
            HAL_Delay(500);
            break;

        case STATE_WAIT_HOST_CONNECT:
            // [Implicit Handshake 1: Detect Heartbeat]
            // No more serial commands needed! As long as the USB RX watchdog timestamp is updated,
            // and the delta is less than 50ms, it means ROS2 is connected and spamming data at 100Hz!
            if (last_usb_rx_time != 0 && (HAL_GetTick() - last_usb_rx_time < 50)) {
                system_state = STATE_WAIT_ROS_MOVE;
            }
            break;

        case STATE_WAIT_ROS_MOVE:
            // [Implicit Handshake 2: Feature Value Trigger]
            // We agreed with the future ROS2 node: when the upper arm slowly moves to the drop position,
            // ROS2 intentionally sets the mode byte for the 1st motor to 0x10 (normally only 0, 1, 2).
            // Once the MCU sees 0x10, it understands immediately: ROS2 says it's in position!
            if (Current_Target_Cmd.modes[0] == 0x10) {
                state_timer = HAL_GetTick();
                system_state = STATE_WAIT_SETTLE;
            }
            break;

        case STATE_WAIT_SETTLE:
            // Received in-position signal, hard wait for 2 seconds to let the forearm settle against the limit via gravity
            if (HAL_GetTick() - state_timer > 2000) {
                system_state = STATE_CALIBRATE_ZERO;
            }
            break;

        case STATE_CALIBRATE_ZERO:
            // Physical zeroing
            cm_motor_set_absolute_zero(&hfdcan2, &cm_motor[0], 0);
            HAL_Delay(500); 
            
//            cm_motor[0].ctrl.pos_set = 0.0f;
            init_flag_3 = true; 
            
            // Removed the original send_system_event_to_ros, directly enter takeover state!
            system_state = STATE_NORMAL_RUN;
            break;

        case STATE_NORMAL_RUN:
            // Now it's ROS2's world, the MCU main loop just idles and rests
            break;
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
#ifdef USE_FULL_ASSERT
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
