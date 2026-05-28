#ifndef _ROS_PROTOCOL_H_
#define _ROS_PROTOCOL_H_

#include "stdint.h"

// Define protocol constants
#define PKT_HDR      0xAA55
#define PKT_TAIL     0x0D0A
#define MOTOR_NUM    6

// Define control modes for STM32 router
typedef enum {
    MODE_POS_ONLY   = 0, // Fallback/Init: STM32 uses internal Kp/Kd, overrides V and Tor
    MODE_POS_TORQUE = 1, // Gravity Comp: STM32 uses P and Tor_ff, internal Kp/Kd
    MODE_FULL_MIT   = 2  // Advanced: STM32 trusts all P, V, Kp, Kd, Tor_ff from ROS2
} MotorControlMode_e;

// Define system states
typedef enum {
    SYS_WAIT_CONNECT = 0, // Waiting for ROS2 first frame
    SYS_RUNNING      = 1, // Normal operation
    SYS_SAFE_DROP    = 2  // Watchdog timeout, safe fallback mode
} SystemState_e;

/* =====================================================================
 * CANCEL MEMORY PADDING: CRITICAL FOR CROSS-PLATFORM BYTE ALIGNMENT
 * ===================================================================== */
#pragma pack(push, 1)

// ---------------------------------------------------------
// RX: ROS2 -> STM32 (Target Commands)
// ---------------------------------------------------------
typedef struct {
    float p_des;
    float v_des;
    float kp;
    float kd;
    float tor_ff;
} MitCmd_t; // 5 floats * 4 bytes = 20 bytes per motor

typedef struct {
    uint16_t header;                // 2 bytes (0xAA55)
    uint8_t  modes[MOTOR_NUM];      // 6 bytes (Control mode for each motor)
    MitCmd_t motors[MOTOR_NUM];     // 120 bytes (6 motors * 20 bytes)
    uint16_t crc16;                 // 2 bytes (Checksum or CRC)
    uint16_t tail;                  // 2 bytes (0x0D0A)
} RosRxPacket_t;                    // Total: 132 bytes

// Union for easy USB byte reception and struct parsing
typedef union {
    RosRxPacket_t pkt;
    uint8_t bytes[sizeof(RosRxPacket_t)];
} RosRxUnion_t;

// ---------------------------------------------------------
// TX: STM32 -> ROS2 (Real-time Feedback)
// ---------------------------------------------------------
typedef struct {
    float pos;
    float vel;
    float tor; // Real current converted to Torque
} MotorState_t; // 12 bytes per motor

typedef struct {
    uint16_t header;                // 2 bytes
    uint8_t  sys_state;             // 1 byte (SystemState_e)
    MotorState_t motors[MOTOR_NUM]; // 72 bytes (6 motors * 12 bytes)
    uint16_t crc16;                 // 2 bytes
    uint16_t tail;                  // 2 bytes
} RosTxPacket_t;                    // Total: 79 bytes

typedef union {
    RosTxPacket_t pkt;
    uint8_t bytes[sizeof(RosTxPacket_t)];
} RosTxUnion_t;

#pragma pack(pop)
/* =====================================================================
 * RESTORE MEMORY PADDING
 * ===================================================================== */

// Global variables for double-buffering and watchdog
extern RosRxUnion_t USB_Rx_Buffer;
extern RosRxPacket_t Current_Target_Cmd;
extern uint32_t last_usb_rx_time;
extern SystemState_e current_sys_state;

// Protocol function prototypes
void Protocol_Parse_Stream(uint8_t *data, uint32_t len);

void Protocol_Execute_Router(void);

void Protocol_Send_Feedback(void);

#endif // _ROS_PROTOCOL_H_