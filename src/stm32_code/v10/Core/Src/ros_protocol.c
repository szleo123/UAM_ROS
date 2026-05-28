#include "ros_protocol.h"
#include "string.h"
#include "main.h"
#include "dm_motor_ctrl.h"
#include "cubemars_motor_ctrl.h"
#include "usbd_cdc_if.h"  // 加入这一行以调用 CDC_Transmit_HS

// 1. The "Inbox": USB interrupt writes directly to this byte array
RosRxUnion_t USB_Rx_Buffer;

// 2. The "Execution Vault": TIM3 interrupt reads from this verified struct
RosRxPacket_t Current_Target_Cmd;

// 3. Watchdog timestamp (Updated in USB Rx, checked in TIM3)
uint32_t last_usb_rx_time = 0;

// 4. Global system state
SystemState_e current_sys_state = SYS_WAIT_CONNECT;



/* =====================================================================
 * Standard CRC16-Modbus Algorithm (Cross-platform friendly)
 * ===================================================================== */
uint16_t Calculate_CRC16(uint8_t *data, uint16_t length) 
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* =====================================================================
 * Robust Stream Parser for USB CDC (Handles fragmentation & concatenation)
 * ===================================================================== */
#define RX_PACKET_SIZE sizeof(RosRxPacket_t) // Should be 132 bytes

// Static ring-like buffer for raw stream parsing
static uint8_t stream_buf[256];
static uint16_t stream_len = 0;

void Protocol_Parse_Stream(uint8_t *data, uint32_t len) 
{
    // 1. Append incoming data to stream buffer
    for (uint32_t i = 0; i < len; i++) {
        if (stream_len < sizeof(stream_buf)) {
            stream_buf[stream_len++] = data[i];
        } else {
            // Buffer overflow, reset to prevent crash
            stream_len = 0; 
        }
    }

    // 2. State machine to find valid packets
    while (stream_len >= RX_PACKET_SIZE) {
        
        // 2.1 Check Header (Little Endian: 0xAA55 -> bytes[0]=0x55, bytes[1]=0xAA)
        uint16_t current_header = (stream_buf[1] << 8) | stream_buf[0];
        
        if (current_header == PKT_HDR) {
            
            // 2.2 Check Tail
            uint16_t current_tail = (stream_buf[RX_PACKET_SIZE - 1] << 8) | stream_buf[RX_PACKET_SIZE - 2];
            
            if (current_tail == PKT_TAIL) {
                
                // 2.3 Calculate and Verify CRC16 
                // We calculate CRC over 'modes' and 'motors' (Skip Header 2 bytes)
                // Payload length = Modes(6) + Motors(120) = 126 bytes
                uint16_t payload_length = RX_PACKET_SIZE - 6; 
                uint16_t calc_crc = Calculate_CRC16(&stream_buf[2], payload_length);
                
                uint16_t recv_crc = (stream_buf[RX_PACKET_SIZE - 3] << 8) | stream_buf[RX_PACKET_SIZE - 4];
                
                if (calc_crc == recv_crc) {
                    // ==========================================
                    // PACKET VALID! Copy to Global Double-Buffer
                    // ==========================================
                    memcpy(USB_Rx_Buffer.bytes, stream_buf, RX_PACKET_SIZE);
                    
                    // Feed the Watchdog
                    last_usb_rx_time = HAL_GetTick();
                    
                    // Remove the processed packet from stream
                    stream_len -= RX_PACKET_SIZE;
                    memmove(stream_buf, &stream_buf[RX_PACKET_SIZE], stream_len);
                    continue; // Check if there is another packet immediately behind
                }
            }
        }
        
        // Invalid Header, Tail, or CRC: Shift buffer by 1 byte to re-align
        stream_len--;
        memmove(stream_buf, &stream_buf[1], stream_len);
    }
}

/* =====================================================================
 * Control Engine Router (Called at 100Hz by TIM3)
 * Handles double-buffering, watchdog timeouts, and control mode mapping.
 * ===================================================================== */

// [Crucial] Preserve the CubeMars physical zero offset
#define FOREARM_OFFSET_RAD (0.0f) // Adjust if needed

// Define Safe Fallback Parameters (Impedance for holding position)
#define DM_SAFE_KP  50.0f
#define DM_SAFE_KD  0.6f
#define CM_SAFE_KP  50.0f
#define CM_SAFE_KD  0.6f

// External reference to DJI target position (from main.c)
extern int32_t target_position;

void Protocol_Execute_Router(void)
{
    // [Core Fix] Edge detection: records if the previous state was a timeout.
    // Initially set to true. This prevents position capture right after boot, 
    // perfectly inheriting the locked position from while(1)!
    static bool was_timeout = true; 

    // 1. Watchdog Check (30ms timeout = 3 missed frames at 100Hz)
    uint32_t time_since_last_rx = HAL_GetTick() - last_usb_rx_time;
    bool is_timeout = (time_since_last_rx > 30);

    if (is_timeout) {
        current_sys_state = SYS_SAFE_DROP;
    } else {
        current_sys_state = SYS_RUNNING;
        // Safely copy the latest verified packet from USB interrupt
        __disable_irq();
        Current_Target_Cmd = USB_Rx_Buffer.pkt;
        __enable_irq();
    }

    // 2. Route Data for DM Motors (ROS Indices: 0, 1, 3, 4)
    motor_t* dm_ptrs[4] = {&motor[Motor1], &motor[Motor2], &motor[Motor4], &motor[Motor5]};
    uint8_t  ros_idx[4] = {0, 1, 3, 4};

    for (int i = 0; i < 4; i++) {
        motor_t* m = dm_ptrs[i];
        uint8_t idx = ros_idx[i];
        MitCmd_t* cmd = &Current_Target_Cmd.motors[idx];
        uint8_t mode = Current_Target_Cmd.modes[idx];

        if (is_timeout) {
            // [Core Fix] Capture the physical position ONLY at the exact moment of disconnection!
            if (!was_timeout) {
                m->ctrl.pos_set = m->para.pos; 
            }
            
            // Continuously inject safe damping/stiffness, NEVER modify the position!
            m->ctrl.vel_set = 0.0f;
            m->ctrl.kp_set  = DM_SAFE_KP;
            m->ctrl.kd_set  = DM_SAFE_KD;
            m->ctrl.tor_set = 0.0f;
            
        } else if (mode == MODE_POS_ONLY) {
            m->ctrl.pos_set = cmd->p_des;
            m->ctrl.vel_set = 0.0f;
            m->ctrl.kp_set  = DM_SAFE_KP;
            m->ctrl.kd_set  = DM_SAFE_KD;
            m->ctrl.tor_set = 0.0f;
        } else if (mode == MODE_POS_TORQUE) {
            m->ctrl.pos_set = cmd->p_des;
            m->ctrl.vel_set = 0.0f;
            m->ctrl.kp_set  = DM_SAFE_KP;
            m->ctrl.kd_set  = DM_SAFE_KD;
            m->ctrl.tor_set = cmd->tor_ff; 
        } else if (mode == MODE_FULL_MIT) {
            m->ctrl.pos_set = cmd->p_des;
            m->ctrl.vel_set = cmd->v_des;
            m->ctrl.kp_set  = cmd->kp;    
            m->ctrl.kd_set  = cmd->kd;
            m->ctrl.tor_set = cmd->tor_ff;
        } else {
            m->ctrl.pos_set = m->para.pos;
            m->ctrl.vel_set = 0.0f;
            m->ctrl.kp_set  = DM_SAFE_KP;
            m->ctrl.kd_set  = DM_SAFE_KD;
            m->ctrl.tor_set = 0.0f;
        }
    }

   // 3. Route Data for CubeMars (ROS Index: 2)
    MitCmd_t* cm_cmd = &Current_Target_Cmd.motors[2];
    uint8_t cm_mode = Current_Target_Cmd.modes[2];

    if (is_timeout) {
        // [Core Fix] Similarly, lock the forearm angle once at the moment of disconnection
        if (!was_timeout) {
            cm_motor[0].ctrl.pos_set = cm_motor[0].fb.pos * (3.1415926f / 180.0f); 
        }
        cm_motor[0].ctrl.vel_set = 0.0f;
        cm_motor[0].ctrl.kp_set  = CM_SAFE_KP;
        cm_motor[0].ctrl.kd_set  = CM_SAFE_KD;
        cm_motor[0].ctrl.tor_set = 0.0f;
    } else {
        // Apply Kinematic Offset
        float target_rad = cm_cmd->p_des - FOREARM_OFFSET_RAD;

        if (cm_mode == MODE_POS_ONLY) {
            cm_motor[0].ctrl.pos_set = target_rad;
            cm_motor[0].ctrl.vel_set = 0.0f;
            cm_motor[0].ctrl.kp_set  = CM_SAFE_KP;
            cm_motor[0].ctrl.kd_set  = CM_SAFE_KD;
            cm_motor[0].ctrl.tor_set = 0.0f;
        } else if (cm_mode == MODE_POS_TORQUE) {
            cm_motor[0].ctrl.pos_set = target_rad;
            cm_motor[0].ctrl.vel_set = 0.0f;
            cm_motor[0].ctrl.kp_set  = CM_SAFE_KP;
            cm_motor[0].ctrl.kd_set  = CM_SAFE_KD;
            cm_motor[0].ctrl.tor_set = cm_cmd->tor_ff;
        } else if (cm_mode == MODE_FULL_MIT) {
            cm_motor[0].ctrl.pos_set = target_rad;
            cm_motor[0].ctrl.vel_set = cm_cmd->v_des;
            cm_motor[0].ctrl.kp_set  = cm_cmd->kp;
            cm_motor[0].ctrl.kd_set  = cm_cmd->kd;
            cm_motor[0].ctrl.tor_set = cm_cmd->tor_ff;
        } else {
            // [Crucial Fallback]: Unknown mode (e.g., 0xFF or 0x10), lock in place safely!
            // Must convert physical degrees to target radians with offset to avoid jerking.
            cm_motor[0].ctrl.pos_set = cm_motor[0].fb.pos * (3.1415926f / 180.0f);
            cm_motor[0].ctrl.vel_set = 0.0f;
            cm_motor[0].ctrl.kp_set  = CM_SAFE_KP;
            cm_motor[0].ctrl.kd_set  = CM_SAFE_KD;
            cm_motor[0].ctrl.tor_set = 0.0f;
        }
    }

    // 4. Route Data for DJI 2006 (ROS Index: 5)
    if (!is_timeout) {
        float dji_rad = Current_Target_Cmd.motors[5].p_des;
        target_position = (int32_t)((dji_rad / 3.1415926f) * 8191.0f * 36.0f);
    }
    
    // [Core Fix] Update the state at the end of the function 
    // for the edge detection in the next 10ms cycle!
    was_timeout = is_timeout;
}

/* =====================================================================
 * Feedback Engine (STM32 -> ROS2)
 * Gathers real physics states, packs them, and transmits via USB.
 * ===================================================================== */
RosTxUnion_t USB_Tx_Buffer;

void Protocol_Send_Feedback(void)
{
    // 1. Pack Header and Global State
    USB_Tx_Buffer.pkt.header = PKT_HDR;
    USB_Tx_Buffer.pkt.sys_state = current_sys_state;
    USB_Tx_Buffer.pkt.tail = PKT_TAIL;

    // 2. Pack DM Motors (ROS Indices: 0, 1, 3, 4)
    motor_t* dm_ptrs[4] = {&motor[Motor1], &motor[Motor2], &motor[Motor4], &motor[Motor5]};
    uint8_t  ros_idx[4] = {0, 1, 3, 4};

    for (int i = 0; i < 4; i++) {
        uint8_t idx = ros_idx[i];
        // Read real physics state (Ensure 'para' is the correct feedback struct in your dm_motor_drv)
        USB_Tx_Buffer.pkt.motors[idx].pos = dm_ptrs[i]->para.pos;
        USB_Tx_Buffer.pkt.motors[idx].vel = dm_ptrs[i]->para.vel; 
        USB_Tx_Buffer.pkt.motors[idx].tor = dm_ptrs[i]->para.tor; 
    }

    // 3. Pack CubeMars (ROS Index: 2)
    // CRITICAL: Add the Forearm Offset back so ROS2 gets URDF-aligned Radian coordinates!
    USB_Tx_Buffer.pkt.motors[2].pos = cm_motor[0].fb.pos * (3.1415926f / 180.0f) + FOREARM_OFFSET_RAD; 
    USB_Tx_Buffer.pkt.motors[2].vel = cm_motor[0].fb.vel; // Convert to rad/s if needed
    USB_Tx_Buffer.pkt.motors[2].tor = cm_motor[0].fb.cur; // CubeMars fb uses current, mapped to tor

    // 4. Pack DJI 2006 (ROS Index: 5)
    // Convert Ticks to Radians
    float dji_rad = ((float)moto_dji2006.total_angle / (8191.0f * 36.0f)) * (2.0f * 3.1415926f);
    // Convert RPM to Rad/s
    float dji_rad_s = (float)moto_dji2006.speed_rpm * (2.0f * 3.1415926f / 60.0f);
    
    USB_Tx_Buffer.pkt.motors[5].pos = dji_rad;
    USB_Tx_Buffer.pkt.motors[5].vel = dji_rad_s;
    USB_Tx_Buffer.pkt.motors[5].tor = (float)moto_dji2006.given_current; 

    // 5. Calculate CRC16 (Calculate over sys_state and motors)
    uint16_t payload_length = sizeof(RosTxPacket_t) - 6; // Total(79) - Head(2) - CRC(2) - Tail(2) = 73 bytes
    USB_Tx_Buffer.pkt.crc16 = Calculate_CRC16(&USB_Tx_Buffer.bytes[2], payload_length);

    // 6. Transmit to ROS2 via USB High-Speed PHY
    CDC_Transmit_HS(USB_Tx_Buffer.bytes, sizeof(RosTxPacket_t));
}

