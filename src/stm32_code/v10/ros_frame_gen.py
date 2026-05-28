import struct

def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1; crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def generate_ros_frame(modes, motors):
    payload = struct.pack('<6B', *modes)
    for i in range(6):
        payload += struct.pack('<5f', motors[i][0], motors[i][1], motors[i][2], motors[i][3], motors[i][4])
    crc16 = calculate_crc16(payload)
    full_frame = struct.pack('<H', 0xAA55) + payload + struct.pack('<H', crc16) + struct.pack('<H', 0x0D0A)
    return " ".join([f"{b:02X}" for b in full_frame])

# 1. 绝对安全心跳帧 (模式全为 0xFF，单片机会原地锁死)
modes_hb = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
motors_hb = [[0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(6)]
print("【安全心跳帧】:\n" + generate_ros_frame(modes_hb, motors_hb) + "\n")

# 2. 触发帧 (大臂模式 0x10，单片机也会原地锁死，并触发寻零)
modes_trigger = [0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
print("【触发暗号帧】:\n" + generate_ros_frame(modes_trigger, motors_hb) + "\n")