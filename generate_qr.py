import struct

def calc_crc16_modbus(data: bytearray) -> int:
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    # 注意：Modbus 协议通常是低字节在前 (Low Byte First)
    # 但在 JSON hex 字符串中，我们通常按顺序写。
    # 根据你的文档 "CRC-16 低前高后"，意味着最后两个字节，先发低位，再发高位。
    return crc

def generate_tcu_command(tcu_address, cmd_type=0x03):
    # 构造基础帧: Address + Command + Reserved(3 bytes) + Length
    # 示例: 0A 03 00 00 00 25
    payload = bytearray([tcu_address, cmd_type, 0x00, 0x00, 0x00, 0x25])
    
    # 计算 CRC
    crc = calc_crc16_modbus(payload)
    
    # 拼接 CRC (低位在前)
    payload.append(crc & 0xFF)        # Low
    payload.append((crc >> 8) & 0xFF) # High
    
    return payload.hex().upper()

# --- 配置你的设备参数 ---
TCU_ADDRESS = 10  # 0x0A
MAC_ADDRESS = "D6:65:62:A0:AD:E5" # 替换为真实 MAC
SERVICE_UUID = "0000FFF0-0000-1000-8000-00805F9B34FB" # 替换为真实 UUID
CHAR_UUID    = "0000FFF2-0000-1000-8000-00805F9B34FB" # 替换为真实 UUID

# 生成指令
hex_cmd = generate_tcu_command(TCU_ADDRESS)

# 生成 JSON
import json
qr_content = {
    "t": "ble",
    "m": MAC_ADDRESS,
    "s": SERVICE_UUID,
    "c": CHAR_UUID,
    "d": hex_cmd, # 自动带上了 CRC
    "n": "TCU_Device_10"
}

print("✅ 请使用以下内容生成二维码:")
print(json.dumps(qr_content, separators=(',', ':')))
print(f"\n(生成的 Hex 指令为: {hex_cmd})")