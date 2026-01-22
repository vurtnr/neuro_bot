import json
import qrcode
import struct

# ---------------------------------------------------------
# 1. 协议定义 (极简模式)
# ---------------------------------------------------------
# 原格式: {"t": "ble", "mac": "D6:65:...", "cmd": "0A03..."} (太长，密度大)
# 新格式: {"t": "b",   "m": "D665...",     "c": "0A03..."}   (短小，好识别)
# ---------------------------------------------------------

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
    return crc

def generate_tcu_command(tcu_address, cmd_type=0x03):
    # 构造 Modbus 指令: Address + Cmd + ... + CRC
    payload = bytearray([tcu_address, cmd_type, 0x00, 0x00, 0x00, 0x25])
    crc = calc_crc16_modbus(payload)
    payload.append(crc & 0xFF)
    payload.append((crc >> 8) & 0xFF)
    return payload.hex().upper()

# --- 配置参数 ---
TCU_ADDRESS = 10 
MAC_ADDRESS = "D6:65:62:00:2A:7E" # 请替换为您实际的 MAC

# 1. 生成指令
cmd_hex = generate_tcu_command(TCU_ADDRESS)

# 2. 组装极简数据包
# 去掉冒号，节省 5 个字符
mac_clean = MAC_ADDRESS.replace(":", "") 

minified_data = {
    "t": "b",        # b 代表 ble
    "m": mac_clean,  # m 代表 mac
    "c": cmd_hex     # c 代表 cmd
}

json_str = json.dumps(minified_data, separators=(',', ':')) # 去掉空格，极致压缩
print(f"原始数据: {json_str}")

# 3. 生成二维码
qr = qrcode.QRCode(
    version=None, # 自动选择最小版本
    error_correction=qrcode.constants.ERROR_CORRECT_L, # 使用最低容错 (L)，进一步减少黑块数量，适合清晰环境
    box_size=10,
    border=4,
)
qr.add_data(json_str)
qr.make(fit=True)

img = qr.make_image(fill_color="black", back_color="white")
img.save("robot_qr_lite.png")
print("✅ 已生成 robot_qr_lite.png，请扫描此图！")