import json
import qrcode

# --- 配置参数 ---
MAC_ADDRESS = "D6:65:62:A0:AD:E5"  # 请替换为您实际的 MAC

# 仅生成 MAC 的极简二维码
mac_clean = MAC_ADDRESS.replace(":", "")
minified_data = {
    "t": "b",
    "m": mac_clean,
}

json_str = json.dumps(minified_data, separators=(",", ":"))
print(f"原始数据: {json_str}")

qr = qrcode.QRCode(
    version=None,
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=4,
)
qr.add_data(json_str)
qr.make(fit=True)

img = qr.make_image(fill_color="black", back_color="white")
img.save("robot_qr_lite.png")
print("✅ 已生成 robot_qr_lite.png，请扫描此图！")
