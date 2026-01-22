# 识别到的对象类型 (例如: "qrcode", "face")
string type

# 识别到的具体内容
# 对于 neural_link，这里是一个 JSON 字符串，例如：
# {"t":"ble", "m":"xx:xx", "s":"...", "c":"...", "d":"CMD_HEX"}
string content

# 置信度或距离 (可选)
float32 distance