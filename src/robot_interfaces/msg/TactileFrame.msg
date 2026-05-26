std_msgs/Header header
float32[512] data          # 扁平化 32×16，行优先
uint16 rows=32
uint16 cols=16
uint16 active_count        # 超过阈值的激活点数
float32 peak_value         # 全局峰值
float32 fps                # 实际帧率
bool calibrated            # 是否已完成基线校准