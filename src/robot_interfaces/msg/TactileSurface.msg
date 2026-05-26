std_msgs/Header header
float32[128] front_data    # 8×16 胸前穹顶，扁平化
float32[128] back_data     # 8×16 后背穹顶，扁平化
uint8 front_active         # 胸前激活格点数
uint8 back_active          # 后背激活格点数
float32 front_peak         # 胸前峰值
float32 back_peak          # 后背峰值
float32 front_total        # 胸前总压力
float32 back_total         # 后背总压力