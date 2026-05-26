std_msgs/Header header
string surface             # "chest" | "back" | "none"
string region_v            # "Upper" | "Middle" | "Lower" | "--"
string region_h            # "Left" | "Center" | "Right" | "--"
float32 peak_value         # 触发区域的峰值
float32 total_force        # 总压力
bool is_touch              # 是否构成有效触摸（>threshold）
string label               # 人类可读标签，如 "Chest / Middle / Right"