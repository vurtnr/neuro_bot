#!/bin/bash
# 颜色设置
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN}🚀 NeuroBot 快速启动模式 (不重新编译)...${NC}"
cd ~/neuro_bot_ws

# 1. 刷新环境
source install/setup.bash

# 2. (可选) 双重保险：再次注入库路径，防止 /usr/lib 没生效
# 虽然我们复制了文件到系统目录，但多加这一行环境变量没有任何坏处
SO_FILE=$(find install/robot_interfaces -name "librobot_interfaces__rosidl_typesupport_introspection_c.so" | head -n 1)
if [ -n "$SO_FILE" ]; then
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(dirname "$SO_FILE")
fi

# 3. 启动
echo -e "${GREEN}==============================================${NC}"
echo -e "${GREEN}正在启动 NeuroBot 大脑与四肢...${NC}"
echo -e "${GREEN}==============================================${NC}"
ros2 launch neuro_bot_bringup all_systems.launch.py