#!/bin/bash
# 设置颜色
GREEN='\033[0;32m'
CYAN='\033[0;36m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}🏗️  NeuroBot 全量编译与启动脚本...${NC}"
WORK_DIR=~/neuro_bot_ws
cd $WORK_DIR

# 0. 清理旧缓存 (开发阶段为了稳妥，通常会清理)
if [ -d "build/r2r" ]; then
    echo -e "${YELLOW}[Step 0/5] 清理旧的构建缓存 (r2r)...${NC}"
    rm -rf build/r2r install/r2r
fi

# 1. 核心动作: 优先编译 robot_interfaces
echo -e "${CYAN}[Step 1/5] 正在编译通信协议 (robot_interfaces)...${NC}"
colcon build --packages-select robot_interfaces
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ 接口编译失败！系统终止。${NC}"
    exit 1
fi

# 2. [关键升级] 自动同步系统库 (防止 Rust 找不到库)
echo -e "${CYAN}[Step 2/5] 正在同步库文件到系统目录...${NC}"
SO_FILE=$(find install/robot_interfaces -name "librobot_interfaces__rosidl_typesupport_introspection_c.so" | head -n 1)
if [ -n "$SO_FILE" ]; then
    SO_DIR=$(dirname "$SO_FILE")
    echo -e "${GREEN}✅ 检测到新生成的库，正在注入 /usr/lib (需要密码)...${NC}"
    sudo cp "$SO_DIR"/*.so /usr/lib/
    sudo ldconfig
else
    echo -e "${RED}❌ 严重警告：未找到生成的 .so 文件！后续编译可能会失败。${NC}"
fi

# 3. 刷新环境
echo -e "${CYAN}[Step 3/5] 刷新环境变量...${NC}"
source install/setup.bash

# 4. 编译其余模块 (现在不需要 LD_LIBRARY_PATH 补丁了，因为 Step 2 解决了根源)
echo -e "${CYAN}[Step 4/5] 正在编译功能模块 (R2R, Engine & Core)...${NC}"
colcon build --packages-ignore robot_interfaces
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ 模块编译失败！系统终止。${NC}"
    exit 1
fi

# 5. 一键启动
echo -e "${GREEN}[Step 5/5] 所有模块准备就绪，正在启动 NeuroBot...${NC}"
source install/setup.bash
ros2 launch neuro_bot_bringup all_systems.launch.py