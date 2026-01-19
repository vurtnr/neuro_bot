

# 🤖 NeuroBot 开发环境启动指南

本文档记录了 NeuroBot 在开发阶段的启动流程。

## 🛠️ 0. 编译构建 (Build)

在运行任何节点之前，请确保已编译最新代码。

```bash
cd ~/neuro_bot_ws

# 编译所有包
colcon build

# 或者：仅编译特定修改过的包 (推荐，速度快)
colcon build --packages-select brain_core iot_controller vision_engine

```

---

## 👨‍💻 模式一：分模块启动 (调试推荐)

在开发过程中，建议打开多个终端窗口，分别启动各个模块，以便实时观察各模块的日志输出。

**⚠️ 注意：** 每个新打开的终端都需要先加载工作空间环境：

```bash
source ~/neuro_bot_ws/install/setup.bash

```

### 1. 启动“眼睛” (硬件驱动)

负责拉起摄像头驱动，发布图像话题。

```bash
# Terminal 1
ros2 launch vision_system.launch.py

```

> **检查点**：启动后无报错，且能在浏览器/Rviz 中看到 `/camera_driver/image_raw` 话题。

### 2. 启动“视觉神经” (感知层)

负责识别二维码 (QR Code) 和人脸。

```bash
# Terminal 2
ros2 run vision_engine qr_node

```

> **检查点**：展示二维码时，终端应打印 `🔍 Found QR Code: ...`

### 3. 启动“四肢” (控制层)

负责蓝牙连接与电机控制 (Rust)。

```bash
# Terminal 3
ros2 run iot_controller iot_controller

```

> **检查点**：启动后应显示 `🔗 Bluetooth Service & Command Link Ready`。

### 4. 启动“嘴巴” (音频层)

负责语音识别 (STT) 和 语音合成 (TTS)。

```bash
# Terminal 4
ros2 run audio_engine audio_node

```

### 5. 启动“大脑” (决策层)

负责状态管理、LLM 交互及多模态协调 (Rust)。

```bash
# Terminal 5
ros2 run brain_core brain_core

```

> **检查点**：启动后应显示 `✅ Brain Audio Loop Started` 和 `✅ Brain Vision Loop Started`。

---

## 🚀 模式二：一键启动 (集成测试)

如果已经完成了 `neuro_bot_bringup` 包的配置，可以使用一条命令拉起所有节点。

```bash
ros2 launch neuro_bot_bringup all_systems.launch.py

```

---

## 🧰 常用调试工具

### 1. 蓝牙扫描工具

用于获取蓝牙设备的 Service UUID 和 Characteristic UUID。

```bash
python3 ~/neuro_bot_ws/ble_debug.py

```

### 2. 查看话题列表

确认节点之间是否“连通”。

```bash
ros2 topic list

```

### 3. 监听交互指令

查看大脑是否向控制器发送了指令。

```bash
ros2 topic echo /iot/bluetooth_command

```
# === NeuroBot 接口更新专用脚本 ===

# 1. 重新编译接口包
cd ~/neuro_bot_ws
colcon build --packages-select robot_interfaces

# 2. 【关键】把新生成的库再次覆盖到系统目录
# 这一步是防止系统还在用旧版本
echo "🔄 正在更新系统库文件..."
NEW_SO_DIR=$(find ~/neuro_bot_ws/install/robot_interfaces -name "librobot_interfaces__rosidl_typesupport_introspection_c.so" | head -n 1 | xargs dirname)

if [ -n "$NEW_SO_DIR" ]; then
    sudo cp "$NEW_SO_DIR"/*.so /usr/lib/
    sudo ldconfig
    echo "✅ 系统库已更新到最新版本！"
else
    echo "❌ 编译可能失败了，没找到新文件。"
fi

# 3. 然后再去编译 r2r 或其他业务模块
colcon build --symlink-install --packages-select r2r ...