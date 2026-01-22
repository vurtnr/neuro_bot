# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

**NeuroBot** - 基于 ROS 2 的智能机器人，采用 Rust + Python 混合架构：
- **Rust**: 核心决策 (brain_core)、硬件控制 (iot_controller 蓝牙、face_bridge 串口)
- **Python**: 视觉感知、语音处理 (ASR/TTS)、LLM 服务集成

## 常用命令

```bash
# 编译 (ROS 2 工作空间)
cd ~/neuro_bot_ws
colcon build
colcon build --packages-select brain_core iot_controller vision_engine  # 增量编译

# 一键启动
./start.sh      # 带编译
./run.sh        # 快速启动

# 分模块启动 (调试)
ros2 launch vision_system.launch.py          # 摄像头驱动
ros2 run vision_engine qr_node               # 视觉识别
ros2 run iot_controller iot_controller       # 蓝牙控制
ros2 run audio_engine audio_node             # 语音引擎
ros2 run brain_core brain_core               # 决策大脑

# 调试
ros2 topic list                              # 查看话题
ros2 topic echo /iot/bluetooth_command       # 监听蓝牙指令
python3 ~/neuro_bot_ws/ble_debug.py          # 蓝牙扫描
```

## 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│  决策控制层 (Rust + r2r)                                     │
│  brain_core: 状态机 | LLM交互 | 多模态协调                    │
└─────────────────────────────────────────────────────────────┘
                              │
                    ROS 2 Topic/Service
                              │
┌──────────────────┬──────────────────┬──────────────────────┐
│  视觉引擎 (Python) │  语音引擎 (Python) │  LLM引擎 (Python)     │
│  vision_engine   │  audio_engine    │  llm_engine          │
└──────────────────┴──────────────────┴──────────────────────┘
                              │
                    ROS 2 Topic/Service
                              │
┌─────────────────────────────────────────────────────────────┐
│  硬件执行层 (Rust)                                           │
│  iot_controller: 蓝牙BLE  |  face_bridge: 串口屏幕           │
└─────────────────────────────────────────────────────────────┘
```

## 关键 ROS 2 接口

| 类型 | 名称 | 用途 |
|------|------|------|
| Topic | `/vision/result` | 视觉识别结果 |
| Topic | `/audio/speech` | 语音识别文本 |
| Topic | `/iot/bluetooth_command` | 蓝牙指令 |
| Service | `/llm/ask` | LLM 问答 |
| Service | `/iot/connect` | 蓝牙连接 |

## 环境变量

```bash
export DEEPSEEK_API_KEY='...'     # LLM API
export VOLC_APPID='...'           # 火山引擎 ASR/TTS
export VOLC_TOKEN='...'
```
