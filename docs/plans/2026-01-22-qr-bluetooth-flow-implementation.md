# 二维码扫描 → 蓝牙连接流程实现计划

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**目标:** 实现 vision_engine 识别二维码 → brain_core 状态机管理 → iot_controller 蓝牙连接 → face_bridge 表情切换 + audio_engine 语音播报的完整流程。

**架构:** Rust (brain_core, iot_controller, face_bridge) + Python (vision_engine, audio_engine) 混合架构，通过 ROS 2 Topic/Service 通信。

**技术栈:** ROS 2 Jazzy, Rust r2r, Python rclpy, 蓝牙 BLE

---

## 前置条件

在 Linux 环境（ROS 2）下执行：
```bash
cd ~/neuro_bot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_interfaces brain_core iot_controller face_bridge
```

---

## Task 1: 创建设计文档

**Files:**
- Create: `docs/plans/2026-01-22-qr-bluetooth-flow-design.md`

**Step 1: Write design document**

```markdown
# 二维码扫描 → 蓝牙连接流程设计

## 数据流

```
摄像头 → qr_node (识别) → /vision/result → brain_core (状态机)
                                                      │
                       ┌──────────────────────────────┼──────────────────────────────┐
                       ▼                              ▼                              ▼
                /audio/tts_play                  /robot/face_emotion         /iot/connect
                (语音播报)                       (表情切换)                    (蓝牙连接)
```

## 二维码格式 (极简协议)

```json
{"t":"b","m":"D66562002A7E","c":"0A03000000258490"}
```

- `t`: "b" (BLE)
- `m`: MAC 地址（去掉冒号的 12 位十六进制）
- `c`: Modbus 指令 hex

## brain_core 状态机

```
                    ┌──────────────────────────────────────┐
                    │           收到 VisionResult           │
                    └──────────────────┬───────────────────┘
                                       ▼
                              ┌────────────────┐
                              │  SCANNING      │ ───▶ 表情: BUSY
                              │  (扫描连接中)   │       语音: "已识别出二维码中的 MAC 地址，正在连接蓝牙设备"
                              └───────┬────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    ▼                 │                 ▼
            蓝牙连接成功          超时/错误        设备不存在
                    │                 │                 │
                    ▼                 ▼                 ▼
           ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
           │ SENDING_CMD  │   │    IDLE      │   │    IDLE      │
           │ (下发指令中)  │   │   (空闲)     │   │   (空闲)     │
           └──────┬───────┘   └──────────────┘   └──────────────┘
                  │
                  ▼
           表情: HAPPY
           语音: "蓝牙设备已连接，并下发查询指令"
                  │
                  ▼
           发送指令到 iot_controller
                  │
                  ▼
           ┌──────────────┐
           │    IDLE      │ ◀─────────────────────┐
           │   (空闲)     │                       │
           └──────────────┘                       │
                                          指令发送失败/超时
```

## 语音播报内容

| 阶段 | Topic | 播报内容 |
|------|-------|---------|
| 进入 SCANNING | `/audio/tts_play` | "已识别出二维码中的 MAC 地址，正在连接蓝牙设备" |
| 进入 SENDING_CMD | `/audio/tts_play` | "蓝牙设备已连接，并下发查询指令" |
| 连接/指令失败 | `/audio/tts_play` | "蓝牙连接失败，请重试" |

## 表情切换

| 状态 | Topic | 表情指令 |
|------|-------|---------|
| SCANNING | `/robot/face_emotion` | `{"emotion":"busy"}` |
| SENDING_CMD | `/robot/face_emotion` | `{"emotion":"happy"}` |
| CONNECTION_FAILED | `/robot/face_emotion` | `{"emotion":"idle"}` |
```

**Step 2: Save and commit**

```bash
cd ~/neuro_bot_ws/src/neuro_bot
git add docs/plans/2026-01-22-qr-bluetooth-flow-design.md
git commit -m "docs: add qr-bluetooth-flow design document"
```

---

## Task 2: 修改 brain_core - 新增状态机状态

**Files:**
- Modify: `src/brain_core/src/modules/state.rs`
- Modify: `src/brain_core/src/main.rs`

**Step 1: 添加新状态到状态机**

修改 `src/brain_core/src/modules/state.rs`:

```rust
#[derive(Debug, Clone, PartialEq)]
pub enum RobotState {
    Idle,
    Listening,
    Thinking,
    Speaking,
    Busy,        // 保持原有
    // 新增状态
    Scanning,    // 扫描连接中
    SendingCmd,  // 下发指令中
}

impl Default for RobotState {
    fn default() -> Self {
        RobotState::Idle
    }
}
```

**Step 2: 修改 main.rs 添加 VisionResult 处理**

在 `src/brain_core/src/main.rs` 中添加：

```rust
// 在 ROS 2 初始化后添加表情和语音发布者
let emotion_publisher = node.create_publisher::<FaceEmotion>("/robot/face_emotion")?;
let tts_publisher = node.create_publisher::<String>("/audio/tts_play")?;

// VisionResult 处理任务
let vision_tx = vision_tx.clone();
let emotion_publisher = emotion_publisher.clone();
let tts_publisher = tts_publisher.clone();

tokio::task::spawn(async move {
    let mut vision_sub = node.subscribe::<VisionResult>("/vision/result", qos_profile).unwrap();

    while let Some(msg) = vision_sub.next().await {
        if msg.type_ == "ble" {
            // 解析二维码内容
            if let Ok(payload) = serde_json::from_str::<QrCodePayload>(&msg.content) {
                let mac = format!(
                    "{}:{}:{}:{}:{}:{}",
                    &payload.m[0..2],
                    &payload.m[2..4],
                    &payload.m[4..6],
                    &payload.m[6..8],
                    &payload.m[8..10],
                    &payload.m[10..12]
                );

                // 1. 播报语音：已识别出 MAC 地址
                let tts_msg = String::from("已识别出二维码中的 MAC 地址，正在连接蓝牙设备");
                tts_publisher.publish(&tts_msg).unwrap();

                // 2. 切换表情为 BUSY
                let mut emotion = FaceEmotion::default();
                emotion.emotion = "busy".to_string();
                emotion_publisher.publish(&emotion).unwrap();

                // 3. 发送事件到状态机
                let _ = vision_tx.send(BrainEvent::QrCodeScanned {
                    mac,
                    command: payload.c
                }).await;
            }
        }
    }
});
```

**Step 3: 修改状态机事件处理**

在状态机处理逻辑中添加：

```rust
match event {
    BrainEvent::QrCodeScanned { mac, command } => {
        self.current_state = RobotState::Scanning;
        // 发起蓝牙连接请求
        self.initiate_bluetooth_connect(mac, command).await;
    }
    BrainEvent::BluetoothConnected { device_name, command } => {
        self.current_state = RobotState::SendingCmd;

        // 1. 播报语音：蓝牙已连接
        let tts_msg = String::from("蓝牙设备已连接，并下发查询指令");
        tts_publisher.publish(&tts_msg).unwrap();

        // 2. 切换表情为 HAPPY
        let mut emotion = FaceEmotion::default();
        emotion.emotion = "happy".to_string();
        emotion_publisher.publish(&emotion).unwrap();

        // 3. 下发指令
        self.send_bluetooth_command(command).await;
    }
    BrainEvent::BluetoothCommandSent => {
        // 恢复 IDLE 状态
        let mut emotion = FaceEmotion::default();
        emotion.emotion = "idle".to_string();
        emotion_publisher.publish(&emotion).unwrap();

        self.current_state = RobotState::Idle;
    }
    BrainEvent::BluetoothFailed { reason } => {
        // 1. 播报语音：连接失败
        let tts_msg = format!("蓝牙连接失败，请重试");
        tts_publisher.publish(&tts_msg).unwrap();

        // 2. 切换表情为 IDLE
        let mut emotion = FaceEmotion::default();
        emotion.emotion = "idle".to_string();
        emotion_publisher.publish(&emotion).unwrap();

        self.current_state = RobotState::Idle;
    }
    _ => {}
}
```

**Step 4: Commit**

```bash
git add src/brain_core/src/
git commit -m "feat(brain_core): add Scanning/SendingCmd states and QR handling"
```

---

## Task 3: 修改 face_bridge - 支持新表情

**Files:**
- Modify: `src/face_bridge/src/main.rs`

**Step 1: 更新表情处理**

```rust
// 在订阅回调中添加对新表情的处理
fn handle_emotion(msg: FaceEmotion) {
    match msg.emotion.as_str() {
        "happy" => sender_1("h"),     // 开心
        "busy" => sender_1("b"),      // 忙碌
        "idle" | "neutral" => sender_1("n"), // 中性/空闲
        "thinking" => sender_1("t"),  // 思考
        "speaking" => sender_1("l"),  // 说话
        _ => sender_1("n"),           // 默认中性
    }
}
```

**Step 2: Commit**

```bash
git add src/face_bridge/src/
git commit -m "feat(face_bridge): support happy and busy emotions"
```

---

## Task 4: 编译并测试

**Step 1: 编译**

```bash
cd ~/neuro_bot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select brain_core face_bridge robot_interfaces
```

**Step 2: 测试验证**

```bash
# 启动各节点
ros2 run brain_core brain_core
ros2 run face_bridge face_bridge
ros2 run vision_engine qr_node
ros2 run iot_controller iot_controller
ros2 run audio_engine audio_node

# 测试流程
# 1. 展示二维码
# 2. 观察语音播报
# 3. 观察表情变化
# 4. 检查蓝牙连接日志
```

**Step 3: Commit**

```bash
git commit -m "chore: build and test qr-bluetooth-flow feature"
```

---

## Task 5: 功能验证清单

- [ ] vision_engine 正确识别二维码并发布到 `/vision/result`
- [ ] brain_core 接收 VisionResult 并解析 MAC 地址
- [ ] 语音播报 "已识别出二维码中的 MAC 地址，正在连接蓝牙设备"
- [ ] 表情切换为 BUSY
- [ ] iot_controller 发起蓝牙连接
- [ ] 连接成功后语音播报 "蓝牙设备已连接，并下发查询指令"
- [ ] 表情切换为 HAPPY
- [ ] 指令发送完成后恢复 IDLE 状态
- [ ] 连接/指令失败时播报 "蓝牙连接失败，请重试" 并恢复 IDLE
