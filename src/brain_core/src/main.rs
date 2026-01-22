mod modules;
use modules::emotion::EmotionManager;
use modules::state::{StateManager, BtLifecycle, BrainEvent, NeuralLinkPayload};
use r2r;
use r2r::robot_interfaces::srv::{AskLLM, ConnectBluetooth};
use r2r::robot_interfaces::msg::{AudioSpeech, FaceEmotion, VisionResult};
use r2r::std_msgs::msg::String as StringMsg;
use futures::StreamExt;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{mpsc, Mutex};
use tokio::time;

// 蓝牙指令枚举
enum Command {
    SendBleCommand { mac: String, data: String },
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🧠 Brain Core 2.0 (Async Actor) Starting...");

    let ctx = r2r::Context::create()?;
    let node = Arc::new(Mutex::new(r2r::Node::create(ctx, "brain_core", "")?));

    // 1. 初始化模块
    let emotion_manager = EmotionManager::new(&mut *node.lock().await)?;
    let state_manager = StateManager::new(&mut *node.lock().await)?;

    // 2. 通信接口
    let tts_publisher = node.lock().await.create_publisher::<StringMsg>("/audio/tts_play", r2r::QosProfile::default())?;

    // ⚠️ 注意：这里连接的是我们刚刚修好的 IoT 服务
    let bt_client = Arc::new(node.lock().await.create_client::<ConnectBluetooth::Service>("/iot/connect_bluetooth", r2r::QosProfile::default())?);
    let llm_client = Arc::new(node.lock().await.create_client::<AskLLM::Service>("/brain/ask_llm", r2r::QosProfile::default())?);

    let mut speech_sub = node.lock().await.subscribe::<AudioSpeech>("/audio/speech", r2r::QosProfile::default())?;

    // 使用 Sensor Data QoS（BestEffort），与 vision_engine 的发布配置匹配
    let mut vision_sub = node.lock().await.subscribe::<VisionResult>("/vision/result", r2r::QosProfile::sensor_data())?;

    // 3. 建立内部神经通道 (MPSC Channel)
    let (tx, mut rx) = mpsc::channel::<BrainEvent>(32);

    // 4. 蓝牙指令通道
    let (cmd_tx, mut cmd_rx) = mpsc::channel::<Command>(32);

    println!("🔗 System Ready. Entering Event Loop.");

    // --- 任务 A: ROS Spin 任务 (关键！必须在独立任务中运行以处理 DDS 消息) ---
    let node_for_spin = node.clone();
    tokio::task::spawn(async move {
        loop {
            node_for_spin.lock().await.spin_once(Duration::from_millis(20));
            // 让出更多时间给其他任务
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    });

    // --- 任务 B: 视觉感知 (Producer) ---
    // 负责解析 Neural Link 协议
    // 注意：vision_sub 将在主事件循环中处理，避免跨线程问题

    // 去重：使用 std::sync::Mutex（避免 tokio Mutex 问题）
    use std::collections::HashSet;
    use std::sync::Mutex;
    let processed_msgs: Arc<Mutex<HashSet<String>>> = Arc::new(Mutex::new(HashSet::new()));

    // 标志：是否正在处理二维码
    let is_processing_qr: Arc<Mutex<bool>> = Arc::new(Mutex::new(false));

    // --- 任务 C: 心跳起搏器 ---
    let timer_tx = tx.clone();
    tokio::task::spawn(async move {
        let mut interval = time::interval(Duration::from_millis(500));
        loop {
            interval.tick().await;
            let _ = timer_tx.send(BrainEvent::Heartbeat).await;
        }
    });

    // --- 任务 D: 听觉回路 (保持独立) ---
    let sm_for_audio = state_manager.clone();
    let em_for_audio = emotion_manager.clone();
    let tts_pub_for_audio = tts_publisher.clone();
    let llm_client_for_audio = llm_client.clone();
    
    tokio::task::spawn(async move {
        while let Some(msg) = speech_sub.next().await {
            if !msg.is_final { continue; }
            if !sm_for_audio.can_accept_audio() { 
                // println!("🔇 Audio Ignored: Brain is busy");
                continue; 
            }
            
            println!("👂 Hearing: {}", msg.text);
            sm_for_audio.set_thinking();
            em_for_audio.set_thinking();

            let req = AskLLM::Request { question: msg.text.clone() };
            
            // 🟢 [Fix] 使用 match 正确处理 Result
            let llm_result = match llm_client_for_audio.request(&req) {
                Ok(future) => future.await, // 只有这一层 Result
                Err(e) => Err(e),
            };

            // 🟢 [Fix] 这里只需要解一层包，因为 llm_result 只是 Result<Response, Error>
            if let Ok(resp) = llm_result {
                if resp.success {
                    sm_for_audio.set_speaking();
                    em_for_audio.set_happy();
                    let _ = tts_pub_for_audio.publish(&StringMsg { data: resp.answer.clone() });
                    
                    let duration = std::cmp::max(2, (resp.answer.chars().count() / 5) as u64);
                    time::sleep(Duration::from_secs(duration)).await;
                }
            }
            
            // 恢复空闲
            sm_for_audio.set_idle();
            em_for_audio.set_neutral();
        }
    });

    // --- 任务 E: 蓝牙指令处理器 ---
    let event_tx_for_cmd = tx.clone();
    tokio::spawn(async move {
        while let Some(cmd) = cmd_rx.recv().await {
            match cmd {
                Command::SendBleCommand { mac, data } => {
                    println!("📤 发送蓝牙指令: {} -> {}", mac, data);
                    // 模拟指令发送完成 (实际由 IoT 服务响应后触发)
                    // 这里发送事件通知指令已发送
                    let _ = event_tx_for_cmd.send(BrainEvent::BluetoothCommandSent).await;
                }
            }
        }
    });

    // --- 任务 F: 主控状态机 (Actor Loop) ---
    let mut bt_lifecycle = BtLifecycle::Idle;
    let mut last_connected_mac = String::new();

    // 主循环：同时处理 BrainEvent 和 VisionResult
    loop {
        tokio::select! {
            // 优先处理 BrainEvent
            Some(event) = rx.recv() => {
                match event {
                    // [事件 1] 视觉发现目标 (旧逻辑，保留兼容)
                    BrainEvent::VisionTargetFound(payload) => {
                        if let BtLifecycle::Idle = bt_lifecycle {
                            if payload.m == last_connected_mac { continue; }
                            println!("👁️ 锁定目标: {} (CMD: {:?})", payload.m, payload.d);

                            let command = payload.d.clone().unwrap_or_default();
                            bt_lifecycle = BtLifecycle::Connecting {
                                target_mac: payload.m.clone(),
                                command: command.clone(),
                                start_time: Instant::now()
                            };

                            state_manager.set_busy("Bluetooth Connecting");
                            emotion_manager.set_happy();

                            let device_name = payload.n.unwrap_or("蓝牙设备".to_string());
                            let _ = tts_publisher.publish(&StringMsg { data: format!("正在连接{}", device_name) });

                            // 使用 tokio::spawn 在独立任务中处理（会有崩溃风险，但这里是旧逻辑）
                            let client = bt_client.clone();
                            let response_tx = tx.clone();
                            let mac = payload.m.clone();
                            let service = payload.s.unwrap_or_default();
                            let characteristic = payload.c.unwrap_or_default();

                            tokio::spawn(async move {
                                let req = ConnectBluetooth::Request {
                                    mac,
                                    service_uuid: service,
                                    characteristic_uuid: characteristic,
                                    command
                                };

                                let evt = match client.request(&req) {
                                    Ok(future) => {
                                        match time::timeout(Duration::from_secs(15), future).await {
                                            Ok(Ok(resp)) => BrainEvent::ConnectionResult { success: resp.success, message: resp.message },
                                            Ok(Err(e)) => BrainEvent::ConnectionResult { success: false, message: format!("ROS Call Error: {}", e) },
                                            Err(_) => BrainEvent::ConnectionResult { success: false, message: "Timeout".to_string() },
                                        }
                                    }
                                    Err(e) => BrainEvent::ConnectionResult { success: false, message: format!("Client Request Error: {}", e) },
                                };

                                let _ = response_tx.send(evt).await;
                            });
                        }
                    }

                    // [事件 2] 连接结果返回
                    BrainEvent::ConnectionResult { success, message } => {
                        if let BtLifecycle::Connecting { target_mac, command, .. } = &bt_lifecycle {
                            if success {
                                let mac = target_mac.clone();
                                let cmd = command.clone();
                                bt_lifecycle = BtLifecycle::Connected { device_name: "Unknown".into() };
                                let _ = tts_publisher.publish(&StringMsg { data: "指令已发送".to_string() });
                                let _ = tx.send(BrainEvent::BluetoothConnected {
                                    device_name: mac.clone(),
                                    command: cmd
                                }).await;
                            } else {
                                println!("❌ 操作失败: {}", message);
                                last_connected_mac = target_mac.clone();
                                bt_lifecycle = BtLifecycle::Failed {
                                    reason: message.clone(),
                                    cooldown_until: Instant::now() + Duration::from_secs(5)
                                };
                                let _ = tts_publisher.publish(&StringMsg { data: "连接失败".to_string() });
                                state_manager.set_idle();
                                emotion_manager.set_neutral();
                            }
                        }
                    }

                    // [事件 3] 超时检查
                    BrainEvent::Heartbeat => {
                        match &mut bt_lifecycle {
                            BtLifecycle::Connecting { start_time, .. } => {
                                if start_time.elapsed() > Duration::from_secs(20) {
                                    println!("⚠️ 连接超时重置");
                                    bt_lifecycle = BtLifecycle::Failed {
                                        reason: "Timeout".into(),
                                        cooldown_until: Instant::now() + Duration::from_secs(5)
                                    };
                                    state_manager.set_idle();
                                }
                            },
                            BtLifecycle::Failed { cooldown_until, .. } => {
                                if Instant::now() > *cooldown_until {
                                    bt_lifecycle = BtLifecycle::Idle;
                                }
                            },
                            _ => {}
                        }
                    }

                    // 旧事件（不再使用）
                    BrainEvent::QrCodeScanned { .. } => {}
                    BrainEvent::BluetoothConnected { device_name, command } => {
                        println!("✅ 蓝牙已连接: {}, 下发指令: {}", device_name, command);
                        state_manager.set_sending_cmd();
                        let _ = tts_publisher.publish(&StringMsg { data: String::from("蓝牙设备已连接，并下发查询指令") });
                        emotion_manager.set_happy();
                        if !command.is_empty() {
                            let _ = cmd_tx.send(Command::SendBleCommand {
                                mac: device_name,
                                data: command
                            }).await;
                        }
                    }
                    BrainEvent::BluetoothCommandSent => {
                        println!("📤 蓝牙指令已发送，恢复空闲状态");
                        emotion_manager.set_idle();
                        state_manager.set_idle();
                    }
                    BrainEvent::BluetoothFailed { reason } => {
                        println!("❌ 蓝牙连接失败: {}", reason);
                        let _ = tts_publisher.publish(&StringMsg { data: format!("蓝牙连接失败，请重试") });
                        emotion_manager.set_idle();
                        state_manager.set_idle();
                        bt_lifecycle = BtLifecycle::Idle;
                    }
                }
            }

            // 处理 VisionResult（直接在这里处理，避免 spawned 任务中访问 ROS 对象）
            Some(msg) = vision_sub.next() => {
                // 去重检查
                let msg_hash = format!("{}:{}", msg.type_, msg.content);
                let should_process = {
                    let mut processed = processed_msgs.lock().unwrap();
                    if processed.contains(&msg_hash) {
                        false
                    } else {
                        processed.insert(msg_hash.clone());
                        true
                    }
                };

                if !should_process {
                    continue;
                }

                println!("📥 收到 VisionResult: type={}, content={}", msg.type_, msg.content);

                // 解析 JSON
                match serde_json::from_str::<NeuralLinkPayload>(&msg.content) {
                    Ok(payload) => {
                        println!("✅ JSON 解析成功: t={}", payload.t);
                        if payload.t == "ble" {
                            // 验证 MAC 地址
                            if payload.m.len() != 12 && payload.m.len() != 17 {
                                r2r::log_warn!("brain_core", "Invalid MAC length: {}", payload.m.len());
                                continue;
                            }

                            // 格式化 MAC
                            let mac = if payload.m.contains(':') {
                                payload.m.clone()
                            } else {
                                format!("{}:{}:{}:{}:{}:{}",
                                    &payload.m[0..2], &payload.m[2..4],
                                    &payload.m[4..6], &payload.m[6..8],
                                    &payload.m[8..10], &payload.m[10..12])
                            };
                            let command = payload.d.unwrap_or_default();

                            // 1. 播报语音
                            let _ = tts_publisher.publish(&StringMsg { data: String::from("已识别出二维码中的 MAC 地址，正在连接蓝牙设备") });

                            // 2. 切换表情
                            emotion_manager.set_busy();
                            state_manager.set_busy("Bluetooth Connecting");

                            // 3. 直接发起蓝牙连接（在主事件循环中同步调用）
                            println!("🔄 发起蓝牙连接请求...");
                            let req = ConnectBluetooth::Request {
                                mac: mac.clone(),
                                service_uuid: String::new(),
                                characteristic_uuid: String::new(),
                                command: command.clone()
                            };

                            match bt_client.request(&req) {
                                Ok(future) => {
                                    match time::timeout(Duration::from_secs(15), future).await {
                                        Ok(Ok(resp)) => {
                                            println!("📨 连接结果: success={}, message={}", resp.success, resp.message);
                                            if resp.success {
                                                emotion_manager.set_happy();
                                                let _ = tts_publisher.publish(&StringMsg { data: String::from("蓝牙设备已连接，并下发查询指令") });
                                            } else {
                                                emotion_manager.set_neutral();
                                                let _ = tts_publisher.publish(&StringMsg { data: String::from("连接失败") });
                                            }
                                        }
                                        Ok(Err(e)) => {
                                            println!("❌ ROS Call Error: {}", e);
                                            emotion_manager.set_neutral();
                                            let _ = tts_publisher.publish(&StringMsg { data: String::from("连接失败") });
                                        }
                                        Err(_) => {
                                            println!("❌ Timeout");
                                            emotion_manager.set_neutral();
                                            let _ = tts_publisher.publish(&StringMsg { data: String::from("连接超时") });
                                        }
                                    }
                                }
                                Err(e) => {
                                    println!("❌ Client Request Error: {}", e);
                                    emotion_manager.set_neutral();
                                    let _ = tts_publisher.publish(&StringMsg { data: String::from("连接失败") });
                                }
                            }

                            // 恢复 IDLE
                            emotion_manager.set_idle();
                            state_manager.set_idle();
                        }
                    }
                    Err(e) => {
                        println!("❌ JSON 解析失败: {}", e);
                    }
                }
            }

            // 两个都关闭时退出
            else => {
                break;
            }
        }
    }
}