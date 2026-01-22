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
use tokio::sync::mpsc;
use tokio::time;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🧠 Brain Core 2.0 (Async Actor) Starting...");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "brain_core", "")?;

    // 1. 初始化模块
    let emotion_manager = EmotionManager::new(&mut node)?;
    let state_manager = StateManager::new(&mut node)?;
    
    // 2. 通信接口
    let tts_publisher = node.create_publisher::<StringMsg>("/audio/tts_play", r2r::QosProfile::default())?;
    
    // ⚠️ 注意：这里连接的是我们刚刚修好的 IoT 服务
    let bt_client = Arc::new(node.create_client::<ConnectBluetooth::Service>("/iot/connect_bluetooth", r2r::QosProfile::default())?);
    let llm_client = Arc::new(node.create_client::<AskLLM::Service>("/brain/ask_llm", r2r::QosProfile::default())?);

    let mut speech_sub = node.subscribe::<AudioSpeech>("/audio/speech", r2r::QosProfile::default())?;
    let mut vision_sub = node.subscribe::<VisionResult>("/vision/result", r2r::QosProfile::default())?;

    // 3. 建立内部神经通道 (MPSC Channel)
    let (tx, mut rx) = mpsc::channel::<BrainEvent>(32);

    println!("🔗 System Ready. Entering Event Loop.");

    // --- 任务 A: 视觉感知 (Producer) ---
    // 负责解析 Neural Link 协议
    let vision_tx = tx.clone();
    let emotion_manager_for_vision = emotion_manager.clone();
    let tts_pub_for_vision = tts_publisher.clone();

    tokio::task::spawn(async move {
        while let Some(msg) = vision_sub.next().await {
            // 尝试解析 JSON
            if let Ok(payload) = serde_json::from_str::<NeuralLinkPayload>(&msg.content) {
                if payload.t == "ble" {
                    // 解析 MAC 地址并格式化
                    let mac = format!(
                        "{}:{}:{}:{}:{}:{}",
                        &payload.m[0..2], &payload.m[2..4],
                        &payload.m[4..6], &payload.m[6..8],
                        &payload.m[8..10], &payload.m[10..12]
                    );

                    // 1. 播报语音
                    let _ = tts_pub_for_vision.publish(&StringMsg { data: String::from("已识别出二维码中的 MAC 地址，正在连接蓝牙设备") });

                    // 2. 切换表情为 BUSY
                    emotion_manager_for_vision.set_busy();

                    // 3. 发送事件到状态机
                    let command = payload.d.unwrap_or_default();
                    let _ = vision_tx.send(BrainEvent::QrCodeScanned { mac, command }).await;
                }
            }
        }
    });

    // --- 任务 B: 心跳起搏器 ---
    let timer_tx = tx.clone();
    tokio::task::spawn(async move {
        let mut interval = time::interval(Duration::from_millis(500));
        loop {
            interval.tick().await;
            let _ = timer_tx.send(BrainEvent::Heartbeat).await;
        }
    });

    // --- 任务 C: 听觉回路 (保持独立) ---
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

    // --- 任务 D: 主控状态机 (Actor Loop) ---
    let mut bt_lifecycle = BtLifecycle::Idle;
    let mut last_connected_mac = String::new(); 

    // 主循环：处理所有事件
    while let Some(event) = rx.recv().await {
        match event {
            // [事件 1] 视觉发现目标
            BrainEvent::VisionTargetFound(payload) => {
                // 只有在空闲且非重复时才响应
                if let BtLifecycle::Idle = bt_lifecycle {
                    if payload.m == last_connected_mac { continue; } 
                    
                    println!("👁️ 锁定目标: {} (CMD: {:?})", payload.m, payload.d);
                    
                    bt_lifecycle = BtLifecycle::Connecting { 
                        target_mac: payload.m.clone(), 
                        start_time: Instant::now() 
                    };
                    
                    // 设置忙碌，防止语音打断
                    state_manager.set_busy("Bluetooth Connecting");
                    emotion_manager.set_happy();

                    // 语音播报
                    let device_name = payload.n.unwrap_or("蓝牙设备".to_string());
                    let _ = tts_publisher.publish(&StringMsg { data: format!("正在连接{}", device_name) });

                    // 发起连接 (异步调用 IoT 服务)
                    let client = bt_client.clone();
                    let response_tx = tx.clone();
                    let mac = payload.m.clone();
                    let service = payload.s.unwrap_or_default();
                    let characteristic = payload.c.unwrap_or_default();
                    let command = payload.d.unwrap_or_default();

                    tokio::spawn(async move {
                        // 构造请求
                        let req = ConnectBluetooth::Request { 
                            mac,
                            service_uuid: service,
                            characteristic_uuid: characteristic,
                            command 
                        };

                        // 🟢 [修复点] 先处理 request() 的 Result，拿到 future 再 await
                        let evt = match client.request(&req) {
                            Ok(future) => {
                                // 请求创建成功，现在开始计时等待结果
                                match time::timeout(Duration::from_secs(15), future).await {
                                    Ok(Ok(resp)) => BrainEvent::ConnectionResult { success: resp.success, message: resp.message },
                                    Ok(Err(e)) => BrainEvent::ConnectionResult { success: false, message: format!("ROS Call Error: {}", e) },
                                    Err(_) => BrainEvent::ConnectionResult { success: false, message: "Timeout".to_string() },
                                }
                            }
                            Err(e) => {
                                // 请求连发都没发出去（比如 Service 还没上线）
                                BrainEvent::ConnectionResult { success: false, message: format!("Client Request Error: {}", e) }
                            }
                        };
                        
                        let _ = response_tx.send(evt).await;
                    });
                }
            }

            // [事件 2] 连接结果返回
            BrainEvent::ConnectionResult { success, message } => {
                if let BtLifecycle::Connecting { target_mac, .. } = &bt_lifecycle {
                    if success {
                        println!("✅ 操作成功: {}", message);
                        last_connected_mac = target_mac.clone();
                        bt_lifecycle = BtLifecycle::Connected { device_name: "Unknown".into() };
                        let _ = tts_publisher.publish(&StringMsg { data: "指令已发送".to_string() });
                    } else {
                        println!("❌ 操作失败: {}", message);
                        bt_lifecycle = BtLifecycle::Failed { 
                            reason: message.clone(), 
                            cooldown_until: Instant::now() + Duration::from_secs(5) 
                        };
                        let _ = tts_publisher.publish(&StringMsg { data: "连接失败".to_string() });
                    }
                    // 恢复空闲
                    state_manager.set_idle();
                    emotion_manager.set_neutral();
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

            // [新事件 4] 二维码扫描完成，准备连接蓝牙
            BrainEvent::QrCodeScanned { mac, command } => {
                if let BtLifecycle::Idle = bt_lifecycle {
                    println!("📱 二维码扫描完成: {} (CMD: {})", mac, command);

                    bt_lifecycle = BtLifecycle::Connecting {
                        target_mac: mac.clone(),
                        start_time: Instant::now()
                    };

                    // 发起蓝牙连接 (异步调用 IoT 服务)
                    let client = bt_client.clone();
                    let response_tx = tx.clone();

                    tokio::spawn(async move {
                        let req = ConnectBluetooth::Request {
                            mac,
                            service_uuid: String::new(),
                            characteristic_uuid: String::new(),
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
                            Err(e) => {
                                BrainEvent::ConnectionResult { success: false, message: format!("Client Request Error: {}", e) }
                            }
                        };

                        let _ = response_tx.send(evt).await;
                    });
                }
            }

            // [新事件 5] 蓝牙已连接，下发指令
            BrainEvent::BluetoothConnected { device_name, command } => {
                println!("✅ 蓝牙已连接: {}, 下发指令: {}", device_name, command);
                // 播报语音
                let _ = tts_publisher.publish(&StringMsg { data: String::from("蓝牙设备已连接，并下发查询指令") });
                // 切换表情为 HAPPY
                emotion_manager.set_happy();

                // 下发指令逻辑已在 QrCodeScanned 中处理
            }

            // [新事件 6] 蓝牙指令已发送
            BrainEvent::BluetoothCommandSent => {
                println!("📤 蓝牙指令已发送，恢复空闲状态");
                // 恢复 IDLE 状态
                emotion_manager.set_idle();
                state_manager.set_idle();
            }

            // [新事件 7] 蓝牙连接失败
            BrainEvent::BluetoothFailed { reason } => {
                println!("❌ 蓝牙连接失败: {}", reason);
                // 播报语音: 连接失败
                let _ = tts_publisher.publish(&StringMsg { data: format!("蓝牙连接失败，请重试") });
                // 切换表情为 IDLE
                emotion_manager.set_idle();
                state_manager.set_idle();
                bt_lifecycle = BtLifecycle::Idle;
            }
        }
    }

    loop { node.spin_once(Duration::from_millis(100)); }
}