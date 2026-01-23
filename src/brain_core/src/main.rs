mod modules;
use modules::coordinator::{Action as CoordinatorAction, Coordinator, Event as CoordinatorEvent};
use modules::emotion::EmotionManager;
use modules::state::{BrainEvent, NeuralLinkPayload, StateManager};
use r2r;
use r2r::robot_interfaces::srv::{AskLLM, ConnectBluetooth};
use r2r::robot_interfaces::msg::{AudioSpeech, VisionResult};
use r2r::std_msgs::msg::String as StringMsg;
use futures::StreamExt;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::time;

#[tokio::main(flavor = "current_thread")]
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
    tokio::task::spawn(async move {
        while let Some(msg) = vision_sub.next().await {
            // 尝试解析 JSON
            if let Ok(payload) = serde_json::from_str::<NeuralLinkPayload>(&msg.content) {
                if payload.t == "ble" {
                    // 发送给大脑主线程
                    let _ = vision_tx.send(BrainEvent::VisionFound(payload)).await;
                }
            }
            // (旧的纯MAC地址逻辑已废弃，强制要求使用 JSON 协议)
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
    let audio_tx = tx.clone();
    tokio::task::spawn(async move {
        while let Some(msg) = speech_sub.next().await {
            if !msg.is_final {
                continue;
            }
            let _ = audio_tx.send(BrainEvent::AudioFinal(msg.text)).await;
        }
    });

    // --- 任务 D: 主控状态机 (Actor Loop) ---
    let mut coordinator = Coordinator::new();

    // 主循环：处理所有事件（独立任务，避免阻塞 spin）
    tokio::spawn(async move {
        while let Some(event) = rx.recv().await {
            match event {
                // [事件 1] 视觉发现目标
                BrainEvent::VisionFound(payload) => {
                    let actions = coordinator.on_event(CoordinatorEvent::VisionFound(payload));
                    for action in actions {
                        match action {
                            CoordinatorAction::Speak(text) => {
                                let _ = tts_publisher.publish(&StringMsg { data: text });
                            }
                            CoordinatorAction::StartLlm(_) => {}
                            CoordinatorAction::SetEmotion(emotion) => {
                                match emotion.as_str() {
                                    "happy" => emotion_manager.set_happy(),
                                    "thinking" => emotion_manager.set_thinking(),
                                    "listening" => emotion_manager.set_listening(),
                                    _ => emotion_manager.set_neutral(),
                                }
                            }
                            CoordinatorAction::SetRobotState { state, detail } => {
                                match state.as_str() {
                                    "BUSY" => state_manager.set_busy(&detail),
                                    "THINKING" => state_manager.set_thinking(),
                                    "SPEAKING" => state_manager.set_speaking(),
                                    _ => state_manager.set_idle(),
                                }
                            }
                            CoordinatorAction::RequestBle(req) => {
                                println!("👁️ 锁定目标: {} (CMD: {:?})", req.mac, req.command);
                                let client = bt_client.clone();
                                let response_tx = tx.clone();
                                tokio::spawn(async move {
                                    let svc = ConnectBluetooth::Request {
                                        mac: req.mac,
                                        service_uuid: req.service_uuid,
                                        characteristic_uuid: req.characteristic_uuid,
                                        command: req.command,
                                    };

                                    let evt = match client.request(&svc) {
                                        Ok(future) => {
                                            match time::timeout(Duration::from_secs(15), future).await {
                                                Ok(Ok(resp)) => BrainEvent::BleResult {
                                                    success: resp.success,
                                                    message: resp.message,
                                                },
                                                Ok(Err(e)) => BrainEvent::BleResult {
                                                    success: false,
                                                    message: format!("ROS Call Error: {}", e),
                                                },
                                                Err(_) => BrainEvent::BleResult {
                                                    success: false,
                                                    message: "Timeout".to_string(),
                                                },
                                            }
                                        }
                                        Err(e) => BrainEvent::BleResult {
                                            success: false,
                                            message: format!("Client Request Error: {}", e),
                                        },
                                    };

                                    let _ = response_tx.send(evt).await;
                                });
                            }
                        }
                    }
                }

                // [事件 2] 连接结果返回
                BrainEvent::BleResult { success, message } => {
                    println!("🔄 BLE 结果: {}", message);
                    let actions = coordinator.on_event(CoordinatorEvent::BleResult { success, message });
                    for action in actions {
                        match action {
                            CoordinatorAction::Speak(text) => {
                                let _ = tts_publisher.publish(&StringMsg { data: text });
                            }
                            CoordinatorAction::StartLlm(_) => {}
                            CoordinatorAction::SetEmotion(emotion) => {
                                match emotion.as_str() {
                                    "happy" => emotion_manager.set_happy(),
                                    "thinking" => emotion_manager.set_thinking(),
                                    "listening" => emotion_manager.set_listening(),
                                    _ => emotion_manager.set_neutral(),
                                }
                            }
                            CoordinatorAction::SetRobotState { state, detail } => {
                                match state.as_str() {
                                    "BUSY" => state_manager.set_busy(&detail),
                                    "THINKING" => state_manager.set_thinking(),
                                    "SPEAKING" => state_manager.set_speaking(),
                                    _ => state_manager.set_idle(),
                                }
                            }
                            CoordinatorAction::RequestBle(_) => {}
                        }
                    }
                }
                BrainEvent::AudioFinal(text) => {
                    let actions = coordinator.on_event(CoordinatorEvent::AudioFinal(text.clone()));
                    for action in actions {
                        match action {
                            CoordinatorAction::Speak(_) => {}
                            CoordinatorAction::StartLlm(question) => {
                                println!("👂 Hearing: {}", question);
                                let client = llm_client.clone();
                                let response_tx = tx.clone();
                                tokio::spawn(async move {
                                    let req = AskLLM::Request { question };
                                    let evt = match client.request(&req) {
                                        Ok(future) => match future.await {
                                            Ok(resp) => BrainEvent::AudioLlmResult {
                                                success: resp.success,
                                                answer: resp.answer,
                                            },
                                            Err(e) => BrainEvent::AudioLlmResult {
                                                success: false,
                                                answer: format!("ROS Call Error: {}", e),
                                            },
                                        },
                                        Err(e) => BrainEvent::AudioLlmResult {
                                            success: false,
                                            answer: format!("Client Request Error: {}", e),
                                        },
                                    };
                                    let _ = response_tx.send(evt).await;
                                });
                            }
                            CoordinatorAction::SetEmotion(emotion) => {
                                match emotion.as_str() {
                                    "happy" => emotion_manager.set_happy(),
                                    "thinking" => emotion_manager.set_thinking(),
                                    "listening" => emotion_manager.set_listening(),
                                    _ => emotion_manager.set_neutral(),
                                }
                            }
                            CoordinatorAction::SetRobotState { state, detail } => {
                                match state.as_str() {
                                    "BUSY" => state_manager.set_busy(&detail),
                                    "THINKING" => state_manager.set_thinking(),
                                    "SPEAKING" => state_manager.set_speaking(),
                                    _ => state_manager.set_idle(),
                                }
                            }
                            CoordinatorAction::RequestBle(_) => {}
                        }
                    }
                }
                BrainEvent::AudioLlmResult { success, answer } => {
                    let actions = coordinator.on_event(CoordinatorEvent::AudioLlmResult {
                        success,
                        answer: answer.clone(),
                    });
                    for action in actions {
                        match action {
                            CoordinatorAction::Speak(text) => {
                                let _ = tts_publisher.publish(&StringMsg { data: text.clone() });
                                let duration = std::cmp::max(2, (text.chars().count() / 5) as u64);
                                let done_tx = tx.clone();
                                tokio::spawn(async move {
                                    time::sleep(Duration::from_secs(duration)).await;
                                    let _ = done_tx.send(BrainEvent::AudioDone).await;
                                });
                            }
                            CoordinatorAction::StartLlm(_) => {}
                            CoordinatorAction::SetEmotion(emotion) => {
                                match emotion.as_str() {
                                    "happy" => emotion_manager.set_happy(),
                                    "thinking" => emotion_manager.set_thinking(),
                                    "listening" => emotion_manager.set_listening(),
                                    _ => emotion_manager.set_neutral(),
                                }
                            }
                            CoordinatorAction::SetRobotState { state, detail } => {
                                match state.as_str() {
                                    "BUSY" => state_manager.set_busy(&detail),
                                    "THINKING" => state_manager.set_thinking(),
                                    "SPEAKING" => state_manager.set_speaking(),
                                    _ => state_manager.set_idle(),
                                }
                            }
                            CoordinatorAction::RequestBle(_) => {}
                        }
                    }
                }
                BrainEvent::AudioDone => {
                    let actions = coordinator.on_event(CoordinatorEvent::AudioDone);
                    for action in actions {
                        match action {
                            CoordinatorAction::Speak(_) => {}
                            CoordinatorAction::StartLlm(_) => {}
                            CoordinatorAction::SetEmotion(emotion) => {
                                match emotion.as_str() {
                                    "happy" => emotion_manager.set_happy(),
                                    "thinking" => emotion_manager.set_thinking(),
                                    "listening" => emotion_manager.set_listening(),
                                    _ => emotion_manager.set_neutral(),
                                }
                            }
                            CoordinatorAction::SetRobotState { state, detail } => {
                                match state.as_str() {
                                    "BUSY" => state_manager.set_busy(&detail),
                                    "THINKING" => state_manager.set_thinking(),
                                    "SPEAKING" => state_manager.set_speaking(),
                                    _ => state_manager.set_idle(),
                                }
                            }
                            CoordinatorAction::RequestBle(_) => {}
                        }
                    }
                }

                // [事件 3] 超时检查
                BrainEvent::Heartbeat => {}
            }
        }
    });

    let mut spin_interval = time::interval(Duration::from_millis(10));
    loop {
        spin_interval.tick().await;
        node.spin_once(Duration::from_millis(0));
    }
}
