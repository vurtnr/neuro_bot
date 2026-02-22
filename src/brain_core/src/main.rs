mod modules;
use modules::coordinator::{Action as CoordinatorAction, Coordinator, Event as CoordinatorEvent, Mode as CoordinatorMode};
use modules::emotion::EmotionManager;
use modules::state::{BrainEvent, NeuralLinkPayload, StateManager};
use r2r;
use r2r::robot_interfaces::srv::{AskLLM, ConnectBluetooth};
use r2r::robot_interfaces::msg::{AudioSpeech, VisionResult, NetworkStatus,BodyCommand};
use r2r::std_msgs::msg::String as StringMsg;
use futures::StreamExt;
use std::future::{pending, Future};
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
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

    // 创建发布身体动作指令的 Publisher
    let body_pub = node.create_publisher::<BodyCommand>("/iot/body_command", r2r::QosProfile::default())?;
    
    // ⚠️ 注意：这里连接的是我们刚刚修好的 IoT 服务
    let bt_client = Arc::new(node.create_client::<ConnectBluetooth::Service>("/iot/connect_bluetooth", r2r::QosProfile::default())?);
    let llm_client = Arc::new(node.create_client::<AskLLM::Service>("/brain/ask_llm", r2r::QosProfile::default())?);

    let mut speech_sub = node.subscribe::<AudioSpeech>("/audio/speech", r2r::QosProfile::default())?;
    let mut vision_sub = node.subscribe::<VisionResult>("/vision/result", r2r::QosProfile::default())?;

    // ==========================================
    // 🆕 3. 网络状态监控 (注入点)
    // ==========================================
    let mut network_sub = node.subscribe::<NetworkStatus>("/system/network_status", r2r::QosProfile::default())?;
    let state_for_net = state_manager.clone(); // StateManager 必须 derive Clone
    tokio::spawn(async move {
        println!("📡 Network Monitor Started...");
        while let Some(msg) = network_sub.next().await {
            // 逻辑: 已连接且信号不是未知(99)才算有效在线
            // 您可以根据实际情况调整，比如要求 signal_strength > 10
            let is_online = msg.is_connected && msg.signal_strength != 99;
            state_for_net.set_online(is_online);
        }
    });
    // ==========================================

    println!("🔗 System Ready. Entering Event Loop.");

    let mut coordinator = Coordinator::new();
    let mut pending_ble: Option<Pin<Box<dyn Future<Output = BrainEvent>>>> = None;
    let mut pending_llm: Option<Pin<Box<dyn Future<Output = BrainEvent>>>> = None;
    let mut pending_audio_done: Option<Pin<Box<time::Sleep>>> = None;

    let mut spin_interval = time::interval(Duration::from_millis(10));
    loop {
        let mut event_to_handle: Option<BrainEvent> = None;

        tokio::select! {
            _ = spin_interval.tick() => {
                node.spin_once(Duration::from_millis(0));
            }
            msg = vision_sub.next() => {
                if let Some(msg) = msg {
                    if let Ok(payload) = serde_json::from_str::<NeuralLinkPayload>(&msg.content) {
                        if payload.t == "ble" {
                            event_to_handle = Some(BrainEvent::VisionFound(payload));
                        }
                    }
                }
            }
            msg = speech_sub.next() => {
                if let Some(msg) = msg {
                    if msg.is_final {
                       // ==========================================
                        // 🆕 4. 离线拦截逻辑
                        // ==========================================
                        if !state_manager.is_online() {
                            println!("🚫 离线模式: 拦截语音请求");
                            state_manager.set_busy("Network Offline");
                            let _ = tts_publisher.publish(&StringMsg { 
                                data: "网络信号不佳，我暂时无法连接大脑。".to_string() 
                            });
                            let sm_clone = state_manager.clone();
                            tokio::spawn(async move {
                                time::sleep(Duration::from_secs(3)).await;
                                sm_clone.set_idle();
                            });
                            continue; 
                        }
                        event_to_handle = Some(BrainEvent::AudioFinal(msg.text));
                    }
                }
            }
            event = async {
                if let Some(fut) = pending_ble.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<BrainEvent>().await
                }
            } => {
                pending_ble = None;
                event_to_handle = Some(event);
            }
            event = async {
                if let Some(fut) = pending_llm.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<BrainEvent>().await
                }
            } => {
                pending_llm = None;
                event_to_handle = Some(event);
            }
            _ = async {
                if let Some(fut) = pending_audio_done.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<()>().await
                }
            } => {
                pending_audio_done = None;
                event_to_handle = Some(BrainEvent::AudioDone);
            }
        }

        if let Some(event) = event_to_handle {
            let coordinator_event = match event {
                BrainEvent::VisionFound(payload) => CoordinatorEvent::VisionFound(payload),
                BrainEvent::BleResult { success, message } => {
                    println!("🔄 BLE 结果: {}", message);
                    CoordinatorEvent::BleResult { success, message }
                }
                BrainEvent::AudioFinal(text) => CoordinatorEvent::AudioFinal(text),
                BrainEvent::AudioLlmResult { success, answer } => {
                    CoordinatorEvent::AudioLlmResult { success, answer }
                }
                BrainEvent::AudioDone => CoordinatorEvent::AudioDone,
                BrainEvent::Heartbeat => continue,
            };

            let actions = coordinator.on_event(coordinator_event);
            let schedule_audio_done = matches!(coordinator.mode(), CoordinatorMode::AudioSpeaking);

            for action in actions {
                match action {
                    CoordinatorAction::Speak(text) => {
                        let _ = tts_publisher.publish(&StringMsg { data: text.clone() });
                        if schedule_audio_done {
                            let duration = std::cmp::max(2, (text.chars().count() / 5) as u64);
                            pending_audio_done = Some(Box::pin(time::sleep(Duration::from_secs(duration))));
                        }
                    }
                    CoordinatorAction::StartLlm(question) => {
                        if pending_llm.is_none() {
                            println!("👂 Hearing: {}", question);
                            let client = llm_client.clone();
                            pending_llm = Some(Box::pin(async move {
                                let req = AskLLM::Request { question };
                                match client.request(&req) {
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
                                }
                            }));
                        }
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
                    CoordinatorAction::RequestBle(req) => {
                        if pending_ble.is_none() {
                            println!("👁️ 锁定目标: {} (CMD: {:?})", req.mac, req.command);
                            let client = bt_client.clone();
                            pending_ble = Some(Box::pin(async move {
                                let svc = ConnectBluetooth::Request {
                                    mac: req.mac,
                                    service_uuid: req.service_uuid,
                                    characteristic_uuid: req.characteristic_uuid,
                                    command: req.command,
                                };
                                match client.request(&svc) {
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
                                }
                            }));
                        }
                    }
                    CoordinatorAction::BodyMove { cmd, params } => {
                        let msg = BodyCommand {
                            cmd: cmd.clone(),
                            params: params.clone(),
                        };
                        
                        if let Err(e) = body_pub.publish(&msg) {
                            eprintln!("❌ 发送舵机指令失败: {}", e);
                        } else {
                            println!("🚀 下发身体动作指令: cmd={}, params={}", cmd, params);
                        }
                    }
                }
            }
        }
    }
}
