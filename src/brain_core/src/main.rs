mod modules;
use modules::emotion::EmotionManager;
use modules::state::StateManager;
use r2r;
use r2r::robot_interfaces::srv::{AskLLM, ConnectBluetooth};
use r2r::robot_interfaces::msg::{AudioSpeech, VisionResult};
use r2r::std_msgs::msg::String as StringMsg;
use futures::StreamExt;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::time;
use regex::Regex;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🧠 Brain Core (Modular Architecture) Starting...");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "brain_core", "")?;

    let emotion_manager = EmotionManager::new(&mut node)?;
    let state_manager = StateManager::new(&mut node)?;

    let tts_publisher = node.create_publisher::<StringMsg>("/audio/tts_play", r2r::QosProfile::default())?;
    
    // 关键服务：连接蓝牙
    let bt_client = Arc::new(node.create_client::<ConnectBluetooth::Service>("/iot/connect_bluetooth", r2r::QosProfile::default())?);
    let llm_client = Arc::new(node.create_client::<AskLLM::Service>("/brain/ask_llm", r2r::QosProfile::default())?);

    let mut speech_sub = node.subscribe::<AudioSpeech>("/audio/speech", r2r::QosProfile::default())?;
    let mut vision_sub = node.subscribe::<VisionResult>("/vision/result", r2r::QosProfile::default())?;

    println!("🔗 Waiting for dependencies...");
    
    // 资源克隆
    let em_for_vision = emotion_manager.clone();
    let tts_pub_for_vision = tts_publisher.clone();
    let bt_client_for_vision = bt_client.clone();
    let sm_for_vision = state_manager.clone();
    let sm_for_audio = state_manager.clone();

    // Task 1: 听觉回路
    tokio::task::spawn(async move {
        while let Some(msg) = speech_sub.next().await {
            if !msg.is_final { continue; }
            // 守卫检查：忙碌时屏蔽语音
            if !sm_for_audio.can_accept_audio() {
                println!("🔇 Ignored speech: System Busy");
                continue;
            }
            println!("👂 Input: \"{}\"", msg.text);
            sm_for_audio.set_thinking();
            emotion_manager.set_thinking();
            
            let client = llm_client.clone();
            let mut s_mgr = sm_for_audio.clone();
            let mut e_mgr = emotion_manager.clone();
            let tts_pub = tts_publisher.clone();
            let question = msg.text.clone();

            tokio::spawn(async move {
                let request = AskLLM::Request { question };
                // 这里的 .expect().await 是正确的
                match client.request(&request).expect("Client fail").await {
                    Ok(response) => {
                        if response.success {
                            s_mgr.set_speaking();
                            e_mgr.set_happy();
                            let _ = tts_pub.publish(&StringMsg { data: response.answer.clone() });
                            let duration = std::cmp::max(2, (response.answer.chars().count() / 5) as u64);
                            time::sleep(Duration::from_secs(duration)).await;
                        }
                    }
                    Err(e) => println!("🔥 LLM Error: {}", e),
                }
                s_mgr.set_idle();
                e_mgr.set_neutral();
            });
        }
    });

    // Task 2: 视觉回路
    tokio::task::spawn(async move {
        let mac_regex = Regex::new(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$").expect("Invalid Regex");
        let mut last_content = String::new();
        let mut last_seen_time = Instant::now();
        let mut is_first = true;

        while let Some(msg) = vision_sub.next().await {
            let now = Instant::now();
            let cooldown = Duration::from_secs(5);
            
            // 守卫检查
            if !sm_for_vision.can_accept_vision_task() { continue; }

            if msg.content != last_content || now.duration_since(last_seen_time) > cooldown || is_first {
                last_content = msg.content.clone();
                last_seen_time = now;
                is_first = false;

                if msg.type_ == "qrcode" {
                    let clean_content = msg.content.trim().to_string();
                    if mac_regex.is_match(&clean_content) {
                        println!("⚡ Detected MAC: {}", clean_content);
                        
                        // 1. 设置忙碌状态
                        sm_for_vision.set_busy("Connecting Bluetooth");
                        em_for_vision.set_happy();
                        
                        // 2. 播报
                        let _ = tts_pub_for_vision.publish(&StringMsg { data: "识别到蓝牙地址，准备连接".to_string() });
                        time::sleep(Duration::from_secs(4)).await;

                        // 3. 调用服务 (注意：这里去掉了错误的 .await)
                        let req = ConnectBluetooth::Request { address: clean_content.clone() };
                        match bt_client_for_vision.request(&req) {
                            Ok(future) => {
                                match future.await {
                                    Ok(res) => {
                                        let text = if res.success { "蓝牙连接成功".to_string() } else { format!("连接失败：{}", res.message) };
                                        let _ = tts_pub_for_vision.publish(&StringMsg { data: text });
                                    },
                                    Err(_) => println!("❌ BT Timeout"),
                                }
                            },
                            Err(e) => println!("❌ BT Client Error: {}", e),
                        }
                        
                        time::sleep(Duration::from_secs(3)).await;
                        sm_for_vision.set_idle();
                        em_for_vision.set_neutral();
                    } else {
                        // 普通二维码
                        let _ = tts_pub_for_vision.publish(&StringMsg { data: format!("发现内容：{}", clean_content) });
                    }
                }
            }
        }
    });

    loop { node.spin_once(Duration::from_millis(100)); }
}