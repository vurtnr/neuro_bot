mod modules;
use modules::emotion::EmotionManager;
use modules::state::StateManager;

use r2r;
// [修改] 引入 VisionResult 消息类型
use r2r::robot_interfaces::msg::{AudioSpeech, VisionResult};
use r2r::robot_interfaces::srv::AskLLM;
use r2r::std_msgs::msg::String as StringMsg;

use futures::StreamExt;
use std::sync::Arc; 
// [修改] 引入 Instant 用于计算冷却时间
use std::time::{Duration, Instant}; 
use tokio::time;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🧠 Brain Core (Modular Architecture) Starting...");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "brain_core", "")?;

    // 实例化模块
    let emotion_manager = EmotionManager::new(&mut node)?;
    let state_manager = StateManager::new(&mut node)?;

    let tts_publisher =
        node.create_publisher::<StringMsg>("/audio/tts_play", r2r::QosProfile::default())?;

    // [保留你的修复] 使用 Arc::new 包裹 Client
    let llm_client = Arc::new(
        node.create_client::<AskLLM::Service>("/brain/ask_llm", r2r::QosProfile::default())?,
    );

    let mut speech_sub =
        node.subscribe::<AudioSpeech>("/audio/speech", r2r::QosProfile::default())?;

    // [新增] 订阅视觉结果 Topic
    let mut vision_sub = 
        node.subscribe::<VisionResult>("/vision/result", r2r::QosProfile::default())?;

    println!("🔗 Waiting for dependencies...");

    // ================================================================
    // [关键步骤] 资源克隆
    // 原有的 Audio 任务会 move 走 emotion_manager 和 tts_publisher。
    // 所以我们需要在它们被 move 之前，先克隆一份给视觉任务用。
    // ================================================================
    let em_for_vision = emotion_manager.clone();
    let tts_pub_for_vision = tts_publisher.clone();

    // ================================================================
    // 👂 任务 1: 听觉回路 (保留你原有的逻辑)
    // ================================================================
    tokio::task::spawn(async move {
        println!("✅ Brain Audio Logic Loop Started.");

        while let Some(msg) = speech_sub.next().await {
            if !msg.is_final {
                continue;
            }

            println!("👂 Input: \"{}\"", msg.text);

            // 状态一：思考
            state_manager.set_thinking();
            emotion_manager.set_thinking();

            // [保留你的修复] 这里 clone 的是 Arc 指针
            let client = llm_client.clone();

            let mut s_mgr = state_manager.clone();
            let mut e_mgr = emotion_manager.clone();
            let tts_pub = tts_publisher.clone();
            let question = msg.text.clone();

            tokio::spawn(async move {
                let request = AskLLM::Request { question };

                println!("🤔 Requesting LLM...");
                match client.request(&request).expect("Client fail").await {
                    Ok(response) => {
                        if response.success {
                            println!("💡 Answer: \"{}\"", response.answer);

                            // 状态二：说话
                            s_mgr.set_speaking();
                            e_mgr.set_happy();

                            let tts_msg = StringMsg {
                                data: response.answer.clone(),
                            };
                            if let Err(e) = tts_pub.publish(&tts_msg) {
                                eprintln!("❌ TTS Publish Error: {}", e);
                            }

                            // 估算说话时间
                            let duration_secs =
                                std::cmp::max(2, (response.answer.chars().count() / 5) as u64);
                            time::sleep(Duration::from_secs(duration_secs)).await;
                        } else {
                            println!("❌ LLM Refused: {}", response.answer);
                        }
                    }
                    Err(e) => {
                        println!("🔥 LLM Service Call Failed: {}", e);
                    }
                }

                // 状态三：归位
                println!("💤 Returning to Idle");
                s_mgr.set_idle();
                e_mgr.set_neutral();
            });
        }
    });

    // ================================================================
    // 👁️ 任务 2: 视觉回路 (新增的部分)
    // ================================================================
    tokio::task::spawn(async move {
        println!("✅ Brain Vision Logic Loop Started.");
        
        // 视觉记忆：防止同一张二维码一直刷屏
        let mut last_content = String::new();
        let mut last_seen_time = Instant::now();
        let mut is_first = true; // 第一次看到即使时间很短也播报

        while let Some(msg) = vision_sub.next().await {
            let now = Instant::now();
            let cooldown = Duration::from_secs(5); // 冷却时间 5 秒

            // 逻辑：如果内容变了，或者距离上次播报超过5秒
            if msg.content != last_content || now.duration_since(last_seen_time) > cooldown || is_first {
                
                println!("👁️ Saw [{}]: {}", msg.type_, msg.content);
                
                // 更新记忆
                last_content = msg.content.clone();
                last_seen_time = now;
                is_first = false;

                // 只有二维码才触发语音
                if msg.type_ == "qrcode" {
                    // 1. 变表情：开心
                    em_for_vision.set_happy(); 
                    
                    // 2. 组织语言
                    let text_to_say = format!("我看到了二维码，内容是：{}", msg.content);
                    let tts_msg = StringMsg { data: text_to_say };
                    
                    println!("🗣️ Announcing QR Code...");
                    
                    // 3. 发送 TTS
                    if let Err(e) = tts_pub_for_vision.publish(&tts_msg) {
                         eprintln!("❌ Vision TTS Error: {}", e);
                    }
                    
                    // 4. 稍微保持一会儿状态，然后恢复
                    time::sleep(Duration::from_secs(3)).await;
                    em_for_vision.set_neutral();
                }
            }
        }
    });

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}