mod modules;
use modules::emotion::EmotionManager;
use modules::state::StateManager;

use r2r;
use r2r::robot_interfaces::msg::AudioSpeech;
use r2r::robot_interfaces::srv::AskLLM;
use r2r::std_msgs::msg::String as StringMsg;

use futures::StreamExt;
use std::sync::Arc; // [修复] 只引入 Arc，不需要 Mutex
use std::time::Duration;
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

    // [关键修复] 使用 Arc::new 包裹 Client
    // 这样 llm_client 的类型变成了 Arc<Client<...>>，它是可以被 clone 的
    let llm_client = Arc::new(
        node.create_client::<AskLLM::Service>("/brain/ask_llm", r2r::QosProfile::default())?,
    );

    let mut speech_sub =
        node.subscribe::<AudioSpeech>("/audio/speech", r2r::QosProfile::default())?;

    println!("🔗 Waiting for dependencies...");

    tokio::task::spawn(async move {
        println!("✅ Brain Logic Loop Started.");

        while let Some(msg) = speech_sub.next().await {
            if !msg.is_final {
                continue;
            }

            println!("👂 Input: \"{}\"", msg.text);

            // 状态一：思考
            state_manager.set_thinking();
            emotion_manager.set_thinking();

            // [修复后] 这里 clone 的是 Arc 指针，而不是 Client 本身，这是合法的且开销极小
            let client = llm_client.clone();

            let mut s_mgr = state_manager.clone();
            let mut e_mgr = emotion_manager.clone();
            let tts_pub = tts_publisher.clone();
            let question = msg.text.clone();

            tokio::spawn(async move {
                let request = AskLLM::Request { question };

                println!("🤔 Requesting LLM...");
                // client 是 Arc<Client>，它会自动解引用调用 request
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

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}
