mod modules;
use modules::emotion::EmotionManager;
use modules::state::StateManager;

use r2r;
use futures::StreamExt;
use tokio::time;
use std::time::Duration;
use std::sync::Arc; 

use r2r::robot_interfaces::srv::AskLLM;
use r2r::robot_interfaces::msg::AudioSpeech;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🧠 Brain Core (connected to LLM) is starting...");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "brain_core", "")?;

    let emotion_manager = EmotionManager::new(&mut node)?;
    let state_manager = StateManager::new(&mut node)?;

    // 使用 Arc 包裹 Client，使其支持跨线程 clone
    let llm_client = Arc::new(
        node.create_client::<AskLLM::Service>("/ask_llm", r2r::QosProfile::default())?
    );
    
    println!("🔗 Waiting for LLM Service to be available...");

    let mut speech_sub = node.subscribe::<AudioSpeech>("/audio/speech", r2r::QosProfile::default())?;

    tokio::task::spawn(async move {
        println!("✅ Brain Loop Started. Waiting for voice...");
        
        while let Some(msg) = speech_sub.next().await {
            println!("------------------------------------------------");
            println!("👂 听到声音: \"{}\"", msg.text);

            state_manager.set_thinking();
            emotion_manager.set_thinking();
            println!("🧠 正在思考... (Requesting LLM)");

            let client = llm_client.clone(); 
            let mut s_mgr = state_manager.clone();
            let mut e_mgr = emotion_manager.clone();
            let question = msg.text.clone();

            tokio::spawn(async move {
                let request = AskLLM::Request { question };

                // 🛠️ 修复点：先获取 Future，再 await
                // client.request() 返回的是 Result<Future, Error>
                match client.request(&request) {
                    Ok(future) => {
                        // 请求发送成功，现在等待 (await) 结果
                        match future.await {
                            Ok(response) => {
                                if response.success {
                                    println!("💡 LLM 回复: \"{}\"", response.answer);
                                    
                                    s_mgr.set_speaking();
                                    e_mgr.set_happy();
                                    
                                    time::sleep(Duration::from_secs(3)).await;
                                    
                                    s_mgr.set_idle();
                                    e_mgr.set_neutral();
                                } else {
                                    println!("❌ LLM 处理失败 (success=false)");
                                    e_mgr.set_neutral();
                                    s_mgr.set_idle();
                                }
                            }
                            Err(e) => {
                                println!("🔥 等待回复时出错 (可能是超时): {}", e);
                                e_mgr.set_neutral();
                                s_mgr.set_idle();
                            }
                        }
                    }
                    Err(e) => {
                        // 还没发出去就报错了
                        println!("🚫 请求发送失败 (Client可能未连接): {}", e);
                        e_mgr.set_neutral();
                        s_mgr.set_idle();
                    }
                }
            });
        }
    });

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}