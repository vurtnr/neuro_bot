use r2r;
use r2r::robot_interfaces::msg::{FaceEmotion, RobotState};
use futures::StreamExt;
use tokio::task;
// use serialport; // 如果您已经集成了串口库，请解开注释

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("😳 Face Bridge Starting...");
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "face_bridge", "")?;

    // 订阅指令与状态
    let mut emotion_sub = node.subscribe::<FaceEmotion>("/robot/face_emotion", r2r::QosProfile::default())?;
    let mut state_sub = node.subscribe::<RobotState>("/robot/state", r2r::QosProfile::default())?;

    // 模拟串口发送函数 (请替换为您实际的串口写入代码)
    let send_serial = |cmd: &str| {
        println!("[Serial] Sending command: '{}'", cmd);
        // let mut port = serialport::new("/dev/ttyUSB0", 115200)...
        // port.write(cmd.as_bytes())...
    };

    // 任务1: 处理手动表情指令
    let sender_1 = send_serial.clone(); // 实际使用时需注意闭包所有权或使用 channel
    task::spawn(async move {
        while let Some(msg) = emotion_sub.next().await {
            // 这里根据 msg.emotion 映射到 'n', 'h' 等
            // 简单示例:
            match msg.emotion.as_str() {
                "happy" => println!("[Serial] -> 'h'"),
                _ => println!("[Serial] -> 'n'"),
            }
        }
    });

    // 任务2: 处理系统状态 (自动表情)
    task::spawn(async move {
        while let Some(msg) = state_sub.next().await {
            println!("🔄 FaceBridge received state: {}", msg.state);
            match msg.state.as_str() {
                "THINKING" => {
                    println!("[Serial] -> 't'"); // 发送 't' 给屏幕
                },
                "BUSY" => {
                    println!("[Serial] -> 'b'"); // 🟢 关键：发送 'b' (蓝色忙碌)
                },
                "SPEAKING" => {
                    // 说话时可能保持开心或监听
                    println!("[Serial] -> 'l'"); 
                },
                "IDLE" => {
                    println!("[Serial] -> 'n'"); // 恢复默认
                },
                _ => {}
            }
        }
    });

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
    }
}