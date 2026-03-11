use r2r;
use r2r::robot_interfaces::msg::{FaceEmotion, RobotState};
use futures::StreamExt;
use tokio::task;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;
use serialport;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("😳 Face Bridge Starting (Hardware Enabled)...");
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "face_bridge", "")?;

    // 订阅指令与状态
    let mut emotion_sub = node.subscribe::<FaceEmotion>("/robot/face_emotion", r2r::QosProfile::default())?;
    let mut state_sub = node.subscribe::<RobotState>("/robot/state", r2r::QosProfile::default())?;

    // --- 🟢 1. 创建串口通信线程 (负责真实的硬件通信) ---
    // 创建一个通道：tx (发送端), rx (接收端)
    let (tx, rx) = mpsc::channel::<String>();
    
    thread::spawn(move || {
        // ⚠️ 注意：请根据您的实际情况修改端口号
        // 常见的是 "/dev/ttyUSB0" 或 "/dev/ttyACM0"
        let port_name = "/dev/ttyAMA0"; 
        let baud_rate = 115200;

        println!("🔌 Attempting to open Serial Port: {} @ {}", port_name, baud_rate);
        
        // 尝试打开串口
        match serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(100))
            .open() {
            Ok(mut port) => {
                println!("✅ Serial Port Connected Successfully!");
                
                // 持续监听通道里的消息
                while let Ok(cmd) = rx.recv() {
                    // Python 的 readline 需要换行符作为结束标志，所以加上 \n
                    let output = format!("{}\n", cmd);
                    
                    // 真正的写入硬件操作
                    match port.write(output.as_bytes()) {
                        Ok(_) => println!("[Hardware] Sent command: '{}'", cmd),
                        Err(e) => eprintln!("❌ Serial Write Failed: {}", e),
                    }
                }
            },
            Err(e) => {
                eprintln!("❌ CRITICAL ERROR: Failed to open serial port: {}", e);
                eprintln!("   Hint 1: Is the USB cable plugged in?");
                eprintln!("   Hint 2: Do you have permissions? Try 'sudo chmod 777 {}'", port_name);
            }
        }
    });

    // 定义一个辅助闭包，方便发送指令到上面的线程
    let send_cmd = move |cmd: &str| {
        // 将指令发送给串口线程
        let _ = tx.send(cmd.to_string());
    };

    // --- 🟢 2. 任务1: 处理手动表情指令 ---
    let sender_1 = send_cmd.clone();
    task::spawn(async move {
        while let Some(msg) = emotion_sub.next().await {
            // 收到 ROS 表情指令 -> 发送给串口
            match msg.emotion.as_str() {
                "happy" => sender_1("h"),
                _ => sender_1("n"), // 默认恢复中性
            }
        }
    });

    // --- 🟢 3. 任务2: 处理系统状态 (自动表情) ---
    let sender_2 = send_cmd.clone();
    task::spawn(async move {
        while let Some(msg) = state_sub.next().await {
            println!("🔄 FaceBridge received state: {}", msg.state);
            // 收到 ROS 状态 -> 映射为屏幕单字符指令
            match msg.state.as_str() {
                "THINKING" => sender_2("t"), // 思考状态 -> 发送 't'
                "BUSY" => sender_2("b"),     // 忙碌状态 -> 发送 'b' (蓝色眼睛)
                "SPEAKING" => sender_2("l"), // 说话/监听 -> 发送 'l' (波形)
                "IDLE" => sender_2("n"),     // 空闲 -> 发送 'n'
                _ => {}
            }
        }
    });

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}