use r2r;
use futures::StreamExt;
use std::time::Duration;
use serialport;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("😳 Face Bridge (Simple Protocol) Starting...");

    // 1. 连接硬件 (使用你验证过的 ID)
    let port_name = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_42618f1794ff5844-if00";
    let baud_rate = 115200;

    println!("🔌 Connecting to: {}", port_name);

    let mut serial_port = match serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(100))
        .open() 
    {
        Ok(mut port) => {
            println!("✅ Hardware Connected!");
            // 必须拉高 DTR，MicroPython 才会理我们
            let _ = port.write_data_terminal_ready(true);
            Some(port)
        }
        Err(e) => {
            println!("⚠️ Connect Error: {}", e);
            None
        }
    };

    // 2. ROS 节点
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "face_bridge", "")?;
    let mut emotion_sub = node.subscribe::<r2r::std_msgs::msg::String>("/robot/emotion", r2r::QosProfile::default())?;

    // 3. 监听循环
    tokio::task::spawn(async move {
        println!("👂 Listening for emotions...");
        
        while let Some(msg) = emotion_sub.next().await {
            let emotion = msg.data.as_str();
            println!("⚡ [ROS] Emotion: {}", emotion);
            
            // 极简协议：只发一个字母
            let cmd = match emotion {
                "neutral"  => "n",
                "happy"    => "h",
                "thinking" => "t",
                "listening"=> "t", // 暂时也用思考脸
                _          => "", 
            };

            if !cmd.is_empty() {
                if let Some(ref mut port) = serial_port {
                    // 发送数据
                    match port.write(cmd.as_bytes()) {
                        Ok(_) => {
                            let _ = port.flush(); // 强制发送
                            println!("   >>> Sent '{}' to RP2350", cmd);
                        },
                        Err(e) => println!("   ❌ Serial Error: {}", e),
                    }
                }
            }
        }
    });

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}