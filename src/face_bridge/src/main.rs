use r2r;
use futures::StreamExt;
use std::time::Duration;
use serialport;
use std::io::Read; // 引入 Read trait
use r2r::robot_interfaces::msg::FaceEmotion;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("😳 Face Bridge (Debug Mode) Starting...");

    // 1. 硬件连接
    let port_name = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_42618f1794ff5844-if00";
    let baud_rate = 115200;
    
    let serial_port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(100))
        .open();

    // 用于发送的端口变量
    let mut tx_port = match serial_port {
        Ok(mut port) => {
            println!("✅ Serial Connected!");
            let _ = port.write_data_terminal_ready(true);
            Some(port)
        },
        Err(ref e) => {
            println!("⚠️ Serial Connection Failed: {}", e);
            None
        }
    };

    // --- 🔍 新增：监听线程 (看看板子在说什么) ---
    if let Some(ref mut port) = tx_port {
        // 克隆一个端口句柄用于读取
        if let Ok(mut rx_port) = port.try_clone() {
            std::thread::spawn(move || {
                let mut buffer: Vec<u8> = vec![0; 1024];
                println!("👂 Serial Monitor Started (Listening to RP2350)...");
                loop {
                    match rx_port.read(buffer.as_mut_slice()) {
                        Ok(t) if t > 0 => {
                            // 将收到的字节转为字符串并打印
                            let s = String::from_utf8_lossy(&buffer[..t]);
                            // 使用黄色打印，突出显示板子的回复
                            print!("\x1b[33m{}\x1b[0m", s); 
                        },
                        Ok(_) => {},
                        Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {},
                        Err(e) => println!("Read Error: {}", e),
                    }
                    std::thread::sleep(Duration::from_millis(10));
                }
            });
        }
    }
    // ------------------------------------------

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "face_bridge", "")?;
    let mut emotion_sub = node.subscribe::<FaceEmotion>("/robot/face_emotion", r2r::QosProfile::default())?;

    tokio::task::spawn(async move {
        println!("👂 Waiting for ROS emotions...");
        while let Some(msg) = emotion_sub.next().await {
            let emotion_str = msg.emotion.as_str();
            
            // 尝试 \r\n (回车+换行)，兼容性更好
            let cmd = match emotion_str {
                "neutral"   => "n\r\n", 
                "happy"     => "h\r\n", 
                "speaking"  => "h\r\n",
                "thinking"  => "t\r\n", 
                "listening" => "l\r\n", 
                _           => "", 
            };

            if !cmd.is_empty() {
                if let Some(ref mut port) = tx_port {
                    print!("⚡ Sending: {:?} -> ", cmd.trim()); // 打印发送内容
                    if let Err(e) = port.write(cmd.as_bytes()) {
                         println!("❌ Write Error: {}", e);
                    } else {
                         let _ = port.flush();
                         println!("✅ Sent");
                    }
                }
            }
        }
    });

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}