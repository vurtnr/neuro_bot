// use futures::StreamExt;
// use r2r;
// use serialport;
// use std::time::Duration;

// // 引入自定义消息类型
// use r2r::robot_interfaces::msg::FaceEmotion;

// #[tokio::main]
// async fn main() -> Result<(), Box<dyn std::error::Error>> {
//     env_logger::init();
//     println!("🤡 Face Bridge (Connected to Brain) Starting...");

//     // =================================================================
//     // ⚠️ 关键修改点：请把下面的字符串替换为您在 Step 1 中复制的真实路径
//     // =================================================================
//     let port_name = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_42618f1794ff5844-if00";

//     let baud_rate = 115200;

//     println!("🔌 Attempting to connect to Serial: {}", port_name);

//     // 尝试连接串口
//     let mut serial_port = match serialport::new(port_name, baud_rate)
//         .timeout(Duration::from_millis(100))
//         .open()
//     {
//         Ok(mut port) => {
//             println!("✅ Hardware Connected! (RP2350 Ready)");
            
//             // 1. 设置 DTR (这通常会触发板子重启)
//             let _ = port.write_data_terminal_ready(true);
            
//             // --- ⚠️ 关键修改：增加 2 秒延时，等待板子重启完毕 ---
//             println!("⏳ Waiting 2s for RP2350 to boot...");
//             std::thread::sleep(Duration::from_secs(2)); 
//             // -----------------------------------------------

//             // 2. 发送 'n' 并强制刷新缓冲区
//             let _ = port.write(b"n");
//             let _ = port.flush(); // 👈 必加：确保数据立即发出去
            
//             println!("   >>> Sent Init 'n' to Screen");
//             Some(port)
//         }
//         Err(e) => {
//             println!("⚠️ CRITICAL WARNING: Serial connect failed: {}", e);
//             println!("   -> Is the path correct? Run 'ls /dev/serial/by-id/'");
//             println!("   -> Did you grant permission? Run 'sudo usermod -aG dialout $USER'");
//             println!("   (Running in Simulation Mode - No screen output)");
//             None
//         }
//     };

//     // 初始化 ROS 2 节点
//     let ctx = r2r::Context::create()?;
//     let mut node = r2r::Node::create(ctx, "face_bridge", "")?;

//     // 订阅 /robot/emotion 话题 (来自 Brain Core)
//     let mut emotion_sub =
//         node.subscribe::<FaceEmotion>("/robot/emotion", r2r::QosProfile::default())?;

//     // 异步任务：监听消息并转发
//     tokio::task::spawn(async move {
//         println!("👂 Listening for /robot/emotion...");

//         while let Some(msg) = emotion_sub.next().await {
//             let emotion_str = msg.emotion.as_str();
//             println!("⚡ [Received] Emotion: {}", emotion_str);

//             // 协议映射 (Rust -> Python)
//             // 'n'=Neutral, 'h'=Happy, 't'=Thinking, 'l'=Listening
//             let cmd = match emotion_str {
//                 "neutral" => "n",
//                 "happy" => "h",
//                 "thinking" => "t",
//                 "listening" => "l", // 现在我们有专门的监听指令了
//                 _ => "",
//             };

//             // 发送给 RP2350
//             if !cmd.is_empty() {
//                 if let Some(ref mut port) = serial_port {
//                     match port.write(cmd.as_bytes()) {
//                         Ok(_) => {
//                             let _ = port.flush();
//                             println!("   >>> Sent '{}' to Screen", cmd);
//                         }
//                         Err(e) => println!("   ❌ Serial Write Error: {}", e),
//                     }
//                 }
//             }
//         }
//     });

//     // 保持节点运行
//     loop {
//         node.spin_once(Duration::from_millis(100));
//     }
// }
use r2r;
use futures::StreamExt;
use std::time::Duration;
use std::io::Write; // 引入 Write trait，用于 write_all 和 flush

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("(O_O) Face Bridge Starting...");

    // === 1. 串口配置 ===
    // ⚠️ 请确认这个路径！Linux 通常是 /dev/ttyACM0
    let port_name = "/dev/ttyACM0"; 
    let baud_rate = 115200;

    println!("正在连接串口: {} ...", port_name);

    // 打开串口，并立即处理 Result，将其转换为 Option
    // 这样就避免了 "partially moved" 问题，因为我们生成了一个全新的变量 serial_option
    let mut serial_option = match serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(100))
        .open() 
    {
        Ok(mut port) => {
            println!("✅ 串口连接成功!");
            // 启动时先重置为 Neutral
            let _ = port.write_all(b"n\n"); 
            Some(port) // 将 port 所有权转移给 serial_option
        }
        Err(e) => {
            eprintln!("⚠️ 警告: 串口连接失败 ({})，将只运行 ROS 逻辑，不显示画面。", e);
            None // 连接失败，这里就是 None
        }
    };

    // === 2. ROS 节点配置 ===
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "face_bridge", "")?;

    // 订阅 emotion 话题
    let mut sub = node.subscribe::<r2r::robot_interfaces::msg::FaceEmotion>("/robot_face/emotion", r2r::QosProfile::default())?;

    println!("👂 Listening for emotion messages...");

    // 开启异步任务处理消息
    // async move 会把 serial_option 的所有权移入这个任务中
    tokio::task::spawn(async move {
        while let Some(msg) = sub.next().await {
            // 获取表情字符串
            let emotion_str = &msg.emotion; 
            
            println!("收到 ROS 指令: {}", emotion_str);

            // 协议映射 (Rust -> Python)
            let cmd = match emotion_str.as_str() {
                "neutral"   => "n\n",
                "happy"     => "h\n",
                "thinking"  => "t\n",
                "listening" => "l\n",
                _           => {
                    println!("未知表情，忽略");
                    ""
                }
            };

            // 发送给 Pico
            if !cmd.is_empty() {
                // 从 Option 中借用 mutable reference
                if let Some(ref mut port) = serial_option {
                    match port.write_all(cmd.as_bytes()) {
                        Ok(_) => {
                            let _ = port.flush(); // 确保数据发出去
                            println!("   -> 发送串口: {:?}", cmd.trim());
                        },
                        Err(e) => eprintln!("   ❌ 串口写入失败: {}", e),
                    }
                } else {
                    // 如果 serial_option 是 None (说明启动时没连上)
                    // 可以在这里决定是否要静默，或者打印提示
                    // eprintln!("(未连接串口，无法发送指令)");
                }
            }
        }
    });

    // 保持节点运行
    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
    }
}