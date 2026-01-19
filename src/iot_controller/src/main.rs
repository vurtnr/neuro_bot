mod modules;
use modules::bluetooth::BluetoothManager; // <--- 引入模块
use r2r;
use r2r::robot_interfaces::srv::ConnectBluetooth;
use r2r::robot_interfaces::msg::BluetoothCommand;
use futures::StreamExt;
use std::sync::{Arc};
use tokio::sync::Mutex; // 使用 Tokio 的异步 Mutex

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🤖 IoT Controller (Rust) Starting...");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "iot_controller", "")?;

    // 🟢 初始化蓝牙管理器
    // 使用 Arc<Mutex<...>> 使得它可以在多个异步任务中共享
    let bt_manager = Arc::new(Mutex::new(BluetoothManager::new()));

    // 1. 创建 ROS 服务: 连接请求
    // 对应 Brain Core 发来的 "ConnectBluetooth" (带 MAC 地址)
    let mut connect_service = node.create_service::<ConnectBluetooth::Service>("/iot/connect_bluetooth", r2r::QosProfile::default())?;
    
    // 2. 创建 ROS 订阅: 控制指令
    // 对应 Brain Core 发来的 "BluetoothCommand" (带指令内容)
    let mut command_sub = node.subscribe::<BluetoothCommand>("/iot/bluetooth_command", r2r::QosProfile::default())?;

    println!("🔗 Bluetooth Service & Command Link Ready.");

    // ================================================================
    // 👂 任务 1: 处理连接请求 (Service)
    // ================================================================
    let bt_mgr_clone_1 = bt_manager.clone();
    tokio::spawn(async move {
        println!("✅ Service Listener Started.");
        while let Some(req) = connect_service.next().await {
            let mut mgr = bt_mgr_clone_1.lock().await; // 锁住管理器
            println!("📥 收到连接请求，目标 MAC: {}", req.message.address);
            
            // 调用蓝牙连接逻辑
            let result = mgr.connect(&req.message.address).await;
            
            let (success, msg) = match result {
                Ok(info) => (true, info),
                Err(e) => (false, e.to_string()),
            };
            
            println!("🔄 连接结果: {} ({})", success, msg);

            // 回复 Brain Core
            let _ = req.respond(ConnectBluetooth::Response {
                success,
                message: msg,
            });
        }
    });

    // ================================================================
    // 👂 任务 2: 处理控制指令 (Topic)
    // ================================================================
    let bt_mgr_clone_2 = bt_manager.clone();
    tokio::spawn(async move {
        println!("✅ Command Listener Started.");
        while let Some(msg) = command_sub.next().await {
            let mut mgr = bt_mgr_clone_2.lock().await;
            println!("📥 收到指令请求: cmd='{}', mac='{}'", msg.command, msg.mac); // 打印 mac 方便调试

            // 🟢 [核心逻辑修复] 根据指令内容分流
            if msg.command == "connect" {
                println!("🔗 发起连接请求 -> {}", msg.mac);
                // 调用连接逻辑 (复用 Service 的逻辑)
                let _ = mgr.connect(&msg.mac).await; 
                // 注意：这里 connect 可能需要处理 Result，简单起见先忽略返回值，或者打印日志
            } else {
                // 其他指令 (如 LED控制等) 走 send_command
                if let Err(e) = mgr.send_command(&msg.command).await {
                    eprintln!("🔥 指令发送失败: {}", e);
                }
            }
        }
    });

    // 主循环
    loop {
        node.spin_once(std::time::Duration::from_millis(100));
    }
}