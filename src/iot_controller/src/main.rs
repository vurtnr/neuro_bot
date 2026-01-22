mod modules;
use modules::bluetooth::BluetoothManager;
use r2r;
use r2r::robot_interfaces::srv::ConnectBluetooth;
// use r2r::robot_interfaces::msg::BluetoothCommand; // ⚠️ 旧的 Topic 方式暂时屏蔽，因为 V1 协议强依赖 UUID
use futures::StreamExt;
use std::sync::Arc;
use tokio::sync::Mutex;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🤖 IoT Controller (Rust) Starting [Neural Link V1]...");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "iot_controller", "")?;

    // 初始化蓝牙管理器
    let bt_manager = Arc::new(Mutex::new(BluetoothManager::new()));

    // 1. 创建 ROS 服务: 连接并执行
    // 对应 Brain Core 2.0 发来的请求
    let mut connect_service = node.create_service::<ConnectBluetooth::Service>("/iot/connect_bluetooth", r2r::QosProfile::default())?;
    
    // 2. 旧的 Topic 订阅暂时屏蔽 (如果代码中有用到 BluetoothCommand 的地方建议先注释掉)
    // let mut command_sub = node.subscribe::<BluetoothCommand>("/iot/bluetooth_command", r2r::QosProfile::default())?;

    println!("🔗 Bluetooth Service Ready. Waiting for Neural Link commands...");

    // ================================================================
    // 👂 任务 1: 处理连接请求 (Service)
    // ================================================================
    let bt_mgr_clone_1 = bt_manager.clone();
    tokio::spawn(async move {
        println!("✅ Service Listener Started.");
        while let Some(req) = connect_service.next().await {
            // 直接在 async 上下文中处理，不要嵌套 block_on
            let target_mac = req.message.mac.clone();
            let service_uuid = req.message.service_uuid.clone();
            let char_uuid = req.message.characteristic_uuid.clone();
            let cmd_hex = req.message.command.clone();

            println!("📥 收到指令: MAC={} CMD={}", target_mac, cmd_hex);

            // 在独立任务中执行蓝牙操作，使用超时防止永久阻塞
            let mgr = bt_mgr_clone_1.clone();
            let connect_result = tokio::time::timeout(
                std::time::Duration::from_secs(20),
                async move {
                    let mut mgr_guard = mgr.lock().await;
                    mgr_guard.connect_and_execute(&target_mac, &service_uuid, &char_uuid, &cmd_hex).await
                }
            ).await;

            let (success, msg) = match connect_result {
                Ok(Ok(info)) => (true, info),
                Ok(Err(e)) => (false, format!("Bluetooth error: {}", e)),
                Err(_) => (false, "Connection timeout (20s)".to_string()),
            };

            println!("🔄 执行结果: {} ({})", success, msg);

            // 回复结果
            let _ = req.respond(ConnectBluetooth::Response {
                success,
                message: msg,
            });
        }
    });

    // ================================================================
    // 👂 任务 2: 处理控制指令 (Topic) - 已弃用
    // ================================================================
    // 旧逻辑已不兼容 V1 协议（缺少 UUID），暂时注释以通过编译
    /*
    let bt_mgr_clone_2 = bt_manager.clone();
    tokio::spawn(async move {
        while let Some(msg) = command_sub.next().await {
            println!("⚠️ 忽略旧版 Topic 指令: {}", msg.command);
        }
    });
    */

    // ================================================================
    // 👂 任务 3: ROS Spin 循环 (独立任务)
    // ================================================================
    let node_for_spin = Arc::new(tokio::sync::Mutex::new(node));
    let node_spin = node_for_spin.clone();
    tokio::spawn(async move {
        loop {
            let mut node = node_spin.lock().await;
            node.spin_once(std::time::Duration::from_millis(20));
            drop(node);
            tokio::time::sleep(std::time::Duration::from_millis(5)).await;
        }
    });

    // 主任务保持活跃
    loop {
        tokio::time::sleep(std::time::Duration::from_secs(60)).await;
    }
}