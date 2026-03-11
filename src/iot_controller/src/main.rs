mod modules;
use modules::cellular::CellularManager;
use modules::bluetooth::BluetoothManager;
use modules::servo_serial::ServoSerialManager;
use futures::StreamExt;
use r2r;
use r2r::robot_interfaces::msg::{BodyCommand, NetworkStatus};
use r2r::robot_interfaces::srv::ConnectBluetooth;
use r2r::std_msgs::msg::String as StringMsg;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;

fn init_servo_manager(device: &str, baud_rate: u32) -> Option<Arc<ServoSerialManager>> {
    match ServoSerialManager::new(device, baud_rate) {
        Ok(mgr) => Some(Arc::new(mgr)),
        Err(e) => {
            eprintln!("⚠️ 警告: 无法连接 USB 舵机控制器 ({})", device);
            eprintln!("   错误信息: {}", e);
            eprintln!("   请检查: 1.USB线连接 2.权限(sudo chmod 666 {})", device);
            eprintln!("   当前继续以 BLE-only 模式启动 iot_controller。");
            None
        }
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🤖 IoT Controller (Rust) Starting [Neural Link V1]...");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "iot_controller", "")?;

    // --- 1. 初始化 USB 总线舵机控制器 ---
    // 根据之前的测试，使用的是 /dev/ttyUSB0，波特率 115200
    let usb_device = "/dev/ttyUSB0";
    let servo_manager = init_servo_manager(usb_device, 115200);

    // --- 2. 订阅身体控制指令 (Topic) ---
    // 监听来自 Brain Core 的 /iot/body_command
    let mut body_sub =
        node.subscribe::<BodyCommand>("/iot/body_command", r2r::QosProfile::default())?;

    // 启动一个异步任务处理舵机指令
    let sm_clone = servo_manager.clone();
    tokio::spawn(async move {
        println!("🦾 舵机指令监听器已启动...");
        while let Some(msg) = body_sub.next().await {
            println!("📥 收到动作指令: [{}] 参数: [{}]", msg.cmd, msg.params);
            if let Some(servo_manager) = sm_clone.as_ref() {
                match msg.cmd.as_str() {
                    "WAVE" => {
                        servo_manager.action_wave().await;
                    }
                    "RESET" => {
                        servo_manager.reset().await;
                    }
                    "GIMBAL" => {
                        if let Ok(angle) = msg.params.parse::<i32>() {
                            servo_manager.set_gimbal(angle).await;
                        }
                    }
                    _ => println!("❓ 未知指令: {}", msg.cmd),
                }
            } else {
                println!("⚠️ 舵机硬件未就绪，忽略动作指令: {}", msg.cmd);
            }
        }
    });

    // --- (以下是原有的蓝牙和网络代码，保持不变) ---
    let bt_manager = Arc::new(Mutex::new(BluetoothManager::new()));
    let mut connect_service = node.create_service::<ConnectBluetooth::Service>(
        "/iot/connect_bluetooth",
        r2r::QosProfile::services_default(),
    )?;

    let tts_publisher =
        node.create_publisher::<StringMsg>("/audio/tts_play", r2r::QosProfile::default())?;
    let cellular_pub = node
        .create_publisher::<NetworkStatus>("/system/network_status", r2r::QosProfile::default())?;
    let cellular_manager = CellularManager::new();

    // 放入后台任务运行 (这样不会阻塞蓝牙)
    tokio::spawn(async move {
        cellular_manager.run(cellular_pub).await;
    });
    // ==========================================

    println!("🔗 Bluetooth Service Ready...");

    // 2. 旧的 Topic 订阅暂时屏蔽 (如果代码中有用到 BluetoothCommand 的地方建议先注释掉)
    // let mut command_sub = node.subscribe::<BluetoothCommand>("/iot/bluetooth_command", r2r::QosProfile::default())?;

    println!("🔗 Bluetooth Service Ready. Waiting for Neural Link commands...");

    println!("✅ Service Listener Started.");

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

    let spin_handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(Duration::from_millis(100));
    });

    while let Some(req) = connect_service.next().await {
        let mut mgr = bt_manager.lock().await;

        // 🟢 [Fix 1] 适配新字段: 从 req.message 中获取 mac, service_uuid, characteristic_uuid, command
        let target_mac = &req.message.mac;
        let service_uuid = &req.message.service_uuid;
        let char_uuid = &req.message.characteristic_uuid;
        let cmd_hex = &req.message.command;

        println!("📥 收到指令: MAC={} CMD={}", target_mac, cmd_hex);

        // 🟢 [Fix 2] 调用新的通用执行方法 connect_and_execute
        let result = mgr
            .connect_and_execute(target_mac, service_uuid, char_uuid, cmd_hex)
            .await;

        let (success, msg) = match result {
            Ok(info) => {
                if let Some(tts) = info.tts {
                    let _ = tts_publisher.publish(&StringMsg { data: tts });
                }
                (true, info.message)
            }
            Err(e) => (false, e.to_string()),
        };

        println!("🔄 执行结果: {} ({})", success, msg);

        // 回复结果
        let _ = req.respond(ConnectBluetooth::Response { success, message: msg });
    }

    spin_handle.await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::init_servo_manager;

    #[test]
    fn init_servo_manager_returns_none_when_device_is_missing() {
        let manager = init_servo_manager("/definitely/missing/servo-device", 115200);

        assert!(manager.is_none());
    }
}
