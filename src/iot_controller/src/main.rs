mod modules;
use modules::bluetooth::BluetoothManager;
use r2r;
use modules::cellular::CellularManager;
use r2r::robot_interfaces::srv::ConnectBluetooth;
use r2r::robot_interfaces::msg::{NetworkStatus, BodyCommand};
use r2r::std_msgs::msg::String as StringMsg;
use modules::servo_serial::ServoSerialManager;
// use r2r::robot_interfaces::msg::BluetoothCommand; // ⚠️ 旧的 Topic 方式暂时屏蔽，因为 V1 协议强依赖 UUID
use futures::StreamExt;
use std::sync::{Arc};
use tokio::sync::Mutex;
use std::time::Duration;

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🤖 IoT Controller (Rust) Starting [Neural Link V1]...");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "iot_controller", "")?;

    // --- 1. 初始化 USB 总线舵机控制器 ---
        // 根据之前的测试，使用的是 /dev/ttyUSB0，波特率 115200
    let usb_device = "/dev/ttyUSB0";
    let servo_manager = match ServoSerialManager::new(usb_device, 115200) {
        Ok(mgr) => Arc::new(mgr),
        Err(e) => {
            eprintln!("⚠️ 警告: 无法连接 USB 舵机控制器 ({})", usb_device);
            eprintln!("   错误信息: {}", e);
            eprintln!("   请检查: 1.USB线连接 2.权限(sudo chmod 666 {})", usb_device);
            // 发生错误时 panic 提醒接线
            panic!("硬件连接失败，请检查 USB 连接！");
        }
    };

    // --- 2. 订阅身体控制指令 (Topic) ---
    // 监听来自 Brain Core 的 /iot/body_command
    let mut body_sub = node.subscribe::<BodyCommand>("/iot/body_command", r2r::QosProfile::default())?;

    // 启动一个异步任务处理舵机指令
    let sm_clone = servo_manager.clone();
    tokio::spawn(async move {
        println!("🦾 舵机指令监听器已启动...");
        while let Some(msg) = body_sub.next().await {
            println!("📥 收到动作指令: [{}] 参数: [{}]", msg.cmd, msg.params);
            match msg.cmd.as_str() {
                "WAVE" => {
                    sm_clone.action_wave().await;
                }
                "RESET" => {
                    sm_clone.reset().await;
                }
                "GIMBAL" => {
                    if let Ok(angle) = msg.params.parse::<i32>() {
                        sm_clone.set_gimbal(angle).await;
                    }
                }
                _ => println!("❓ 未知指令: {}", msg.cmd),
            }
        }
    });

    // --- (以下是原有的蓝牙和网络代码，保持不变) ---
    let bt_manager = Arc::new(Mutex::new(BluetoothManager::new()));
    let mut connect_service = node.create_service::<ConnectBluetooth::Service>(
        "/iot/connect_bluetooth",
        r2r::QosProfile::services_default(),
    )?;



    // 初始化蓝牙管理器
    let bt_manager = Arc::new(Mutex::new(BluetoothManager::new()));

    // 1. 创建 ROS 服务: 连接并执行
    // 对应 Brain Core 2.0 发来的请求
    let mut connect_service = node.create_service::<ConnectBluetooth::Service>(
        "/iot/connect_bluetooth",
        r2r::QosProfile::services_default(),
    )?;

    let tts_publisher =
        node.create_publisher::<StringMsg>("/audio/tts_play", r2r::QosProfile::default())?;


    let cellular_pub = node.create_publisher::<NetworkStatus>("/system/network_status", r2r::QosProfile::default())?;
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
        let _ = req.respond(ConnectBluetooth::Response {
            success,
            message: msg,
        });
    }



    spin_handle.await?;
    Ok(())
}
