use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter, WriteType, Characteristic}; // 🟢 引入 Characteristic
use btleplug::platform::{Manager, Peripheral};
use std::error::Error;
use std::time::Duration;
use tokio::time;
use uuid::Uuid;

// 🟢 你的设备 UUID 配置
const SERVICE_UUID_STR: &str = "0000FFF0-0000-1000-8000-00805F9B34FB";
const CHAR_UUID_STR: &str    = "0000FFF2-0000-1000-8000-00805F9B34FB";

pub struct BluetoothManager {
    target_device: Option<Peripheral>,
    // 🟢 新增：存储找到的特征值对象，而不仅仅是 UUID
    write_char: Option<Characteristic>,
    target_char_uuid: Uuid,
}

impl BluetoothManager {
    pub fn new() -> Self {
        let target_char_uuid = Uuid::parse_str(CHAR_UUID_STR).expect("UUID 格式错误");
        Self { 
            target_device: None,
            write_char: None, // 初始化为空
            target_char_uuid,
        }
    }

    pub async fn connect(&mut self, target_mac_str: &str) -> Result<String, Box<dyn Error>> {
        let manager = Manager::new().await?;
        let adapters = manager.adapters().await?;
        let central = adapters.into_iter().nth(0).ok_or("❌ 未找到蓝牙适配器")?;

        println!("📡 开始扫描蓝牙设备 (5秒)...");
        central.start_scan(ScanFilter::default()).await?;
        time::sleep(Duration::from_secs(5)).await;

        let peripherals = central.peripherals().await?;
        let normalized_target = target_mac_str.replace(":", "").to_uppercase();

        for p in peripherals {
            let properties = p.properties().await?;
            let Some(props) = properties else { continue };
            let address_str = p.address().to_string().replace(":", "").to_uppercase();
            let name = props.local_name.unwrap_or_else(|| "Unknown".to_string());

            if address_str == normalized_target {
                println!("🔗 找到设备 [{}]，正在连接...", name);
                central.stop_scan().await?;
                p.connect().await?;
                
                println!("✅ 连接建立! 正在发现服务...");
                p.discover_services().await?;

                // 🟢 关键修复：遍历所有特征值，找到匹配 UUID 的那个对象
                let chars = p.characteristics();
                let matched_char = chars.into_iter().find(|c| c.uuid == self.target_char_uuid);

                if let Some(c) = matched_char {
                    println!("✅ 成功定位写入特征值: {:?}", c.uuid);
                    self.write_char = Some(c); // 保存对象
                } else {
                    return Err(format!("❌ 连接成功，但在设备上没找到特征值 UUID: {}", self.target_char_uuid).into());
                }

                self.target_device = Some(p);
                return Ok(format!("已连接到设备: {}", name));
            }
        }
        
        Err(format!("❌ 未扫描到对应 MAC 地址 的设备").into())
    }

    pub async fn send_command(&self, command: &str) -> Result<(), Box<dyn Error>> {
        // 🟢 关键修复：同时检查设备连接和特征值对象是否存在
        if let (Some(device), Some(characteristic)) = (&self.target_device, &self.write_char) {
            println!("📤 发送蓝牙指令: {}", command);
            let data = command.as_bytes(); 

            // 🟢 这里的第一个参数现在是 &Characteristic 类型了，修复了报错
            device.write(
                characteristic,
                data,
                WriteType::WithoutResponse,
            ).await?;
            
            Ok(())
        } else {
            Err("⚠️ 蓝牙设备未连接或特征值未找到".into())
        }
    }
}