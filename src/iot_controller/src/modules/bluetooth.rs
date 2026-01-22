use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter, WriteType, Characteristic};
use btleplug::platform::{Manager, Peripheral};
use std::error::Error;
use std::time::Duration;
use tokio::time;
use uuid::Uuid;

pub struct BluetoothManager {
    target_device: Option<Peripheral>,
    write_char: Option<Characteristic>,
}

impl BluetoothManager {
    pub fn new() -> Self {
        Self { 
            target_device: None,
            write_char: None, 
        }
    }

    /// 核心连接函数：支持动态 UUID 和 连接后即时发送指令
    pub async fn connect_and_execute(
        &mut self, 
        mac_str: &str, 
        service_uuid_str: &str,
        char_uuid_str: &str,
        command_hex: &str
    ) -> Result<String, Box<dyn Error>> {
        
        // 1. 解析传入的 UUID
        let target_service_uuid = Uuid::parse_str(service_uuid_str).map_err(|_| "Service UUID 格式错误")?;
        let target_char_uuid = Uuid::parse_str(char_uuid_str).map_err(|_| "Characteristic UUID 格式错误")?;

        let manager = Manager::new().await?;
        let adapters = manager.adapters().await?;
        let central = adapters.into_iter().nth(0).ok_or("❌ 未找到蓝牙适配器")?;

        // 2. 扫描设备
        println!("📡 扫描目标: {} (5s)...", mac_str);
        central.start_scan(ScanFilter::default()).await?;
        time::sleep(Duration::from_secs(5)).await; // 扫描 5 秒

        let peripherals = central.peripherals().await?;
        let normalized_target = mac_str.replace(":", "").to_uppercase();

        for p in peripherals {
            let address_str = p.address().to_string().replace(":", "").to_uppercase();
            
            if address_str == normalized_target {
                println!("🔗 找到设备，正在连接...");
                central.stop_scan().await?;
                p.connect().await?;
                
                println!("✅ 连接建立! 正在发现服务...");
                p.discover_services().await?;

                // 3. 动态寻找特征值
                let chars = p.characteristics();
                let matched_char = chars.into_iter().find(|c| c.uuid == target_char_uuid && c.service_uuid == target_service_uuid);

                if let Some(c) = matched_char {
                    println!("✅ 锁定特征值: {:?}", c.uuid);
                    self.write_char = Some(c.clone());
                    self.target_device = Some(p.clone());

                    // 4. 如果有指令，立即执行写入 (即连即发)
                    if !command_hex.is_empty() {
                        println!("⚡ 检测到即时指令，准备发送...");
                        self.send_hex_command(&p, &c, command_hex).await?;
                        return Ok(format!("已连接并发送指令: {}", command_hex));
                    }

                    return Ok("已连接 (无指令发送)".to_string());
                } else {
                    return Err(format!("❌ 未找到指定特征值: {}", char_uuid_str).into());
                }
            }
        }
        
        Err(format!("❌ 未扫描到设备: {}", mac_str).into())
    }

    // 内部辅助：发送 Hex 字符串
    async fn send_hex_command(&self, device: &Peripheral, characteristic: &Characteristic, hex_cmd: &str) -> Result<(), Box<dyn Error>> {
        let data = Self::hex_to_bytes(hex_cmd)?;
        println!("📤 发送 HEX: {:02X?}", data);
        device.write(characteristic, &data, WriteType::WithoutResponse).await?;
        Ok(())
    }

    // 简单的 Hex 转 Bytes 工具 (避免引入额外 crate)
    fn hex_to_bytes(hex: &str) -> Result<Vec<u8>, Box<dyn Error>> {
        if hex.len() % 2 != 0 {
            return Err("Hex 字符串长度必须为偶数".into());
        }
        (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).map_err(|e| e.into()))
            .collect()
    }
}