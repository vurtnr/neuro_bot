use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter, WriteType, Characteristic, CharPropFlags};
use btleplug::platform::{Manager, Peripheral};
use futures::StreamExt;
use serde::Serialize;
use std::error::Error;
use std::path::Path;
use std::time::Duration;
use tokio::time;
use uuid::Uuid;

pub struct BleExecutionResult {
    pub message: String,
    pub tts: Option<String>,
}

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

    /// 核心连接函数：支持动态 UUID 或 自动发现可写特征值
    pub async fn connect_and_execute(
        &mut self, 
        mac_str: &str, 
        service_uuid_str: &str,
        char_uuid_str: &str,
        command_hex: &str
    ) -> Result<BleExecutionResult, Box<dyn Error>> {
        // 1. 解析传入的 UUID (空/占位符则视为自动发现)
        let target_service_uuid = normalize_uuid_input(service_uuid_str)
            .map(|value| Uuid::parse_str(value).map_err(|_| "Service UUID 格式错误"))
            .transpose()?;

        let target_char_uuid = normalize_uuid_input(char_uuid_str)
            .map(|value| Uuid::parse_str(value).map_err(|_| "Characteristic UUID 格式错误"))
            .transpose()?;

        let mut command_hex = normalize_command_input(command_hex).to_string();

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
                
                // --- 核心修改：匹配逻辑升级 ---
                // 寻找满足条件的特征值：
                // A. 如果指定了 UUID，必须完全匹配
                // B. 如果没指定 UUID，寻找第一个"可写"的特征值
                let matched_char = chars.iter().find(|c| {
                    match (target_service_uuid, target_char_uuid) {
                        (Some(s_uuid), Some(c_uuid)) => {
                            c.uuid == c_uuid && c.service_uuid == s_uuid
                        },
                        _ => {
                            // 自动模式：只要能写就行
                            c.properties.contains(CharPropFlags::WRITE) || 
                            c.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE)
                        }
                    }
                }).cloned();

                if let Some(c) = matched_char {
                    println!("✅ 锁定特征值: {:?} (Service: {:?})", c.uuid, c.service_uuid);
                    println!("   属性: {:?}", c.properties);

                    self.write_char = Some(c.clone());
                    self.target_device = Some(p.clone());

                    if command_hex.is_empty() {
                        let tcu = self.resolve_tcu_from_advertisement(&p).await?;
                        if let Err(e) = persist_device_info(mac_str, tcu) {
                            eprintln!("⚠️ 持久化设备信息失败: {}", e);
                        } else {
                            println!("💾 已保存设备信息: MAC={}, TCU={}", mac_str, tcu);
                        }
                        command_hex = build_query_command(tcu);
                        println!("🧩 生成查询指令: {}", command_hex);
                    }

                    let command_bytes = if command_hex.is_empty() {
                        None
                    } else {
                        Some(Self::hex_to_bytes(&command_hex)?)
                    };
                    let expects_response = command_bytes
                        .as_ref()
                        .map(|bytes| bytes.len() >= 2 && bytes[1] == 0x03)
                        .unwrap_or(false);

                    let notify_uuid = Uuid::parse_str("0000FFF1-0000-1000-8000-00805F9B34FB")?;
                    let notify_char = chars.iter().find(|c| c.uuid == notify_uuid).cloned();
                    let mut notifications = None;

                    if expects_response {
                        let notify_char = notify_char.ok_or("❌ 未找到通知特征值")?;
                        if !(notify_char.properties.contains(CharPropFlags::NOTIFY)
                            || notify_char.properties.contains(CharPropFlags::INDICATE))
                        {
                            return Err("❌ 通知特征值不支持通知".into());
                        }
                        p.subscribe(&notify_char).await?;
                        println!("✅ 订阅通知特征值: {:?}", notify_char.uuid);
                        notifications = Some(p.notifications().await?);
                    }

                    // 4. 如果有指令，立即执行写入 (即连即发)
                    if !command_hex.is_empty() {
                        println!("⚡ 检测到即时指令，准备发送...");
                        self.send_hex_command(&p, &c, &command_hex).await?;

                        let mut tts = None;
                        if expects_response {
                            let notify_uuid = notify_uuid;
                            let mut stream = notifications.ok_or("❌ 未初始化通知流")?;
                            let deadline = time::Instant::now() + Duration::from_secs(5);
                            loop {
                                let remaining = deadline.saturating_duration_since(time::Instant::now());
                                if remaining.is_zero() {
                                    return Err("❌ 未收到通知".into());
                                }
                                let next = time::timeout(remaining, stream.next()).await;
                                let notification = match next {
                                    Ok(Some(value)) => value,
                                    Ok(None) => return Err("❌ 通知流结束".into()),
                                    Err(_) => return Err("❌ 未收到通知".into()),
                                };
                                if notification.uuid != notify_uuid {
                                    continue;
                                }
                                println!("📥 收到通知: {:02X?}", notification.value);
                                let parsed = parse_response_payload(&notification.value)?;
                                let tts_text = build_tts(&parsed);
                                println!("🗣️ TTS: {}", tts_text);
                                tts = Some(tts_text);
                                break;
                            }
                        }

                        return Ok(BleExecutionResult {
                            message: format!("已连接并发送指令: {}", command_hex),
                            tts,
                        });
                    }

                    return Ok(BleExecutionResult {
                        message: "已连接 (无指令发送)".to_string(),
                        tts: None,
                    });
                } else {
                    return Err(format!("❌ 未找到合适的可写特征值 (UUID 指定: {:?})", char_uuid_str).into());
                }
            }
        }
        
        Err(format!("❌ 未扫描到设备: {}", mac_str).into())
    }

    async fn resolve_tcu_from_advertisement(
        &self,
        peripheral: &Peripheral,
    ) -> Result<u8, Box<dyn Error>> {
        let props = peripheral
            .properties()
            .await?
            .ok_or("❌ 未获取到广播信息")?;

        let protocol = extract_protocol_from_manufacturer_data(&props.manufacturer_data)
            .ok_or("❌ 未找到厂商广播数据")?;

        let tcu = parse_tcu_from_protocol(&protocol)?;
        println!("🧩 解析 TCU 地址: {}", tcu);
        Ok(tcu)
    }

    // 内部辅助：发送 Hex 字符串
    async fn send_hex_command(&self, device: &Peripheral, characteristic: &Characteristic, hex_cmd: &str) -> Result<(), Box<dyn Error>> {
        let data = Self::hex_to_bytes(hex_cmd)?;
        println!("📤 发送 HEX: {:02X?}", data);
        
        // --- 核心修改：根据特征值属性自动选择写入方式 ---
        let write_type = if characteristic.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE) {
            WriteType::WithoutResponse
        } else {
            WriteType::WithResponse
        };

        device.write(characteristic, &data, write_type).await?;
        Ok(())
    }

    // 简单的 Hex 转 Bytes 工具
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

fn crc16_modbus(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for b in data {
        crc ^= *b as u16;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

fn build_query_command(tcu_address: u8) -> String {
    let mut payload = vec![tcu_address, 0x03, 0x00, 0x00, 0x00, 0x25];
    let crc = crc16_modbus(&payload);
    payload.push((crc & 0xFF) as u8);
    payload.push((crc >> 8) as u8);
    payload
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<String>()
}

fn verify_protocol_checksum(protocol: &[u8]) -> bool {
    if protocol.len() != 26 {
        return false;
    }
    let checksum = protocol[2] as u16;
    let sum: u16 = protocol[3..].iter().map(|b| *b as u16).sum();
    let low = sum & 0xFF;
    let high = (sum >> 8) & 0xFF;
    let result = (low + high) & 0xFF;
    checksum == result
}

fn key_from_rand(rand0: u8, rand1: u8) -> [u8; 7] {
    let mut key = [0u8; 7];
    key[0] = rand0.wrapping_add(rand1);
    key[1] = rand0 ^ rand1;
    key[2] = rand0 ^ 0x69;
    key[3] = key[1];
    key[4] = rand0 ^ 0x16;
    key[5] = rand1 ^ 0x58;
    key[6] = rand0 ^ rand1 ^ 0x69;
    key
}

fn parse_tcu_from_protocol(protocol: &[u8]) -> Result<u8, Box<dyn Error>> {
    if protocol.len() != 26 {
        return Err("❌ 广播数据长度错误".into());
    }
    if protocol[0] != 0x88 || protocol[1] != 0x11 {
        return Err("❌ 广播头不匹配".into());
    }
    if !verify_protocol_checksum(protocol) {
        return Err("❌ 广播校验失败".into());
    }

    let rand0 = protocol[3];
    let rand1 = protocol[4];
    let key = key_from_rand(rand0, rand1);
    let encrypted = &protocol[5..26];

    let mut decrypted = [0u8; 21];
    for i in 0..21 {
        decrypted[i] = encrypted[i] ^ key[i % 7];
    }

    let tcu = decrypted[16];
    if tcu == 0 || tcu > 150 {
        return Err("❌ TCU 地址非法".into());
    }

    Ok(tcu)
}

fn extract_protocol_from_manufacturer_data(
    manufacturer_data: &std::collections::HashMap<u16, Vec<u8>>,
) -> Option<Vec<u8>> {
    if let Some(value) = manufacturer_data.get(&0x1188) {
        if value.len() >= 26 && value[0] == 0x88 && value[1] == 0x11 {
            return Some(value[..26].to_vec());
        }
        if value.len() == 24 {
            let mut protocol = Vec::with_capacity(26);
            protocol.push(0x88);
            protocol.push(0x11);
            protocol.extend_from_slice(value);
            return Some(protocol);
        }
    }

    for value in manufacturer_data.values() {
        if value.len() >= 26 && value[0] == 0x88 && value[1] == 0x11 {
            return Some(value[..26].to_vec());
        }
    }

    None
}

#[derive(Serialize)]
struct PersistedDeviceInfo {
    mac: String,
    tcu: u8,
}

fn persist_device_info(mac: &str, tcu: u8) -> Result<(), Box<dyn Error>> {
    let home = std::env::var("HOME").unwrap_or_else(|_| "/home/pi".to_string());
    let path = Path::new(&home).join("data/ble_devices.json");
    persist_device_info_to_path(&path, mac, tcu)
}

fn persist_device_info_to_path(path: &Path, mac: &str, tcu: u8) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }

    let info = PersistedDeviceInfo {
        mac: mac.to_string(),
        tcu,
    };
    let json = serde_json::to_string(&info)?;
    let tmp_path = path.with_extension("json.tmp");
    std::fs::write(&tmp_path, json)?;
    std::fs::rename(tmp_path, path)?;
    Ok(())
}

struct ParsedResponse {
    tcu_address: u8,
    work_mode: u16,
    fault_code: u16,
    target_angle: f32,
    actual_angle: f32,
    longitude: f32,
    latitude: f32,
    timezone: i8,
}

fn verify_response_crc(payload: &[u8]) -> bool {
    if payload.len() < 4 {
        return false;
    }
    let crc_index = payload.len() - 2;
    let expected = crc16_modbus(&payload[..crc_index]);
    let got = (payload[crc_index] as u16) | ((payload[crc_index + 1] as u16) << 8);
    expected == got
}

fn parse_i16_be(bytes: &[u8]) -> i16 {
    i16::from_be_bytes([bytes[0], bytes[1]])
}

fn parse_u16_be(bytes: &[u8]) -> u16 {
    u16::from_be_bytes([bytes[0], bytes[1]])
}

fn parse_response_payload(payload: &[u8]) -> Result<ParsedResponse, Box<dyn Error>> {
    if payload.len() != 62 && payload.len() != 79 {
        return Err("❌ 响应长度非法".into());
    }
    if !verify_response_crc(payload) {
        return Err("❌ 响应 CRC 校验失败".into());
    }

    let tcu_address = payload[0];
    let work_mode = parse_u16_be(&payload[1..3]);
    let fault_code = parse_u16_be(&payload[3..5]);
    let target_angle = parse_i16_be(&payload[9..11]) as f32 / 10.0;
    let actual_angle = parse_i16_be(&payload[11..13]) as f32 / 10.0;
    let longitude = parse_i16_be(&payload[20..22]) as f32 / 100.0;
    let latitude = parse_i16_be(&payload[22..24]) as f32 / 100.0;
    let timezone = payload[24] as i8;

    Ok(ParsedResponse {
        tcu_address,
        work_mode,
        fault_code,
        target_angle,
        actual_angle,
        longitude,
        latitude,
        timezone,
    })
}

fn fault_code_to_text(code: u16) -> String {
    let mut parts = Vec::new();
    if code & (1 << 0) != 0 {
        parts.push("主从倾角差异");
    }
    if code & (1 << 2) != 0 {
        parts.push("电机损坏");
    }
    if code & (1 << 3) != 0 {
        parts.push("倾角故障");
    }
    if code & (1 << 4) != 0 {
        parts.push("电机过流");
    }
    if code & (1 << 5) != 0 {
        parts.push("东限角警报");
    }
    if code & (1 << 6) != 0 {
        parts.push("西限角警报");
    }
    if code & (1 << 7) != 0 {
        parts.push("RTC故障");
    }
    if code & (1 << 8) != 0 {
        parts.push("电量有限警报");
    }
    if code & (1 << 9) != 0 {
        parts.push("低电量警报");
    }
    if code & (1 << 10) != 0 {
        parts.push("开关电源损坏");
    }
    if code & (1 << 14) != 0 {
        parts.push("无线模块故障");
    }
    if code & (1 << 15) != 0 {
        parts.push("通信故障");
    }

    if parts.is_empty() {
        "无故障".to_string()
    } else {
        parts.join("、")
    }
}

fn build_tts(parsed: &ParsedResponse) -> String {
    let faults = fault_code_to_text(parsed.fault_code);
    format!(
        "目标角度 {:.1} 度，实际角度 {:.1} 度，经度 {:.2}，纬度 {:.2}，时区 {}，工作模式 0x{:04X}，故障：{}。",
        parsed.target_angle,
        parsed.actual_angle,
        parsed.longitude,
        parsed.latitude,
        parsed.timezone,
        parsed.work_mode,
        faults
    )
}

fn normalize_uuid_input(value: &str) -> Option<&str> {
    let trimmed = value.trim();
    if trimmed.is_empty()
        || trimmed.eq_ignore_ascii_case("auto")
        || trimmed == "00000000-0000-0000-0000-000000000000"
    {
        None
    } else {
        Some(trimmed)
    }
}

fn normalize_command_input(value: &str) -> &str {
    let trimmed = value.trim();
    if trimmed.is_empty()
        || trimmed.eq_ignore_ascii_case("noop")
        || trimmed.eq_ignore_ascii_case("none")
        || trimmed.eq_ignore_ascii_case("auto")
    {
        ""
    } else {
        trimmed
    }
}

#[cfg(test)]
mod tests {
    use super::{
        build_query_command, extract_protocol_from_manufacturer_data, fault_code_to_text,
        parse_response_payload, parse_tcu_from_protocol, persist_device_info_to_path,
        verify_protocol_checksum, verify_response_crc,
    };
    use std::collections::HashMap;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn valid_protocol_sample() -> [u8; 26] {
        // Precomputed valid protocol (rand0=0x12, rand1=0x34, tcu=0x0A).
        [
            0x88, 0x11, 0x20, 0x12, 0x34, 0x15, 0x68, 0x4B, 0x17, 0x36,
            0x5F, 0x7B, 0x73, 0x10, 0x4C, 0x1E, 0x3D, 0x2D, 0x0D, 0x05,
            0x27, 0x71, 0x26, 0x04, 0x6C, 0x4F,
        ]
    }

    fn invalid_checksum_sample() -> [u8; 26] {
        // Example from docs (checksum intentionally invalid)
        [
            0x88, 0x11, 0xA7, 0x12, 0x34, 0xE2, 0xC7, 0x83, 0xD7, 0xF0,
            0x9C, 0x8D, 0xE8, 0xC5, 0x81, 0xD5, 0xF2, 0x9E, 0x8F, 0xE9,
            0xC7, 0x83, 0xD6, 0x73, 0x06, 0x66,
        ]
    }

    fn manufacturer_value_without_header() -> [u8; 24] {
        [
            0xA7, 0x12, 0x34, 0xE2, 0xC7, 0x83, 0xD7, 0xF0, 0x9C, 0x8D,
            0xE8, 0xC5, 0x81, 0xD5, 0xF2, 0x9E, 0x8F, 0xE9, 0xC7, 0x83,
            0xD6, 0x73, 0x06, 0x66,
        ]
    }

    fn sample_response_payload() -> Vec<u8> {
        let mut payload = vec![0u8; 62];
        payload[0] = 0x37; // TCU address
        payload[1..3].copy_from_slice(&[0x00, 0x20]); // work mode
        payload[3..5].copy_from_slice(&[0x00, 0x10]); // fault code
        payload[9..11].copy_from_slice(&[0x00, 0x7B]); // target angle 12.3
        payload[11..13].copy_from_slice(&[0x00, 0x79]); // actual angle 12.1
        payload[20..22].copy_from_slice(&[0x2D, 0x76]); // longitude 116.38
        payload[22..24].copy_from_slice(&[0x0F, 0x96]); // latitude 39.90
        payload[24] = 8; // timezone

        let crc = crc16_modbus(&payload[..payload.len() - 2]);
        let len = payload.len();
        payload[len - 2] = (crc & 0xFF) as u8;
        payload[len - 1] = (crc >> 8) as u8;
        payload
    }

    fn temp_path() -> PathBuf {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let mut dir = std::env::temp_dir();
        dir.push(format!("ble_devices_test_{}_{}", std::process::id(), nanos));
        dir
    }

    #[test]
    fn checksum_rejects_invalid_data() {
        let sample = invalid_checksum_sample();
        assert!(!verify_protocol_checksum(&sample));
    }

    #[test]
    fn tcu_parsed_from_protocol() {
        let sample = valid_protocol_sample();
        let tcu = parse_tcu_from_protocol(&sample).expect("tcu parse failed");
        assert_eq!(tcu, 0x0A);
    }

    #[test]
    fn query_command_uses_crc16() {
        let cmd = build_query_command(0x0A);
        assert_eq!(cmd, "0A0300000025856A");
    }

    #[test]
    fn manufacturer_data_with_company_id_prefixes_header() {
        let mut data = HashMap::new();
        let value = manufacturer_value_without_header();
        data.insert(0x1188, value.to_vec());
        let protocol =
            extract_protocol_from_manufacturer_data(&data).expect("protocol not found");
        assert_eq!(protocol.len(), 26);
        assert_eq!(&protocol[..2], &[0x88, 0x11]);
        assert_eq!(&protocol[2..], value.as_slice());
    }

    #[test]
    fn manufacturer_data_with_header_value_is_used() {
        let mut data = HashMap::new();
        let value = invalid_checksum_sample().to_vec();
        data.insert(0x1188, value.clone());
        let protocol =
            extract_protocol_from_manufacturer_data(&data).expect("protocol not found");
        assert_eq!(protocol, value);
    }

    #[test]
    fn response_crc_rejects_invalid_data() {
        let mut payload = sample_response_payload();
        let len = payload.len();
        payload[len - 1] = 0x00;
        assert!(!verify_response_crc(&payload));
    }

    #[test]
    fn response_parses_required_fields() {
        let payload = sample_response_payload();
        let parsed = parse_response_payload(&payload).expect("parse failed");
        assert_eq!(parsed.tcu_address, 0x37);
        assert_eq!(parsed.work_mode, 0x0020);
        assert_eq!(parsed.fault_code, 0x0010);
        assert!((parsed.target_angle - 12.3).abs() < 0.01);
        assert!((parsed.actual_angle - 12.1).abs() < 0.01);
        assert!((parsed.longitude - 116.38).abs() < 0.01);
        assert!((parsed.latitude - 39.90).abs() < 0.01);
        assert_eq!(parsed.timezone, 8);
    }

    #[test]
    fn fault_code_to_text_maps_bits() {
        let text = fault_code_to_text(0x4010);
        assert!(text.contains("电机过流"));
        assert!(text.contains("无线模块故障"));
    }

    #[test]
    fn persist_device_info_writes_json() {
        let dir = temp_path();
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("ble_devices.json");
        persist_device_info_to_path(&path, "D6:65:62:00:2A:7E", 55).unwrap();
        let content = std::fs::read_to_string(&path).unwrap();
        let value: serde_json::Value = serde_json::from_str(&content).unwrap();
        assert_eq!(value["mac"], "D6:65:62:00:2A:7E");
        assert_eq!(value["tcu"], 55);
    }
}
