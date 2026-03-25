use btleplug::api::{
    Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::StreamExt;
use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fmt;
use std::path::{Path, PathBuf};
use std::time::Duration;
use tokio::{process::Command, time};
use uuid::Uuid;

const AUTO_MODE_TARGET_WORK_MODE: u16 = 0x0140;
const MANUAL_DELTA_DEFAULT: i32 = 10;
const MANUAL_DELTA_MIN: i32 = 0;
const MANUAL_DELTA_MAX: i32 = 90;
const MANUAL_TARGET_MIN: i32 = -128;
const MANUAL_TARGET_MAX: i32 = 127;
const NOTIFY_CHARACTERISTIC_UUID: &str = "0000FFF1-0000-1000-8000-00805F9B34FB";
const MANUAL_VERIFICATION_MIN_DELTA: f32 = 0.5;
const MANUAL_VERIFICATION_TARGET_TOLERANCE: f32 = 1.5;
const MANUAL_VERIFICATION_MAX_ATTEMPTS: usize = 3;
const MANUAL_VERIFICATION_DELAY_MS: u64 = 1200;
const BLE_SCAN_ATTEMPTS: usize = 3;
const BLE_SCAN_WINDOW_SECS: u64 = 5;
const BLE_CONNECT_SETTLE_MS: u64 = 900;
const BLE_CONNECT_RETRY_DELAY_MS: u64 = 500;
const BLE_LOCAL_ABORT_RETRY_DELAY_MS: u64 = 1500;
const BLEAK_HELPER_TIMEOUT_SECS: u64 = 12;
const BLEAK_NOTIFICATION_TIMEOUT_MS: u64 = 5000;
const VENDOR_SERVICE_UUID: &str = "0000FFF0-0000-1000-8000-00805F9B34FB";
const VENDOR_NOTIFY_UUID: &str = "0000FFF1-0000-1000-8000-00805F9B34FB";
const VENDOR_WRITE_PRIMARY_UUID: &str = "0000FFF2-0000-1000-8000-00805F9B34FB";
const VENDOR_WRITE_SECONDARY_UUID: &str = "0000FFF3-0000-1000-8000-00805F9B34FB";

pub struct BleExecutionResult {
    pub message: String,
    pub tts: Option<String>,
    pub actual_angle: Option<f32>,
    pub target_angle: Option<f32>,
}

pub struct ManualAngleExecutionResult {
    pub message: String,
    pub error_code: String,
    pub actual_angle_used: f32,
    pub verified_actual_angle: f32,
    pub verified_changed: bool,
    pub target_angle: i32,
    pub delta_angle_used: i32,
    pub tts: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct BleCharacteristicCandidate {
    service_uuid: Uuid,
    characteristic_uuid: Uuid,
    is_writable: bool,
}

impl BleCharacteristicCandidate {
    fn new(service_uuid: Uuid, characteristic_uuid: Uuid, is_writable: bool) -> Self {
        Self {
            service_uuid,
            characteristic_uuid,
            is_writable,
        }
    }
}

#[derive(Debug, Clone)]
struct BleakSession {
    mac: String,
    service_uuid: Option<String>,
    write_char_uuid: String,
    notify_char_uuid: Option<String>,
    tcu_address: Option<u8>,
}

#[derive(Debug, Serialize)]
struct BleakHelperRequest {
    mac: String,
    service_uuid: Option<String>,
    write_char_uuid: Option<String>,
    notify_char_uuid: Option<String>,
    command_hex: Option<String>,
    expect_notification: bool,
    notification_timeout_ms: u64,
}

#[derive(Debug, Deserialize)]
struct BleakHelperResponse {
    success: bool,
    message: String,
    service_uuid: Option<String>,
    write_char_uuid: Option<String>,
    notify_char_uuid: Option<String>,
    notification_hex: Option<String>,
}

#[derive(Debug)]
pub struct ManualAngleFailure {
    pub message: String,
    pub error_code: String,
    pub tts: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ManualAngleDirection {
    West,
    East,
}

impl ManualAngleDirection {
    fn as_str(&self) -> &'static str {
        match self {
            Self::West => "west",
            Self::East => "east",
        }
    }

    fn label(&self) -> &'static str {
        match self {
            Self::West => "向西",
            Self::East => "向东",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ManualAngleValidationErrorKind {
    InvalidDirection,
    InvalidDeltaAngle,
    TargetAngleOutOfRange,
}

#[derive(Debug, Clone)]
struct ManualAngleValidationError {
    kind: ManualAngleValidationErrorKind,
    message: String,
}

impl fmt::Display for ManualAngleValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl Error for ManualAngleValidationError {}

impl ManualAngleValidationError {
    fn invalid_direction(direction: &str) -> Self {
        Self {
            kind: ManualAngleValidationErrorKind::InvalidDirection,
            message: format!("不支持的姿态方向: {direction}"),
        }
    }

    fn invalid_delta_angle(delta_angle: i32) -> Self {
        Self {
            kind: ManualAngleValidationErrorKind::InvalidDeltaAngle,
            message: format!("手动角度仅支持 0 到 90 的整数，当前收到 {delta_angle}"),
        }
    }

    fn target_angle_out_of_range(target_angle: i32) -> Self {
        Self {
            kind: ManualAngleValidationErrorKind::TargetAngleOutOfRange,
            message: format!("目标角度超出协议可编码范围，当前计算结果为 {target_angle}°"),
        }
    }
}

impl From<ManualAngleValidationError> for ManualAngleFailure {
    fn from(value: ManualAngleValidationError) -> Self {
        let error_code = match value.kind {
            ManualAngleValidationErrorKind::InvalidDirection => "invalid_direction",
            ManualAngleValidationErrorKind::InvalidDeltaAngle => "invalid_delta_angle",
            ManualAngleValidationErrorKind::TargetAngleOutOfRange => "target_angle_out_of_range",
        };

        let tts = match value.kind {
            ManualAngleValidationErrorKind::InvalidDirection => {
                Some("姿态调整方向无效，请重新选择向西或向东。".to_string())
            }
            ManualAngleValidationErrorKind::InvalidDeltaAngle => {
                Some("姿态调整角度无效，请输入零到九十度的整数。".to_string())
            }
            ManualAngleValidationErrorKind::TargetAngleOutOfRange => {
                Some("当前计算出的目标角度超出设备允许范围，本次调整已取消。".to_string())
            }
        };

        Self {
            message: value.to_string(),
            error_code: error_code.to_string(),
            tts,
        }
    }
}

pub struct BluetoothManager {
    target_device: Option<Peripheral>,
    write_char: Option<Characteristic>,
    bleak_session: Option<BleakSession>,
}

impl BluetoothManager {
    pub fn new() -> Self {
        Self {
            target_device: None,
            write_char: None,
            bleak_session: None,
        }
    }

    /// 核心连接函数：支持动态 UUID 或 自动发现可写特征值
    pub async fn connect_and_execute(
        &mut self,
        mac_str: &str,
        service_uuid_str: &str,
        char_uuid_str: &str,
        command_hex: &str,
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
        if adapters.is_empty() {
            return Err("❌ 未找到蓝牙适配器".into());
        }

        let normalized_target = normalize_mac(mac_str);
        let mut matched: Option<(Adapter, Peripheral)> = None;

        for (adapter_index, central) in adapters.into_iter().enumerate() {
            println!("📡 使用蓝牙适配器 #{} 扫描目标 {}", adapter_index, mac_str);
            if let Some(peripheral) =
                Self::scan_target_peripheral(&central, &normalized_target).await?
            {
                matched = Some((central, peripheral));
                break;
            }
        }

        let Some((central, p)) = matched else {
            return Err(format!("❌ 未扫描到设备: {}", mac_str).into());
        };

        let mut resolved_tcu_address = None;
        if command_hex.is_empty() {
            let tcu = self.resolve_tcu_from_advertisement(&p).await?;
            if let Err(e) = persist_device_info(mac_str, tcu) {
                eprintln!("⚠️ 持久化设备信息失败: {}", e);
            } else {
                println!("💾 已保存设备信息: MAC={}, TCU={}", mac_str, tcu);
            }
            command_hex = build_query_command(tcu);
            resolved_tcu_address = Some(tcu);
            println!("🧩 生成查询指令: {}", command_hex);
        }

        println!("🔗 找到设备，正在连接...");
        if let Err(e) = central.stop_scan().await {
            eprintln!("⚠️ 停止扫描失败: {}", e);
        }
        time::sleep(Duration::from_millis(BLE_CONNECT_SETTLE_MS)).await;
        if let Err(error) = Self::connect_with_retry(&central, &p, 3).await {
            let error_message = error.to_string();
            if should_use_bleak_fallback(&error_message) {
                eprintln!(
                    "⚠️ btleplug 建连失败，切换 Bleak fallback: {}",
                    error_message
                );
                return self
                    .execute_with_bleak_fallback(
                        mac_str,
                        target_service_uuid,
                        target_char_uuid,
                        &command_hex,
                        resolved_tcu_address,
                    )
                    .await;
            }
            return Err(error);
        }

        println!("✅ 连接建立! 正在发现服务...");
        p.discover_services().await?;

        // 3. 动态寻找特征值
        let chars = p.characteristics().into_iter().collect::<Vec<_>>();

        // --- 核心修改：匹配逻辑升级 ---
        // 寻找满足条件的特征值：
        // A. 如果指定了 UUID，必须完全匹配
        // B. 如果没指定 UUID，寻找第一个"可写"的特征值
        let matched_char =
            select_write_characteristic(&chars, target_service_uuid, target_char_uuid);

        if let Some(c) = matched_char {
            println!(
                "✅ 锁定特征值: {:?} (Service: {:?})",
                c.uuid, c.service_uuid
            );
            println!("   属性: {:?}", c.properties);

            self.write_char = Some(c.clone());
            self.target_device = Some(p.clone());
            self.bleak_session = None;

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
                let mut actual_angle = None;
                let mut target_angle = None;
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
                        actual_angle = Some(parsed.actual_angle);
                        target_angle = Some(parsed.target_angle);
                        println!("🗣️ TTS: {}", tts_text);
                        tts = Some(tts_text);
                        break;
                    }
                }

                return Ok(BleExecutionResult {
                    message: format!("已连接并发送指令: {}", command_hex),
                    tts,
                    actual_angle,
                    target_angle,
                });
            }

            return Ok(BleExecutionResult {
                message: "已连接 (无指令发送)".to_string(),
                tts: None,
                actual_angle: None,
                target_angle: None,
            });
        } else {
            return Err(
                format!("❌ 未找到合适的可写特征值 (UUID 指定: {:?})", char_uuid_str).into(),
            );
        }
    }

    pub async fn disconnect_current(&mut self) -> Result<String, Box<dyn Error>> {
        let target_device = self.target_device.take();
        self.write_char = None;
        let had_bleak_session = self.bleak_session.take().is_some();

        let Some(peripheral) = target_device else {
            if had_bleak_session {
                return Ok("蓝牙连接已断开".to_string());
            }
            return Ok("当前无活动蓝牙连接".to_string());
        };

        if peripheral.is_connected().await.unwrap_or(false) {
            peripheral.disconnect().await?;
            println!("🔌 已断开当前蓝牙连接");
            return Ok("蓝牙连接已断开".to_string());
        }

        Ok("当前蓝牙连接已处于断开状态".to_string())
    }

    pub async fn execute_manual_angle(
        &mut self,
        direction: &str,
        raw_delta_angle: Option<i32>,
    ) -> Result<ManualAngleExecutionResult, ManualAngleFailure> {
        let direction = parse_manual_angle_direction(direction)?;
        let delta_angle = normalize_manual_delta_angle(raw_delta_angle)?;

        if let Some(session) = self.bleak_session.clone() {
            return self
                .execute_manual_angle_with_bleak(&session, direction, delta_angle)
                .await;
        }

        let Some(peripheral) = self.target_device.clone() else {
            return Err(ManualAngleFailure {
                message: "当前没有已连接的设备，无法执行姿态调整。".to_string(),
                error_code: "device_not_connected".to_string(),
                tts: Some("当前还没有连接设备，暂时不能执行姿态调整。".to_string()),
            });
        };

        if !peripheral.is_connected().await.unwrap_or(false) {
            self.target_device = None;
            self.write_char = None;
            return Err(ManualAngleFailure {
                message: "蓝牙设备当前未连接，请重新扫码并连接设备。".to_string(),
                error_code: "device_not_connected".to_string(),
                tts: Some("设备连接已经断开，请重新扫码后再试。".to_string()),
            });
        }

        let Some(write_char) = self.write_char.clone() else {
            return Err(ManualAngleFailure {
                message: "当前连接缺少可写通道，无法执行姿态调整。".to_string(),
                error_code: "missing_write_characteristic".to_string(),
                tts: Some("当前设备缺少可写通道，本次姿态调整无法执行。".to_string()),
            });
        };

        let tcu_address = self
            .resolve_current_tcu_address(&peripheral)
            .await
            .map_err(|error| ManualAngleFailure {
                message: format!("无法获取当前设备的 TCU 地址: {error}"),
                error_code: "missing_tcu_address".to_string(),
                tts: Some("当前设备地址信息不完整，无法执行姿态调整。".to_string()),
            })?;

        let query_result = self
            .query_current_state(&peripheral, &write_char, tcu_address)
            .await
            .map_err(|error| ManualAngleFailure {
                message: format!("读取当前设备角度失败: {error}"),
                error_code: "query_failed".to_string(),
                tts: Some("当前设备状态读取失败，本次姿态调整已取消。".to_string()),
            })?;

        let target_angle = compute_manual_target_angle(
            query_result.actual_angle,
            query_result.target_angle,
            direction.as_str(),
            delta_angle,
        )?;
        let command_hex = build_manual_angle_command(tcu_address, target_angle)
            .map_err(ManualAngleFailure::from)?;

        self.send_hex_command(&peripheral, &write_char, &command_hex)
            .await
            .map_err(|error| ManualAngleFailure {
                message: format!("姿态调整指令下发失败: {error}"),
                error_code: "command_write_failed".to_string(),
                tts: Some("姿态调整指令发送失败，请稍后重试。".to_string()),
            })?;

        let verification = self
            .verify_manual_angle_effect(
                &peripheral,
                &write_char,
                tcu_address,
                query_result.actual_angle,
                direction,
                target_angle,
            )
            .await
            .map_err(|error| ManualAngleFailure {
                message: format!("姿态调整指令已下发，但复核失败: {error}"),
                error_code: "verification_query_failed".to_string(),
                tts: Some(
                    "姿态调整指令已发送，但复核当前角度失败，请稍后检查设备状态。".to_string(),
                ),
            })?;

        if !verification.changed {
            return Err(ManualAngleFailure {
                message: format!(
                    "姿态调整指令已下发，但复核时未检测到实际角度变化。调整前 {:.1}°，当前 {:.1}°。",
                    query_result.actual_angle, verification.actual_angle
                ),
                error_code: "verification_failed".to_string(),
                tts: Some(
                    "姿态调整指令已经下发，但当前还没有检测到角度变化，请检查设备执行状态。"
                        .to_string(),
                ),
            });
        }

        let direction_text = direction.label();
        Ok(ManualAngleExecutionResult {
            message: format!(
                "已读取当前实际角度 {:.1}°、当前目标角度 {:.1}°，计算新目标角度 {}°。复核完成，当前实际角度 {:.1}°，{}调整已生效。",
                query_result.actual_angle,
                query_result.target_angle,
                target_angle,
                verification.actual_angle,
                direction_text
            ),
            error_code: String::new(),
            actual_angle_used: query_result.actual_angle,
            verified_actual_angle: verification.actual_angle,
            verified_changed: verification.changed,
            target_angle,
            delta_angle_used: delta_angle,
            tts: Some(format!(
                "已读取当前实际角度 {:.1} 度，当前目标角度 {:.1} 度，{}调整 {} 度，新目标角度 {} 度。复核完成，当前实际角度 {:.1} 度，姿态调整成功。",
                query_result.actual_angle,
                query_result.target_angle,
                direction_text,
                delta_angle,
                target_angle,
                verification.actual_angle
            )),
        })
    }

    async fn execute_with_bleak_fallback(
        &mut self,
        mac_str: &str,
        target_service_uuid: Option<Uuid>,
        target_char_uuid: Option<Uuid>,
        command_hex: &str,
        resolved_tcu_address: Option<u8>,
    ) -> Result<BleExecutionResult, Box<dyn Error>> {
        let command_bytes = if command_hex.is_empty() {
            None
        } else {
            Some(Self::hex_to_bytes(command_hex)?)
        };
        let expects_response = command_bytes
            .as_ref()
            .map(|bytes| bytes.len() >= 2 && bytes[1] == 0x03)
            .unwrap_or(false);
        let response = Self::run_bleak_helper(BleakHelperRequest {
            mac: mac_str.to_string(),
            service_uuid: target_service_uuid.map(|uuid| uuid.to_string()),
            write_char_uuid: target_char_uuid.map(|uuid| uuid.to_string()),
            notify_char_uuid: expects_response.then(|| VENDOR_NOTIFY_UUID.to_string()),
            command_hex: (!command_hex.is_empty()).then(|| command_hex.to_string()),
            expect_notification: expects_response,
            notification_timeout_ms: BLEAK_NOTIFICATION_TIMEOUT_MS,
        })
        .await?;

        if !response.success {
            return Err(response.message.into());
        }

        let write_char_uuid = response
            .write_char_uuid
            .clone()
            .ok_or("❌ Bleak fallback 未返回可写特征值")?;
        self.target_device = None;
        self.write_char = None;
        self.bleak_session = Some(BleakSession {
            mac: mac_str.to_string(),
            service_uuid: response.service_uuid.clone(),
            write_char_uuid,
            notify_char_uuid: response.notify_char_uuid.clone(),
            tcu_address: resolved_tcu_address,
        });

        let mut result = BleExecutionResult {
            message: response.message,
            tts: None,
            actual_angle: None,
            target_angle: None,
        };

        if expects_response {
            let notification_hex = response
                .notification_hex
                .ok_or("❌ Bleak fallback 未返回通知数据")?;
            let payload = Self::hex_to_bytes(&notification_hex)?;
            println!("📥 [bleak] 收到通知: {:02X?}", payload);
            let parsed = parse_response_payload(&payload)?;
            let tts_text = build_tts(&parsed);
            println!("🗣️ TTS: {}", tts_text);
            result.tts = Some(tts_text);
            result.actual_angle = Some(parsed.actual_angle);
            result.target_angle = Some(parsed.target_angle);
            if let Some(session) = self.bleak_session.as_mut() {
                session.tcu_address = Some(parsed.tcu_address);
            }
        }

        Ok(result)
    }

    async fn execute_manual_angle_with_bleak(
        &mut self,
        session: &BleakSession,
        direction: ManualAngleDirection,
        delta_angle: i32,
    ) -> Result<ManualAngleExecutionResult, ManualAngleFailure> {
        let tcu_address = self
            .resolve_current_tcu_address_for_bleak(session)
            .map_err(|error| ManualAngleFailure {
                message: format!("无法获取当前设备的 TCU 地址: {error}"),
                error_code: "missing_tcu_address".to_string(),
                tts: Some("当前设备地址信息不完整，无法执行姿态调整。".to_string()),
            })?;

        let query_result = self
            .query_current_state_with_bleak(session, tcu_address)
            .await
            .map_err(|error| ManualAngleFailure {
                message: format!("读取当前设备角度失败: {error}"),
                error_code: "query_failed".to_string(),
                tts: Some("当前设备状态读取失败，本次姿态调整已取消。".to_string()),
            })?;

        let target_angle = compute_manual_target_angle(
            query_result.actual_angle,
            query_result.target_angle,
            direction.as_str(),
            delta_angle,
        )?;
        let command_hex = build_manual_angle_command(tcu_address, target_angle)
            .map_err(ManualAngleFailure::from)?;

        self.send_command_with_bleak(session, &command_hex)
            .await
            .map_err(|error| ManualAngleFailure {
                message: format!("姿态调整指令下发失败: {error}"),
                error_code: "command_write_failed".to_string(),
                tts: Some("姿态调整指令发送失败，请稍后重试。".to_string()),
            })?;

        let verification = self
            .verify_manual_angle_effect_with_bleak(
                session,
                tcu_address,
                query_result.actual_angle,
                direction,
                target_angle,
            )
            .await
            .map_err(|error| ManualAngleFailure {
                message: format!("姿态调整指令已下发，但复核失败: {error}"),
                error_code: "verification_query_failed".to_string(),
                tts: Some(
                    "姿态调整指令已发送，但复核当前角度失败，请稍后检查设备状态。".to_string(),
                ),
            })?;

        if !verification.changed {
            return Err(ManualAngleFailure {
                message: format!(
                    "姿态调整指令已下发，但复核时未检测到实际角度变化。调整前 {:.1}°，当前 {:.1}°。",
                    query_result.actual_angle, verification.actual_angle
                ),
                error_code: "verification_failed".to_string(),
                tts: Some(
                    "姿态调整指令已经下发，但当前还没有检测到角度变化，请检查设备执行状态。"
                        .to_string(),
                ),
            });
        }

        let direction_text = direction.label();
        Ok(ManualAngleExecutionResult {
            message: format!(
                "已读取当前实际角度 {:.1}°、当前目标角度 {:.1}°，计算新目标角度 {}°。复核完成，当前实际角度 {:.1}°，{}调整已生效。",
                query_result.actual_angle,
                query_result.target_angle,
                target_angle,
                verification.actual_angle,
                direction_text
            ),
            error_code: String::new(),
            actual_angle_used: query_result.actual_angle,
            verified_actual_angle: verification.actual_angle,
            verified_changed: verification.changed,
            target_angle,
            delta_angle_used: delta_angle,
            tts: Some(format!(
                "已读取当前实际角度 {:.1} 度，当前目标角度 {:.1} 度，{}调整 {} 度，新目标角度 {} 度。复核完成，当前实际角度 {:.1} 度，姿态调整成功。",
                query_result.actual_angle,
                query_result.target_angle,
                direction_text,
                delta_angle,
                target_angle,
                verification.actual_angle
            )),
        })
    }

    fn resolve_current_tcu_address_for_bleak(
        &self,
        session: &BleakSession,
    ) -> Result<u8, Box<dyn Error>> {
        if let Some(tcu) = session.tcu_address {
            return Ok(tcu);
        }

        let info = load_persisted_device_info()?;
        if normalize_mac(&info.mac) == normalize_mac(&session.mac) {
            return Ok(info.tcu);
        }

        Err("❌ 当前缺少已缓存的 TCU 地址".into())
    }

    async fn query_current_state_with_bleak(
        &self,
        session: &BleakSession,
        tcu_address: u8,
    ) -> Result<ParsedResponse, Box<dyn Error>> {
        let response = Self::run_bleak_helper(BleakHelperRequest {
            mac: session.mac.clone(),
            service_uuid: session.service_uuid.clone(),
            write_char_uuid: Some(session.write_char_uuid.clone()),
            notify_char_uuid: session.notify_char_uuid.clone(),
            command_hex: Some(build_query_command(tcu_address)),
            expect_notification: true,
            notification_timeout_ms: BLEAK_NOTIFICATION_TIMEOUT_MS,
        })
        .await?;

        if !response.success {
            return Err(response.message.into());
        }

        let notification_hex = response
            .notification_hex
            .ok_or("❌ Bleak 查询未返回通知数据")?;
        let payload = Self::hex_to_bytes(&notification_hex)?;
        println!("📥 [bleak] 收到设备查询响应: {:02X?}", payload);
        parse_response_payload(&payload)
    }

    async fn send_command_with_bleak(
        &self,
        session: &BleakSession,
        command_hex: &str,
    ) -> Result<(), Box<dyn Error>> {
        let response = Self::run_bleak_helper(BleakHelperRequest {
            mac: session.mac.clone(),
            service_uuid: session.service_uuid.clone(),
            write_char_uuid: Some(session.write_char_uuid.clone()),
            notify_char_uuid: session.notify_char_uuid.clone(),
            command_hex: Some(command_hex.to_string()),
            expect_notification: false,
            notification_timeout_ms: BLEAK_NOTIFICATION_TIMEOUT_MS,
        })
        .await?;

        if response.success {
            return Ok(());
        }

        Err(response.message.into())
    }

    async fn verify_manual_angle_effect_with_bleak(
        &self,
        session: &BleakSession,
        tcu_address: u8,
        initial_actual_angle: f32,
        direction: ManualAngleDirection,
        target_angle: i32,
    ) -> Result<ManualAngleVerification, Box<dyn Error>> {
        let mut last_actual_angle = initial_actual_angle;

        for _ in 0..MANUAL_VERIFICATION_MAX_ATTEMPTS {
            time::sleep(Duration::from_millis(MANUAL_VERIFICATION_DELAY_MS)).await;
            let verified_state = self
                .query_current_state_with_bleak(session, tcu_address)
                .await?;
            last_actual_angle = verified_state.actual_angle;

            if did_manual_angle_take_effect(
                initial_actual_angle,
                verified_state.actual_angle,
                direction,
                target_angle,
            ) {
                return Ok(ManualAngleVerification {
                    actual_angle: verified_state.actual_angle,
                    changed: true,
                });
            }
        }

        Ok(ManualAngleVerification {
            actual_angle: last_actual_angle,
            changed: false,
        })
    }

    async fn run_bleak_helper(
        request: BleakHelperRequest,
    ) -> Result<BleakHelperResponse, Box<dyn Error>> {
        let helper_path = bleak_fallback_helper_path();
        if !helper_path.exists() {
            return Err(format!("❌ Bleak fallback 脚本不存在: {}", helper_path.display()).into());
        }

        let request_json = serde_json::to_string(&request)?;
        let output = time::timeout(
            Duration::from_secs(BLEAK_HELPER_TIMEOUT_SECS),
            Command::new("python3")
                .arg(&helper_path)
                .arg("--request")
                .arg(request_json)
                .output(),
        )
        .await
        .map_err(|_| "❌ Bleak fallback 执行超时")??;

        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            let stdout = String::from_utf8_lossy(&output.stdout);
            let detail = if !stderr.trim().is_empty() {
                stderr.trim().to_string()
            } else {
                stdout.trim().to_string()
            };
            return Err(format!("❌ Bleak fallback 执行失败: {}", detail).into());
        }

        let stdout = String::from_utf8(output.stdout)?;
        let response = serde_json::from_str::<BleakHelperResponse>(stdout.trim())?;
        Ok(response)
    }

    async fn connect_with_retry(
        central: &Adapter,
        peripheral: &Peripheral,
        max_attempts: usize,
    ) -> Result<(), Box<dyn Error>> {
        let mut last_error = String::new();

        if peripheral.is_connected().await.unwrap_or(false) {
            println!("ℹ️ 设备当前已连接，先断开旧连接后重试...");
            if let Err(e) = peripheral.disconnect().await {
                eprintln!("⚠️ 断开旧连接失败: {}", e);
            }
            time::sleep(Duration::from_millis(300)).await;
        }

        for attempt in 1..=max_attempts {
            if let Err(error) = central.stop_scan().await {
                eprintln!("⚠️ 连接前停止扫描失败: {}", error);
            }
            match peripheral.connect().await {
                Ok(_) => return Ok(()),
                Err(e) => {
                    last_error = e.to_string();
                    eprintln!(
                        "⚠️ BLE 连接失败 (第 {}/{} 次): {}",
                        attempt, max_attempts, last_error
                    );
                    let _ = peripheral.disconnect().await;
                    if attempt < max_attempts {
                        let retry_delay_ms = connect_retry_delay_ms(&last_error);
                        if is_local_connection_abort(&last_error) {
                            eprintln!(
                                "⚠️ 检测到本地主动中止连接，等待蓝牙适配器稳定 {}ms 后重试...",
                                retry_delay_ms
                            );
                        }
                        time::sleep(Duration::from_millis(retry_delay_ms)).await;
                    }
                }
            }
        }

        Err(format!("❌ BLE 连接失败: {}", last_error).into())
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

    async fn resolve_current_tcu_address(
        &self,
        peripheral: &Peripheral,
    ) -> Result<u8, Box<dyn Error>> {
        if let Ok(info) = load_persisted_device_info() {
            let current_mac = normalize_mac(&peripheral.address().to_string());
            if normalize_mac(&info.mac) == current_mac {
                return Ok(info.tcu);
            }
        }

        self.resolve_tcu_from_advertisement(peripheral).await
    }

    async fn query_current_state(
        &self,
        peripheral: &Peripheral,
        characteristic: &Characteristic,
        tcu_address: u8,
    ) -> Result<ParsedResponse, Box<dyn Error>> {
        let notify_uuid = Uuid::parse_str(NOTIFY_CHARACTERISTIC_UUID)?;
        let notify_char = peripheral
            .characteristics()
            .iter()
            .find(|c| c.uuid == notify_uuid)
            .cloned()
            .ok_or("❌ 未找到通知特征值")?;

        if !(notify_char.properties.contains(CharPropFlags::NOTIFY)
            || notify_char.properties.contains(CharPropFlags::INDICATE))
        {
            return Err("❌ 通知特征值不支持通知".into());
        }

        peripheral.subscribe(&notify_char).await?;
        let result = async {
            let mut notifications = peripheral.notifications().await?;
            let query_command = build_query_command(tcu_address);
            self.send_hex_command(peripheral, characteristic, &query_command)
                .await?;

            let deadline = time::Instant::now() + Duration::from_secs(5);
            loop {
                let remaining = deadline.saturating_duration_since(time::Instant::now());
                if remaining.is_zero() {
                    return Err("❌ 未收到设备查询响应".into());
                }

                let next = time::timeout(remaining, notifications.next()).await;
                let notification = match next {
                    Ok(Some(value)) => value,
                    Ok(None) => return Err("❌ 通知流结束".into()),
                    Err(_) => return Err("❌ 未收到设备查询响应".into()),
                };

                if notification.uuid != notify_uuid {
                    continue;
                }

                println!("📥 收到设备查询响应: {:02X?}", notification.value);
                return parse_response_payload(&notification.value);
            }
        }
        .await;

        let _ = peripheral.unsubscribe(&notify_char).await;
        result
    }

    async fn verify_manual_angle_effect(
        &self,
        peripheral: &Peripheral,
        characteristic: &Characteristic,
        tcu_address: u8,
        initial_actual_angle: f32,
        direction: ManualAngleDirection,
        target_angle: i32,
    ) -> Result<ManualAngleVerification, Box<dyn Error>> {
        let mut last_actual_angle = initial_actual_angle;

        for _ in 0..MANUAL_VERIFICATION_MAX_ATTEMPTS {
            time::sleep(Duration::from_millis(MANUAL_VERIFICATION_DELAY_MS)).await;
            let verified_state = self
                .query_current_state(peripheral, characteristic, tcu_address)
                .await?;
            last_actual_angle = verified_state.actual_angle;

            if did_manual_angle_take_effect(
                initial_actual_angle,
                verified_state.actual_angle,
                direction,
                target_angle,
            ) {
                return Ok(ManualAngleVerification {
                    actual_angle: verified_state.actual_angle,
                    changed: true,
                });
            }
        }

        Ok(ManualAngleVerification {
            actual_angle: last_actual_angle,
            changed: false,
        })
    }

    // 内部辅助：发送 Hex 字符串
    async fn send_hex_command(
        &self,
        device: &Peripheral,
        characteristic: &Characteristic,
        hex_cmd: &str,
    ) -> Result<(), Box<dyn Error>> {
        let data = Self::hex_to_bytes(hex_cmd)?;
        println!("📤 发送 HEX: {:02X?}", data);

        // --- 核心修改：根据特征值属性自动选择写入方式 ---
        let write_type = if characteristic
            .properties
            .contains(CharPropFlags::WRITE_WITHOUT_RESPONSE)
        {
            WriteType::WithoutResponse
        } else {
            WriteType::WithResponse
        };

        device.write(characteristic, &data, write_type).await?;
        Ok(())
    }

    async fn scan_target_peripheral(
        central: &Adapter,
        normalized_target: &str,
    ) -> Result<Option<Peripheral>, Box<dyn Error>> {
        if let Err(error) = central.stop_scan().await {
            eprintln!("⚠️ 扫描前停止旧扫描失败: {}", error);
        }

        for attempt in 1..=BLE_SCAN_ATTEMPTS {
            println!(
                "📡 扫描轮次 {}/{}，窗口 {}s",
                attempt, BLE_SCAN_ATTEMPTS, BLE_SCAN_WINDOW_SECS
            );
            central.start_scan(ScanFilter::default()).await?;
            time::sleep(Duration::from_secs(BLE_SCAN_WINDOW_SECS)).await;

            let peripherals = central.peripherals().await?;
            println!("📶 本轮发现 {} 个 BLE 设备", peripherals.len());

            for peripheral in &peripherals {
                let address = normalize_mac(&peripheral.address().to_string());
                if address == normalized_target {
                    return Ok(Some(peripheral.clone()));
                }
            }

            let sample_addresses = peripherals
                .iter()
                .take(5)
                .map(|peripheral| peripheral.address().to_string())
                .collect::<Vec<_>>();
            if sample_addresses.is_empty() {
                println!("📭 本轮未发现任何 BLE 外设");
            } else {
                println!("📋 本轮设备样例: {}", sample_addresses.join(", "));
            }

            if let Err(error) = central.stop_scan().await {
                eprintln!("⚠️ 扫描轮次结束后停止扫描失败: {}", error);
            }
            time::sleep(Duration::from_millis(300)).await;
        }

        Ok(None)
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

fn is_local_connection_abort(error: &str) -> bool {
    error
        .to_ascii_lowercase()
        .contains("le-connection-abort-by-local")
}

fn connect_retry_delay_ms(error: &str) -> u64 {
    if is_local_connection_abort(error) {
        BLE_LOCAL_ABORT_RETRY_DELAY_MS
    } else {
        BLE_CONNECT_RETRY_DELAY_MS
    }
}

fn should_use_bleak_fallback(error: &str) -> bool {
    is_local_connection_abort(error)
}

fn bleak_fallback_helper_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("scripts")
        .join("bleak_fallback.py")
}

fn select_preferred_write_candidate(
    candidates: &[BleCharacteristicCandidate],
) -> Option<BleCharacteristicCandidate> {
    let vendor_service_uuid = Uuid::parse_str(VENDOR_SERVICE_UUID).ok()?;
    let vendor_write_primary_uuid = Uuid::parse_str(VENDOR_WRITE_PRIMARY_UUID).ok()?;
    let vendor_write_secondary_uuid = Uuid::parse_str(VENDOR_WRITE_SECONDARY_UUID).ok()?;

    let mut best: Option<(usize, &BleCharacteristicCandidate)> = None;
    for candidate in candidates.iter().filter(|candidate| candidate.is_writable) {
        let priority = if candidate.service_uuid == vendor_service_uuid
            && candidate.characteristic_uuid == vendor_write_primary_uuid
        {
            0
        } else if candidate.service_uuid == vendor_service_uuid
            && candidate.characteristic_uuid == vendor_write_secondary_uuid
        {
            1
        } else if candidate.service_uuid == vendor_service_uuid {
            2
        } else {
            10
        };

        if best
            .as_ref()
            .map(|(best_priority, _)| priority < *best_priority)
            .unwrap_or(true)
        {
            best = Some((priority, candidate));
        }
    }

    best.map(|(_, candidate)| candidate.clone())
}

fn select_write_characteristic(
    chars: &[Characteristic],
    target_service_uuid: Option<Uuid>,
    target_char_uuid: Option<Uuid>,
) -> Option<Characteristic> {
    match (target_service_uuid, target_char_uuid) {
        (Some(service_uuid), Some(char_uuid)) => chars
            .iter()
            .find(|characteristic| {
                characteristic.service_uuid == service_uuid && characteristic.uuid == char_uuid
            })
            .cloned(),
        _ => {
            let candidates = chars
                .iter()
                .map(|characteristic| {
                    BleCharacteristicCandidate::new(
                        characteristic.service_uuid,
                        characteristic.uuid,
                        characteristic.properties.contains(CharPropFlags::WRITE)
                            || characteristic
                                .properties
                                .contains(CharPropFlags::WRITE_WITHOUT_RESPONSE),
                    )
                })
                .collect::<Vec<_>>();
            let selected = select_preferred_write_candidate(&candidates)?;
            chars
                .iter()
                .find(|characteristic| {
                    characteristic.service_uuid == selected.service_uuid
                        && characteristic.uuid == selected.characteristic_uuid
                })
                .cloned()
        }
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

fn parse_manual_angle_direction(
    direction: &str,
) -> Result<ManualAngleDirection, ManualAngleFailure> {
    match direction.trim().to_ascii_lowercase().as_str() {
        "west" => Ok(ManualAngleDirection::West),
        "east" => Ok(ManualAngleDirection::East),
        other => Err(ManualAngleValidationError::invalid_direction(other).into()),
    }
}

fn normalize_manual_delta_angle(
    raw_delta_angle: Option<i32>,
) -> Result<i32, ManualAngleValidationError> {
    let delta_angle = raw_delta_angle.unwrap_or(MANUAL_DELTA_DEFAULT);
    if !(MANUAL_DELTA_MIN..=MANUAL_DELTA_MAX).contains(&delta_angle) {
        return Err(ManualAngleValidationError::invalid_delta_angle(delta_angle));
    }
    Ok(delta_angle)
}

fn compute_manual_target_angle(
    actual_angle: f32,
    current_target_angle: f32,
    direction: &str,
    delta_angle: i32,
) -> Result<i32, ManualAngleValidationError> {
    let rounded_actual_angle = actual_angle.round() as i32;
    let rounded_current_target_angle = current_target_angle.round() as i32;
    let direction =
        parse_manual_angle_direction(direction).map_err(|error| ManualAngleValidationError {
            kind: ManualAngleValidationErrorKind::InvalidDirection,
            message: error.message,
        })?;

    let base_angle = match direction {
        ManualAngleDirection::West => rounded_actual_angle.max(rounded_current_target_angle),
        ManualAngleDirection::East => rounded_actual_angle.min(rounded_current_target_angle),
    };

    let target_angle = match direction {
        ManualAngleDirection::West => base_angle + delta_angle,
        ManualAngleDirection::East => base_angle - delta_angle,
    };

    if !(MANUAL_TARGET_MIN..=MANUAL_TARGET_MAX).contains(&target_angle) {
        return Err(ManualAngleValidationError::target_angle_out_of_range(
            target_angle,
        ));
    }

    Ok(target_angle)
}

fn build_manual_angle_command(
    tcu_address: u8,
    target_angle: i32,
) -> Result<String, ManualAngleValidationError> {
    if !(MANUAL_TARGET_MIN..=MANUAL_TARGET_MAX).contains(&target_angle) {
        return Err(ManualAngleValidationError::target_angle_out_of_range(
            target_angle,
        ));
    }

    let mut payload = vec![
        tcu_address,
        0x06,
        0x08,
        (AUTO_MODE_TARGET_WORK_MODE >> 8) as u8,
        (AUTO_MODE_TARGET_WORK_MODE & 0xFF) as u8,
        (target_angle as i8) as u8,
    ];
    let crc = crc16_modbus(&payload);
    payload.push((crc & 0xFF) as u8);
    payload.push((crc >> 8) as u8);
    Ok(payload
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<String>())
}

struct ManualAngleVerification {
    actual_angle: f32,
    changed: bool,
}

fn did_manual_angle_take_effect(
    initial_actual_angle: f32,
    verified_actual_angle: f32,
    direction: ManualAngleDirection,
    target_angle: i32,
) -> bool {
    let changed =
        (verified_actual_angle - initial_actual_angle).abs() >= MANUAL_VERIFICATION_MIN_DELTA;
    if !changed {
        return false;
    }

    let moved_in_expected_direction = match direction {
        ManualAngleDirection::West => verified_actual_angle > initial_actual_angle,
        ManualAngleDirection::East => verified_actual_angle < initial_actual_angle,
    };

    let close_to_target =
        (verified_actual_angle - target_angle as f32).abs() <= MANUAL_VERIFICATION_TARGET_TOLERANCE;

    moved_in_expected_direction || close_to_target
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

#[derive(Deserialize, Serialize)]
struct PersistedDeviceInfo {
    mac: String,
    tcu: u8,
}

fn load_persisted_device_info() -> Result<PersistedDeviceInfo, Box<dyn Error>> {
    let path = Path::new("/neuro_bot_ws/data/ble_devices.json");
    let content = std::fs::read_to_string(path)?;
    let info = serde_json::from_str::<PersistedDeviceInfo>(&content)?;
    Ok(info)
}

fn normalize_mac(value: &str) -> String {
    value.replace(":", "").to_uppercase()
}

fn persist_device_info(mac: &str, tcu: u8) -> Result<(), Box<dyn Error>> {
    let path = Path::new("/neuro_bot_ws/data/ble_devices.json");
    persist_device_info_to_path(path, mac, tcu)
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
        bleak_fallback_helper_path, build_manual_angle_command, build_query_command,
        compute_manual_target_angle, connect_retry_delay_ms, did_manual_angle_take_effect,
        extract_protocol_from_manufacturer_data, fault_code_to_text, is_local_connection_abort,
        normalize_manual_delta_angle, parse_response_payload, parse_tcu_from_protocol,
        persist_device_info_to_path, select_preferred_write_candidate, should_use_bleak_fallback,
        verify_protocol_checksum, verify_response_crc, BleCharacteristicCandidate,
        ManualAngleDirection, BLE_CONNECT_RETRY_DELAY_MS, BLE_LOCAL_ABORT_RETRY_DELAY_MS,
    };
    use std::collections::HashMap;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};
    use uuid::Uuid;

    fn valid_protocol_sample() -> [u8; 26] {
        // Precomputed valid protocol (rand0=0x12, rand1=0x34, tcu=0x0A).
        [
            0x88, 0x11, 0x20, 0x12, 0x34, 0x15, 0x68, 0x4B, 0x17, 0x36, 0x5F, 0x7B, 0x73, 0x10,
            0x4C, 0x1E, 0x3D, 0x2D, 0x0D, 0x05, 0x27, 0x71, 0x26, 0x04, 0x6C, 0x4F,
        ]
    }

    fn invalid_checksum_sample() -> [u8; 26] {
        // Example from docs (checksum intentionally invalid)
        [
            0x88, 0x11, 0xA7, 0x12, 0x34, 0xE2, 0xC7, 0x83, 0xD7, 0xF0, 0x9C, 0x8D, 0xE8, 0xC5,
            0x81, 0xD5, 0xF2, 0x9E, 0x8F, 0xE9, 0xC7, 0x83, 0xD6, 0x73, 0x06, 0x66,
        ]
    }

    fn manufacturer_value_without_header() -> [u8; 24] {
        [
            0xA7, 0x12, 0x34, 0xE2, 0xC7, 0x83, 0xD7, 0xF0, 0x9C, 0x8D, 0xE8, 0xC5, 0x81, 0xD5,
            0xF2, 0x9E, 0x8F, 0xE9, 0xC7, 0x83, 0xD6, 0x73, 0x06, 0x66,
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
    fn manual_delta_defaults_to_ten_when_missing() {
        let delta = normalize_manual_delta_angle(None).expect("delta should default");
        assert_eq!(delta, 10);
    }

    #[test]
    fn manual_delta_rejects_out_of_range_values() {
        let error = normalize_manual_delta_angle(Some(91)).expect_err("delta should fail");
        assert!(error.to_string().contains("0 到 90"));
    }

    #[test]
    fn manual_target_angle_uses_rounded_actual_angle() {
        let target =
            compute_manual_target_angle(12.6, 11.2, "west", 5).expect("target should compute");
        assert_eq!(target, 18);
    }

    #[test]
    fn manual_target_angle_advances_from_existing_westward_target_when_it_is_ahead() {
        let target =
            compute_manual_target_angle(19.4, 29.0, "west", 10).expect("target should compute");
        assert_eq!(target, 39);
    }

    #[test]
    fn manual_target_angle_advances_from_existing_eastward_target_when_it_is_ahead() {
        let target =
            compute_manual_target_angle(19.4, 9.0, "east", 10).expect("target should compute");
        assert_eq!(target, -1);
    }

    #[test]
    fn manual_target_angle_rejects_protocol_overflow() {
        let error =
            compute_manual_target_angle(120.4, 126.0, "west", 10).expect_err("should reject");
        assert!(error.to_string().contains("目标角度超出"));
    }

    #[test]
    fn manual_angle_command_uses_signed_angle_and_crc() {
        let cmd = build_manual_angle_command(0x01, -2).expect("command should build");
        assert_eq!(cmd, "0106080140FE6A2A");
    }

    #[test]
    fn manual_angle_verification_accepts_westward_change() {
        assert!(did_manual_angle_take_effect(
            12.0,
            13.1,
            ManualAngleDirection::West,
            18,
        ));
    }

    #[test]
    fn manual_angle_verification_rejects_unchanged_angle() {
        assert!(!did_manual_angle_take_effect(
            12.0,
            12.2,
            ManualAngleDirection::West,
            18,
        ));
    }

    #[test]
    fn local_connection_abort_is_detected_case_insensitively() {
        assert!(is_local_connection_abort("le-connection-abort-by-local"));
        assert!(is_local_connection_abort("LE-CONNECTION-ABORT-BY-LOCAL"));
        assert!(!is_local_connection_abort(
            "le-connection-failed-to-be-established"
        ));
    }

    #[test]
    fn local_connection_abort_uses_longer_retry_delay() {
        assert_eq!(
            connect_retry_delay_ms("le-connection-abort-by-local"),
            BLE_LOCAL_ABORT_RETRY_DELAY_MS
        );
        assert_eq!(
            connect_retry_delay_ms("other-error"),
            BLE_CONNECT_RETRY_DELAY_MS
        );
    }

    #[test]
    fn bleak_fallback_only_triggers_for_local_abort_errors() {
        assert!(should_use_bleak_fallback("le-connection-abort-by-local"));
        assert!(should_use_bleak_fallback(
            "BLE connect failed: LE-CONNECTION-ABORT-BY-LOCAL"
        ));
        assert!(!should_use_bleak_fallback("❌ 未扫描到设备"));
        assert!(!should_use_bleak_fallback(
            "le-connection-failed-to-be-established"
        ));
    }

    #[test]
    fn preferred_write_candidate_prefers_fff2_over_other_writable_chars() {
        let service_uuid = Uuid::parse_str("0000fff0-0000-1000-8000-00805f9b34fb").unwrap();
        let generic_service_uuid = Uuid::parse_str("00001801-0000-1000-8000-00805f9b34fb").unwrap();
        let fff2 = BleCharacteristicCandidate::new(
            service_uuid,
            Uuid::parse_str("0000fff2-0000-1000-8000-00805f9b34fb").unwrap(),
            true,
        );
        let fff3 = BleCharacteristicCandidate::new(
            service_uuid,
            Uuid::parse_str("0000fff3-0000-1000-8000-00805f9b34fb").unwrap(),
            true,
        );
        let generic = BleCharacteristicCandidate::new(
            generic_service_uuid,
            Uuid::parse_str("00002a05-0000-1000-8000-00805f9b34fb").unwrap(),
            true,
        );

        let selected = select_preferred_write_candidate(&[generic, fff3, fff2]).unwrap();
        assert_eq!(
            selected.characteristic_uuid,
            Uuid::parse_str("0000fff2-0000-1000-8000-00805f9b34fb").unwrap()
        );
    }

    #[test]
    fn preferred_write_candidate_falls_back_to_first_writable_when_vendor_uuids_absent() {
        let service_uuid = Uuid::parse_str("00001801-0000-1000-8000-00805f9b34fb").unwrap();
        let first = BleCharacteristicCandidate::new(
            service_uuid,
            Uuid::parse_str("00002a05-0000-1000-8000-00805f9b34fb").unwrap(),
            true,
        );
        let second = BleCharacteristicCandidate::new(
            service_uuid,
            Uuid::parse_str("00002a06-0000-1000-8000-00805f9b34fb").unwrap(),
            true,
        );

        let selected = select_preferred_write_candidate(&[first.clone(), second]).unwrap();
        assert_eq!(selected.characteristic_uuid, first.characteristic_uuid);
    }

    #[test]
    fn bleak_helper_path_resolves_inside_iot_controller_package() {
        let helper_path = bleak_fallback_helper_path();
        assert!(helper_path.ends_with("scripts/bleak_fallback.py"));
    }

    #[test]
    fn manufacturer_data_with_company_id_prefixes_header() {
        let mut data = HashMap::new();
        let value = manufacturer_value_without_header();
        data.insert(0x1188, value.to_vec());
        let protocol = extract_protocol_from_manufacturer_data(&data).expect("protocol not found");
        assert_eq!(protocol.len(), 26);
        assert_eq!(&protocol[..2], &[0x88, 0x11]);
        assert_eq!(&protocol[2..], value.as_slice());
    }

    #[test]
    fn manufacturer_data_with_header_value_is_used() {
        let mut data = HashMap::new();
        let value = invalid_checksum_sample().to_vec();
        data.insert(0x1188, value.clone());
        let protocol = extract_protocol_from_manufacturer_data(&data).expect("protocol not found");
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
