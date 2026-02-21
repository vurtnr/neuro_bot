// src/iot_controller/src/modules/cellular.rs

use r2r;
use r2r::robot_interfaces::msg::NetworkStatus;
// use std::sync::{Arc, Mutex};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::time::{sleep, Duration};
use tokio_serial::SerialPortBuilderExt;
use regex::Regex;

pub struct CellularManager {
    port_name: String,
    baud_rate: u32,
}

impl CellularManager {
    pub fn new() -> Self {
        Self {
            // ⚠️ 注意: SIM7600 的 AT 命令口通常是 ttyUSB2
            // 如果后续报错 Device Busy，可能需要换端口或配置 udev
            port_name: "/dev/ttyUSB2".to_string(),
            baud_rate: 115200,
        }
    }

    pub async fn run(&self, publisher: r2r::Publisher<NetworkStatus>) {
        println!("📡 Cellular Module Started. Listening on {}", self.port_name);

        // 正则表达式预编译
        let re_csq = Regex::new(r"\+CSQ: (\d+),(\d+)").unwrap();
        // GPS 格式: +CGPSINFO: [lat],[N/S],[long],[E/W],...
        // 示例: 3113.3432,N,12121.2333,E,...
        let re_gps = Regex::new(r"\+CGPSINFO: (\d+\.\d+),([NS]),(\d+\.\d+),([EW])").unwrap();

        loop {
            match tokio_serial::new(&self.port_name, self.baud_rate).open_native_async() {
                Ok(mut port) => {
                    println!("✅ 4G Serial Connected!");

                    // 1. 开启 GPS (幂等操作，多发几次没关系)
                    let _ = port.write_all(b"AT+CGPS=1\r\n").await;
                    sleep(Duration::from_millis(500)).await;

                    let mut buffer = [0u8; 1024];

                    loop {
                        // === 循环查询任务 ===

                        // A. 查询信号
                        if let Err(_) = port.write_all(b"AT+CSQ\r\n").await { break; }
                        sleep(Duration::from_millis(100)).await; // 等待回复

                        // B. 查询 GPS
                        if let Err(_) = port.write_all(b"AT+CGPSINFO\r\n").await { break; }
                        sleep(Duration::from_millis(200)).await; // 等待回复

                        // C. 读取数据
                        match port.read(&mut buffer).await {
                            Ok(n) if n > 0 => {
                                let output = String::from_utf8_lossy(&buffer[..n]);
                                // println!("Raw 4G: {:?}", output); // 调试时可打开

                                let mut status_msg = NetworkStatus::default();
                                status_msg.interface_name = "wwan0".to_string(); // 默认名字
                                status_msg.is_connected = true; // 既然能跑这里的代码，大概率是有网的（简化逻辑）
                                status_msg.tech = "4G".to_string();

                                // 解析信号
                                if let Some(caps) = re_csq.captures(&output) {
                                    if let Ok(rssi) = caps[1].parse::<i8>() {
                                        status_msg.signal_strength = rssi;
                                    }
                                } else {
                                    status_msg.signal_strength = 99; // 未知
                                }

                                // 解析 GPS
                                if let Some(caps) = re_gps.captures(&output) {
                                    // SIM7600 返回的是 DDMM.MMMM 格式，简单起见先直接透传
                                    // 严谨做法需要转换成 DD.DDDD 格式
                                    // 这里我们先做简单的字符串转 float，后续再优化坐标系
                                    let raw_lat = caps[1].parse::<f64>().unwrap_or(0.0);
                                    let raw_lon = caps[3].parse::<f64>().unwrap_or(0.0);

                                    // 简单的格式转换 (DDMM.MMMM -> DD.DDDD)
                                    let lat_deg = (raw_lat / 100.0).floor();
                                    let lat_min = raw_lat % 100.0;
                                    status_msg.latitude = lat_deg + (lat_min / 60.0);
                                    if &caps[2] == "S" { status_msg.latitude = -status_msg.latitude; }

                                    let lon_deg = (raw_lon / 100.0).floor();
                                    let lon_min = raw_lon % 100.0;
                                    status_msg.longitude = lon_deg + (lon_min / 60.0);
                                    if &caps[4] == "W" { status_msg.longitude = -status_msg.longitude; }
                                }

                                // 发布 ROS 消息
                                let _ = publisher.publish(&status_msg);
                            }
                            Ok(_) => {}, // 空读取
                            Err(e) => {
                                println!("❌ Serial Read Error: {}", e);
                                break; // 跳出内层循环，触发重连
                            }
                        }

                        // 1Hz 频率
                        sleep(Duration::from_secs(1)).await;
                    }
                }
                Err(e) => {
                    eprintln!("⚠️ Cannot open 4G port ({}): {}. Retrying in 5s...", self.port_name, e);
                    sleep(Duration::from_secs(5)).await;
                }
            }
        }
    }
}
