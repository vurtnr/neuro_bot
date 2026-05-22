use std::sync::Arc;
use std::time::Duration;
use tokio::io::AsyncWriteExt;
use tokio::sync::Mutex;
use tokio_serial::{SerialPortBuilderExt, SerialStream};

pub struct ServoSerialManager {
    port: Arc<Mutex<SerialStream>>,
}

impl ServoSerialManager {
    // 初始化串口，传入设备路径 (如 /dev/ttyUSB0) 和波特率 (115200)
    pub fn new(device: &str, baud_rate: u32) -> Result<Self, Box<dyn std::error::Error>> {
        let port = tokio_serial::new(device, baud_rate).open_native_async()?;

        println!("🔌 总线舵机控制器已连接 (USB): {} @ {}", device, baud_rate);

        Ok(Self {
            port: Arc::new(Mutex::new(port)),
        })
    }

    // 发送原始指令，自动添加回车换行
    async fn send_raw(&self, cmd: String) {
        let mut port = self.port.lock().await;
        // 添加 \r\n 确保指令被立即刷新和识别
        let full_cmd = format!("{}\r\n", cmd);
        if let Err(e) = port.write_all(full_cmd.as_bytes()).await {
            eprintln!("❌ 串口写入失败: {}", e);
        } else {
            println!("📤 TX: {}", cmd.trim());
        }
    }

    // 格式化指令: #<ID>P<PWM>T<TIME>!
    fn fmt_cmd(id: u8, pwm: u16, time: u16) -> String {
        format!(
            "#{:03}P{:04}T{:04}!",
            id,
            pwm.clamp(500, 2500),
            time.clamp(0, 9999)
        )
    }

    // 动作: 全体归位 (0, 1, 5号舵机)
    pub async fn reset(&self) {
        println!("🤖 执行: 全体归位");
        let cmd = format!(
            "{{{}{}{}}}",
            Self::fmt_cmd(0, 1500, 1000),
            Self::fmt_cmd(1, 1500, 1000),
            Self::fmt_cmd(5, 1500, 1000)
        );
        self.send_raw(cmd).await;
    }

    // 动作: 挥手 (控制 ID 0 和 ID 1)
    pub async fn action_wave(&self) {
        println!("👋 执行动作: 挥手");
        // 1. 归中
        self.reset().await;
        tokio::time::sleep(Duration::from_millis(1000)).await;

        // 2. 循环挥动
        for _ in 0..3 {
            // 张开: 0号(左手)向外, 1号(右手)向内
            let cmd_open = format!(
                "{{{}{}}}",
                Self::fmt_cmd(0, 2000, 400),
                Self::fmt_cmd(1, 1000, 400)
            );
            self.send_raw(cmd_open).await;
            tokio::time::sleep(Duration::from_millis(500)).await;

            // 交叉
            let cmd_cross = format!(
                "{{{}{}}}",
                Self::fmt_cmd(0, 1000, 400),
                Self::fmt_cmd(1, 2000, 400)
            );
            self.send_raw(cmd_cross).await;
            tokio::time::sleep(Duration::from_millis(500)).await;
        }

        // 3. 恢复
        self.reset().await;
    }

    // 动作: 云台控制 (控制 ID 5)
    pub async fn set_gimbal(&self, angle: i32) {
        // 角度 -90~90 映射到 PWM 500~2500
        let pwm = 1500 + (angle as f32 * 11.11) as i32;
        let cmd = Self::fmt_cmd(5, pwm as u16, 500);
        self.send_raw(cmd).await;
    }
}
