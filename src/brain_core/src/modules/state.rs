use r2r::robot_interfaces::msg::RobotState;
use r2r::{Publisher, QosProfile};
use serde::Deserialize;
use std::sync::{Arc, Mutex};
use std::time::Instant;

// --- 全局状态管理 ---

#[derive(Clone)]
pub struct StateManager {
    publisher: Publisher<RobotState>,
    current_state: Arc<Mutex<i32>>, // 内部仍保留 ID 用于逻辑判断 (0=IDLE)
    // 🆕 新增: 网络在线状态标记 (独立于状态机，不影响 IDLE 判断)
    is_online: Arc<Mutex<bool>>,
}

impl StateManager {
    pub fn new(node: &mut r2r::Node) -> Result<Self, r2r::Error> {
        let publisher =
            node.create_publisher::<RobotState>("/robot/state", QosProfile::default())?;
        Ok(Self {
            publisher,
            current_state: Arc::new(Mutex::new(0)),
            // 🆕 初始化: 默认先假设在线，直到收到第一次 NetworkStatus 更新
            is_online: Arc::new(Mutex::new(true)),
        })
    }

    // 🆕 新增: 设置网络状态
    pub fn set_online(&self, online: bool) {
        let mut lock = self.is_online.lock().unwrap();
        if *lock != online {
            *lock = online;
            // 可选: 这里可以打印日志或发布状态变更，但为了简化暂不处理
            println!("🌐 System Online Status Changed: {}", online);
        }
    }

    // 🆕 新增: 获取网络状态
    pub fn is_online(&self) -> bool {
        *self.is_online.lock().unwrap()
    }

    // 🟢 [Fix] 更新 Helper 方法，传入对应的字符串状态
    pub fn set_idle(&self) {
        self.publish_state(0, "IDLE", "Ready");
    }
    pub fn set_listening(&self) {
        self.publish_state(1, "LISTENING", "Waiting for speech");
    }
    pub fn set_thinking(&self) {
        self.publish_state(2, "THINKING", "Processing");
    }
    pub fn set_speaking(&self) {
        self.publish_state(3, "SPEAKING", "TTS Active");
    }

    // 🟢 [Fix] set_busy 现在真正使用了 reason 参数
    pub fn set_busy(&self, reason: &str) {
        // ID 2 对应非 IDLE 状态，防止打断
        self.publish_state(2, "BUSY", reason);
    }

    // 状态检查 (保持不变)
    pub fn can_accept_audio(&self) -> bool {
        let s = *self.current_state.lock().unwrap();
        s == 0 // 只有 IDLE 时才接受语音
    }

    pub fn can_accept_vision_task(&self) -> bool {
        let s = *self.current_state.lock().unwrap();
        s == 0 // 只有 IDLE 时才处理视觉连接
    }

    // 🟢 [Fix] 核心发布函数重构
    // 参数变化：接受 state_str (对应 msg.state) 和 detail_str (对应 msg.detail)
    fn publish_state(&self, id: i32, state_str: &str, detail_str: &str) {
        // 1. 更新内部原子状态 (用于逻辑判断)
        *self.current_state.lock().unwrap() = id;

        // 2. 构造符合新定义的 ROS 消息
        let msg = RobotState {
            state: state_str.to_string(),   // 赋值 string
            detail: detail_str.to_string(), // 赋值 string (原 message 字段已改为 detail)
        };
        let _ = self.publisher.publish(&msg);
    }
}

// --- 蓝牙异步状态机定义 (保持不变) ---

#[derive(Debug, Clone, PartialEq)]
pub enum BtLifecycle {
    Idle,
    Connecting {
        target_mac: String,
        start_time: Instant,
    },
    Connected {
        device_name: String,
    },
    Failed {
        reason: String,
        cooldown_until: Instant,
    },
}

#[derive(Debug)]
pub enum BrainEvent {
    VisionFound(NeuralLinkPayload),
    BleResult { success: bool, message: String },
    AudioFinal(String),
    AudioLlmResult { success: bool, answer: String },
    AudioDone,
    Heartbeat,
    SkinTouch {
        surface: String,
        region_v: String,
        region_h: String,
        peak_value: f32,
        total_force: f32,
    },
}

#[derive(Debug, Deserialize, Clone)]
pub struct NeuralLinkPayload {
    pub t: String,
    #[serde(alias = "mac")]
    pub m: String,

    // 兼容 {"s": "..."} 和 {"service": "..."}
    #[serde(alias = "service")]
    pub s: Option<String>,

    // 兼容 {"c": "..."} 和 {"char": "..."} (特征值 UUID)
    #[serde(alias = "char")]
    #[serde(alias = "characteristic")]
    pub c: Option<String>,

    // 兼容 {"d": "..."} 和 {"cmd": "..."} (指令数据)
    // ⚠️ Vision 层的 "cmd" 字段会映射到这里的 'd'
    #[serde(alias = "cmd")]
    #[serde(alias = "data")]
    pub d: Option<String>,

    // 兼容设备名称
    #[serde(alias = "name")]
    pub n: Option<String>,
}
