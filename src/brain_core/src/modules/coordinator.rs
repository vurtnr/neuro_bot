use super::state::NeuralLinkPayload;
use serde::Deserialize;

// --- 1. 定义 LLM 结构化输出的 JSON 结构 ---
// 使用 #[serde(default)] 保证如果 LLM 漏掉某些字段时不会解析崩溃
#[derive(Debug, Deserialize)]
struct LlmResponse {
    #[serde(default)]
    r#type: String,  // "chat", "action", "control"
    #[serde(default)]
    intent: String,  // 用于外部控制: "POWER_ON", "POWER_OFF" 等
    #[serde(default)]
    cmd: String,     // 用于肢体动作: "WAVE", "GIMBAL"
    #[serde(default)]
    params: String,  // 附加参数
    #[serde(default)]
    reply: String,   // 语音播报内容
}

#[derive(Debug, Clone, PartialEq)]
pub enum Mode {
    Idle,
    BleConnecting,
    AudioThinking,
    AudioSpeaking,
}

#[derive(Debug, Clone)]
pub enum Event {
    VisionFound(NeuralLinkPayload),
    BleResult { success: bool, message: String },
    AudioFinal(String),
    AudioLlmResult { success: bool, answer: String },
    AudioDone,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    Speak(String),
    StartLlm(String),
    SetEmotion(String),
    SetRobotState { state: String, detail: String },
    RequestBle(BleRequest),
    BodyMove { cmd: String, params: String }, // 控制总线舵机
}

#[derive(Debug, Clone, PartialEq)]
pub struct BleRequest {
    pub mac: String,
    pub service_uuid: String,
    pub characteristic_uuid: String,
    pub command: String,
}

pub struct Coordinator {
    mode: Mode,
    current_gimbal_angle: i32,                // 记忆：当前云台角度
    connected_device: Option<BleRequest>,     // ✨记忆：当前连接的外部设备上下文
}

fn normalize_uuid_field(value: Option<String>) -> String {
    let trimmed = value.unwrap_or_default().trim().to_string();
    if trimmed.is_empty() { "AUTO".to_string() } else { trimmed }
}

fn normalize_command_field(value: Option<String>) -> String {
    let trimmed = value.unwrap_or_default().trim().to_string();
    if trimmed.is_empty() { "NOOP".to_string() } else { trimmed }
}

impl Coordinator {
    pub fn new() -> Self {
        Self {
            mode: Mode::Idle,
            current_gimbal_angle: 0,
            connected_device: None, // 初始时没有任何设备连接
        }
    }

    pub fn mode(&self) -> Mode {
        self.mode.clone()
    }

    pub fn on_event(&mut self, event: Event) -> Vec<Action> {
        match (&self.mode, event) {

            // ==========================================
            // 阶段 1：视觉发现设备 -> 记录上下文并连接
            // ==========================================
            (Mode::Idle, Event::VisionFound(payload)) => {
                self.mode = Mode::BleConnecting;

                let req = BleRequest {
                    mac: payload.m,
                    service_uuid: normalize_uuid_field(payload.s),
                    characteristic_uuid: normalize_uuid_field(payload.c),
                    command: normalize_command_field(payload.d),
                };

                // ✨ 记忆这个设备的通信特征，假设马上会连接成功
                self.connected_device = Some(req.clone());

                vec![
                    Action::Speak("已识别到设备，开始连接".to_string()),
                    Action::SetEmotion("happy".to_string()),
                    Action::SetRobotState {
                        state: "BUSY".to_string(),
                        detail: "Bluetooth Connecting".to_string(),
                    },
                    Action::RequestBle(req),
                ]
            }

            // ==========================================
            // 阶段 2：连接结果处理 -> 失败则清除记忆
            // ==========================================
            (Mode::BleConnecting, Event::BleResult { success, .. }) => {
                self.mode = Mode::Idle;

                if !success {
                    // 如果连接失败，清除设备记忆
                    self.connected_device = None;
                }

                let speech = if success { "蓝牙设备连接成功" } else { "蓝牙设备连接失败" };
                vec![
                    Action::Speak(speech.to_string()),
                    Action::SetEmotion("neutral".to_string()),
                    Action::SetRobotState {
                        state: "IDLE".to_string(),
                        detail: "Ready".to_string(),
                    },
                ]
            }

            // ==========================================
            // 阶段 3：语音输入处理
            // ==========================================
            (Mode::Idle, Event::AudioFinal(text)) => {
                self.mode = Mode::AudioThinking;
                vec![
                    Action::SetEmotion("thinking".to_string()),
                    Action::SetRobotState {
                        state: "THINKING".to_string(),
                        detail: "Processing".to_string(),
                    },
                    Action::StartLlm(text),
                ]
            }

            // ==========================================
            // 阶段 4：LLM意图识别与协议映射核心
            // ==========================================
            (Mode::AudioThinking, Event::AudioLlmResult { success, answer }) => {
                if !success {
                    self.mode = Mode::Idle;
                    return vec![Action::Speak("大脑连接断开了".to_string())];
                }

                self.mode = Mode::AudioSpeaking;
                let mut actions = Vec::new();

                // 尝试提取并解析 JSON (防止 LLM 输出带有 Markdown 标记)
                let json_str = if let (Some(start), Some(end)) = (answer.find('{'), answer.rfind('}')) {
                    &answer[start..=end]
                } else {
                    &answer
                };

                match serde_json::from_str::<LlmResponse>(json_str) {
                    Ok(llm_resp) => {
                        // 1. 优先添加语音播报回复
                        actions.push(Action::Speak(llm_resp.reply.clone()));

                        // 2. 路由派发动作
                        match llm_resp.r#type.as_str() {
                            // 🤖 【分支 A：控制外部 IoT 设备】
                            "control" => {
                                if let Some(mut target_req) = self.connected_device.clone() {
                                    // ⚡️ 这里是协议映射表（意图 -> Hex代码）
                                    // 未来可以将这部分抽离到配置文件中
                                    let hex_cmd = match llm_resp.intent.as_str() {
                                        "POWER_ON" => "01050000FF008C3A",   // 示例：Modbus 继电器开
                                        "POWER_OFF" => "010500000000CDCA",  // 示例：Modbus 继电器关
                                        "READ_DATA" => "010300000002C40B",  // 示例：Modbus 读传感器
                                        _ => "NOOP"
                                    };

                                    if hex_cmd != "NOOP" {
                                        println!("📡 外部控制路由: {} -> {}", llm_resp.intent, hex_cmd);
                                        target_req.command = hex_cmd.to_string();
                                        actions.push(Action::RequestBle(target_req));
                                    } else {
                                        println!("⚠️ 收到未知的控制意图: {}", llm_resp.intent);
                                    }
                                } else {
                                    // 如果当前没有记忆中的连接设备，覆盖之前的 reply
                                    actions.clear();
                                    actions.push(Action::Speak("我还不知道你要控制哪个设备，请先让我看一看它的二维码。".to_string()));
                                }
                            }

                            // 🦾 【分支 B：控制自身肢体】
                            "action" => {
                                match llm_resp.cmd.as_str() {
                                    "WAVE" => {
                                        actions.push(Action::BodyMove { cmd: "WAVE".to_string(), params: "".to_string() });
                                    }
                                    "GIMBAL" => {
                                        let target_angle = if llm_resp.params == "RESET" {
                                            0
                                        } else {
                                            let delta: i32 = llm_resp.params.parse().unwrap_or(0);
                                            (self.current_gimbal_angle + delta).clamp(-90, 90)
                                        };
                                        self.current_gimbal_angle = target_angle;
                                        actions.push(Action::BodyMove { cmd: "GIMBAL".to_string(), params: target_angle.to_string() });
                                    }
                                    _ => {}
                                }
                            }

                            // 💬 【分支 C：纯聊天】
                            _ => {
                                // "chat" 已经在上方被推入 Speak 动作中了
                            }
                        }
                    }
                    Err(_) => {
                        // 解析 JSON 失败，说明是普通聊天内容（没有大括号），直接播报
                        actions.push(Action::Speak(answer));
                    }
                }

                actions.push(Action::SetRobotState {
                    state: "SPEAKING".to_string(),
                    detail: "Task Executing".to_string(),
                });
                actions
            }

            (Mode::AudioSpeaking, Event::AudioDone) => {
                self.mode = Mode::Idle;
                vec![
                    Action::SetEmotion("neutral".to_string()),
                    Action::SetRobotState {
                        state: "IDLE".to_string(),
                        detail: "Ready".to_string(),
                    },
                ]
            }

            _ => Vec::new(),
        }
    }
}
