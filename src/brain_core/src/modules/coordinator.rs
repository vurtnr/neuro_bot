use super::state::NeuralLinkPayload;

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
}

fn normalize_uuid_field(value: Option<String>) -> String {
    let trimmed = value.unwrap_or_default().trim().to_string();
    if trimmed.is_empty() {
        "AUTO".to_string()
    } else {
        trimmed
    }
}

impl Coordinator {
    pub fn new() -> Self {
        Self { mode: Mode::Idle }
    }

    pub fn mode(&self) -> Mode {
        self.mode.clone()
    }

    pub fn on_event(&mut self, event: Event) -> Vec<Action> {
        match (&self.mode, event) {
            (Mode::Idle, Event::VisionFound(payload)) => {
                self.mode = Mode::BleConnecting;
                vec![
                    Action::Speak("已识别到设备，开始连接".to_string()),
                    Action::SetEmotion("happy".to_string()),
                    Action::SetRobotState {
                        state: "BUSY".to_string(),
                        detail: "Bluetooth Connecting".to_string(),
                    },
                    Action::RequestBle(BleRequest {
                        mac: payload.m,
                        service_uuid: normalize_uuid_field(payload.s),
                        characteristic_uuid: normalize_uuid_field(payload.c),
                        command: payload.d.unwrap_or_default(),
                    }),
                ]
            }
            (Mode::BleConnecting, Event::BleResult { success, .. }) => {
                self.mode = Mode::Idle;
                let speech = if success {
                    "蓝牙设备连接成功"
                } else {
                    "蓝牙设备连接失败"
                };
                vec![
                    Action::Speak(speech.to_string()),
                    Action::SetEmotion("neutral".to_string()),
                    Action::SetRobotState {
                        state: "IDLE".to_string(),
                        detail: "Ready".to_string(),
                    },
                ]
            }
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
            (Mode::AudioThinking, Event::AudioLlmResult { success, answer }) => {
                if success {
                    self.mode = Mode::AudioSpeaking;
                    vec![
                        Action::Speak(answer),
                        Action::SetEmotion("happy".to_string()),
                        Action::SetRobotState {
                            state: "SPEAKING".to_string(),
                            detail: "TTS Active".to_string(),
                        },
                    ]
                } else {
                    self.mode = Mode::Idle;
                    vec![
                        Action::SetEmotion("neutral".to_string()),
                        Action::SetRobotState {
                            state: "IDLE".to_string(),
                            detail: "Ready".to_string(),
                        },
                    ]
                }
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

#[cfg(test)]
mod tests {
    use super::{Action, Coordinator, Event, Mode};
    use super::NeuralLinkPayload;

    fn sample_payload() -> NeuralLinkPayload {
        NeuralLinkPayload {
            t: "ble".to_string(),
            m: "D6:65:62:00:2A:7E".to_string(),
            s: Some("".to_string()),
            c: Some("".to_string()),
            d: Some("0A030000".to_string()),
            n: Some("设备".to_string()),
        }
    }

    #[test]
    fn qr_from_idle_starts_ble_flow() {
        let mut coord = Coordinator::new();
        let actions = coord.on_event(Event::VisionFound(sample_payload()));
        assert!(actions.iter().any(|a| matches!(a, Action::Speak(_))));
        assert!(actions.iter().any(|a| matches!(a, Action::RequestBle(_))));
        assert_eq!(coord.mode(), Mode::BleConnecting);
    }

    #[test]
    fn qr_ignored_while_busy() {
        let mut coord = Coordinator::new();
        coord.on_event(Event::VisionFound(sample_payload()));
        let actions = coord.on_event(Event::VisionFound(sample_payload()));
        assert!(actions.is_empty());
        assert_eq!(coord.mode(), Mode::BleConnecting);
    }

    #[test]
    fn ble_success_reports_and_returns_idle() {
        let mut coord = Coordinator::new();
        coord.on_event(Event::VisionFound(sample_payload()));
        let actions = coord.on_event(Event::BleResult {
            success: true,
            message: "OK".into(),
        });
        assert!(actions.iter().any(|a| matches!(a, Action::Speak(_))));
        assert_eq!(coord.mode(), Mode::Idle);
    }

    #[test]
    fn ble_failure_reports_and_returns_idle() {
        let mut coord = Coordinator::new();
        coord.on_event(Event::VisionFound(sample_payload()));
        let actions = coord.on_event(Event::BleResult {
            success: false,
            message: "ERR".into(),
        });
        assert!(actions.iter().any(|a| matches!(a, Action::Speak(_))));
        assert_eq!(coord.mode(), Mode::Idle);
    }

    #[test]
    fn ble_request_uses_payload_fields() {
        let mut coord = Coordinator::new();
        let actions = coord.on_event(Event::VisionFound(sample_payload()));
        let request = actions
            .into_iter()
            .find_map(|action| match action {
                Action::RequestBle(req) => Some(req),
                _ => None,
            })
            .expect("missing ble request");

        assert_eq!(request.mac, "D6:65:62:00:2A:7E");
        assert_eq!(request.service_uuid, "AUTO");
        assert_eq!(request.characteristic_uuid, "AUTO");
        assert_eq!(request.command, "0A030000");
    }

    #[test]
    fn audio_ignored_while_ble_connecting() {
        let mut coord = Coordinator::new();
        coord.on_event(Event::VisionFound(sample_payload()));
        let actions = coord.on_event(Event::AudioFinal("hi".to_string()));
        assert!(actions.is_empty());
        assert_eq!(coord.mode(), Mode::BleConnecting);
    }
}
