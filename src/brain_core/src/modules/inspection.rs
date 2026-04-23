use super::coordinator::BleRequest;
use super::state::NeuralLinkPayload;
use std::time::{Duration, Instant};

const STAGE_ACCEPTED: &str = "accepted";
const STAGE_PERMISSION_PROMPTING: &str = "permission_prompting";
const STAGE_PERMISSION_LISTENING: &str = "permission_listening";
const STAGE_PERMISSION_RETRYING: &str = "permission_retrying";
const STAGE_PERMISSION_DENIED: &str = "permission_denied";
const STAGE_PERMISSION_UNRESOLVED: &str = "permission_unresolved";
const STAGE_WAITING_FOR_QR: &str = "waiting_for_qr";
const STAGE_QR_DETECTED: &str = "qr_detected";
const STAGE_BLE_CONNECTING: &str = "ble_connecting";
const STAGE_QUERYING_DEVICE: &str = "querying_device";
const STAGE_SUCCESS: &str = "success";
const STAGE_FAILED: &str = "failed";
const INSPECTION_PERMISSION_PROMPT: &str = "是否可以获取该设备数据？";
const INSPECTION_SCAN_ANNOUNCEMENT: &str =
    "已获得授权，现在进入扫码识别设备阶段。请向机器人提供设备二维码图片，并保持在镜头范围内。";

#[derive(Debug, Clone)]
pub struct InspectionRequest {
    pub request_id: String,
    pub site_id: String,
    pub node_id: String,
    pub node_label: String,
}

#[derive(Debug, Clone)]
pub struct InspectionAngleSnapshot {
    pub actual_angle: f32,
    pub target_angle: f32,
}

#[derive(Debug, Clone)]
pub struct InspectionStatusUpdate {
    pub request_id: String,
    pub stage: String,
    pub success: bool,
    pub reason: String,
    pub message: String,
    pub angle_snapshot: Option<InspectionAngleSnapshot>,
}

#[derive(Debug, Clone)]
pub enum Action {
    PublishStatus(InspectionStatusUpdate),
    Speak(String),
    RequestBle(BleRequest),
}

#[derive(Debug, Clone, PartialEq)]
pub enum PermissionVerdict {
    Consent,
    Refusal,
    Unclear,
}

#[derive(Debug, Clone)]
pub enum Event {
    StartRequested(InspectionRequest),
    AnnouncementFinished,
    PermissionPromptFinished,
    PermissionVerdict {
        verdict: PermissionVerdict,
    },
    VisionFound(NeuralLinkPayload),
    BleResult {
        success: bool,
        message: String,
        angle_snapshot: Option<InspectionAngleSnapshot>,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum Mode {
    Idle,
    AnnouncingPermission,
    WaitingForPermission,
    RetryingPermission,
    WaitingForQr,
    BleQuerying,
}

#[derive(Debug, Clone)]
enum SessionState {
    Idle,
    AnnouncingPermission {
        request: InspectionRequest,
        attempt: u8,
    },
    WaitingForPermission {
        request: InspectionRequest,
        attempt: u8,
    },
    WaitingForQr {
        request: InspectionRequest,
        deadline: Instant,
    },
    BleQuerying {
        request: InspectionRequest,
    },
}

pub struct StartOutcome {
    pub accepted: bool,
    pub message: String,
    pub actions: Vec<Action>,
}

pub struct InspectionCoordinator {
    state: SessionState,
    scan_timeout: Duration,
}

fn normalize_uuid_field(value: Option<String>) -> String {
    let trimmed = value.unwrap_or_default().trim().to_string();
    if trimmed.is_empty() {
        "AUTO".to_string()
    } else {
        trimmed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_request() -> InspectionRequest {
        InspectionRequest {
            request_id: "req-1".to_string(),
            site_id: "site-1".to_string(),
            node_id: "cabinet-1".to_string(),
            node_label: "储能电柜 E1".to_string(),
        }
    }

    #[test]
    fn start_announces_before_waiting_for_qr() {
        let mut coordinator = InspectionCoordinator::new(Duration::from_secs(30));

        let outcome = coordinator.start(build_request());

        assert!(outcome.accepted);
        assert_eq!(coordinator.mode(), Mode::AnnouncingPermission);
        assert!(matches!(
            outcome.actions.first(),
            Some(Action::PublishStatus(update)) if update.stage == STAGE_ACCEPTED
        ));
        assert!(matches!(
            outcome.actions.get(1),
            Some(Action::PublishStatus(update)) if update.stage == STAGE_PERMISSION_PROMPTING
        ));
        assert!(matches!(
            outcome.actions.get(2),
            Some(Action::Speak(text))
                if text == INSPECTION_PERMISSION_PROMPT
        ));
        assert!(!outcome.actions.iter().any(|action| matches!(
            action,
            Action::PublishStatus(update) if update.stage == STAGE_WAITING_FOR_QR
        )));
    }

    #[test]
    fn announcement_finished_opens_waiting_for_qr_stage() {
        let mut coordinator = InspectionCoordinator::new(Duration::from_secs(30));

        let _ = coordinator.start(build_request());
        let actions = coordinator.on_event(Event::AnnouncementFinished);

        assert_eq!(coordinator.mode(), Mode::WaitingForPermission);
        assert!(matches!(
            actions.first(),
            Some(Action::PublishStatus(update)) if update.stage == STAGE_PERMISSION_LISTENING
        ));
    }

    #[test]
    fn successful_ble_result_publishes_angle_snapshot() {
        let mut coordinator = InspectionCoordinator::new(Duration::from_secs(30));
        let request = build_request();

        let _ = coordinator.start(request.clone());
        let _ = coordinator.on_event(Event::AnnouncementFinished);
        let _ = coordinator.on_event(Event::PermissionVerdict {
            verdict: PermissionVerdict::Consent,
        });
        let _ = coordinator.on_event(Event::VisionFound(NeuralLinkPayload {
            t: "b".to_string(),
            m: "D6:65:62:A0:AD:E5".to_string(),
            s: None,
            c: None,
            d: None,
            n: None,
        }));

        let actions = coordinator.on_event(Event::BleResult {
            success: true,
            message: "inspection_query_ok actual_angle=12.1 target_angle=12.3".to_string(),
            angle_snapshot: Some(InspectionAngleSnapshot {
                actual_angle: 12.1,
                target_angle: 12.3,
            }),
        });

        assert_eq!(coordinator.mode(), Mode::Idle);
        assert!(matches!(
            actions.first(),
            Some(Action::PublishStatus(update))
                if update.stage == STAGE_SUCCESS
                && update.success
                && update.message == "inspection_query_ok actual_angle=12.1 target_angle=12.3"
                && matches!(
                    update.angle_snapshot.as_ref(),
                    Some(snapshot)
                        if (snapshot.actual_angle - 12.1).abs() < 0.01
                            && (snapshot.target_angle - 12.3).abs() < 0.01
            )
        ));
    }

    #[test]
    fn consent_moves_from_permission_gate_into_waiting_for_qr() {
        let mut coordinator = InspectionCoordinator::new(Duration::from_secs(30));
        let _ = coordinator.start(build_request());

        let _ = coordinator.on_event(Event::AnnouncementFinished);
        let actions = coordinator.on_event(Event::PermissionVerdict {
            verdict: PermissionVerdict::Consent,
        });

        assert_eq!(coordinator.mode(), Mode::WaitingForQr);
        assert!(matches!(
            actions
                .iter()
                .find(|action| matches!(action, Action::PublishStatus(update) if update.stage == STAGE_WAITING_FOR_QR)),
            Some(Action::PublishStatus(update)) if update.stage == STAGE_WAITING_FOR_QR
        ));
    }

    #[test]
    fn consent_announces_scan_qr_instruction() {
        let mut coordinator = InspectionCoordinator::new(Duration::from_secs(30));
        let _ = coordinator.start(build_request());
        let _ = coordinator.on_event(Event::AnnouncementFinished);

        let actions = coordinator.on_event(Event::PermissionVerdict {
            verdict: PermissionVerdict::Consent,
        });

        assert!(matches!(
            actions.iter().find(|action| matches!(action, Action::Speak(_))),
            Some(Action::Speak(text)) if text == INSPECTION_SCAN_ANNOUNCEMENT
        ));
    }

    #[test]
    fn refusal_ends_the_session_with_permission_denied() {
        let mut coordinator = InspectionCoordinator::new(Duration::from_secs(30));
        let _ = coordinator.start(build_request());
        let _ = coordinator.on_event(Event::AnnouncementFinished);

        let actions = coordinator.on_event(Event::PermissionVerdict {
            verdict: PermissionVerdict::Refusal,
        });

        assert_eq!(coordinator.mode(), Mode::Idle);
        assert!(matches!(
            actions.last(),
            Some(Action::PublishStatus(update)) if update.stage == STAGE_PERMISSION_DENIED
        ));
    }

    #[test]
    fn second_unclear_answer_ends_as_permission_unresolved() {
        let mut coordinator = InspectionCoordinator::new(Duration::from_secs(30));
        let _ = coordinator.start(build_request());
        let _ = coordinator.on_event(Event::AnnouncementFinished);
        let _ = coordinator.on_event(Event::PermissionVerdict {
            verdict: PermissionVerdict::Unclear,
        });
        let _ = coordinator.on_event(Event::AnnouncementFinished);

        let actions = coordinator.on_event(Event::PermissionVerdict {
            verdict: PermissionVerdict::Unclear,
        });

        assert_eq!(coordinator.mode(), Mode::Idle);
        assert!(matches!(
            actions.last(),
            Some(Action::PublishStatus(update)) if update.stage == STAGE_PERMISSION_UNRESOLVED
        ));
    }

    #[test]
    fn permission_stages_require_speech_but_scan_stages_do_not() {
        let mut coordinator = InspectionCoordinator::new(Duration::from_secs(30));
        let _ = coordinator.start(build_request());

        assert!(coordinator.needs_permission_speech());

        let _ = coordinator.on_event(Event::PermissionPromptFinished);
        assert!(coordinator.needs_permission_speech());

        let _ = coordinator.on_event(Event::PermissionVerdict {
            verdict: PermissionVerdict::Consent,
        });
        assert!(!coordinator.needs_permission_speech());
    }
}

fn normalize_command_field(value: Option<String>) -> String {
    let trimmed = value.unwrap_or_default().trim().to_string();
    if trimmed.is_empty() {
        "NOOP".to_string()
    } else {
        trimmed
    }
}

impl InspectionCoordinator {
    pub fn new(scan_timeout: Duration) -> Self {
        Self {
            state: SessionState::Idle,
            scan_timeout,
        }
    }

    pub fn mode(&self) -> Mode {
        match &self.state {
            SessionState::Idle => Mode::Idle,
            SessionState::AnnouncingPermission { attempt, .. } if *attempt > 1 => {
                Mode::RetryingPermission
            }
            SessionState::AnnouncingPermission { .. } => Mode::AnnouncingPermission,
            SessionState::WaitingForPermission { attempt, .. } if *attempt > 1 => {
                Mode::RetryingPermission
            }
            SessionState::WaitingForPermission { .. } => Mode::WaitingForPermission,
            SessionState::WaitingForQr { .. } => Mode::WaitingForQr,
            SessionState::BleQuerying { .. } => Mode::BleQuerying,
        }
    }

    pub fn has_active_session(&self) -> bool {
        !matches!(self.state, SessionState::Idle)
    }

    pub fn needs_permission_speech(&self) -> bool {
        matches!(
            self.state,
            SessionState::AnnouncingPermission { .. } | SessionState::WaitingForPermission { .. }
        )
    }

    pub fn start(&mut self, request: InspectionRequest) -> StartOutcome {
        if self.has_active_session() {
            return StartOutcome {
                accepted: false,
                message: "robot_busy".to_string(),
                actions: Vec::new(),
            };
        }

        let actions = self.on_event(Event::StartRequested(request));
        StartOutcome {
            accepted: true,
            message: "Inspection accepted".to_string(),
            actions,
        }
    }

    pub fn on_event(&mut self, event: Event) -> Vec<Action> {
        match (self.state.clone(), event) {
            (SessionState::Idle, Event::StartRequested(request)) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::AnnouncingPermission {
                    request,
                    attempt: 1,
                };

                vec![
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id: request_id.clone(),
                        stage: STAGE_ACCEPTED.to_string(),
                        success: false,
                        reason: String::new(),
                        message: "Inspection session accepted".to_string(),
                        angle_snapshot: None,
                    }),
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id: request_id.clone(),
                        stage: STAGE_PERMISSION_PROMPTING.to_string(),
                        success: false,
                        reason: String::new(),
                        message: "Requesting spoken permission before reading device data"
                            .to_string(),
                        angle_snapshot: None,
                    }),
                    Action::Speak(INSPECTION_PERMISSION_PROMPT.to_string()),
                ]
            }
            (
                SessionState::AnnouncingPermission { request, attempt },
                Event::AnnouncementFinished | Event::PermissionPromptFinished,
            ) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::WaitingForPermission { request, attempt };

                vec![Action::PublishStatus(InspectionStatusUpdate {
                    request_id,
                    stage: STAGE_PERMISSION_LISTENING.to_string(),
                    success: false,
                    reason: String::new(),
                    message: "Waiting for spoken permission response".to_string(),
                    angle_snapshot: None,
                })]
            }
            (
                SessionState::WaitingForPermission { request, .. },
                Event::PermissionVerdict {
                    verdict: PermissionVerdict::Consent,
                },
            ) => {
                let request_id = request.request_id.clone();
                let node_label = request.node_label.clone();
                self.state = SessionState::WaitingForQr {
                    request,
                    deadline: Instant::now() + self.scan_timeout,
                };

                vec![
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id,
                        stage: STAGE_WAITING_FOR_QR.to_string(),
                        success: false,
                        reason: String::new(),
                        message: format!("Waiting for robot to identify {node_label}"),
                        angle_snapshot: None,
                    }),
                    Action::Speak(INSPECTION_SCAN_ANNOUNCEMENT.to_string()),
                ]
            }
            (
                SessionState::WaitingForPermission { request, .. },
                Event::PermissionVerdict {
                    verdict: PermissionVerdict::Refusal,
                },
            ) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::Idle;

                vec![Action::PublishStatus(InspectionStatusUpdate {
                    request_id,
                    stage: STAGE_PERMISSION_DENIED.to_string(),
                    success: false,
                    reason: "user_refused".to_string(),
                    message: "User explicitly refused device data access".to_string(),
                    angle_snapshot: None,
                })]
            }
            (
                SessionState::WaitingForPermission { request, attempt },
                Event::PermissionVerdict {
                    verdict: PermissionVerdict::Unclear,
                },
            ) if attempt < 2 => {
                let request_id = request.request_id.clone();
                self.state = SessionState::AnnouncingPermission {
                    request,
                    attempt: attempt + 1,
                };

                vec![
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id: request_id.clone(),
                        stage: STAGE_PERMISSION_RETRYING.to_string(),
                        success: false,
                        reason: "permission_unclear".to_string(),
                        message: "Retrying spoken permission request".to_string(),
                        angle_snapshot: None,
                    }),
                    Action::Speak(INSPECTION_PERMISSION_PROMPT.to_string()),
                ]
            }
            (
                SessionState::WaitingForPermission { request, .. },
                Event::PermissionVerdict {
                    verdict: PermissionVerdict::Unclear,
                },
            ) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::Idle;

                vec![Action::PublishStatus(InspectionStatusUpdate {
                    request_id,
                    stage: STAGE_PERMISSION_UNRESOLVED.to_string(),
                    success: false,
                    reason: "permission_timeout".to_string(),
                    message: "No explicit spoken permission received after two attempts"
                        .to_string(),
                    angle_snapshot: None,
                })]
            }
            (SessionState::WaitingForQr { request, .. }, Event::VisionFound(payload)) => {
                let request_id = request.request_id.clone();
                let ble_request = BleRequest {
                    mac: payload.m,
                    service_uuid: normalize_uuid_field(payload.s),
                    characteristic_uuid: normalize_uuid_field(payload.c),
                    command: normalize_command_field(payload.d),
                };
                self.state = SessionState::BleQuerying {
                    request: request.clone(),
                };

                vec![
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id: request_id.clone(),
                        stage: STAGE_QR_DETECTED.to_string(),
                        success: false,
                        reason: String::new(),
                        message: format!("QR detected for {}", request.node_label),
                        angle_snapshot: None,
                    }),
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id: request_id.clone(),
                        stage: STAGE_BLE_CONNECTING.to_string(),
                        success: false,
                        reason: String::new(),
                        message: format!("Connecting to device MAC {}", ble_request.mac),
                        angle_snapshot: None,
                    }),
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id,
                        stage: STAGE_QUERYING_DEVICE.to_string(),
                        success: false,
                        reason: String::new(),
                        message: "Querying device over BLE".to_string(),
                        angle_snapshot: None,
                    }),
                    Action::RequestBle(ble_request),
                ]
            }
            (
                SessionState::BleQuerying { request },
                Event::BleResult {
                    success,
                    message,
                    angle_snapshot,
                },
            ) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::Idle;

                if success {
                    vec![Action::PublishStatus(InspectionStatusUpdate {
                        request_id,
                        stage: STAGE_SUCCESS.to_string(),
                        success: true,
                        reason: String::new(),
                        message,
                        angle_snapshot,
                    })]
                } else {
                    vec![Action::PublishStatus(InspectionStatusUpdate {
                        request_id,
                        stage: STAGE_FAILED.to_string(),
                        success: false,
                        reason: "device_query_failed".to_string(),
                        message,
                        angle_snapshot: None,
                    })]
                }
            }
            _ => Vec::new(),
        }
    }

    pub fn poll_timeout(&mut self) -> Vec<Action> {
        match self.state.clone() {
            SessionState::WaitingForQr { request, deadline } if Instant::now() >= deadline => {
                let request_id = request.request_id.clone();
                self.state = SessionState::Idle;
                vec![Action::PublishStatus(InspectionStatusUpdate {
                    request_id,
                    stage: STAGE_FAILED.to_string(),
                    success: false,
                    reason: "scan_timeout".to_string(),
                    message: "QR scan timed out".to_string(),
                    angle_snapshot: None,
                })]
            }
            _ => Vec::new(),
        }
    }
}
