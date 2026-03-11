use super::coordinator::BleRequest;
use super::state::NeuralLinkPayload;
use std::time::{Duration, Instant};

const STAGE_ACCEPTED: &str = "accepted";
const STAGE_WAITING_FOR_QR: &str = "waiting_for_qr";
const STAGE_QR_DETECTED: &str = "qr_detected";
const STAGE_BLE_CONNECTING: &str = "ble_connecting";
const STAGE_QUERYING_DEVICE: &str = "querying_device";
const STAGE_SUCCESS: &str = "success";
const STAGE_FAILED: &str = "failed";

#[derive(Debug, Clone)]
pub struct InspectionRequest {
    pub request_id: String,
    pub site_id: String,
    pub node_id: String,
    pub node_label: String,
}

#[derive(Debug, Clone)]
pub struct InspectionStatusUpdate {
    pub request_id: String,
    pub stage: String,
    pub success: bool,
    pub reason: String,
    pub message: String,
}

#[derive(Debug, Clone)]
pub enum Action {
    PublishStatus(InspectionStatusUpdate),
    RequestBle(BleRequest),
}

#[derive(Debug, Clone)]
pub enum Event {
    StartRequested(InspectionRequest),
    VisionFound(NeuralLinkPayload),
    BleResult { success: bool, message: String },
}

#[derive(Debug, Clone, PartialEq)]
pub enum Mode {
    Idle,
    WaitingForQr,
    BleQuerying,
}

#[derive(Debug, Clone)]
enum SessionState {
    Idle,
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
            SessionState::WaitingForQr { .. } => Mode::WaitingForQr,
            SessionState::BleQuerying { .. } => Mode::BleQuerying,
        }
    }

    pub fn has_active_session(&self) -> bool {
        !matches!(self.state, SessionState::Idle)
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
                let node_label = request.node_label.clone();
                self.state = SessionState::WaitingForQr {
                    request,
                    deadline: Instant::now() + self.scan_timeout,
                };

                vec![
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id: request_id.clone(),
                        stage: STAGE_ACCEPTED.to_string(),
                        success: false,
                        reason: String::new(),
                        message: "Inspection session accepted".to_string(),
                    }),
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id,
                        stage: STAGE_WAITING_FOR_QR.to_string(),
                        success: false,
                        reason: String::new(),
                        message: format!("Waiting for robot to identify {node_label}"),
                    }),
                ]
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
                    }),
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id: request_id.clone(),
                        stage: STAGE_BLE_CONNECTING.to_string(),
                        success: false,
                        reason: String::new(),
                        message: format!("Connecting to device MAC {}", ble_request.mac),
                    }),
                    Action::PublishStatus(InspectionStatusUpdate {
                        request_id,
                        stage: STAGE_QUERYING_DEVICE.to_string(),
                        success: false,
                        reason: String::new(),
                        message: "Querying device over BLE".to_string(),
                    }),
                    Action::RequestBle(ble_request),
                ]
            }
            (SessionState::BleQuerying { request }, Event::BleResult { success, message }) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::Idle;

                if success {
                    vec![Action::PublishStatus(InspectionStatusUpdate {
                        request_id,
                        stage: STAGE_SUCCESS.to_string(),
                        success: true,
                        reason: String::new(),
                        message: "Inspection completed successfully".to_string(),
                    })]
                } else {
                    vec![Action::PublishStatus(InspectionStatusUpdate {
                        request_id,
                        stage: STAGE_FAILED.to_string(),
                        success: false,
                        reason: "device_query_failed".to_string(),
                        message,
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
                })]
            }
            _ => Vec::new(),
        }
    }
}
