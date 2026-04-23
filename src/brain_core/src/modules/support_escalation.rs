pub const SUPPORT_ESCALATION_NODE_ID: &str = "technical-support-escalation";
const STAGE_SUPPORT_ESCALATION_REQUESTED: &str = "support_escalation_requested";
const STAGE_SUPPORT_ESCALATION_SENT: &str = "support_escalation_sent";
const STAGE_SUPPORT_ESCALATION_CANCELLED: &str = "support_escalation_cancelled";
pub const SUPPORT_ESCALATION_PROMPT: &str =
    "当前异常状态工单无法由机器人和AI自动确认根因。是否需要发送给天合光能运维部门寻求技术支持？";
pub const SUPPORT_ESCALATION_SENT: &str =
    "已将异常状态工单发送给天合光能运维部门，运维工程师会接入分析并提供技术支持。";
pub const SUPPORT_ESCALATION_CANCELLED: &str =
    "已取消发送异常状态工单。如需技术支持，可以再次点击异常状态工单。";
pub const SUPPORT_ESCALATION_UNCLEAR: &str =
    "我没有听清是否需要发送给天合光能运维部门，请直接回答发送或不发送。";

#[derive(Debug, Clone)]
pub struct SupportEscalationRequest {
    pub request_id: String,
    pub site_id: String,
    pub node_id: String,
    pub node_label: String,
}

#[derive(Debug, Clone)]
pub struct SupportEscalationStatusUpdate {
    pub request_id: String,
    pub stage: String,
    pub success: bool,
    pub reason: String,
    pub message: String,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ConfirmationVerdict {
    Confirm,
    Cancel,
    Unclear,
}

#[derive(Debug, Clone)]
pub enum Event {
    StartRequested(SupportEscalationRequest),
    ConfirmationVerdict { verdict: ConfirmationVerdict },
}

#[derive(Debug, Clone)]
pub enum Action {
    PublishStatus(SupportEscalationStatusUpdate),
    Speak(String),
}

#[derive(Debug, Clone)]
enum SessionState {
    Idle,
    AwaitingConfirmation { request: SupportEscalationRequest },
}

pub struct StartOutcome {
    pub accepted: bool,
    pub message: String,
    pub actions: Vec<Action>,
}

pub struct SupportEscalationCoordinator {
    state: SessionState,
}

impl SupportEscalationCoordinator {
    pub fn new() -> Self {
        Self {
            state: SessionState::Idle,
        }
    }

    pub fn has_active_session(&self) -> bool {
        !matches!(self.state, SessionState::Idle)
    }

    pub fn needs_confirmation_speech(&self) -> bool {
        matches!(self.state, SessionState::AwaitingConfirmation { .. })
    }

    pub fn start(&mut self, request: SupportEscalationRequest) -> StartOutcome {
        if self.has_active_session() {
            return StartOutcome {
                accepted: false,
                message: "support_escalation_busy".to_string(),
                actions: Vec::new(),
            };
        }

        let actions = self.on_event(Event::StartRequested(request));
        StartOutcome {
            accepted: true,
            message: "support_escalation_accepted".to_string(),
            actions,
        }
    }

    pub fn on_event(&mut self, event: Event) -> Vec<Action> {
        match (self.state.clone(), event) {
            (SessionState::Idle, Event::StartRequested(request)) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::AwaitingConfirmation { request };
                vec![
                    Action::PublishStatus(SupportEscalationStatusUpdate {
                        request_id,
                        stage: STAGE_SUPPORT_ESCALATION_REQUESTED.to_string(),
                        success: false,
                        reason: String::new(),
                        message: SUPPORT_ESCALATION_PROMPT.to_string(),
                    }),
                    Action::Speak(SUPPORT_ESCALATION_PROMPT.to_string()),
                ]
            }
            (
                SessionState::AwaitingConfirmation { request },
                Event::ConfirmationVerdict {
                    verdict: ConfirmationVerdict::Confirm,
                },
            ) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::Idle;
                vec![
                    Action::PublishStatus(SupportEscalationStatusUpdate {
                        request_id,
                        stage: STAGE_SUPPORT_ESCALATION_SENT.to_string(),
                        success: true,
                        reason: String::new(),
                        message: SUPPORT_ESCALATION_SENT.to_string(),
                    }),
                    Action::Speak(SUPPORT_ESCALATION_SENT.to_string()),
                ]
            }
            (
                SessionState::AwaitingConfirmation { request },
                Event::ConfirmationVerdict {
                    verdict: ConfirmationVerdict::Cancel,
                },
            ) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::Idle;
                vec![
                    Action::PublishStatus(SupportEscalationStatusUpdate {
                        request_id,
                        stage: STAGE_SUPPORT_ESCALATION_CANCELLED.to_string(),
                        success: false,
                        reason: "user_cancelled".to_string(),
                        message: SUPPORT_ESCALATION_CANCELLED.to_string(),
                    }),
                    Action::Speak(SUPPORT_ESCALATION_CANCELLED.to_string()),
                ]
            }
            (
                SessionState::AwaitingConfirmation { request },
                Event::ConfirmationVerdict {
                    verdict: ConfirmationVerdict::Unclear,
                },
            ) => {
                self.state = SessionState::AwaitingConfirmation { request };
                vec![Action::Speak(SUPPORT_ESCALATION_UNCLEAR.to_string())]
            }
            _ => Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_request() -> SupportEscalationRequest {
        SupportEscalationRequest {
            request_id: "support-1".to_string(),
            site_id: "qinghai-gonghexian".to_string(),
            node_id: SUPPORT_ESCALATION_NODE_ID.to_string(),
            node_label: "异常状态工单".to_string(),
        }
    }

    #[test]
    fn start_asks_for_trina_support_confirmation() {
        let mut coordinator = SupportEscalationCoordinator::new();
        let outcome = coordinator.start(build_request());

        assert!(outcome.accepted);
        assert!(coordinator.needs_confirmation_speech());
        assert!(matches!(
            outcome.actions.get(1),
            Some(Action::Speak(text)) if text == SUPPORT_ESCALATION_PROMPT
        ));
    }

    #[test]
    fn confirmed_request_speaks_sent_message_and_finishes() {
        let mut coordinator = SupportEscalationCoordinator::new();
        let _ = coordinator.start(build_request());

        let actions = coordinator.on_event(Event::ConfirmationVerdict {
            verdict: ConfirmationVerdict::Confirm,
        });

        assert!(!coordinator.has_active_session());
        assert!(matches!(
            actions.first(),
            Some(Action::PublishStatus(update))
                if update.stage == STAGE_SUPPORT_ESCALATION_SENT && update.success
        ));
        assert!(matches!(
            actions.get(1),
            Some(Action::Speak(text)) if text == SUPPORT_ESCALATION_SENT
        ));
    }
}
