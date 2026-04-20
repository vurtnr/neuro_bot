use std::time::Duration;

const STAGE_PATROL_STARTED: &str = "patrol_started";
const STAGE_PATROL_ANOMALY_DETECTED: &str = "patrol_anomaly_detected";
const STAGE_PATROL_COMPLETED: &str = "patrol_completed";
const STAGE_PATROL_FAILED: &str = "patrol_failed";

pub const SITE_PATROL_START_ANNOUNCEMENT: &str =
    "已接收巡检任务，当前进入场站巡检模式";
pub const SITE_PATROL_ANOMALY_ANNOUNCEMENT: &str =
    "青海场站发现参数异常设备，已上报监控平台";
pub const SITE_PATROL_COMPLETION_ANNOUNCEMENT: &str =
    "异常设备工单处理完成，已提交至合作商工单平台，我会持续跟进，巡检任务结束。";
pub const SITE_PATROL_ANOMALY_MESSAGE: &str =
    "发生巡检事件：检测到青海场站支架NCU N5 参数异常，已同步至场站监控，点击查看详细信息。";

#[derive(Debug, Clone)]
pub struct SitePatrolRequest {
    pub request_id: String,
    pub site_id: String,
    pub site_name: String,
    pub node_id: String,
    pub node_label: String,
}

#[derive(Debug, Clone)]
pub struct SitePatrolStatusUpdate {
    pub request_id: String,
    pub stage: String,
    pub success: bool,
    pub reason: String,
    pub message: String,
}

#[derive(Debug, Clone)]
pub enum Action {
    PublishStatus(SitePatrolStatusUpdate),
    Speak(String),
    ScheduleAnomaly(Duration),
}

#[derive(Debug, Clone)]
pub enum Event {
    StartRequested(SitePatrolRequest),
    AnomalyTimerElapsed,
    WorkOrderCompleted,
}

#[derive(Debug, Clone)]
enum SessionState {
    Idle,
    Active { request: SitePatrolRequest },
    AwaitingWorkOrderResolution { request: SitePatrolRequest },
}

pub struct StartOutcome {
    pub accepted: bool,
    pub message: String,
    pub actions: Vec<Action>,
}

pub struct SitePatrolCoordinator {
    state: SessionState,
    anomaly_delay: Duration,
}

impl SitePatrolCoordinator {
    pub fn new(anomaly_delay: Duration) -> Self {
        Self {
            state: SessionState::Idle,
            anomaly_delay,
        }
    }

    pub fn has_active_session(&self) -> bool {
        !matches!(self.state, SessionState::Idle)
    }

    pub fn start(&mut self, request: SitePatrolRequest) -> StartOutcome {
        if self.has_active_session() {
            return StartOutcome {
                accepted: false,
                message: "robot_busy".to_string(),
                actions: vec![Action::PublishStatus(SitePatrolStatusUpdate {
                    request_id: request.request_id,
                    stage: STAGE_PATROL_FAILED.to_string(),
                    success: false,
                    reason: "robot_busy".to_string(),
                    message: "机器人当前正在执行巡检任务，请稍后重试".to_string(),
                })],
            };
        }

        let actions = self.on_event(Event::StartRequested(request));
        StartOutcome {
            accepted: true,
            message: "site_patrol_accepted".to_string(),
            actions,
        }
    }

    pub fn on_event(&mut self, event: Event) -> Vec<Action> {
        match (self.state.clone(), event) {
            (SessionState::Idle, Event::StartRequested(request)) => {
                let request_id = request.request_id.clone();
                self.state = SessionState::Active { request };

                vec![
                    Action::PublishStatus(SitePatrolStatusUpdate {
                        request_id,
                        stage: STAGE_PATROL_STARTED.to_string(),
                        success: true,
                        reason: String::new(),
                        message: SITE_PATROL_START_ANNOUNCEMENT.to_string(),
                    }),
                    Action::Speak(SITE_PATROL_START_ANNOUNCEMENT.to_string()),
                    Action::ScheduleAnomaly(self.anomaly_delay),
                ]
            }
            (SessionState::Active { request }, Event::AnomalyTimerElapsed) => {
                self.state = SessionState::AwaitingWorkOrderResolution {
                    request: request.clone(),
                };

                vec![
                    Action::PublishStatus(SitePatrolStatusUpdate {
                        request_id: request.request_id.clone(),
                        stage: STAGE_PATROL_ANOMALY_DETECTED.to_string(),
                        success: false,
                        reason: "anomaly_detected".to_string(),
                        message: SITE_PATROL_ANOMALY_MESSAGE.to_string(),
                    }),
                    Action::Speak(SITE_PATROL_ANOMALY_ANNOUNCEMENT.to_string()),
                ]
            }
            (
                SessionState::AwaitingWorkOrderResolution { request },
                Event::WorkOrderCompleted,
            ) => {
                self.state = SessionState::Idle;

                vec![
                    Action::PublishStatus(SitePatrolStatusUpdate {
                        request_id: request.request_id,
                        stage: STAGE_PATROL_COMPLETED.to_string(),
                        success: true,
                        reason: String::new(),
                        message: "异常设备工单已完成，场站巡检流程闭环".to_string(),
                    }),
                    Action::Speak(SITE_PATROL_COMPLETION_ANNOUNCEMENT.to_string()),
                ]
            }
            _ => Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_request() -> SitePatrolRequest {
        SitePatrolRequest {
            request_id: "req-1".to_string(),
            site_id: "qinghai-gonghexian".to_string(),
            site_name: "青海场站".to_string(),
            node_id: "ncu-5".to_string(),
            node_label: "N5".to_string(),
        }
    }

    #[test]
    fn start_patrol_publishes_started_and_schedules_anomaly() {
        let mut coordinator = SitePatrolCoordinator::new(Duration::from_secs(10));
        let outcome = coordinator.start(build_request());

        assert!(outcome.accepted);
        assert!(coordinator.has_active_session());
        assert!(matches!(
            outcome.actions.first(),
            Some(Action::PublishStatus(update))
                if update.stage == STAGE_PATROL_STARTED
                    && update.message == SITE_PATROL_START_ANNOUNCEMENT
        ));
        assert!(matches!(
            outcome.actions.get(1),
            Some(Action::Speak(text)) if text == SITE_PATROL_START_ANNOUNCEMENT
        ));
        assert!(matches!(
            outcome.actions.get(2),
            Some(Action::ScheduleAnomaly(delay)) if *delay == Duration::from_secs(10)
        ));
    }

    #[test]
    fn anomaly_timer_publishes_anomaly_and_keeps_session_active_until_completion() {
        let mut coordinator = SitePatrolCoordinator::new(Duration::from_secs(10));
        let _ = coordinator.start(build_request());

        let actions = coordinator.on_event(Event::AnomalyTimerElapsed);

        assert!(coordinator.has_active_session());
        assert!(matches!(
            actions.first(),
            Some(Action::PublishStatus(update))
                if update.stage == STAGE_PATROL_ANOMALY_DETECTED
                    && update.reason == "anomaly_detected"
        ));
        assert!(matches!(
            actions.get(1),
            Some(Action::Speak(text)) if text == SITE_PATROL_ANOMALY_ANNOUNCEMENT
        ));
        assert_eq!(actions.len(), 2);
    }

    #[test]
    fn work_order_completion_releases_site_patrol_voice_lock() {
        let mut coordinator = SitePatrolCoordinator::new(Duration::from_secs(10));
        let _ = coordinator.start(build_request());
        let _ = coordinator.on_event(Event::AnomalyTimerElapsed);

        let actions = coordinator.on_event(Event::WorkOrderCompleted);

        assert!(!coordinator.has_active_session());
        assert!(matches!(
            actions.first(),
            Some(Action::PublishStatus(update))
                if update.stage == STAGE_PATROL_COMPLETED
                    && update.message == "异常设备工单已完成，场站巡检流程闭环"
        ));
        assert!(matches!(
            actions.get(1),
            Some(Action::Speak(text)) if text == SITE_PATROL_COMPLETION_ANNOUNCEMENT
        ));
    }
}
