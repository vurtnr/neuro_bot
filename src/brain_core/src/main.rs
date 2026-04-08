mod modules;

use futures::StreamExt;
use modules::coordinator::{
    Action as CoordinatorAction, Coordinator, Event as CoordinatorEvent, Mode as CoordinatorMode,
};
use modules::emotion::EmotionManager;
use modules::inspection::{
    Action as InspectionAction, Event as InspectionEvent, InspectionAngleSnapshot,
    InspectionCoordinator, InspectionRequest, InspectionStatusUpdate,
};
use modules::site_patrol::{
    Action as SitePatrolAction, Event as SitePatrolEvent, SitePatrolCoordinator,
    SitePatrolRequest, SitePatrolStatusUpdate,
};
use modules::state::{BrainEvent, NeuralLinkPayload, StateManager};
use r2r;
use r2r::robot_interfaces::msg::{
    AudioSpeech, BodyCommand, InspectionStatus, NetworkStatus, VisionResult,
};
use r2r::robot_interfaces::srv::{
    AskLLM, CompleteInspection, ConnectBluetooth, DisconnectBluetooth, StartInspection,
};
use r2r::std_msgs::msg::String as StringMsg;
use std::future::{pending, Future};
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::time;

struct InspectionBleOutcome {
    success: bool,
    message: String,
    angle_snapshot: Option<InspectionAngleSnapshot>,
}

fn spawn_control_ble_request(
    client: Arc<r2r::Client<ConnectBluetooth::Service>>,
    req: modules::coordinator::BleRequest,
) -> Pin<Box<dyn Future<Output = BrainEvent>>> {
    Box::pin(async move {
        let svc = ConnectBluetooth::Request {
            mac: req.mac,
            service_uuid: req.service_uuid,
            characteristic_uuid: req.characteristic_uuid,
            command: req.command,
        };
        match client.request(&svc) {
            Ok(future) => match time::timeout(Duration::from_secs(15), future).await {
                Ok(Ok(resp)) => BrainEvent::BleResult {
                    success: resp.success,
                    message: resp.message,
                },
                Ok(Err(e)) => BrainEvent::BleResult {
                    success: false,
                    message: format!("ROS Call Error: {}", e),
                },
                Err(_) => BrainEvent::BleResult {
                    success: false,
                    message: "Timeout".to_string(),
                },
            },
            Err(e) => BrainEvent::BleResult {
                success: false,
                message: format!("Client Request Error: {}", e),
            },
        }
    })
}

fn spawn_inspection_ble_request(
    client: Arc<r2r::Client<ConnectBluetooth::Service>>,
    req: modules::coordinator::BleRequest,
) -> Pin<Box<dyn Future<Output = InspectionBleOutcome>>> {
    Box::pin(async move {
        println!(
            "🔎 [inspection] 发起 BLE 请求: mac={} service_uuid={} characteristic_uuid={} command={}",
            req.mac, req.service_uuid, req.characteristic_uuid, req.command
        );
        let svc = ConnectBluetooth::Request {
            mac: req.mac,
            service_uuid: req.service_uuid,
            characteristic_uuid: req.characteristic_uuid,
            command: req.command,
        };
        match client.request(&svc) {
            Ok(future) => match time::timeout(Duration::from_secs(30), future).await {
                Ok(Ok(resp)) => {
                    println!(
                        "✅ [inspection] BLE 请求完成: success={} message={}",
                        resp.success, resp.message
                    );
                    InspectionBleOutcome {
                        success: resp.success,
                        message: resp.message,
                        angle_snapshot: if resp.has_device_angles {
                            Some(InspectionAngleSnapshot {
                                actual_angle: resp.actual_angle,
                                target_angle: resp.target_angle,
                            })
                        } else {
                            None
                        },
                    }
                }
                Ok(Err(e)) => {
                    let message = format!("ROS Call Error: {}", e);
                    eprintln!("❌ [inspection] {}", message);
                    InspectionBleOutcome {
                        success: false,
                        message,
                        angle_snapshot: None,
                    }
                }
                Err(_) => {
                    eprintln!("❌ [inspection] BLE 请求超时");
                    InspectionBleOutcome {
                        success: false,
                        message: "Timeout".to_string(),
                        angle_snapshot: None,
                    }
                }
            },
            Err(e) => {
                let message = format!("Client Request Error: {}", e);
                eprintln!("❌ [inspection] {}", message);
                InspectionBleOutcome {
                    success: false,
                    message,
                    angle_snapshot: None,
                }
            }
        }
    })
}

fn spawn_disconnect_ble_request(
    client: Arc<r2r::Client<DisconnectBluetooth::Service>>,
) -> Pin<Box<dyn Future<Output = (bool, String)>>> {
    Box::pin(async move {
        println!("🔌 [inspection] 发起 BLE 断连请求");
        let svc = DisconnectBluetooth::Request {};
        match client.request(&svc) {
            Ok(future) => match time::timeout(Duration::from_secs(15), future).await {
                Ok(Ok(resp)) => {
                    println!(
                        "✅ [inspection] BLE 断连完成: success={} message={}",
                        resp.success, resp.message
                    );
                    (resp.success, resp.message)
                }
                Ok(Err(e)) => {
                    let message = format!("ROS Call Error: {}", e);
                    eprintln!("❌ [inspection] {}", message);
                    (false, message)
                }
                Err(_) => {
                    eprintln!("❌ [inspection] BLE 断连超时");
                    (false, "Disconnect timeout".to_string())
                }
            },
            Err(e) => {
                let message = format!("Client Request Error: {}", e);
                eprintln!("❌ [inspection] {}", message);
                (false, message)
            }
        }
    })
}

fn publish_inspection_status(
    publisher: &r2r::Publisher<InspectionStatus>,
    state_manager: &StateManager,
    update: InspectionStatusUpdate,
) {
    match update.stage.as_str() {
        "accepted" => state_manager.set_busy("Preparing Inspection"),
        "waiting_for_qr" => state_manager.set_busy("Waiting for QR"),
        "qr_detected" => state_manager.set_busy("QR Detected"),
        "ble_connecting" => state_manager.set_busy("BLE Connecting"),
        "querying_device" => state_manager.set_busy("Querying Device"),
        "success" | "failed" => state_manager.set_idle(),
        _ => {}
    }

    let message = InspectionStatus {
        request_id: update.request_id,
        stage: update.stage,
        success: update.success,
        reason: update.reason,
        message: update.message,
        has_device_angles: update.angle_snapshot.is_some(),
        actual_angle: update
            .angle_snapshot
            .as_ref()
            .map(|snapshot| snapshot.actual_angle)
            .unwrap_or(0.0),
        target_angle: update
            .angle_snapshot
            .as_ref()
            .map(|snapshot| snapshot.target_angle)
            .unwrap_or(0.0),
    };
    let _ = publisher.publish(&message);
}

fn estimate_speech_duration(text: &str) -> Duration {
    Duration::from_secs(std::cmp::max(2, (text.chars().count() / 5) as u64))
}

fn publish_site_patrol_status(
    publisher: &r2r::Publisher<InspectionStatus>,
    state_manager: &StateManager,
    update: SitePatrolStatusUpdate,
) {
    match update.stage.as_str() {
        "patrol_started" => state_manager.set_busy("Site Patrol Active"),
        "patrol_completed" | "patrol_failed" => state_manager.set_idle(),
        _ => {}
    }

    let message = InspectionStatus {
        request_id: update.request_id,
        stage: update.stage,
        success: update.success,
        reason: update.reason,
        message: update.message,
        has_device_angles: false,
        actual_angle: 0.0,
        target_angle: 0.0,
    };
    let _ = publisher.publish(&message);
}

fn build_voice_site_patrol_request() -> SitePatrolRequest {
    let request_millis = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_millis())
        .unwrap_or(0);

    SitePatrolRequest {
        request_id: format!("voice-site-patrol-{request_millis}"),
        site_id: "qinghai-gonghexian".to_string(),
        site_name: "青海场站".to_string(),
        node_id: "ncu-5".to_string(),
        node_label: "N5".to_string(),
    }
}

fn is_start_site_patrol_intent(text: &str) -> bool {
    let normalized = text
        .chars()
        .filter(|ch| !ch.is_whitespace())
        .collect::<String>();

    if normalized.is_empty() {
        return false;
    }

    if ["不要", "不用", "取消", "停止", "结束"]
        .iter()
        .any(|candidate| normalized.contains(candidate))
    {
        return false;
    }

    let explicit_phrases = [
        "开始巡检",
        "现在开始巡检",
        "开始当前巡检",
        "执行巡检",
        "进行巡检",
        "开始场站巡检",
        "进行场站巡检",
        "执行场站巡检",
        "开始巡检任务",
        "进行巡检任务",
        "执行巡检任务",
        "进行场站巡检任务",
        "执行场站巡检任务",
        "开始场站巡检任务",
    ];

    if explicit_phrases
        .iter()
        .any(|candidate| normalized.contains(candidate))
    {
        return true;
    }

    let contains_patrol_keyword =
        normalized.contains("巡检") || normalized.contains("巡检任务");
    let contains_start_keyword = normalized.contains("开始")
        || normalized.contains("进行")
        || normalized.contains("执行");
    let contains_station_scope =
        normalized.contains("场站") || normalized.contains("当前");

    contains_patrol_keyword && contains_start_keyword && contains_station_scope
}

fn handle_site_patrol_actions(
    actions: Vec<SitePatrolAction>,
    inspection_status_pub: &r2r::Publisher<InspectionStatus>,
    state_manager: &StateManager,
    tts_publisher: &r2r::Publisher<StringMsg>,
    pending_site_patrol_anomaly: &mut Option<Pin<Box<time::Sleep>>>,
) {
    for action in actions {
        match action {
            SitePatrolAction::PublishStatus(update) => {
                publish_site_patrol_status(inspection_status_pub, state_manager, update);
            }
            SitePatrolAction::Speak(text) => {
                let _ = tts_publisher.publish(&StringMsg { data: text });
            }
            SitePatrolAction::ScheduleAnomaly(delay) => {
                *pending_site_patrol_anomaly = Some(Box::pin(time::sleep(delay)));
            }
        }
    }
}

const COMPLETE_INSPECTION_ANNOUNCEMENT: &str =
    "本次巡检处理完成，设备已归档。我将断开当前连接，等待下一次任务。";

#[cfg(test)]
mod tests {
    use super::is_start_site_patrol_intent;

    #[test]
    fn matches_common_site_patrol_phrases() {
        assert!(is_start_site_patrol_intent("开始巡检"));
        assert!(is_start_site_patrol_intent("进行场站巡检任务"));
        assert!(is_start_site_patrol_intent("执行当前巡检"));
        assert!(is_start_site_patrol_intent("现在开始场站巡检"));
    }

    #[test]
    fn rejects_negative_or_unrelated_phrases() {
        assert!(!is_start_site_patrol_intent("不要开始巡检"));
        assert!(!is_start_site_patrol_intent("取消巡检任务"));
        assert!(!is_start_site_patrol_intent("读取设备数据"));
        assert!(!is_start_site_patrol_intent("今天天气怎么样"));
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("🧠 Brain Core 2.0 (Async Actor) Starting...");

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "brain_core", "")?;

    let emotion_manager = EmotionManager::new(&mut node)?;
    let state_manager = StateManager::new(&mut node)?;

    let tts_publisher =
        node.create_publisher::<StringMsg>("/audio/tts_play", r2r::QosProfile::default())?;
    let body_pub =
        node.create_publisher::<BodyCommand>("/iot/body_command", r2r::QosProfile::default())?;
    let inspection_status_pub = node
        .create_publisher::<InspectionStatus>("/inspection/status", r2r::QosProfile::default())?;

    let bt_client = Arc::new(node.create_client::<ConnectBluetooth::Service>(
        "/iot/connect_bluetooth",
        r2r::QosProfile::default(),
    )?);
    let disconnect_bt_client = Arc::new(node.create_client::<DisconnectBluetooth::Service>(
        "/iot/disconnect_bluetooth",
        r2r::QosProfile::default(),
    )?);
    let llm_client = Arc::new(
        node.create_client::<AskLLM::Service>("/brain/ask_llm", r2r::QosProfile::default())?,
    );

    let mut speech_sub =
        node.subscribe::<AudioSpeech>("/audio/speech", r2r::QosProfile::default())?;
    let mut vision_sub =
        node.subscribe::<VisionResult>("/vision/result", r2r::QosProfile::default())?;
    let mut network_sub =
        node.subscribe::<NetworkStatus>("/system/network_status", r2r::QosProfile::default())?;
    let mut inspection_service = node.create_service::<StartInspection::Service>(
        "/inspection/start",
        r2r::QosProfile::services_default(),
    )?;
    let mut complete_inspection_service = node.create_service::<CompleteInspection::Service>(
        "/inspection/complete",
        r2r::QosProfile::services_default(),
    )?;

    let state_for_net = state_manager.clone();
    tokio::spawn(async move {
        println!("📡 Network Monitor Started...");
        while let Some(msg) = network_sub.next().await {
            let is_online = msg.is_connected && msg.signal_strength != 99;
            state_for_net.set_online(is_online);
        }
    });

    println!("🔗 System Ready. Entering Event Loop.");

    let mut coordinator = Coordinator::new();
    let mut inspection = InspectionCoordinator::new(Duration::from_secs(30));
    let mut site_patrol = SitePatrolCoordinator::new(Duration::from_secs(10));
    let mut pending_control_ble: Option<Pin<Box<dyn Future<Output = BrainEvent>>>> = None;
    let mut pending_inspection_ble: Option<Pin<Box<dyn Future<Output = InspectionBleOutcome>>>> =
        None;
    let mut pending_inspection_announcement_done: Option<Pin<Box<time::Sleep>>> = None;
    let mut pending_site_patrol_anomaly: Option<Pin<Box<time::Sleep>>> = None;
    let mut pending_completion_announcement_done: Option<Pin<Box<time::Sleep>>> = None;
    let mut pending_completion_disconnect: Option<Pin<Box<dyn Future<Output = (bool, String)>>>> =
        None;
    let mut pending_llm: Option<Pin<Box<dyn Future<Output = BrainEvent>>>> = None;
    let mut pending_audio_done: Option<Pin<Box<time::Sleep>>> = None;
    let mut site_patrol_voice_lock_notified = false;

    let mut spin_interval = time::interval(Duration::from_millis(10));
    loop {
        let mut event_to_handle: Option<BrainEvent> = None;

        tokio::select! {
            _ = spin_interval.tick() => {
                node.spin_once(Duration::from_millis(0));

                for action in inspection.poll_timeout() {
                    match action {
                        InspectionAction::PublishStatus(update) => {
                            publish_inspection_status(&inspection_status_pub, &state_manager, update);
                        }
                        InspectionAction::Speak(text) => {
                            let _ = tts_publisher.publish(&StringMsg { data: text.clone() });
                            pending_inspection_announcement_done =
                                Some(Box::pin(time::sleep(estimate_speech_duration(&text))));
                        }
                        InspectionAction::RequestBle(req) => {
                            if pending_inspection_ble.is_none() {
                                pending_inspection_ble = Some(spawn_inspection_ble_request(bt_client.clone(), req));
                            }
                        }
                    }
                }
            }
            req = inspection_service.next() => {
                if let Some(req) = req {
                    if req.message.node_id == "site-patrol" {
                        let outcome = site_patrol.start(SitePatrolRequest {
                            request_id: req.message.request_id.clone(),
                            site_id: req.message.site_id.clone(),
                            site_name: "青海场站".to_string(),
                            node_id: "ncu-5".to_string(),
                            node_label: "N5".to_string(),
                        });
                        let _ = req.respond(StartInspection::Response {
                            accepted: outcome.accepted,
                            message: outcome.message.clone(),
                        });
                        if outcome.accepted {
                            site_patrol_voice_lock_notified = false;
                        }
                        handle_site_patrol_actions(
                            outcome.actions,
                            &inspection_status_pub,
                            &state_manager,
                            &tts_publisher,
                            &mut pending_site_patrol_anomaly,
                        );
                        continue;
                    }

                    let request = InspectionRequest {
                        request_id: req.message.request_id.clone(),
                        site_id: req.message.site_id.clone(),
                        node_id: req.message.node_id.clone(),
                        node_label: req.message.node_label.clone(),
                    };

                    let outcome = inspection.start(request);
                    let _ = req.respond(StartInspection::Response {
                        accepted: outcome.accepted,
                        message: outcome.message.clone(),
                    });

                    for action in outcome.actions {
                        match action {
                            InspectionAction::PublishStatus(update) => {
                                publish_inspection_status(&inspection_status_pub, &state_manager, update);
                            }
                            InspectionAction::Speak(text) => {
                                let _ = tts_publisher.publish(&StringMsg { data: text.clone() });
                                pending_inspection_announcement_done =
                                    Some(Box::pin(time::sleep(estimate_speech_duration(&text))));
                            }
                            InspectionAction::RequestBle(req) => {
                                if pending_inspection_ble.is_none() {
                                    pending_inspection_ble = Some(spawn_inspection_ble_request(bt_client.clone(), req));
                                }
                            }
                        }
                    }
                }
            }
            req = complete_inspection_service.next() => {
                if let Some(req) = req {
                    if site_patrol.has_active_session() {
                        let _ = req.respond(CompleteInspection::Response {
                            accepted: true,
                            message: "site patrol work order completion accepted".to_string(),
                        });
                        handle_site_patrol_actions(
                            site_patrol.on_event(SitePatrolEvent::WorkOrderCompleted),
                            &inspection_status_pub,
                            &state_manager,
                            &tts_publisher,
                            &mut pending_site_patrol_anomaly,
                        );
                        site_patrol_voice_lock_notified = false;
                        continue;
                    }

                    println!(
                        "🧾 [inspection] 收到工单完成指令: request_id={} site={} node={}",
                        req.message.request_id,
                        req.message.site_id,
                        req.message.node_id,
                    );
                    state_manager.set_busy("Finishing Inspection");
                    let _ = tts_publisher.publish(&StringMsg {
                        data: COMPLETE_INSPECTION_ANNOUNCEMENT.to_string(),
                    });
                    pending_completion_announcement_done = Some(Box::pin(time::sleep(
                        estimate_speech_duration(COMPLETE_INSPECTION_ANNOUNCEMENT),
                    )));
                    let _ = req.respond(CompleteInspection::Response {
                        accepted: true,
                        message: "inspection completion accepted".to_string(),
                    });
                }
            }
            msg = vision_sub.next() => {
                if let Some(msg) = msg {
                    if let Ok(payload) = serde_json::from_str::<NeuralLinkPayload>(&msg.content) {
                        if payload.t == "ble" {
                            if inspection.has_active_session() {
                                for action in inspection.on_event(InspectionEvent::VisionFound(payload)) {
                                    match action {
                                        InspectionAction::PublishStatus(update) => {
                                            publish_inspection_status(&inspection_status_pub, &state_manager, update);
                                        }
                                        InspectionAction::Speak(text) => {
                                            let _ = tts_publisher.publish(&StringMsg { data: text.clone() });
                                            pending_inspection_announcement_done =
                                                Some(Box::pin(time::sleep(estimate_speech_duration(&text))));
                                        }
                                        InspectionAction::RequestBle(req) => {
                                            if pending_inspection_ble.is_none() {
                                                pending_inspection_ble = Some(spawn_inspection_ble_request(bt_client.clone(), req));
                                            }
                                        }
                                    }
                                }
                            } else {
                                event_to_handle = Some(BrainEvent::VisionFound(payload));
                            }
                        }
                    }
                }
            }
            _ = async {
                if let Some(fut) = pending_completion_announcement_done.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<()>().await
                }
            } => {
                pending_completion_announcement_done = None;
                if pending_completion_disconnect.is_none() {
                    pending_completion_disconnect = Some(
                        spawn_disconnect_ble_request(disconnect_bt_client.clone())
                    );
                }
            }
            msg = speech_sub.next() => {
                if let Some(msg) = msg {
                    if msg.is_final {
                        if site_patrol.has_active_session() {
                            if !site_patrol_voice_lock_notified {
                                site_patrol_voice_lock_notified = true;
                                let _ = tts_publisher.publish(&StringMsg {
                                    data: "当前正在执行巡检任务，暂不接收语音指令".to_string(),
                                });
                            }
                            continue;
                        }

                        if inspection.has_active_session() {
                            println!("🤖 Inspection active, ignoring audio request");
                            continue;
                        }

                        if !state_manager.is_online() {
                            println!("🚫 离线模式: 拦截语音请求");
                            state_manager.set_busy("Network Offline");
                            let _ = tts_publisher.publish(&StringMsg {
                                data: "网络信号不佳，我暂时无法连接大脑。".to_string(),
                            });
                            let sm_clone = state_manager.clone();
                            tokio::spawn(async move {
                                time::sleep(Duration::from_secs(3)).await;
                                sm_clone.set_idle();
                            });
                            continue;
                        }

                        if is_start_site_patrol_intent(&msg.text) {
                            let outcome = site_patrol.start(build_voice_site_patrol_request());
                            if outcome.accepted {
                                site_patrol_voice_lock_notified = false;
                            } else if !outcome.message.is_empty() {
                                let _ = tts_publisher.publish(&StringMsg {
                                    data: "机器人当前正在执行巡检任务，请稍后重试".to_string(),
                                });
                            }
                            handle_site_patrol_actions(
                                outcome.actions,
                                &inspection_status_pub,
                                &state_manager,
                                &tts_publisher,
                                &mut pending_site_patrol_anomaly,
                            );
                            continue;
                        }

                        event_to_handle = Some(BrainEvent::AudioFinal(msg.text));
                    }
                }
            }
            _ = async {
                if let Some(fut) = pending_site_patrol_anomaly.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<()>().await
                }
            } => {
                pending_site_patrol_anomaly = None;
                handle_site_patrol_actions(
                    site_patrol.on_event(SitePatrolEvent::AnomalyTimerElapsed),
                    &inspection_status_pub,
                    &state_manager,
                    &tts_publisher,
                    &mut pending_site_patrol_anomaly,
                );
            }
            result = async {
                if let Some(fut) = pending_completion_disconnect.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<(bool, String)>().await
                }
            } => {
                pending_completion_disconnect = None;
                if result.0 {
                    println!("✅ [inspection] 工单收尾完成: {}", result.1);
                } else {
                    eprintln!("❌ [inspection] 工单收尾断连失败: {}", result.1);
                }
                state_manager.set_idle();
            }
            event = async {
                if let Some(fut) = pending_control_ble.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<BrainEvent>().await
                }
            } => {
                pending_control_ble = None;
                event_to_handle = Some(event);
            }
            result = async {
                if let Some(fut) = pending_inspection_ble.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<InspectionBleOutcome>().await
                }
            } => {
                pending_inspection_ble = None;
                for action in inspection.on_event(InspectionEvent::BleResult {
                    success: result.success,
                    message: result.message,
                    angle_snapshot: result.angle_snapshot,
                }) {
                    match action {
                        InspectionAction::PublishStatus(update) => {
                            publish_inspection_status(&inspection_status_pub, &state_manager, update);
                        }
                        InspectionAction::Speak(text) => {
                            let _ = tts_publisher.publish(&StringMsg { data: text.clone() });
                            pending_inspection_announcement_done =
                                Some(Box::pin(time::sleep(estimate_speech_duration(&text))));
                        }
                        InspectionAction::RequestBle(req) => {
                            if pending_inspection_ble.is_none() {
                                pending_inspection_ble = Some(spawn_inspection_ble_request(bt_client.clone(), req));
                            }
                        }
                    }
                }
            }
            _ = async {
                if let Some(fut) = pending_inspection_announcement_done.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<()>().await
                }
            } => {
                pending_inspection_announcement_done = None;
                for action in inspection.on_event(InspectionEvent::AnnouncementFinished) {
                    match action {
                        InspectionAction::PublishStatus(update) => {
                            publish_inspection_status(&inspection_status_pub, &state_manager, update);
                        }
                        InspectionAction::Speak(text) => {
                            let _ = tts_publisher.publish(&StringMsg { data: text.clone() });
                            pending_inspection_announcement_done =
                                Some(Box::pin(time::sleep(estimate_speech_duration(&text))));
                        }
                        InspectionAction::RequestBle(req) => {
                            if pending_inspection_ble.is_none() {
                                pending_inspection_ble = Some(spawn_inspection_ble_request(bt_client.clone(), req));
                            }
                        }
                    }
                }
            }
            event = async {
                if let Some(fut) = pending_llm.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<BrainEvent>().await
                }
            } => {
                pending_llm = None;
                event_to_handle = Some(event);
            }
            _ = async {
                if let Some(fut) = pending_audio_done.as_mut() {
                    fut.as_mut().await
                } else {
                    pending::<()>().await
                }
            } => {
                pending_audio_done = None;
                event_to_handle = Some(BrainEvent::AudioDone);
            }
        }

        if let Some(event) = event_to_handle {
            let coordinator_event = match event {
                BrainEvent::VisionFound(payload) => CoordinatorEvent::VisionFound(payload),
                BrainEvent::BleResult { success, message } => {
                    println!("🔄 BLE 结果: {}", message);
                    CoordinatorEvent::BleResult { success, message }
                }
                BrainEvent::AudioFinal(text) => CoordinatorEvent::AudioFinal(text),
                BrainEvent::AudioLlmResult { success, answer } => {
                    CoordinatorEvent::AudioLlmResult { success, answer }
                }
                BrainEvent::AudioDone => CoordinatorEvent::AudioDone,
                BrainEvent::Heartbeat => continue,
            };

            let actions = coordinator.on_event(coordinator_event);
            let schedule_audio_done = matches!(coordinator.mode(), CoordinatorMode::AudioSpeaking);

            for action in actions {
                match action {
                    CoordinatorAction::Speak(text) => {
                        let _ = tts_publisher.publish(&StringMsg { data: text.clone() });
                        if schedule_audio_done {
                            let duration = std::cmp::max(2, (text.chars().count() / 5) as u64);
                            pending_audio_done =
                                Some(Box::pin(time::sleep(Duration::from_secs(duration))));
                        }
                    }
                    CoordinatorAction::StartLlm(question) => {
                        if pending_llm.is_none() {
                            println!("👂 Hearing: {}", question);
                            let client = llm_client.clone();
                            pending_llm = Some(Box::pin(async move {
                                let req = AskLLM::Request { question };
                                match client.request(&req) {
                                    Ok(future) => match future.await {
                                        Ok(resp) => BrainEvent::AudioLlmResult {
                                            success: resp.success,
                                            answer: resp.answer,
                                        },
                                        Err(e) => BrainEvent::AudioLlmResult {
                                            success: false,
                                            answer: format!("ROS Call Error: {}", e),
                                        },
                                    },
                                    Err(e) => BrainEvent::AudioLlmResult {
                                        success: false,
                                        answer: format!("Client Request Error: {}", e),
                                    },
                                }
                            }));
                        }
                    }
                    CoordinatorAction::SetEmotion(emotion) => match emotion.as_str() {
                        "happy" => emotion_manager.set_happy(),
                        "thinking" => emotion_manager.set_thinking(),
                        "listening" => emotion_manager.set_listening(),
                        _ => emotion_manager.set_neutral(),
                    },
                    CoordinatorAction::SetRobotState { state, detail } => match state.as_str() {
                        "BUSY" => state_manager.set_busy(&detail),
                        "THINKING" => state_manager.set_thinking(),
                        "SPEAKING" => state_manager.set_speaking(),
                        _ => state_manager.set_idle(),
                    },
                    CoordinatorAction::RequestBle(req) => {
                        if pending_control_ble.is_none() {
                            println!("👁️ 锁定目标: {} (CMD: {:?})", req.mac, req.command);
                            pending_control_ble =
                                Some(spawn_control_ble_request(bt_client.clone(), req));
                        }
                    }
                    CoordinatorAction::BodyMove { cmd, params } => {
                        let msg = BodyCommand {
                            cmd: cmd.clone(),
                            params: params.clone(),
                        };

                        if let Err(e) = body_pub.publish(&msg) {
                            eprintln!("❌ 发送舵机指令失败: {}", e);
                        } else {
                            println!("🚀 下发身体动作指令: cmd={}, params={}", cmd, params);
                        }
                    }
                }
            }
        }
    }
}
