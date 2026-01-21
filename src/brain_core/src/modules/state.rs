use r2r;
use r2r::robot_interfaces::msg::RobotState;
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, PartialEq)]
pub enum InternalState {
    Idle,
    Listening,
    Thinking,
    Speaking,
    Busy(String), // 关键改动：带原因的忙碌状态
}

pub struct StateManager {
    publisher: r2r::Publisher<RobotState>,
    current_state: Arc<Mutex<InternalState>>,
}

impl StateManager {
    pub fn new(node: &mut r2r::Node) -> Result<Self, r2r::Error> {
        let publisher =
            node.create_publisher::<RobotState>("/robot/state", r2r::QosProfile::default())?;
        Ok(Self {
            publisher,
            current_state: Arc::new(Mutex::new(InternalState::Idle)),
        })
    }

    pub fn set_idle(&self) {
        self.update_state(InternalState::Idle, "IDLE", "");
    }
    pub fn set_listening(&self) {
        self.update_state(InternalState::Listening, "LISTENING", "");
    }
    pub fn set_thinking(&self) {
        self.update_state(InternalState::Thinking, "THINKING", "Processing LLM");
    }
    pub fn set_speaking(&self) {
        self.update_state(InternalState::Speaking, "SPEAKING", "TTS Playing");
    }

    // 关键方法：设置忙碌
    pub fn set_busy(&self, reason: &str) {
        self.update_state(InternalState::Busy(reason.to_string()), "BUSY", reason);
    }

    // 关键守卫：只有空闲或监听时才允许听
    pub fn can_accept_audio(&self) -> bool {
        let state = self.current_state.lock().unwrap();
        match *state {
            InternalState::Idle | InternalState::Listening => true,
            _ => false,
        }
    }

    pub fn can_accept_vision_task(&self) -> bool {
        let state = self.current_state.lock().unwrap();
        match *state {
            InternalState::Busy(_) => false,
            _ => true,
        }
    }

    fn update_state(&self, new_state: InternalState, state_str: &str, detail: &str) {
        let mut state = self.current_state.lock().unwrap();
        if *state != new_state {
            *state = new_state;
            println!("🔄 State Update: {} ({})", state_str, detail);
            let msg = RobotState {
                state: state_str.to_string(),
                detail: detail.to_string(),
            };
            let _ = self.publisher.publish(&msg);
        }
    }
}

impl Clone for StateManager {
    fn clone(&self) -> Self {
        Self {
            publisher: self.publisher.clone(),
            current_state: self.current_state.clone(),
        }
    }
}
