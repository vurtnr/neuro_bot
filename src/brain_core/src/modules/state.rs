use r2r;
use r2r::robot_interfaces::msg::RobotState;

#[derive(Clone)]
pub struct StateManager {
    publisher: r2r::Publisher<RobotState>,
}

impl StateManager {
    pub fn new(node: &mut r2r::Node) -> Result<Self, r2r::Error> {
        let publisher = node.create_publisher::<RobotState>(
            "/robot/state", 
            r2r::QosProfile::default()
        )?;
        Ok(Self { publisher })
    }

    pub fn set_idle(&self) {
        // 关键修改：添加 "as i32"
        self.publish(RobotState::IDLE as i32); 
    }

    pub fn set_listening(&self) {
        // 关键修改：添加 "as i32"
        self.publish(RobotState::LISTENING as i32);
    }

    pub fn set_thinking(&self) {
        // 关键修改：添加 "as i32"
        self.publish(RobotState::THINKING as i32);
    }

    pub fn set_speaking(&self) {
        // 关键修改：添加 "as i32"
        self.publish(RobotState::SPEAKING as i32);
    }

    fn publish(&self, state_code: i32) {
        let mut msg = RobotState::default();
        msg.state = state_code; 
        
        if let Err(e) = self.publisher.publish(&msg) {
            r2r::log_error!("brain_core", "Failed to broadcast state: {}", e);
        } else {
            r2r::log_info!("brain_core", "Robot State updated to: {}", state_code);
        }
    }
}