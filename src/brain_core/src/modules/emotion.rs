//
use r2r;
// 引入接口定义
use r2r::robot_interfaces::msg::FaceEmotion;

#[derive(Clone)]
pub struct EmotionManager {
    publisher: r2r::Publisher<FaceEmotion>,
}

impl EmotionManager {
    pub fn new(node: &mut r2r::Node) -> Result<Self, r2r::Error> {
        // Topic 对齐: /robot/face_emotion
        let publisher = node
            .create_publisher::<FaceEmotion>("/robot/face_emotion", r2r::QosProfile::default())?;
        Ok(Self { publisher })
    }

    pub fn set_happy(&self) {
        self.publish("happy");
    }

    pub fn set_thinking(&self) {
        self.publish("thinking");
    }

    pub fn set_neutral(&self) {
        self.publish("neutral");
    }
    
    // 新增：监听状态 (对应屏幕的 Magenta 色)
    pub fn set_listening(&self) {
        self.publish("listening");
    }

    fn publish(&self, emotion_str: &str) {
        let mut msg = FaceEmotion::default();
        msg.emotion = emotion_str.to_string();

        if let Err(e) = self.publisher.publish(&msg) {
            r2r::log_error!("brain_core", "Failed to send emotion: {}", e);
        } else {
            // 减少日志刷屏，仅调试时开启
            // r2r::log_info!("brain_core", "Emotion -> {}", emotion_str);
        }
    }
}