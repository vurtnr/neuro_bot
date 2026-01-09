use r2r::{self, Publisher};

//接口库引入
use r2r::robot_interfaces::msg::FaceEmotion;

#[derive(Clone)]
pub struct EmotionManager {
    publisher: r2r::Publisher<FaceEmotion>,
}

impl EmotionManager {
    pub fn new(node: &mut r2r::Node) -> Result<Self, r2r::Error> {
        let publisher = node
            .create_publisher::<FaceEmotion>("/robot/face_emotion", r2r::QosProfile::default())?;
        Ok(Self {publisher})
    }

    pub fn set_happy(&self){
        self.publish("happy");
    }

    pub fn set_thinking(&self){
        self.publish("thinking");
    }

    pub fn set_neutral(&self){
        self.publish("neutral");
    }

    fn publish(&self,emotion_str: &str){
        let mut msg = FaceEmotion::default();
        msg.emotion = emotion_str.to_string();

        if let Err(e) = self.publisher.publish(&msg) {
            r2r::log_error!("brain_core","Failed to send emotion: {}",e);
        }else {
            r2r::log_info!("brain_core", "Expresson changed to: {}", emotion_str);
        }
    }
}
