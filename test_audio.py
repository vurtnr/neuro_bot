#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import AudioSpeech
import time

class MockAudioEngine(Node):
    def __init__(self):
        super().__init__('mock_audio_engine')
        # 创建发布者，话题必须和 brain_core 订阅的一致
        self.publisher_ = self.create_publisher(AudioSpeech, '/audio/speech', 10)
        self.timer = self.create_timer(3.0, self.publish_speech) # 每3秒发一次
        self.count = 0

    def publish_speech(self):
        msg = AudioSpeech()
        msg.text = f"你好 Rust, 我是 Python! (消息 #{self.count})"
        msg.confidence = 0.99
        msg.is_final = True
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.text}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MockAudioEngine()
    print("🎤 Mock Audio Engine Started. Speaking to Brain Core...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()