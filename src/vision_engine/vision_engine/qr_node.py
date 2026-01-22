import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
# 🟢 [修复点] 去掉 r2r 前缀，使用标准的 ROS2 Python 引用
from robot_interfaces.msg import VisionResult 
from cv_bridge import CvBridge
import cv2
import json

class QRNode(Node):
    def __init__(self):
        super().__init__('vision_qr_node')
        
        # 配置 QoS 为 Best Effort (适配摄像头)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera_driver/image_raw', 
            self.listener_callback,
            qos_profile)
            
        self.publisher_ = self.create_publisher(VisionResult, '/vision/result', 10)
        self.bridge = CvBridge()
        self.get_logger().info('👁️ Vision Engine Ready (Listening to /camera/image_raw with Best Effort)')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 使用 OpenCV 微信二维码检测器
            detector = cv2.wechat_qrcode_WeChatQRCode()
            res, points = detector.detectAndDecode(cv_image)
            
            if len(res) > 0:
                for data in res:
                    if not data: continue
                    
                    try:
                        # 验证是否为我们的协议格式
                        json_obj = json.loads(data)
                        if "t" in json_obj and json_obj["t"] == "ble":
                            self.get_logger().info(f'🎯 QR Found: {data}')
                            
                            result_msg = VisionResult()
                            result_msg.type = "ble"
                            result_msg.content = data
                            result_msg.distance = 0.5 
                            self.publisher_.publish(result_msg)
                            
                    except json.JSONDecodeError:
                        pass 

        except Exception as e:
            self.get_logger().error(f'CV Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()