#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar
from robot_interfaces.msg import VisionResult
# 🟢 必须导入 QoS 设置，否则可能连不上摄像头
from rclpy.qos import qos_profile_sensor_data 

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('vision_qr_node')
        self.get_logger().info("👁️ Vision Engine (QR Module) Started")

        # 订阅摄像头画面
        self.subscription = self.create_subscription(
            Image,
            '/camera_driver/image_raw',  # ✅ 已修正为正确的话题名
            self.image_callback,
            qos_profile_sensor_data)     # ✅ 使用传感器专用 QoS (Best Effort)
        
        self.result_pub = self.create_publisher(VisionResult, '/vision/result', 10)
        
        self.bridge = CvBridge()
        self.scan_cooldown = 0 

    def image_callback(self, msg):
        self.scan_cooldown += 1
        if self.scan_cooldown < 10: # 稍微降低频率，每10帧识别一次
            return
        self.scan_cooldown = 0

        try:
            # 1. 将 ROS 图像转为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. 调用 pyzbar 识别二维码
            decoded_objects = pyzbar.decode(cv_image)

            if not decoded_objects:
                # 没识别到时，偶尔打印一下证明在工作 (调试用)
                # self.get_logger().info("Searching...", throttle_duration_sec=2.0)
                pass

            for obj in decoded_objects:
                qr_content = obj.data.decode("utf-8")
                
                self.get_logger().info(f"🔍 Found QR Code: {qr_content}")

                # 3. 发布结果给大脑
                result_msg = VisionResult()
                result_msg.type = "qrcode"
                result_msg.content = qr_content
                # 简单估算距离 (二维码越宽离得越近)
                result_msg.distance = 100.0 / (obj.rect.width + 0.1) 
                
                self.result_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()