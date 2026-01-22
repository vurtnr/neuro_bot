import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from robot_interfaces.msg import VisionResult, RobotState
from cv_bridge import CvBridge
import cv2
import json
import numpy as np

# 尝试导入 pyzbar
try:
    from pyzbar.pyzbar import decode, ZBarSymbol
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False


class QRNode(Node):
    def __init__(self):
        super().__init__('vision_qr_node')

        # 必须使用 Best Effort 配合摄像头
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

        # 订阅机器人状态，用于在 IDLE 时重置处理状态
        self.state_sub = self.create_subscription(
            RobotState,
            '/robot/state',
            self.state_callback,
            QoSProfile(depth=10))

        self.bridge = CvBridge()

        # 状态标志
        self.frame_count = 0
        self.is_processing = False  # True=正在处理，不响应新二维码
        self.last_published_content = None

        if PYZBAR_AVAILABLE:
            self.get_logger().info('✅ 视觉引擎就绪 (pyzbar 极速模式)')
        else:
            self.get_logger().warn('⚠️ 警告: 未检测到 pyzbar，将使用 OpenCV (识别率较低)')
            self.detector = cv2.QRCodeDetector()

        self.get_logger().info('🔌 已订阅话题: /camera_driver/image_raw, /robot/state')

    def state_callback(self, msg):
        """接收机器人状态，当变为 IDLE 时重置处理状态"""
        if msg.state == "IDLE" and self.is_processing:
            self.is_processing = False
            self.get_logger().info('🔄 视觉引擎已重置，可响应新二维码')

    def restore_mac(self, compact_mac):
        """将 D66562... 还原为 D6:65:62..."""
        if len(compact_mac) == 12:
            return ":".join(compact_mac[i:i+2] for i in range(0, 12, 2))
        return compact_mac

    def listener_callback(self, msg):
        # 如果正在处理中，跳过识别
        if self.is_processing:
            return

        self.frame_count += 1
        if self.frame_count % 60 == 0:
            self.get_logger().info(f'📺 监控中... (分辨率: {msg.width}x{msg.height})')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 图像增强: 转灰度 + 直方图均衡化
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            enhanced = cv2.equalizeHist(gray)

            detected_contents = []

            # --- 核心识别逻辑 ---
            if PYZBAR_AVAILABLE:
                objs = decode(enhanced, symbols=[ZBarSymbol.QRCODE])
                for obj in objs:
                    detected_contents.append(obj.data.decode("utf-8"))
            else:
                try:
                    data, points, _ = self.detector.detectAndDecode(cv_image)
                    if points is not None and data:
                        detected_contents.append(data)
                except Exception as e:
                    self.get_logger().warn(f'OpenCV 识别失败: {e}')

            # 没有识别到二维码，直接返回
            if not detected_contents:
                return

            # --- 识别到二维码，设置处理状态 ---
            self.is_processing = True
            self.get_logger().info(f'🔎 识别到内容: {detected_contents}')

            for data in detected_contents:
                if not data:
                    continue

                # 容错：修复单引号
                if data.startswith("{") and "'" in data:
                    data = data.replace("'", '"')

                try:
                    obj = json.loads(data)

                    # 1. 完整协议 {"t": "ble", ...}
                    if obj.get("t") == "ble":
                        self.publish_result(data)

                    # 2. 极简协议 {"t": "b", ...} -> 转换为完整版
                    elif obj.get("t") == "b":
                        self.get_logger().info(f'⚡️ 捕获极简指令: {data}')

                        full_msg = {
                            "t": "ble",
                            "mac": self.restore_mac(obj.get("m", "")),
                            "d": obj.get("c", "")
                        }
                        full_json = json.dumps(full_msg)
                        self.publish_result(full_json)

                except json.JSONDecodeError as e:
                    self.get_logger().warn(f'JSON 解析失败: {e}, 数据: {data[:50]}...')

        except Exception as e:
            self.get_logger().error(f'System Error: {e}')
            # 发生错误时重置状态，避免卡死
            self.is_processing = False

    def publish_result(self, content_str):
        # 去重：只有内容变化时才发布
        if content_str == self.last_published_content:
            return

        self.last_published_content = content_str
        self.get_logger().info(f'🚀 发送控制指令: {content_str}')
        msg = VisionResult()
        msg.type = "ble"
        msg.content = content_str
        msg.distance = 0.5
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
