import json
from threading import Lock

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import InspectionStatus, VisionResult
from robot_interfaces.srv import CaptureSnapshot
from sensor_msgs.msg import Image

from vision_engine.inspection_gate import InspectionGate
from vision_engine.qr_config import resolve_camera_topic
from vision_engine.qr_dedupe import QrContentDeduper
from vision_engine.snapshot_utils import build_snapshot_payload

# 尝试导入 pyzbar
try:
    from pyzbar.pyzbar import ZBarSymbol, decode
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
            depth=1,
        )

        self.declare_parameter('image_topic', '/camera_driver/image_raw')
        self.declare_parameter('repeat_suppression_frames', 10)
        self.declare_parameter('require_active_inspection', False)
        image_topic = resolve_camera_topic(self.get_parameter('image_topic').value)
        suppression_frames = int(
            self.get_parameter('repeat_suppression_frames').value
        )
        require_active_inspection = bool(
            self.get_parameter('require_active_inspection').value
        )
        self.deduper = QrContentDeduper(suppression_frames=suppression_frames)
        self.inspection_gate = InspectionGate(
            require_active_session=require_active_inspection
        )

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            qos_profile,
        )
        self.inspection_subscription = self.create_subscription(
            InspectionStatus,
            '/inspection/status',
            self.handle_inspection_status,
            10,
        )

        self.publisher_ = self.create_publisher(VisionResult, '/vision/result', 10)
        self.snapshot_service = self.create_service(
            CaptureSnapshot,
            '/inspection/capture_snapshot',
            self.handle_capture_snapshot,
        )
        self.bridge = CvBridge()
        self.frame_lock = Lock()
        self.latest_frame = None

        # 调试开关
        self.last_log_time = 0
        self.frame_count = 0

        if PYZBAR_AVAILABLE:
            self.get_logger().info('✅ 视觉引擎就绪 (pyzbar 极速模式)')
        else:
            self.get_logger().warn('⚠️ 警告: 未检测到 pyzbar，将使用 OpenCV (识别率较低)')
            self.detector = cv2.QRCodeDetector()

        self.get_logger().info(f'📸 订阅图像话题: {image_topic}')
        if require_active_inspection:
            self.get_logger().info('🛡️ 仅在巡检任务激活时发布二维码结果')

    def restore_mac(self, compact_mac):
        """将 D66562... 还原为 D6:65:62..."""
        if len(compact_mac) == 12:
            return ':'.join(compact_mac[i:i + 2] for i in range(0, 12, 2))
        return compact_mac

    def listener_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 60 == 0:
            self.get_logger().info(
                f'📺 监控中... (分辨率: {msg.width}x{msg.height})'
            )

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.frame_lock:
                self.latest_frame = cv_image.copy()

            # 图像增强: 转灰度 + 直方图均衡化 (对低分辨率极有帮助)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            enhanced = cv2.equalizeHist(gray)

            detected_contents = []

            # --- 核心识别逻辑 ---
            if PYZBAR_AVAILABLE:
                # 只看二维码，速度更快
                objs = decode(enhanced, symbols=[ZBarSymbol.QRCODE])
                for obj in objs:
                    detected_contents.append(obj.data.decode('utf-8'))
            else:
                # OpenCV 备选方案
                try:
                    data, points, _ = self.detector.detectAndDecode(cv_image)
                    if points is not None and data:
                        detected_contents.append(data)
                except Exception:
                    pass

            # --- 结果解析与协议转换 ---
            for data in detected_contents:
                if not data:
                    continue

                # 容错：修复单引号
                if data.startswith('{') and "'" in data:
                    data = data.replace("'", '"')

                try:
                    obj = json.loads(data)

                    # 1. 兼容完整协议 {"t": "ble", ...}
                    if obj.get('t') == 'ble':
                        if not self.inspection_gate.should_publish():
                            continue
                        if self.deduper.should_publish(data, self.frame_count):
                            self.publish_result(data)

                    # 2. 🟢 兼容极简协议 {"t": "b", ...} -> 自动转回完整版
                    elif obj.get('t') == 'b':
                        # 还原完整结构，让 brain_core 无感
                        full_msg = {
                            't': 'ble',
                            'mac': self.restore_mac(obj.get('m', '')),
                            'cmd': obj.get('c', ''),
                        }
                        full_json = json.dumps(full_msg)
                        if not self.inspection_gate.should_publish():
                            continue
                        if self.deduper.should_publish(full_json, self.frame_count):
                            self.get_logger().info(f'⚡️ 捕获极简指令: {data}')
                            self.publish_result(full_json)

                except json.JSONDecodeError:
                    pass

        except Exception as e:
            self.get_logger().error(f'System Error: {e}')

    def handle_capture_snapshot(self, request, response):
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()

        if frame is None:
            response.success = False
            response.message = 'no_camera_frame_available'
            return response

        try:
            payload = build_snapshot_payload(frame)
        except Exception as exc:
            self.get_logger().error(f'抓拍失败: {exc}')
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = (
            f"captured_snapshot_for_{request.site_id}_{request.node_id}"
        )
        response.image_base64 = payload['image_base64']
        response.mime_type = payload['mime_type']
        response.captured_at = payload['captured_at']
        response.width = payload['width']
        response.height = payload['height']
        self.get_logger().info(
            f'📸 已抓拍: site={request.site_id} node={request.node_id} label={request.node_label}'
        )
        return response

    def handle_inspection_status(self, msg):
        self.inspection_gate.update(msg.request_id, msg.stage)
        if msg.stage in ('accepted', 'waiting_for_qr'):
            self.get_logger().info(
                f'🟢 巡检会话激活: request_id={msg.request_id} stage={msg.stage}'
            )
        elif msg.stage in ('success', 'failed'):
            self.get_logger().info(
                f'⚪ 巡检会话结束: request_id={msg.request_id} stage={msg.stage}'
            )

    def publish_result(self, content_str):
        if not self.inspection_gate.should_publish():
            return
        self.get_logger().info(f'🚀 发送控制指令: {content_str}')
        msg = VisionResult()
        msg.type = 'ble'
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
