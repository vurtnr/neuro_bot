from __future__ import annotations

from dataclasses import dataclass
import time

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import InspectionStatus, SkinPressure
from robot_interfaces.srv import (
    CaptureSnapshot,
    CompleteInspection,
    ManualAngleControl,
    StartInspection,
)

from inspection_bridge.session_store import SessionStore

QINGHAI_SITE_ID = "qinghai-gonghexian"
QINGHAI_SITE_NAME = "青海场站"
QINGHAI_ANOMALY_NODE_ID = "ncu-5"
QINGHAI_ANOMALY_NODE_LABEL = "N5"
QINGHAI_SUPPORT_ESCALATION_NODE_ID = "technical-support-escalation"
QINGHAI_SUPPORT_ESCALATION_NODE_LABEL = "异常状态工单"


@dataclass
class StartInspectionCommand:
    request_id: str
    site_id: str
    node_id: str
    node_label: str


@dataclass
class CaptureSnapshotCommand:
    request_id: str
    site_id: str
    node_id: str
    node_label: str


@dataclass
class CompleteInspectionCommand:
    request_id: str
    site_id: str
    node_id: str
    node_label: str


@dataclass
class ManualAngleControlCommand:
    request_id: str
    site_id: str
    node_id: str
    node_label: str
    direction: str
    has_delta_angle: bool
    delta_angle: int


class RosInspectionBridge(Node):
    def __init__(self, session_store: SessionStore) -> None:
        super().__init__("inspection_bridge")
        self.session_store = session_store
        self.client = self.create_client(StartInspection, "/inspection/start")
        self.capture_client = self.create_client(
            CaptureSnapshot, "/inspection/capture_snapshot"
        )
        self.complete_client = self.create_client(
            CompleteInspection, "/inspection/complete"
        )
        self.manual_angle_client = self.create_client(
            ManualAngleControl, "/iot/manual_angle_control"
        )
        self.subscription = self.create_subscription(
            InspectionStatus,
            "/inspection/status",
            self._handle_status,
            10,
        )
        self.skin_subscription = self.create_subscription(
            SkinPressure,
            "/skin/pressure",
            self._handle_skin_pressure,
            10,
        )

    def start_inspection(
        self,
        command: StartInspectionCommand,
        timeout_seconds: float = 5.0,
    ) -> tuple[bool, str]:
        if not self.client.wait_for_service(timeout_sec=timeout_seconds):
            return False, "inspection service unavailable"

        request = StartInspection.Request()
        request.request_id = command.request_id
        request.site_id = command.site_id
        request.node_id = command.node_id
        request.node_label = command.node_label

        future = self.client.call_async(request)
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if future.done():
                response = future.result()
                return response.accepted, response.message
            time.sleep(0.05)
        return False, "inspection service timeout"

    def capture_snapshot(
        self,
        command: CaptureSnapshotCommand,
        timeout_seconds: float = 5.0,
    ) -> dict:
        if not self.capture_client.wait_for_service(timeout_sec=timeout_seconds):
            return {
                "success": False,
                "message": "capture service unavailable",
            }

        request = CaptureSnapshot.Request()
        request.request_id = command.request_id
        request.site_id = command.site_id
        request.node_id = command.node_id
        request.node_label = command.node_label

        future = self.capture_client.call_async(request)
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if future.done():
                try:
                    response = future.result()
                except Exception as exc:
                    return {
                        "success": False,
                        "message": f"capture service error: {exc}",
                    }
                return {
                    "success": response.success,
                    "message": response.message,
                    "imageBase64": response.image_base64,
                    "mimeType": response.mime_type,
                    "capturedAt": response.captured_at,
                    "width": response.width,
                    "height": response.height,
                }
            time.sleep(0.05)

        return {
            "success": False,
            "message": "capture service timeout",
        }

    def complete_inspection(
        self,
        command: CompleteInspectionCommand,
        timeout_seconds: float = 5.0,
    ) -> tuple[bool, str]:
        if not self.complete_client.wait_for_service(timeout_sec=timeout_seconds):
            return False, "complete inspection service unavailable"

        request = CompleteInspection.Request()
        request.request_id = command.request_id
        request.site_id = command.site_id
        request.node_id = command.node_id
        request.node_label = command.node_label

        future = self.complete_client.call_async(request)
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if future.done():
                response = future.result()
                return response.accepted, response.message
            time.sleep(0.05)
        return False, "complete inspection service timeout"

    def manual_angle_control(
        self,
        command: ManualAngleControlCommand,
        timeout_seconds: float = 12.0,
    ) -> dict:
        if not self.manual_angle_client.wait_for_service(timeout_sec=timeout_seconds):
            return {
                "success": False,
                "message": "manual angle service unavailable",
                "errorCode": "service_unavailable",
            }

        request = ManualAngleControl.Request()
        request.request_id = command.request_id
        request.site_id = command.site_id
        request.node_id = command.node_id
        request.node_label = command.node_label
        request.direction = command.direction
        request.has_delta_angle = command.has_delta_angle
        request.delta_angle = command.delta_angle

        future = self.manual_angle_client.call_async(request)
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if future.done():
                try:
                    response = future.result()
                except Exception as exc:
                    return {
                        "success": False,
                        "message": f"manual angle service error: {exc}",
                        "errorCode": "service_call_error",
                    }
                return {
                    "success": response.success,
                    "message": response.message,
                    "errorCode": response.error_code,
                    "actualAngleUsed": response.actual_angle_used,
                    "verifiedActualAngle": response.verified_actual_angle,
                    "verifiedChanged": response.verified_changed,
                    "targetAngle": response.target_angle,
                    "deltaAngleUsed": response.delta_angle_used,
                }
            time.sleep(0.05)

        return {
            "success": False,
            "message": "manual angle service timeout",
            "errorCode": "service_timeout",
        }

    def _handle_status(self, msg: InspectionStatus) -> None:
        event = {
            "requestId": msg.request_id,
            "event": msg.stage,
            "success": msg.success,
            "reason": msg.reason,
            "message": msg.message,
            "hasDeviceAngles": msg.has_device_angles,
            "actualAngle": msg.actual_angle,
            "targetAngle": msg.target_angle,
        }

        if msg.stage.startswith("patrol_"):
            event.update(
                {
                    "siteId": QINGHAI_SITE_ID,
                    "siteName": QINGHAI_SITE_NAME,
                    "nodeId": QINGHAI_ANOMALY_NODE_ID,
                    "nodeLabel": QINGHAI_ANOMALY_NODE_LABEL,
                }
            )

        if msg.stage.startswith("support_escalation_"):
            event.update(
                {
                    "siteId": QINGHAI_SITE_ID,
                    "siteName": QINGHAI_SITE_NAME,
                    "nodeId": QINGHAI_SUPPORT_ESCALATION_NODE_ID,
                    "nodeLabel": QINGHAI_SUPPORT_ESCALATION_NODE_LABEL,
                }
            )

        self.session_store.append_event(msg.request_id, event)

    def _handle_skin_pressure(self, msg: SkinPressure) -> None:
        event = {
            "event": msg.event_type,
            "surface": msg.surface,
            "region": msg.region,
            "peakRow": msg.peak_row,
            "peakCol": msg.peak_col,
            "peakAdc": msg.peak_adc,
            "normalizedPressure": msg.normalized_pressure,
            "totalPressure": msg.total_pressure,
            "activeCount": msg.active_count,
            "validTouch": msg.valid_touch,
            "frameValues": list(msg.frame_values),
            "timestamp": (
                f"{msg.stamp.sec}.{msg.stamp.nanosec:09d}"
                if hasattr(msg, "stamp")
                else ""
            ),
        }
        self.session_store.append_skin_event(event)
