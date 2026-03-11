from __future__ import annotations

from dataclasses import dataclass
import time

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import InspectionStatus
from robot_interfaces.srv import StartInspection

from inspection_bridge.session_store import SessionStore


@dataclass
class StartInspectionCommand:
    request_id: str
    site_id: str
    node_id: str
    node_label: str


class RosInspectionBridge(Node):
    def __init__(self, session_store: SessionStore) -> None:
        super().__init__("inspection_bridge")
        self.session_store = session_store
        self.client = self.create_client(StartInspection, "/inspection/start")
        self.subscription = self.create_subscription(
            InspectionStatus,
            "/inspection/status",
            self._handle_status,
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

    def _handle_status(self, msg: InspectionStatus) -> None:
        self.session_store.append_event(
            msg.request_id,
            {
                "requestId": msg.request_id,
                "event": msg.stage,
                "success": msg.success,
                "reason": msg.reason,
                "message": msg.message,
            },
        )
