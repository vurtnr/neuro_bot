from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from queue import Empty
from threading import Thread

import rclpy

from inspection_bridge.ros_adapter import (
    CaptureSnapshotCommand,
    CompleteInspectionCommand,
    ManualAngleControlCommand,
    RosInspectionBridge,
    StartInspectionCommand,
)
from inspection_bridge.session_store import SessionStore


class InspectionRequestHandler(BaseHTTPRequestHandler):
    server: "InspectionHttpServer"

    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", self.server.cors_origin)
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        super().end_headers()

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self.end_headers()

    def do_POST(self) -> None:
        if self.path == "/inspection-sessions/start":
            self._handle_start_inspection()
            return

        if self.path == "/work-order-captures":
            self._handle_capture_snapshot()
            return

        if self.path == "/work-order-completions":
            self._handle_complete_work_order()
            return

        if self.path == "/manual-angle-controls":
            self._handle_manual_angle_control()
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def _handle_start_inspection(self) -> None:
        payload = self._read_json_body()
        if payload is None:
            return

        required_fields = self._parse_required_fields(payload)
        if required_fields is None:
            return

        request_id, site_id, node_id, node_label = required_fields
        self.server.session_store.ensure_session(request_id)
        accepted, message = self.server.ros_bridge.start_inspection(
            StartInspectionCommand(
                request_id=request_id,
                site_id=site_id,
                node_id=node_id,
                node_label=node_label,
            )
        )

        if not accepted:
            self.server.session_store.append_event(
                request_id,
                {
                    "requestId": request_id,
                    "event": "failed",
                    "success": False,
                    "reason": message,
                    "message": "机器人忙碌或巡检服务不可用，请重试",
                },
            )
            self._write_json(
                HTTPStatus.CONFLICT,
                {"accepted": False, "message": message},
            )
            return

        self._write_json(
            HTTPStatus.ACCEPTED,
            {"accepted": True, "message": message, "requestId": request_id},
        )

    def _handle_capture_snapshot(self) -> None:
        payload = self._read_json_body()
        if payload is None:
            return

        required_fields = self._parse_required_fields(payload)
        if required_fields is None:
            return

        request_id, site_id, node_id, node_label = required_fields
        result = self.server.ros_bridge.capture_snapshot(
            CaptureSnapshotCommand(
                request_id=request_id,
                site_id=site_id,
                node_id=node_id,
                node_label=node_label,
            )
        )

        if not result.get("success"):
            self._write_json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                result,
            )
            return

        self._write_json(HTTPStatus.OK, result)

    def _handle_complete_work_order(self) -> None:
        payload = self._read_json_body()
        if payload is None:
            return

        required_fields = self._parse_required_fields(payload)
        if required_fields is None:
            return

        request_id, site_id, node_id, node_label = required_fields
        accepted, message = self.server.ros_bridge.complete_inspection(
            CompleteInspectionCommand(
                request_id=request_id,
                site_id=site_id,
                node_id=node_id,
                node_label=node_label,
            )
        )

        if not accepted:
            self._write_json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                {"success": False, "message": message},
            )
            return

        self._write_json(
            HTTPStatus.ACCEPTED,
            {"success": True, "message": message},
        )

    def _handle_manual_angle_control(self) -> None:
        payload = self._read_json_body()
        if payload is None:
            return

        required_fields = self._parse_required_fields(payload)
        if required_fields is None:
            return

        direction = str(payload.get("direction", "")).strip().lower()
        if direction not in {"west", "east"}:
            self._write_json(
                HTTPStatus.BAD_REQUEST,
                {
                    "success": False,
                    "message": "invalid_direction",
                    "errorCode": "invalid_direction",
                },
            )
            return

        raw_delta_angle = payload.get("deltaAngle")
        has_delta_angle = raw_delta_angle is not None and raw_delta_angle != ""
        delta_angle = 0

        if has_delta_angle:
            if isinstance(raw_delta_angle, bool) or not isinstance(raw_delta_angle, int):
                self._write_json(
                    HTTPStatus.BAD_REQUEST,
                    {
                        "success": False,
                        "message": "deltaAngle must be an integer",
                        "errorCode": "invalid_delta_angle",
                    },
                )
                return
            delta_angle = raw_delta_angle

        request_id, site_id, node_id, node_label = required_fields
        result = self.server.ros_bridge.manual_angle_control(
            ManualAngleControlCommand(
                request_id=request_id,
                site_id=site_id,
                node_id=node_id,
                node_label=node_label,
                direction=direction,
                has_delta_angle=has_delta_angle,
                delta_angle=delta_angle,
            )
        )

        if not result.get("success"):
            self._write_json(HTTPStatus.CONFLICT, result)
            return

        self._write_json(HTTPStatus.OK, result)

    def do_GET(self) -> None:
        prefix = "/inspection-sessions/"
        suffix = "/events"
        if not self.path.startswith(prefix) or not self.path.endswith(suffix):
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        request_id = self.path[len(prefix) : -len(suffix)]
        if not request_id:
            self.send_error(HTTPStatus.BAD_REQUEST)
            return

        queue, history, terminal = self.server.session_store.subscribe(request_id)
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        try:
            for item in history:
                self.wfile.write(self.server.session_store.format_sse(item))
                self.wfile.flush()

            if terminal:
                return

            while True:
                try:
                    item = queue.get(timeout=15)
                except Empty:
                    self.wfile.write(b": keep-alive\n\n")
                    self.wfile.flush()
                    continue

                self.wfile.write(self.server.session_store.format_sse(item))
                self.wfile.flush()

                if item.get("event") in {"success", "failed"}:
                    return
        finally:
            self.server.session_store.unsubscribe(request_id, queue)

    def log_message(self, format: str, *args) -> None:
        self.server.ros_bridge.get_logger().info(format % args)

    def _read_json_body(self) -> dict | None:
        content_length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(content_length) if content_length else b"{}"

        try:
            return json.loads(body.decode("utf-8"))
        except json.JSONDecodeError:
            self._write_json(
                HTTPStatus.BAD_REQUEST,
                {"success": False, "message": "invalid_json"},
            )
            return None

    def _parse_required_fields(self, payload: dict) -> tuple[str, str, str, str] | None:
        request_id = str(payload.get("requestId", "")).strip()
        site_id = str(payload.get("siteId", "")).strip()
        node_id = str(payload.get("nodeId", "")).strip()
        node_label = str(payload.get("nodeLabel", "")).strip()

        if not request_id or not site_id or not node_id or not node_label:
            self._write_json(
                HTTPStatus.BAD_REQUEST,
                {"success": False, "message": "missing_required_fields"},
            )
            return None

        return request_id, site_id, node_id, node_label

    def _write_json(self, status: HTTPStatus, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class InspectionHttpServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        session_store: SessionStore,
        ros_bridge: RosInspectionBridge,
        cors_origin: str,
    ) -> None:
        super().__init__(server_address, InspectionRequestHandler)
        self.session_store = session_store
        self.ros_bridge = ros_bridge
        self.cors_origin = cors_origin


def main(args=None) -> None:
    rclpy.init(args=args)
    session_store = SessionStore()
    ros_bridge = RosInspectionBridge(session_store)

    host = os.getenv("INSPECTION_BRIDGE_HOST", "0.0.0.0")
    port = int(os.getenv("INSPECTION_BRIDGE_PORT", "8000"))
    cors_origin = os.getenv("INSPECTION_BRIDGE_CORS_ORIGIN", "*")

    server = InspectionHttpServer((host, port), session_store, ros_bridge, cors_origin)
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    ros_bridge.get_logger().info(
        f"Inspection bridge listening on http://{host}:{port}"
    )

    try:
        rclpy.spin(ros_bridge)
    finally:
        server.shutdown()
        server.server_close()
        ros_bridge.destroy_node()
        rclpy.shutdown()
