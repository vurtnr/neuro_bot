from __future__ import annotations

import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import SkinPressure

from skin_engine.baseline import BaselineCorrector
from skin_engine.detector import classify_pressure_frame
from skin_engine.protocol import (
    CMD_START,
    CMD_STOP,
    DEFAULT_BAUD,
    DemoSource,
    FrameAssembler,
    SerialFrameReader,
)
from skin_engine.stabilizer import TouchStabilizer


class SkinNode(Node):
    def __init__(self) -> None:
        super().__init__("skin_engine")
        self.declare_parameter("port", os.getenv("SKIN_SERIAL_PORT", "/dev/ttyUSB0"))
        self.declare_parameter("baud", int(os.getenv("SKIN_SERIAL_BAUD", DEFAULT_BAUD)))
        self.declare_parameter("demo", os.getenv("SKIN_DEMO", "0") == "1")
        self.declare_parameter("touch_threshold", float(os.getenv("SKIN_TOUCH_THRESHOLD", "150")))
        self.declare_parameter("pain_threshold", float(os.getenv("SKIN_PAIN_THRESHOLD", "900")))
        self.declare_parameter("publish_hz", float(os.getenv("SKIN_EVENT_HZ", "10")))
        self.declare_parameter("baseline_frames", int(os.getenv("SKIN_BASELINE_FRAMES", "20")))
        self.declare_parameter(
            "baseline_percentile",
            float(os.getenv("SKIN_BASELINE_PERCENTILE", "99")),
        )
        self.declare_parameter("baseline_margin", float(os.getenv("SKIN_BASELINE_MARGIN", "300")))
        self.declare_parameter("baseline_mode", os.getenv("SKIN_BASELINE_MODE", "positive"))
        self.declare_parameter("confirm_frames", int(os.getenv("SKIN_CONFIRM_FRAMES", "3")))
        self.declare_parameter("release_frames", int(os.getenv("SKIN_RELEASE_FRAMES", "4")))
        self.declare_parameter("reorder", os.getenv("SKIN_NO_REORDER", "0") != "1")

        self.publisher = self.create_publisher(SkinPressure, "/skin/pressure", 10)
        self._source = self._create_source()
        self._assembler = FrameAssembler(reorder=bool(self.get_parameter("reorder").value))
        self._baseline = BaselineCorrector(
            target_frames=int(self.get_parameter("baseline_frames").value),
            noise_percentile=float(self.get_parameter("baseline_percentile").value),
            margin=float(self.get_parameter("baseline_margin").value),
            mode=str(self.get_parameter("baseline_mode").value),
        )
        self._stabilizer = TouchStabilizer(
            confirm_frames=int(self.get_parameter("confirm_frames").value),
            release_frames=int(self.get_parameter("release_frames").value),
        )
        self._last_publish = 0.0
        self._running = True
        self._source = None
        self._thread = threading.Thread(target=self._run_capture_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("Electronic skin pressure node started")

    def destroy_node(self) -> bool:
        self._running = False
        try:
            if self._source is not None:
                self._source.write(CMD_STOP)
                self._source.close()
        except Exception:
            pass
        return super().destroy_node()

    def _create_source(self):
        if bool(self.get_parameter("demo").value):
            return DemoSource()

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        return SerialFrameReader(port, baud)

    def _run_capture_loop(self) -> None:
        publish_interval = 1.0 / max(float(self.get_parameter("publish_hz").value), 1.0)
        touch_threshold = float(self.get_parameter("touch_threshold").value)
        pain_threshold = float(self.get_parameter("pain_threshold").value)

        while self._running:
            if self._source is None:
                self._source = self._try_create_source()
                if self._source is None:
                    time.sleep(1.0)
                    continue

            try:
                packets = self._source.read_packets()
            except Exception as exc:
                self.get_logger().warning(f"skin capture read failed: {exc}")
                self._close_source()
                time.sleep(0.1)
                continue

            for packet in packets:
                frame = self._assembler.push_packet(packet)
                if frame is None:
                    continue
                corrected = self._apply_baseline(frame)
                now = time.monotonic()
                if now - self._last_publish < publish_interval:
                    continue
                self._last_publish = now
                event = classify_pressure_frame(
                    corrected,
                    touch_threshold=touch_threshold,
                    pain_threshold=pain_threshold,
                )
                event = self._stabilizer.apply(event)
                self.publisher.publish(self._to_message(event))

    def _apply_baseline(self, frame: np.ndarray) -> np.ndarray:
        was_ready = self._baseline.ready
        corrected = self._baseline.apply(frame)
        if not was_ready and self._baseline.ready:
            self.get_logger().info("Electronic skin baseline calibrated")
        return corrected

    def _to_message(self, event) -> SkinPressure:
        msg = SkinPressure()
        msg.event_type = event.event_type
        msg.surface = event.surface
        msg.region = event.region
        msg.peak_row = event.peak_row
        msg.peak_col = event.peak_col
        msg.peak_adc = event.peak_adc
        msg.normalized_pressure = event.normalized_pressure
        msg.total_pressure = event.total_pressure
        msg.active_count = event.active_count
        msg.valid_touch = event.valid_touch
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def _try_create_source(self):
        try:
            source = self._create_source()
            source.write(CMD_START)
            return source
        except Exception as exc:
            self.get_logger().warning(f"skin capture source unavailable: {exc}")
            return None

    def _close_source(self) -> None:
        if self._source is None:
            return
        try:
            self._source.write(CMD_STOP)
            self._source.close()
        except Exception:
            pass
        self._source = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SkinNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
