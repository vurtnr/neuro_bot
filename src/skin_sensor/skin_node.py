#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NeuroBot Skin Node — ROS2触觉感知节点
融合 protocol.py + mapper.py，发布 TactileFrame / TactileSurface / TactileEvent
"""

import json
import os
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header

from robot_interfaces.msg import TactileEvent, TactileFrame, TactileSurface

from .mapper import RobotSkinMapper, DOME_MASK, TOUCH_Z_THRESHOLD
from .protocol import SerialFrameReader, decode_packet_fast, CMD_START, CMD_STOP


class SkinNode(Node):
    def __init__(self):
        super().__init__('skin_sensor')

        # ── 参数 ──
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('threshold', TOUCH_Z_THRESHOLD)
        self.declare_parameter('range_max', 16384.0)
        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('spatial_mix', 0.18)
        self.declare_parameter('calibrate_frames', 20)
        self.declare_parameter('calibrate_on_start', True)
        self.declare_parameter('publish_raw', False)      # 是否发布原始帧（调试用）
        self.declare_parameter('baseline_path', '')     # 校准基线持久化路径

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.threshold = self.get_parameter('threshold').value
        self.range_max = self.get_parameter('range_max').value
        self.ema_alpha = float(np.clip(self.get_parameter('ema_alpha').value, 0.0, 1.0))
        self.spatial_mix = float(np.clip(self.get_parameter('spatial_mix').value, 0.0, 1.0))
        self.calibrate_frames = self.get_parameter('calibrate_frames').value
        self.calibrate_on_start = self.get_parameter('calibrate_on_start').value
        self.publish_raw = self.get_parameter('publish_raw').value
        self.baseline_path = self.get_parameter('baseline_path').value

        # ── QoS ──
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── 发布者 ──
        self.pub_event = self.create_publisher(TactileEvent, '/skin/events', qos)
        self.pub_surface = self.create_publisher(TactileSurface, '/skin/surfaces', qos)
        if self.publish_raw:
            self.pub_raw = self.create_publisher(TactileFrame, '/skin/raw_frame', qos)

        # ── 内部状态 ──
        self.reader = SerialFrameReader(self.port, self.baud)
        self.mapper = RobotSkinMapper(
            flip_chest_row=True, flip_back_row=True, use_transpose=False
        )

        self.raw_grid = np.zeros((32, 16), dtype=np.uint16)
        self.disp_grid = np.zeros((32, 16), dtype=np.float32)
        self.smooth_grid = np.zeros((32, 16), dtype=np.float32)
        self.ema_ready = False

        self.calibrating = False
        self.calib_count = 0
        self.calib_sum = np.zeros((32, 16), dtype=np.float64)
        self.baseline = np.zeros((32, 16), dtype=np.float32)
        self.has_baseline = False

        # 尝试加载持久化基线
        self._load_baseline()

        # 启动校准
        if self.calibrate_on_start and not self.has_baseline:
            self._start_calibration()

        # ── 工作线程 ──
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"🤖 SkinNode started on {self.port}@{self.baud} | "
            f"threshold={self.threshold} | calibrate={self.calibrate_on_start}"
        )

    def _load_baseline(self):
        if not self.baseline_path:
            return
        p = Path(self.baseline_path)
        if p.exists():
            try:
                data = json.loads(p.read_text())
                self.baseline = np.array(data['baseline'], dtype=np.float32).reshape(32, 16)
                self.has_baseline = True
                self.get_logger().info(f"✅ Loaded baseline from {self.baseline_path}")
            except Exception as exc:
                self.get_logger().warn(f"⚠️ Failed to load baseline: {exc}")

    def _save_baseline(self):
        if not self.baseline_path:
            return
        try:
            Path(self.baseline_path).write_text(
                json.dumps({'baseline': self.baseline.flatten().tolist()})
            )
            self.get_logger().info(f"💾 Saved baseline to {self.baseline_path}")
        except Exception as exc:
            self.get_logger().warn(f"⚠️ Failed to save baseline: {exc}")

    def _start_calibration(self):
        self.calibrating = True
        self.calib_count = 0
        self.calib_sum.fill(0)
        self.has_baseline = False
        self.ema_ready = False
        self.get_logger().info("🔧 Calibration started...")

    def _box_mean_3x3(self, src: np.ndarray, dst: np.ndarray) -> None:
        padded = np.pad(src, ((1, 1), (1, 1)), mode="edge")
        dst[:] = (
            padded[0:-2, 0:-2] + padded[0:-2, 1:-1] + padded[0:-2, 2:]
            + padded[1:-1, 0:-2] + padded[1:-1, 1:-1] + padded[1:-1, 2:]
            + padded[2:, 0:-2] + padded[2:, 1:-1] + padded[2:, 2:]
        ) * (1.0 / 9.0)

    def _loop(self):
        """后台线程：串口读取 → 解码 → 平滑 → 映射 → 发布"""
        band = -1
        synced = False
        no_tail_mode = False
        pre_sync_count = 0
        last_pkt_time = 0.0
        pkt_count = 0
        t0 = time.perf_counter()

        while self._running:
            pkts = self.reader.read_packets()
            if not pkts:
                if synced and no_tail_mode and (time.perf_counter() - last_pkt_time) > 0.15:
                    synced = False
                    band = -1
                    pre_sync_count = 0
                continue

            for pkt in pkts:
                plen = len(pkt)
                is_tail = plen == 221 + 5
                last_pkt_time = time.perf_counter()

                if not synced:
                    if is_tail:
                        decode_packet_fast(pkt, self.raw_grid, 3)
                        band = 0
                        synced = True
                        no_tail_mode = False
                        pre_sync_count = 0
                    else:
                        pre_sync_count += 1
                        if pre_sync_count >= 5:
                            decode_packet_fast(pkt, self.raw_grid, 0)
                            band = 1
                            synced = True
                            no_tail_mode = True
                            pre_sync_count = 0
                    continue

                if is_tail:
                    decode_packet_fast(pkt, self.raw_grid, 3)
                    band = 0
                else:
                    decode_packet_fast(pkt, self.raw_grid, band)
                    band = (band + 1) % 4

                pkt_count += 1
                if band != 0:
                    continue  # 帧未完整

                # ── 帧完整，开始处理 ──
                np.copyto(self.disp_grid, self.raw_grid, casting="unsafe")

                # 校准
                if self.calibrating:
                    self.calib_sum += self.disp_grid
                    self.calib_count += 1
                    if self.calib_count >= self.calibrate_frames:
                        self.baseline = (self.calib_sum / self.calib_count).astype(np.float32)
                        self.has_baseline = True
                        self.calibrating = False
                        self._save_baseline()
                        self.get_logger().info(
                            f"✅ Calibration done ({self.calib_count} frames)"
                        )
                    continue

                # 减基线
                if self.has_baseline:
                    self.disp_grid -= self.baseline
                    np.clip(self.disp_grid, 0, None, out=self.disp_grid)

                # EMA平滑
                if not self.ema_ready:
                    np.copyto(self.smooth_grid, self.disp_grid)
                    self.ema_ready = True
                else:
                    self.smooth_grid[:] = (
                        self.smooth_grid * (1.0 - self.ema_alpha)
                        + self.disp_grid * self.ema_alpha
                    )

                # 空间混合
                work = np.zeros_like(self.smooth_grid)
                if self.spatial_mix > 0.0:
                    self._box_mean_3x3(self.smooth_grid, work)
                    out_grid = self.smooth_grid * (1.0 - self.spatial_mix) + work * self.spatial_mix
                else:
                    out_grid = self.smooth_grid.copy()

                # 阈值截断（用于统计激活点）
                clipped = out_grid.copy()
                if self.threshold > 0:
                    clipped[clipped < self.threshold] = 0.0

                # ── 发布原始帧（调试用） ──
                if self.publish_raw:
                    active = int(np.count_nonzero(clipped))
                    peak = float(np.max(out_grid))
                    msg = TactileFrame(
                        header=self._stamp(),
                        data=out_grid.flatten().tolist(),
                        rows=32, cols=16,
                        active_count=active,
                        peak_value=peak,
                        fps=float(pkt_count / max(time.perf_counter() - t0, 0.001)),
                        calibrated=self.has_baseline,
                    )
                    self.pub_raw.publish(msg)

                # ── 映射到穹顶 ──
                vf, vb = self.mapper.map_frame(out_grid)
                front_active = int(np.count_nonzero(np.where(DOME_MASK, vf, 0) > self.threshold))
                back_active = int(np.count_nonzero(np.where(DOME_MASK, vb, 0) > self.threshold))

                surface_msg = TactileSurface(
                    header=self._stamp(),
                    front_data=vf.flatten().tolist(),
                    back_data=vb.flatten().tolist(),
                    front_active=front_active,
                    back_active=back_active,
                    front_peak=float(np.max(vf)) if front_active > 0 else 0.0,
                    back_peak=float(np.max(vb)) if back_active > 0 else 0.0,
                    front_total=float(np.sum(vf, where=DOME_MASK)),
                    back_total=float(np.sum(vb, where=DOME_MASK)),
                )
                self.pub_surface.publish(surface_msg)

                # ── 事件检测与发布 ──
                event = self.mapper.detect_event(vf, vb, threshold=self.threshold)
                event_msg = TactileEvent(
                    header=self._stamp(),
                    surface=event['surface'],
                    region_v=event['region_v'],
                    region_h=event['region_h'],
                    peak_value=event['peak_value'],
                    total_force=event['total_force'],
                    is_touch=event['is_touch'],
                    label=event['label'],
                )
                self.pub_event.publish(event_msg)

                # FPS统计
                now = time.perf_counter()
                if now - t0 >= 1.0:
                    fps = pkt_count / (now - t0)
                    pkt_count = 0
                    t0 = now
                    self.get_logger().debug(f"Skin FPS: {fps:.1f}")

    def _stamp(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'robot_skin'
        return h

    def destroy_node(self):
        self._running = False
        self.reader.close()
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SkinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()