#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NeuroBot Skin Protocol — ROS2化改造版
基于 robot_skin_cli_v4.py 协议层提取，增加自动重连与Header时间戳
"""

import struct
import threading
import time
from typing import List, Optional

import numpy as np
import serial


PACKET_MAGIC = bytes((0x01, 0x02, 0x03))
ROWS = 32
COLS = 16
ADC_BYTES = 8 * 24          # 192
BODY_221 = 3 + ADC_BYTES + 24 + 2
FOOTER_226 = bytes((0x21, 0x22, 0x23, 0x0D, 0x0A))
DEFAULT_BAUD = 921600
CMD_START = bytes((0x11, 0xBB, 0x0D, 0x0A))
CMD_STOP = bytes((0xFF, 0xBB, 0x0D, 0x0A))

# 行列重排表（与v4一致）
LAYOUT_ROW_SRC = np.concatenate(
    (np.arange(8, dtype=np.intp), np.arange(15, 7, -1, dtype=np.intp),
     np.arange(16, 24, dtype=np.intp), np.arange(31, 23, -1, dtype=np.intp))
)
LAYOUT_COL_SRC = np.concatenate(
    (np.arange(8, dtype=np.intp), np.arange(15, 7, -1, dtype=np.intp))
)


def decode_packet_fast(raw: bytes, grid: np.ndarray, band: int) -> None:
    """解码单个子包到 raw_grid 的对应 band 位置（band 0~3）"""
    r0 = band * 8
    adc = raw[3:3 + ADC_BYTES]
    for i in range(8):
        s = 24 * i
        for j in range(8):
            off = s + 3 * j
            b0 = adc[off]
            grid[r0 + i, 2 * j] = ((b0 >> 4) << 8) | adc[off + 1]
            grid[r0 + i, 2 * j + 1] = ((b0 & 0x0F) << 8) | adc[off + 2]


class SerialFrameReader:
    """
    串口帧读取器，带自动重连。
    对外接口与 v4 保持一致，方便替换。
    """

    def __init__(self, port: str, baud: int = DEFAULT_BAUD,
                 reconnect_interval: float = 2.0) -> None:
        self.port = port
        self.baud = baud
        self._reconnect_interval = reconnect_interval
        self._buf = bytearray()
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._running = True
        self._connect()

    def _connect(self) -> bool:
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.002,
                write_timeout=1.0
            )
            self._buf.clear()
            self.write(CMD_START)
            return True
        except Exception as exc:
            print(f"[SkinProtocol] Serial open failed: {exc}")
            self._ser = None
            return False

    def close(self) -> None:
        self._running = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(CMD_STOP)
                self._ser.flush()
            except Exception:
                pass
            self._ser.close()

    def write(self, data: bytes) -> None:
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(data)
                    self._ser.flush()
                except Exception:
                    pass

    def read_packets(self) -> List[bytes]:
        """线程安全读取；如果断开则尝试重连"""
        with self._lock:
            if self._ser is None or not self._ser.is_open:
                # 指数退避简单版
                time.sleep(self._reconnect_interval)
                self._connect()
                return []

            try:
                n = max(self._ser.in_waiting, 256)
                chunk = self._ser.read(n)
            except Exception as exc:
                print(f"[SkinProtocol] Read error: {exc}")
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                return []

        if not chunk:
            return []

        self._buf.extend(chunk)
        if len(self._buf) > 65536:
            del self._buf[:-8192]

        pkts: List[bytes] = []
        while True:
            idx = self._buf.find(PACKET_MAGIC)
            if idx < 0:
                keep = len(PACKET_MAGIC) - 1
                if len(self._buf) > keep:
                    del self._buf[:-keep]
                break
            if idx > 0:
                del self._buf[:idx]

            if len(self._buf) < BODY_221:
                break

            # 检查是否带帧尾（完整第四包）
            if len(self._buf) >= BODY_221 + 5:
                tail = self._buf[BODY_221:BODY_221 + 5]
                if bytes(tail) == FOOTER_226:
                    pkts.append(bytes(self._buf[:BODY_221 + 5]))
                    del self._buf[:BODY_221 + 5]
                    continue

            # 普通包（无帧尾）
            pkts.append(bytes(self._buf[:BODY_221]))
            del self._buf[:BODY_221]
        return pkts