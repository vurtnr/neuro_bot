from __future__ import annotations

import time
from typing import Optional

import numpy as np

try:
    from serial import Serial
except ImportError:
    Serial = None  # type: ignore[assignment]

PACKET_MAGIC = bytes((0x01, 0x02, 0x03))
ROWS = 32
COLS = 16
ADC_BYTES = 8 * 24
BODY_221 = 3 + ADC_BYTES + 24 + 2
FOOTER_226 = bytes((0x21, 0x22, 0x23, 0x0D, 0x0A))
DEFAULT_BAUD = 921600
CMD_START = bytes((0x11, 0xBB, 0x0D, 0x0A))
CMD_STOP = bytes((0xFF, 0xBB, 0x0D, 0x0A))
LAYOUT_ROW_SRC = np.concatenate(
    (
        np.arange(8, dtype=np.intp),
        np.arange(15, 7, -1, dtype=np.intp),
        np.arange(16, 24, dtype=np.intp),
        np.arange(31, 23, -1, dtype=np.intp),
    )
)
LAYOUT_COL_SRC = np.concatenate(
    (np.arange(8, dtype=np.intp), np.arange(15, 7, -1, dtype=np.intp))
)


def decode_packet_into(raw: bytes, grid: np.ndarray, band: int) -> None:
    r0 = band * 8
    adc = raw[3 : 3 + ADC_BYTES]
    for row in range(8):
        start = 24 * row
        for pair in range(8):
            off = start + 3 * pair
            b0 = adc[off]
            grid[r0 + row, 2 * pair] = ((b0 >> 4) << 8) | adc[off + 1]
            grid[r0 + row, 2 * pair + 1] = ((b0 & 0x0F) << 8) | adc[off + 2]


class SerialFrameReader:
    def __init__(self, port: str, baud: int) -> None:
        if Serial is None:
            raise RuntimeError("pyserial is required")
        self.ser = Serial(port=port, baudrate=baud, timeout=0.002)
        self._buf = bytearray()

    def close(self) -> None:
        self.ser.close()

    def write(self, data: bytes) -> None:
        if self.ser.is_open:
            self.ser.write(data)
            self.ser.flush()

    def read_packets(self) -> list[bytes]:
        n = max(self.ser.in_waiting, 256)
        chunk = self.ser.read(n)
        if not chunk:
            return []
        self._buf.extend(chunk)
        if len(self._buf) > 65536:
            del self._buf[:-8192]

        packets: list[bytes] = []
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
            if len(self._buf) >= BODY_221 + len(FOOTER_226):
                tail = self._buf[BODY_221 : BODY_221 + len(FOOTER_226)]
                if bytes(tail) == FOOTER_226:
                    packets.append(bytes(self._buf[: BODY_221 + len(FOOTER_226)]))
                    del self._buf[: BODY_221 + len(FOOTER_226)]
                    continue
            packets.append(bytes(self._buf[:BODY_221]))
            del self._buf[:BODY_221]
        return packets


class DemoSource:
    def __init__(self) -> None:
        self._phase = 0
        self._t0 = time.time()

    def write(self, data: bytes) -> None:
        del data

    def close(self) -> None:
        pass

    def read_packets(self) -> list[bytes]:
        time.sleep(0.01)
        phase = self._phase % 4
        data = bytearray(BODY_221 + len(FOOTER_226))
        data[0:3] = PACKET_MAGIC
        grid = np.zeros((8, COLS), dtype=np.uint16)
        if phase == 0:
            t = time.time() - self._t0
            col = 12 + int(2 * np.sin(t * 2.0))
            grid[4, col] = 420
        for row in range(8):
            start = 3 + 24 * row
            for pair in range(8):
                v0 = int(grid[row, 2 * pair])
                v1 = int(grid[row, 2 * pair + 1])
                data[start + 3 * pair] = ((v0 >> 8) << 4) | (v1 >> 8)
                data[start + 3 * pair + 1] = v0 & 0xFF
                data[start + 3 * pair + 2] = v1 & 0xFF
        data[219:221] = b"\x0d\x0a"
        data[221:226] = FOOTER_226
        self._phase += 1
        return [bytes(data)] if phase == 3 else [bytes(data[:BODY_221])]


class FrameAssembler:
    def __init__(self, reorder: bool = True) -> None:
        self.reorder = reorder
        self._raw_grid = np.zeros((ROWS, COLS), dtype=np.uint16)
        self._display_grid = np.zeros((ROWS, COLS), dtype=np.float32)
        self._row_tmp = np.zeros((ROWS, COLS), dtype=np.uint16)
        self._band = -1
        self._synced = False

    def push_packet(self, packet: bytes) -> Optional[np.ndarray]:
        packet_len = len(packet)
        if packet_len not in (BODY_221, BODY_221 + len(FOOTER_226)):
            return None

        is_tail = packet_len == BODY_221 + len(FOOTER_226)
        if not self._synced:
            if is_tail:
                decode_packet_into(packet, self._raw_grid, 3)
                self._band = 0
                self._synced = True
            return None

        if is_tail:
            decode_packet_into(packet, self._raw_grid, 3)
            self._band = 0
        else:
            decode_packet_into(packet, self._raw_grid, self._band)
            self._band = (self._band + 1) % 4
            return None

        if self.reorder:
            np.take(self._raw_grid, LAYOUT_ROW_SRC, axis=0, out=self._row_tmp)
            np.take(self._row_tmp, LAYOUT_COL_SRC, axis=1, out=self._display_grid)
        else:
            np.copyto(self._display_grid, self._raw_grid, casting="unsafe")
        return self._display_grid.copy()
