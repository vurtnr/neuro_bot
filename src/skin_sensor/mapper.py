#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NeuroBot Skin Mapper — 32×16 → 8×16 Dome 映射 + 触觉事件生成
"""

from typing import Tuple

import numpy as np

# 与 v4 一致的穹顶参数
SKIN_ROWS = 8
SKIN_COLS = 16
COLUMN_HEIGHTS = (3, 4, 6, 7, 8, 8, 8, 8, 8, 8, 8, 8, 7, 6, 4, 3)
TOUCH_Z_THRESHOLD = 50.0

SKIN_REGION_V_NAMES = ("Upper", "Middle", "Lower")
SKIN_REGION_H_NAMES = ("Left", "Center", "Right")

# 胸前/后背在 32×16 母阵中的切片（与图片一致）
CHEST_ROW1, CHEST_ROW2 = 1, 16      # 0-based: 0:16
CHEST_COL1, CHEST_COL2 = 9, 16      # 0-based: 8:16
BACK_ROW1, BACK_ROW2 = 17, 32       # 0-based: 16:32
BACK_COL1, BACK_COL2 = 1, 8         # 0-based: 0:8


def _build_dome_mask(rows: int, cols: int, heights: tuple) -> np.ndarray:
    m = np.zeros((rows, cols), dtype=np.bool_)
    for c, h in enumerate(heights):
        hh = int(np.clip(h, 0, rows))
        if hh <= 0:
            continue
        r0 = rows - hh
        m[r0:rows, c] = True
    return m


DOME_MASK = _build_dome_mask(SKIN_ROWS, SKIN_COLS, COLUMN_HEIGHTS)
MASK_FLOAT = DOME_MASK.astype(np.float32)


def _bilinear_resize_2d(src: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
    sy, sx = float(src.shape[0] - 1), float(src.shape[1] - 1)
    gy = np.linspace(0.0, sy, out_h, dtype=np.float64)[:, None]
    gx = np.linspace(0.0, sx, out_w, dtype=np.float64)[None, :]
    y0 = np.minimum(np.floor(gy).astype(np.int64), src.shape[0] - 2)
    x0 = np.minimum(np.floor(gx).astype(np.int64), src.shape[1] - 2)
    fy = (gy - y0).astype(np.float32)
    fx = (gx - x0).astype(np.float32)
    v00 = src[y0, x0]
    v01 = src[y0, x0 + 1]
    v10 = src[y0 + 1, x0]
    v11 = src[y0 + 1, x0 + 1]
    top = v00 * (1.0 - fx) + v01 * fx
    bot = v10 * (1.0 - fx) + v11 * fx
    return top * (1.0 - fy) + bot * fy


class RobotSkinMapper:
    """
    将 32×16 原始母阵映射为胸前/后背两个 8×16 穹顶表面。
    默认配置与 v4 一致：flipud 纠正方向，不转置。
    """

    def __init__(self,
                 swap_chest_back: bool = False,
                 flip_chest_row: bool = True,
                 flip_chest_col: bool = False,
                 flip_back_row: bool = True,
                 flip_back_col: bool = False,
                 use_transpose: bool = False) -> None:
        self.values_front = np.zeros((SKIN_ROWS, SKIN_COLS), dtype=np.float32)
        self.values_back = np.zeros((SKIN_ROWS, SKIN_COLS), dtype=np.float32)
        self.swap_chest_back = swap_chest_back
        self.flip_chest_row = flip_chest_row
        self.flip_chest_col = flip_chest_col
        self.flip_back_row = flip_back_row
        self.flip_back_col = flip_back_col
        self.use_transpose = use_transpose

    def _map_half(self, half: np.ndarray, out: np.ndarray) -> np.ndarray:
        resized = _bilinear_resize_2d(
            np.asarray(half, dtype=np.float32), SKIN_ROWS, SKIN_COLS)
        np.multiply(resized, MASK_FLOAT, out=out)
        return out

    def map_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        grid = np.asarray(frame, dtype=np.float32).reshape(32, 16)

        # 胸前：母阵 rows 0-15, cols 8-15
        chest = grid[CHEST_ROW1 - 1:CHEST_ROW2, CHEST_COL1 - 1:CHEST_COL2]
        if self.use_transpose:
            chest = chest.T
        if self.flip_chest_row:
            chest = np.flipud(chest)
        if self.flip_chest_col:
            chest = np.fliplr(chest)

        # 后背：母阵 rows 16-31, cols 0-7
        back = grid[BACK_ROW1 - 1:BACK_ROW2, BACK_COL1 - 1:BACK_COL2]
        if self.use_transpose:
            back = back.T
        if self.flip_back_row:
            back = np.flipud(back)
        if self.flip_back_col:
            back = np.fliplr(back)

        self._map_half(chest, self.values_front)
        self._map_half(back, self.values_back)

        if self.swap_chest_back:
            return self.values_back.copy(), self.values_front.copy()
        return self.values_front.copy(), self.values_back.copy()

    def detect_event(self, front: np.ndarray, back: np.ndarray,
                     threshold: float = TOUCH_Z_THRESHOLD):
        """
        返回触觉事件字典，与 TactileEvent.msg 字段一一对应。
        """
        front_m = np.where(DOME_MASK, np.asarray(front, dtype=np.float32), np.nan)
        back_m = np.where(DOME_MASK, np.asarray(back, dtype=np.float32), np.nan)

        peak_front = float(np.nanmax(front_m)) if np.any(np.isfinite(front_m)) else 0.0
        peak_back = float(np.nanmax(back_m)) if np.any(np.isfinite(back_m)) else 0.0

        if max(peak_front, peak_back) < float(threshold):
            return {
                "surface": "none",
                "region_v": "--",
                "region_h": "--",
                "peak_value": 0.0,
                "total_force": 0.0,
                "is_touch": False,
                "label": "--",
            }

        if peak_back > peak_front:
            grid, surface = back_m, "back"
        else:
            grid, surface = front_m, "front"

        row, col = (int(x) for x in np.unravel_index(int(np.nanargmax(grid)), grid.shape))
        v = 0 if row < 3 else 1 if row < 6 else 2
        h = 0 if col < 5 else 1 if col < 11 else 2

        total_force = float(np.nansum(grid))
        peak = peak_back if surface == "back" else peak_front

        return {
            "surface": surface,
            "region_v": SKIN_REGION_V_NAMES[v],
            "region_h": SKIN_REGION_H_NAMES[h],
            "peak_value": peak,
            "total_force": total_force,
            "is_touch": True,
            "label": f"{surface.capitalize()} / {SKIN_REGION_V_NAMES[v]} / {SKIN_REGION_H_NAMES[h]}",
        }