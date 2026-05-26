from __future__ import annotations

from dataclasses import dataclass

import numpy as np

ROWS = 32
COLS = 16
CHEST_REGION = (slice(0, 16), slice(8, 16))
BACK_REGION = (slice(16, 32), slice(0, 8))
REGION_VERTICAL_NAMES = ("Upper", "Middle", "Lower")
REGION_HORIZONTAL_NAMES = ("Left", "Center", "Right")


@dataclass(frozen=True)
class SkinPressureEvent:
    event_type: str
    surface: str
    region: str
    peak_row: int
    peak_col: int
    peak_adc: float
    normalized_pressure: float
    total_pressure: float
    active_count: int
    valid_touch: bool


def classify_pressure_frame(
    frame: np.ndarray,
    *,
    touch_threshold: float,
    pain_threshold: float,
) -> SkinPressureEvent:
    grid = np.asarray(frame, dtype=np.float32)
    if grid.shape != (ROWS, COLS):
        raise ValueError(f"Expected frame shape {(ROWS, COLS)}, got {grid.shape}")

    chest = grid[CHEST_REGION]
    back = grid[BACK_REGION]
    chest_peak = float(np.max(chest)) if chest.size else 0.0
    back_peak = float(np.max(back)) if back.size else 0.0

    if chest_peak <= 0.0 and back_peak <= 0.0:
        return _idle_event()

    if back_peak > chest_peak:
        surface = "back"
        local = back
        row_offset = 16
        col_offset = 0
        surface_name = "Back"
    else:
        surface = "chest"
        local = chest
        row_offset = 0
        col_offset = 8
        surface_name = "Chest"

    local_row, local_col = (
        int(value) for value in np.unravel_index(int(np.argmax(local)), local.shape)
    )
    peak_adc = float(local[local_row, local_col])
    active_count = int(np.count_nonzero(np.concatenate((chest.ravel(), back.ravel())) >= touch_threshold))
    total_pressure = float(
        np.sum(np.clip(chest - touch_threshold, 0.0, None))
        + np.sum(np.clip(back - touch_threshold, 0.0, None))
    )

    if peak_adc < touch_threshold:
        return _idle_event()

    event_type = "pain_warning" if peak_adc >= pain_threshold else _touch_type(active_count)
    normalized = float(np.clip(peak_adc / max(float(pain_threshold), 1.0), 0.0, 1.0))

    return SkinPressureEvent(
        event_type=event_type,
        surface=surface,
        region=_region_label(surface_name, local_row, local_col),
        peak_row=row_offset + local_row,
        peak_col=col_offset + local_col,
        peak_adc=peak_adc,
        normalized_pressure=normalized,
        total_pressure=total_pressure,
        active_count=active_count,
        valid_touch=True,
    )


def _touch_type(active_count: int) -> str:
    if active_count < 3:
        return "light_touch"
    return "focused_press"


def _region_label(surface_name: str, row: int, col: int) -> str:
    vertical = 0 if row < 5 else 1 if row < 11 else 2
    horizontal = 0 if col < 3 else 1 if col < 6 else 2
    return (
        f"{surface_name} / {REGION_VERTICAL_NAMES[vertical]} / "
        f"{REGION_HORIZONTAL_NAMES[horizontal]}"
    )


def _idle_event() -> SkinPressureEvent:
    return SkinPressureEvent(
        event_type="idle",
        surface="",
        region="--",
        peak_row=-1,
        peak_col=-1,
        peak_adc=0.0,
        normalized_pressure=0.0,
        total_pressure=0.0,
        active_count=0,
        valid_touch=False,
    )
