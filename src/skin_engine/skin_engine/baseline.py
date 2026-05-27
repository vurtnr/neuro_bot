from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class BaselineCorrector:
    target_frames: int
    noise_percentile: float = 99.0
    margin: float = 300.0
    mode: str = "positive"

    def __post_init__(self) -> None:
        self.target_frames = max(int(self.target_frames), 0)
        self.noise_percentile = float(np.clip(self.noise_percentile, 0.0, 100.0))
        self.margin = max(float(self.margin), 0.0)
        if self.mode not in ("positive", "absolute"):
            raise ValueError("baseline mode must be 'positive' or 'absolute'")
        self._frames: list[np.ndarray] = []
        self._center = np.zeros((32, 16), dtype=np.float32)
        self._noise = np.zeros((32, 16), dtype=np.float32)
        self._ready = self.target_frames <= 0

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def collected_frames(self) -> int:
        return len(self._frames)

    def apply(self, frame: np.ndarray) -> np.ndarray:
        grid = np.asarray(frame, dtype=np.float32)
        if grid.shape != (32, 16):
            raise ValueError(f"Expected frame shape {(32, 16)}, got {grid.shape}")

        if self.target_frames <= 0:
            return grid.copy()

        if not self._ready:
            self._frames.append(grid.copy())
            if len(self._frames) >= self.target_frames:
                self._calibrate()
            return np.zeros_like(grid, dtype=np.float32)

        return self.correct(grid)

    def correct(self, frame: np.ndarray) -> np.ndarray:
        grid = np.asarray(frame, dtype=np.float32)
        if self.mode == "absolute":
            corrected = np.abs(grid - self._center) - self._noise
        else:
            corrected = grid - self._center - self._noise
        np.clip(corrected, 0.0, None, out=corrected)
        return corrected.astype(np.float32, copy=False)

    def _calibrate(self) -> None:
        stack = np.stack(self._frames, axis=0).astype(np.float32, copy=False)
        self._center = np.median(stack, axis=0).astype(np.float32)
        residual = np.abs(stack - self._center)
        noise = np.percentile(residual, self.noise_percentile, axis=0)
        self._noise = (noise + self.margin).astype(np.float32)
        self._frames.clear()
        self._ready = True
