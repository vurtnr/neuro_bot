from __future__ import annotations

from skin_engine.detector import SkinPressureEvent


class TouchStabilizer:
    def __init__(
        self,
        confirm_frames: int = 3,
        release_frames: int = 4,
        cell_tolerance: int = 1,
    ) -> None:
        self.confirm_frames = max(int(confirm_frames), 1)
        self.release_frames = max(int(release_frames), 1)
        self.cell_tolerance = max(int(cell_tolerance), 0)
        self._last_key: tuple[str, int, int] | None = None
        self._count = 0
        self._latched = False
        self._idle_count = 0

    def apply(self, event: SkinPressureEvent) -> SkinPressureEvent:
        if not event.valid_touch or event.event_type == "idle":
            self._idle_count += 1
            if self._idle_count >= self.release_frames:
                self._latched = False
                self._last_key = None
                self._count = 0
            return event

        if self._latched and 0 < self._idle_count < self.release_frames:
            return _idle_event()

        self._idle_count = 0
        key = (event.surface, event.peak_row, event.peak_col)
        if self._same_touch(key, self._last_key):
            self._count += 1
        else:
            self._count = 1
        self._last_key = key

        if self._count < self.confirm_frames:
            return _idle_event()
        self._latched = True
        return event

    def _same_touch(
        self,
        key: tuple[str, int, int],
        last_key: tuple[str, int, int] | None,
    ) -> bool:
        if last_key is None:
            return False
        surface, row, col = key
        last_surface, last_row, last_col = last_key
        return (
            surface == last_surface
            and abs(row - last_row) <= self.cell_tolerance
            and abs(col - last_col) <= self.cell_tolerance
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
