from __future__ import annotations

from skin_engine.detector import SkinPressureEvent


class TouchStabilizer:
    def __init__(self, confirm_frames: int = 3) -> None:
        self.confirm_frames = max(int(confirm_frames), 1)
        self._last_key: tuple[str, int, int] | None = None
        self._count = 0

    def apply(self, event: SkinPressureEvent) -> SkinPressureEvent:
        if not event.valid_touch or event.event_type == "idle":
            self._last_key = None
            self._count = 0
            return event

        key = (event.surface, event.peak_row, event.peak_col)
        if key == self._last_key:
            self._count += 1
        else:
            self._last_key = key
            self._count = 1

        if self._count < self.confirm_frames:
            return _idle_event()
        return event


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
