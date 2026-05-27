from skin_engine.detector import SkinPressureEvent
from skin_engine.skin_node import select_published_event


def test_select_published_event_keeps_detected_touch_visible_while_stabilizing():
    detected = _touch("light_touch", 20, 7, 260.0)
    stabilized = _idle()

    event = select_published_event(detected, stabilized)

    assert event.event_type == "light_touch"
    assert event.valid_touch is True
    assert event.peak_row == 20
    assert event.peak_col == 7


def test_select_published_event_keeps_true_idle_idle():
    event = select_published_event(_idle(), _idle())

    assert event.event_type == "idle"
    assert event.valid_touch is False


def _touch(event_type: str, row: int, col: int, peak: float) -> SkinPressureEvent:
    return SkinPressureEvent(
        event_type=event_type,
        surface="back" if row >= 16 else "chest",
        region="test",
        peak_row=row,
        peak_col=col,
        peak_adc=peak,
        normalized_pressure=min(1.0, peak / 900.0),
        total_pressure=peak,
        active_count=1,
        valid_touch=True,
    )


def _idle() -> SkinPressureEvent:
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
