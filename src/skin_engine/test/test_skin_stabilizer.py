from skin_engine.detector import SkinPressureEvent
from skin_engine.stabilizer import TouchStabilizer


def test_stabilizer_suppresses_short_idle_spikes():
    stabilizer = TouchStabilizer(confirm_frames=3)

    assert stabilizer.apply(_touch(20, 7, 2060.0)).event_type == "idle"
    assert stabilizer.apply(_touch(20, 7, 2060.0)).event_type == "idle"


def test_stabilizer_passes_sustained_touch_after_confirmation():
    stabilizer = TouchStabilizer(confirm_frames=3)

    stabilizer.apply(_touch(20, 7, 2060.0))
    stabilizer.apply(_touch(20, 7, 2060.0))
    event = stabilizer.apply(_touch(20, 7, 2060.0))

    assert event.event_type == "pain_warning"
    assert event.peak_row == 20
    assert event.peak_col == 7


def test_stabilizer_resets_when_peak_cell_changes():
    stabilizer = TouchStabilizer(confirm_frames=3)

    stabilizer.apply(_touch(20, 7, 2060.0))
    stabilizer.apply(_touch(20, 7, 2060.0))
    event = stabilizer.apply(_touch(8, 8, 1332.0))

    assert event.event_type == "idle"


def test_stabilizer_suppresses_release_bounce_until_idle_is_stable():
    stabilizer = TouchStabilizer(confirm_frames=2, release_frames=4)

    stabilizer.apply(_touch(20, 7, 2060.0))
    assert stabilizer.apply(_touch(20, 7, 2060.0)).event_type == "pain_warning"
    assert stabilizer.apply(_idle()).event_type == "idle"
    assert stabilizer.apply(_idle()).event_type == "idle"

    event = stabilizer.apply(_touch(23, 3, 1036.0))

    assert event.event_type == "idle"


def test_stabilizer_allows_new_touch_after_stable_release():
    stabilizer = TouchStabilizer(confirm_frames=2, release_frames=3)

    stabilizer.apply(_touch(20, 7, 2060.0))
    stabilizer.apply(_touch(20, 7, 2060.0))
    stabilizer.apply(_idle())
    stabilizer.apply(_idle())
    stabilizer.apply(_idle())

    assert stabilizer.apply(_touch(23, 3, 1036.0)).event_type == "idle"
    assert stabilizer.apply(_touch(23, 3, 1036.0)).event_type == "pain_warning"


def _touch(row: int, col: int, peak: float) -> SkinPressureEvent:
    return SkinPressureEvent(
        event_type="pain_warning",
        surface="back" if row >= 16 else "chest",
        region="test",
        peak_row=row,
        peak_col=col,
        peak_adc=peak,
        normalized_pressure=1.0,
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
