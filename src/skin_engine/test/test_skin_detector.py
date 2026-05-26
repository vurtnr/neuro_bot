import numpy as np

from skin_engine.detector import (
    BACK_REGION,
    CHEST_REGION,
    classify_pressure_frame,
)


def test_classifies_chest_light_touch_from_valid_robot_cells():
    frame = np.zeros((32, 16), dtype=np.float32)
    frame[4, 12] = 260.0

    event = classify_pressure_frame(
        frame,
        touch_threshold=150.0,
        pain_threshold=900.0,
    )

    assert event.event_type == "light_touch"
    assert event.surface == "chest"
    assert event.peak_row == 4
    assert event.peak_col == 12
    assert event.valid_touch is True
    assert event.normalized_pressure == 260.0 / 900.0
    assert event.active_count == 1
    assert "Chest" in event.region


def test_ignores_unused_cells_even_when_they_have_large_values():
    frame = np.zeros((32, 16), dtype=np.float32)
    frame[4, 2] = 4095.0

    event = classify_pressure_frame(
        frame,
        touch_threshold=150.0,
        pain_threshold=900.0,
    )

    assert event.event_type == "idle"
    assert event.valid_touch is False
    assert event.peak_adc == 0.0


def test_classifies_back_pressure_warning():
    frame = np.zeros((32, 16), dtype=np.float32)
    frame[20, 3] = 1100.0

    event = classify_pressure_frame(
        frame,
        touch_threshold=150.0,
        pain_threshold=900.0,
    )

    assert event.event_type == "pain_warning"
    assert event.surface == "back"
    assert event.peak_row == 20
    assert event.peak_col == 3
    assert event.normalized_pressure == 1.0
    assert event.region.startswith("Back")


def test_region_constants_match_pdf_layout():
    assert CHEST_REGION == (slice(0, 16), slice(8, 16))
    assert BACK_REGION == (slice(16, 32), slice(0, 8))
