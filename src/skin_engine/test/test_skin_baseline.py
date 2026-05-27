import numpy as np

from skin_engine.baseline import BaselineCorrector


def test_baseline_corrector_suppresses_calibrated_idle_spikes():
    corrector = BaselineCorrector(
        target_frames=6,
        noise_percentile=100.0,
        margin=50.0,
        mode="positive",
    )
    base = np.full((32, 16), 2000.0, dtype=np.float32)

    for offset in (0.0, 60.0, -40.0, 120.0, -80.0, 420.0):
        frame = base.copy()
        frame[5, 12] += offset
        corrected = corrector.apply(frame)

    assert corrector.ready is True
    assert float(np.max(corrected)) == 0.0

    idle_frame = base.copy()
    idle_frame[5, 12] += 390.0
    corrected_idle = corrector.apply(idle_frame)

    assert float(np.max(corrected_idle)) == 0.0


def test_baseline_corrector_preserves_press_above_noise_envelope():
    corrector = BaselineCorrector(
        target_frames=6,
        noise_percentile=100.0,
        margin=50.0,
        mode="positive",
    )
    base = np.full((32, 16), 2000.0, dtype=np.float32)

    for offset in (0.0, 60.0, -40.0, 120.0, -80.0, 420.0):
        frame = base.copy()
        frame[5, 12] += offset
        corrector.apply(frame)

    press_frame = base.copy()
    press_frame[5, 12] += 1200.0
    corrected_press = corrector.apply(press_frame)

    assert corrected_press[5, 12] > 700.0
    assert int(np.count_nonzero(corrected_press)) == 1
