from datetime import datetime

import numpy as np

from vision_engine.snapshot_utils import build_snapshot_payload


def test_build_snapshot_payload_returns_base64_jpeg_metadata():
    frame = np.full((24, 32, 3), 180, dtype=np.uint8)

    payload = build_snapshot_payload(frame, captured_at="2026-03-16T12:00:00Z")

    assert payload["mime_type"] == "image/jpeg"
    assert payload["captured_at"] == "2026-03-16T12:00:00Z"
    assert payload["width"] == 32
    assert payload["height"] == 24
    assert isinstance(payload["image_base64"], str)
    assert len(payload["image_base64"]) > 20


def test_build_snapshot_payload_generates_timestamp_when_missing():
    frame = np.zeros((10, 12, 3), dtype=np.uint8)

    payload = build_snapshot_payload(frame)

    assert payload["captured_at"]
    datetime.fromisoformat(payload["captured_at"].replace("Z", "+00:00"))
