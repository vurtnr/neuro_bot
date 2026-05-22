from __future__ import annotations

import base64
from datetime import datetime, timezone

import cv2


def build_snapshot_payload(
    frame,
    *,
    captured_at: str | None = None,
    jpeg_quality: int = 85,
) -> dict[str, object]:
    if frame is None:
        raise ValueError("frame is required")

    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not ok:
        raise ValueError("failed to encode frame")

    height, width = frame.shape[:2]
    timestamp = captured_at or datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    return {
        "image_base64": base64.b64encode(encoded.tobytes()).decode("ascii"),
        "mime_type": "image/jpeg",
        "captured_at": timestamp,
        "width": int(width),
        "height": int(height),
    }
