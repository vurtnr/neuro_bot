def test_resolve_camera_topic_default():
    from vision_engine.qr_config import resolve_camera_topic

    assert resolve_camera_topic(None) == "/camera/image_raw"
    assert resolve_camera_topic("") == "/camera/image_raw"


def test_resolve_camera_topic_custom():
    from vision_engine.qr_config import resolve_camera_topic

    assert resolve_camera_topic("/camera_driver/image_raw") == "/camera_driver/image_raw"
