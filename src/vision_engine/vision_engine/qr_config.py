DEFAULT_CAMERA_TOPIC = '/camera/image_raw'


def resolve_camera_topic(topic):
    if topic is None:
        return DEFAULT_CAMERA_TOPIC
    if isinstance(topic, str) and topic.strip() == '':
        return DEFAULT_CAMERA_TOPIC
    return topic
