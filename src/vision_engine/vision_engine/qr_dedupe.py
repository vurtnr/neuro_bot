class QrContentDeduper:
    def __init__(self, suppression_frames=10):
        self._suppression_frames = int(suppression_frames)
        self._last_seen = {}

    def should_publish(self, content, frame_index):
        if self._suppression_frames <= 0:
            return True

        last_seen = self._last_seen.get(content)
        self._last_seen[content] = frame_index
        self._prune(frame_index)

        if last_seen is None:
            return True

        return (frame_index - last_seen) > self._suppression_frames

    def _prune(self, frame_index):
        cutoff = frame_index - self._suppression_frames
        if cutoff <= 0:
            return
        self._last_seen = {
            content: seen
            for content, seen in self._last_seen.items()
            if seen >= cutoff
        }
