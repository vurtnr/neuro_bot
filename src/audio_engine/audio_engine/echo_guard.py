from dataclasses import dataclass, field


def _normalize_text(text: str) -> str:
    return "".join(char.lower() for char in text if char.isalnum())


@dataclass
class RecentSpeechGuard:
    window_seconds: float = 6.0
    _recent_text: str = field(default="", init=False)
    _expires_at: float = field(default=0.0, init=False)

    def remember_tts(self, text: str, *, playback_finished_at: float) -> None:
        normalized = _normalize_text(text)
        if not normalized:
            self.clear()
            return

        self._recent_text = normalized
        self._expires_at = playback_finished_at + self.window_seconds

    def should_ignore_asr(self, text: str, *, now: float) -> bool:
        normalized = _normalize_text(text)
        if not normalized:
            return False

        if now > self._expires_at:
            self.clear()
            return False

        return normalized == self._recent_text

    def clear(self) -> None:
        self._recent_text = ""
        self._expires_at = 0.0
