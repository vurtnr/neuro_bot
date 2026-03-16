from collections import deque
from dataclasses import dataclass, field
from typing import Deque


def _normalize_text(text: str) -> str:
    return "".join(char.lower() for char in text if char.isalnum())


@dataclass
class SpeakingSessionCounter:
    _active_sessions: int = field(default=0, init=False)

    def start_session(self) -> bool:
        self._active_sessions += 1
        return self._active_sessions == 1

    def finish_session(self) -> bool:
        if self._active_sessions > 0:
            self._active_sessions -= 1
        return self._active_sessions == 0

    @property
    def is_speaking(self) -> bool:
        return self._active_sessions > 0


@dataclass
class _RecentSpeech:
    normalized_text: str
    expires_at: float


@dataclass
class RecentSpeechGuard:
    window_seconds: float = 6.0
    min_partial_chars: int = 6
    _recent_speeches: Deque[_RecentSpeech] = field(default_factory=deque, init=False)

    def remember_tts(self, text: str, *, playback_finished_at: float) -> None:
        normalized = _normalize_text(text)
        if not normalized:
            return

        self._prune(playback_finished_at)
        self._recent_speeches.append(
            _RecentSpeech(
                normalized_text=normalized,
                expires_at=playback_finished_at + self.window_seconds,
            )
        )

    def should_ignore_asr(self, text: str, *, now: float) -> bool:
        normalized = _normalize_text(text)
        if not normalized:
            return False

        self._prune(now)
        return any(
            self._matches_recent_speech(normalized, speech.normalized_text)
            for speech in self._recent_speeches
        )

    def _matches_recent_speech(self, candidate: str, remembered: str) -> bool:
        if candidate == remembered:
            return True

        shorter, longer = sorted((candidate, remembered), key=len)
        return len(shorter) >= self.min_partial_chars and longer.startswith(shorter)

    def _prune(self, now: float) -> None:
        while self._recent_speeches and now > self._recent_speeches[0].expires_at:
            self._recent_speeches.popleft()

    def clear(self) -> None:
        self._recent_speeches.clear()
