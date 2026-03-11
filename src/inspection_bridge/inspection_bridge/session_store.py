from __future__ import annotations

from dataclasses import dataclass, field
import json
from queue import Queue
from threading import Lock
from time import monotonic


@dataclass
class Session:
    request_id: str
    created_at: float = field(default_factory=monotonic)
    history: list[dict] = field(default_factory=list)
    subscribers: list[Queue] = field(default_factory=list)
    terminal: bool = False


class SessionStore:
    def __init__(self) -> None:
        self._sessions: dict[str, Session] = {}
        self._lock = Lock()

    def ensure_session(self, request_id: str) -> Session:
        with self._lock:
            return self._sessions.setdefault(request_id, Session(request_id=request_id))

    def append_event(self, request_id: str, event: dict) -> None:
        with self._lock:
            session = self._sessions.setdefault(request_id, Session(request_id=request_id))
            session.history.append(event)
            if event.get("event") in {"success", "failed"}:
                session.terminal = True
            subscribers = list(session.subscribers)

        for subscriber in subscribers:
            subscriber.put(event)

    def subscribe(self, request_id: str) -> tuple[Queue, list[dict], bool]:
        queue: Queue = Queue()
        with self._lock:
            session = self._sessions.setdefault(request_id, Session(request_id=request_id))
            history = list(session.history)
            terminal = session.terminal
            session.subscribers.append(queue)
        return queue, history, terminal

    def unsubscribe(self, request_id: str, queue: Queue) -> None:
        with self._lock:
            session = self._sessions.get(request_id)
            if not session:
                return
            session.subscribers = [item for item in session.subscribers if item is not queue]

    def format_sse(self, payload: dict) -> bytes:
        return f"data: {json.dumps(payload, ensure_ascii=False)}\n\n".encode("utf-8")
