from queue import Empty

from inspection_bridge.session_store import SessionStore


def test_session_store_replays_history_to_new_subscriber():
    store = SessionStore()
    store.append_event("req-1", {"requestId": "req-1", "event": "accepted"})

    queue, history, terminal = store.subscribe("req-1")

    assert terminal is False
    assert history == [{"requestId": "req-1", "event": "accepted"}]
    store.unsubscribe("req-1", queue)


def test_session_store_marks_terminal_event():
    store = SessionStore()
    queue, _, _ = store.subscribe("req-1")

    store.append_event("req-1", {"requestId": "req-1", "event": "failed"})

    assert queue.get_nowait()["event"] == "failed"
    try:
        queue.get_nowait()
    except Empty:
        pass
    store.unsubscribe("req-1", queue)
