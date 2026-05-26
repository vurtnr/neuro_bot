from queue import Empty

from inspection_bridge.session_store import SessionStore


def test_skin_events_replay_recent_history_and_broadcast_to_subscribers():
    store = SessionStore()
    store.append_skin_event({"event": "light_touch", "peakAdc": 260.0})

    queue, history = store.subscribe_skin()

    assert history == [{"event": "light_touch", "peakAdc": 260.0}]

    store.append_skin_event({"event": "pain_warning", "peakAdc": 1100.0})
    assert queue.get_nowait()["event"] == "pain_warning"

    try:
        queue.get_nowait()
    except Empty:
        pass

    store.unsubscribe_skin(queue)
