from inspection_bridge.session_store import SessionStore


def test_permission_terminal_events_end_session_stream():
    store = SessionStore()
    queue, _, terminal = store.subscribe("req-1")

    assert terminal is False

    store.append_event(
        "req-1",
        {"requestId": "req-1", "event": "permission_denied", "success": False},
    )

    assert queue.get_nowait()["event"] == "permission_denied"

    _, history, terminal = store.subscribe("req-1")
    assert terminal is True
    assert history[-1]["event"] == "permission_denied"


def test_support_escalation_terminal_events_end_session_stream():
    store = SessionStore()
    queue, _, terminal = store.subscribe("support-1")

    assert terminal is False

    store.append_event(
        "support-1",
        {
            "requestId": "support-1",
            "event": "support_escalation_sent",
            "success": True,
        },
    )

    assert queue.get_nowait()["event"] == "support_escalation_sent"

    _, history, terminal = store.subscribe("support-1")
    assert terminal is True
    assert history[-1]["event"] == "support_escalation_sent"
