from vision_engine.inspection_gate import InspectionGate


def test_gate_allows_publish_when_gating_disabled():
    gate = InspectionGate(require_active_session=False)

    assert gate.should_publish() is True
    gate.update("req-1", "waiting_for_qr")
    assert gate.should_publish() is True


def test_gate_blocks_publish_until_session_becomes_active():
    gate = InspectionGate(require_active_session=True)

    assert gate.should_publish() is False

    gate.update("req-1", "accepted")
    assert gate.should_publish() is True


def test_gate_clears_after_terminal_stage():
    gate = InspectionGate(require_active_session=True)

    gate.update("req-1", "waiting_for_qr")
    assert gate.should_publish() is True

    gate.update("req-1", "failed")
    assert gate.should_publish() is False
