def test_dedupe_suppresses_repeated_content():
    from vision_engine.qr_dedupe import QrContentDeduper

    deduper = QrContentDeduper(suppression_frames=5)

    assert deduper.should_publish('A', 1) is True
    assert deduper.should_publish('A', 2) is False
    assert deduper.should_publish('A', 6) is False
    assert deduper.should_publish('A', 12) is True


def test_dedupe_allows_new_content():
    from vision_engine.qr_dedupe import QrContentDeduper

    deduper = QrContentDeduper(suppression_frames=5)

    assert deduper.should_publish('A', 1) is True
    assert deduper.should_publish('B', 1) is True
    assert deduper.should_publish('A', 2) is False
