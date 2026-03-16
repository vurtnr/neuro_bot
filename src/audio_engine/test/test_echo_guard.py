import unittest

from audio_engine.echo_guard import RecentSpeechGuard, SpeakingSessionCounter


class RecentSpeechGuardTests(unittest.TestCase):
    def test_recent_tts_match_is_suppressed(self):
        guard = RecentSpeechGuard(window_seconds=6.0)

        guard.remember_tts("蓝牙设备连接失败，请重试。", playback_finished_at=100.0)

        self.assertTrue(
            guard.should_ignore_asr("蓝牙设备连接失败，请重试！", now=103.0)
        )

    def test_different_text_is_not_suppressed(self):
        guard = RecentSpeechGuard(window_seconds=6.0)

        guard.remember_tts("蓝牙设备连接失败，请重试。", playback_finished_at=100.0)

        self.assertFalse(
            guard.should_ignore_asr("连接失败了，请重新扫码试试。", now=103.0)
        )

    def test_expired_guard_does_not_suppress(self):
        guard = RecentSpeechGuard(window_seconds=6.0)

        guard.remember_tts("蓝牙设备连接失败，请重试。", playback_finished_at=100.0)

        self.assertFalse(
            guard.should_ignore_asr("蓝牙设备连接失败，请重试！", now=107.5)
        )

    def test_partial_prefix_of_recent_tts_is_suppressed(self):
        guard = RecentSpeechGuard(window_seconds=6.0)

        guard.remember_tts("蓝牙设备连接成功，正在查询设备参数。", playback_finished_at=100.0)

        self.assertTrue(
            guard.should_ignore_asr("蓝牙设备连接成功，正在查询", now=103.0)
        )

    def test_older_recent_tts_still_matches_after_newer_tts(self):
        guard = RecentSpeechGuard(window_seconds=6.0)

        guard.remember_tts("蓝牙设备连接成功，正在查询设备参数。", playback_finished_at=100.0)
        guard.remember_tts(
            "目标角度 0.0 度，实际角度 -5.1 度，经度 111.00，纬度 11.00。",
            playback_finished_at=101.0,
        )

        self.assertTrue(
            guard.should_ignore_asr("蓝牙设备连接成功，正在查询", now=103.0)
        )


class SpeakingSessionCounterTests(unittest.TestCase):
    def test_mic_stays_muted_until_all_sessions_finish(self):
        tracker = SpeakingSessionCounter()

        self.assertTrue(tracker.start_session())
        self.assertFalse(tracker.start_session())
        self.assertTrue(tracker.is_speaking)

        self.assertFalse(tracker.finish_session())
        self.assertTrue(tracker.is_speaking)

        self.assertTrue(tracker.finish_session())
        self.assertFalse(tracker.is_speaking)


if __name__ == "__main__":
    unittest.main()
