import unittest

from audio_engine.echo_guard import RecentSpeechGuard


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


if __name__ == "__main__":
    unittest.main()
