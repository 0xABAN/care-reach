"""Synthetic audio tests for the local level endpoint detector."""

import unittest
from array import array

from voice_test import FRAME_BYTES, LevelSegmenter, VoiceTest


def pcm_level(level: int) -> bytes:
    return array("h", [level] * (FRAME_BYTES // 2)).tobytes()


class SegmentationTest(unittest.TestCase):
    def test_stationary_background_does_not_make_eight_second_utterances(self):
        segmenter = LevelSegmenter(silence_frames=18)
        events = []
        for index in range(800):  # 16 seconds, twice the former hard cap.
            level = 129 + ((index * 17) % 136)
            events.append(segmenter.feed(VoiceTest.rms(pcm_level(level))))
        self.assertNotIn("start", events)
        self.assertNotIn("end", events)
        self.assertFalse(segmenter.active)

    def test_phrase_on_background_ends_after_short_silence(self):
        segmenter = LevelSegmenter(silence_frames=18)
        events = []
        for level in ([200] * 60 + [900] * 40 + [200] * 30):
            events.append(segmenter.feed(VoiceTest.rms(pcm_level(level))))
        self.assertEqual(events.count("start"), 1)
        self.assertEqual(events.count("end"), 1)
        self.assertEqual(events.index("end") - 100, 17)  # 18 frames = 360 ms.
        self.assertEqual(segmenter.last_endpoint_frames, 18)

    def test_single_frame_spikes_do_not_start_capture(self):
        segmenter = LevelSegmenter()
        events = [segmenter.feed(200 if index % 15 else 600) for index in range(250)]
        self.assertNotIn("start", events)


if __name__ == "__main__":
    unittest.main()
