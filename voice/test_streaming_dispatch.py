"""No-network checks for streaming dispatch and Pi motor gating."""

import sys
import time
import types
import unittest
from unittest.mock import patch

from pi_voice import MODEL, PiVoiceTest
from voice_test import FRAME_BYTES


class StreamingDispatchTest(unittest.TestCase):
    def setUp(self):
        fake_cartesia = types.SimpleNamespace(Cartesia=lambda api_key: object())
        self.cartesia_patch = patch.dict(sys.modules, {"cartesia": fake_cartesia})
        self.cartesia_patch.start()
        self.rows_patch = patch("voice_test.write_row")
        self.rows_patch.start()
        self.app = PiVoiceTest("fake", "cartesia-key", "openrouter-key", MODEL)
        self.app.streaming_active = True
        self.requests = []

        def dashboard_request(url, data=None, **kwargs):
            self.requests.append((url, data))
            if url.endswith("/api/state"):
                return {"holding": None, "pose": {"active": False}, "motors": [
                    {"id": 6, "motion_enabled": True, "fault": False,
                     "position": 500, "effective_upper": 900, "effective_lower": 100}]}
            return {}

        self.app.dashboard_request = dashboard_request

    def tearDown(self):
        self.rows_patch.stop()
        self.cartesia_patch.stop()

    def final_turn(self):
        self.app.on_streaming_turn("Hey Arm open the gripper", {
            "turn_ms": 1000, "first_partial_ms": 300, "finalize_ms": 50, "stt_ms": 50})
        self.app.handle_streaming_turns()

    def test_only_final_callback_reaches_gate_and_enable_is_one_move(self):
        # Partial and eager updates are never passed to the dispatch callback.
        self.app.handle_streaming_turns()
        self.assertEqual(self.requests, [])
        self.final_turn()
        self.assertEqual(self.requests, [])  # Expired one-move gate: dry run.
        self.assertIn("Dry run", self.app.ui["action"])

        self.app.arm_enabled_until = time.monotonic() + 30
        self.final_turn()
        self.assertEqual(sum(url.endswith("/api/motor6/go") for url, _ in self.requests), 1)
        self.assertEqual(self.app.arm_enabled_until, 0)
        self.final_turn()
        self.assertEqual(sum(url.endswith("/api/motor6/go") for url, _ in self.requests), 1)

    def test_paused_listening_drops_final_turn_and_sends_zero_pcm(self):
        sent = []
        self.app.streaming_stt = types.SimpleNamespace(send=sent.append)
        self.app.listening = False
        self.app.handle_streaming_frame(bytes([7]) * FRAME_BYTES)
        self.assertEqual(sent, [bytes(FRAME_BYTES)])
        self.final_turn()
        self.assertEqual(self.requests, [])

    def test_stream_failure_falls_back_to_batch(self):
        self.app.on_streaming_turn("Hey Arm open the gripper", {"turn_ms": 1000})
        self.app.streaming_stt = types.SimpleNamespace(send=lambda frame: (_ for _ in ()).throw(RuntimeError()))
        self.app.handle_streaming_frame(bytes(FRAME_BYTES))
        self.app.handle_streaming_turns()
        self.assertFalse(self.app.streaming_active)
        self.assertEqual(self.app.stt_mode, "batch")
        self.assertTrue(self.app.stream_close_pending)
        self.assertEqual(self.app.ui["stt_mode"], "batch")
        self.assertEqual(self.requests, [])

    def test_stream_meter_updates_without_batch_segmentation(self):
        sent = []
        self.app.streaming_stt = types.SimpleNamespace(send=sent.append)
        for _ in range(10):
            self.app.handle_streaming_frame(bytes(FRAME_BYTES))
        self.assertEqual(len(sent), 10)
        self.assertEqual(self.app.ui["mic_level"], 0)
        self.assertEqual(self.app.ui["trigger_level"], 0)
        self.assertEqual(len(self.app.segmenter.ambient), 0)


if __name__ == "__main__":
    unittest.main()
