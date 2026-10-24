"""No-network checks for streaming dispatch and voice wake gating."""

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
        self.app.openrouter_disabled = True
        self.requests = []
        self.robot_state = {"holding": None, "pose": {"active": False},
                            "pose_enabled": True, "motors": [
                                {"id": 6, "motion_enabled": True, "fault": False,
                                 "position": 500, "effective_upper": 900,
                                 "effective_lower": 100}]}

        def dashboard_request(url, data=None, **kwargs):
            self.requests.append((url, data))
            if url.endswith("/api/state"):
                return self.robot_state
            return {}

        self.app.dashboard_request = dashboard_request

    def tearDown(self):
        self.rows_patch.stop()
        self.cartesia_patch.stop()

    def final_turn(self, phrase="Hey Arm open the gripper"):
        self.app.on_streaming_turn(phrase, {
            "turn_ms": 1000, "first_partial_ms": 300, "finalize_ms": 50, "stt_ms": 50})
        self.app.handle_streaming_turns()

    def test_final_wake_command_moves_without_click_enable(self):
        # Partial and eager updates are never passed to the dispatch callback.
        self.app.handle_streaming_turns()
        self.assertEqual(self.requests, [])
        self.final_turn()
        self.assertEqual(sum(url.endswith("/api/motor6/go") for url, _ in self.requests), 1)
        self.final_turn()
        self.assertEqual(sum(url.endswith("/api/motor6/go") for url, _ in self.requests), 2)

    def test_no_wake_does_not_move_and_wake_only_opens_short_window(self):
        for phrase in ("dance", "open the hand", "extend", "stop", "cancel",
                       "Hey um dance", "Hey aam dance"):
            self.final_turn(phrase)
        self.assertEqual(self.requests, [])
        self.final_turn("Hey Arm dance")
        self.assertTrue(any(url.endswith("/api/pose/start") for url, _ in self.requests))
        self.requests.clear()
        self.final_turn("Hey Arm")
        self.assertEqual(self.requests, [])
        self.assertGreater(self.app.armed_until, time.monotonic())
        self.final_turn("dance")
        self.assertTrue(any(url.endswith("/api/pose/start") for url, _ in self.requests))
        self.requests.clear()
        self.final_turn("dance")
        self.assertEqual(self.requests, [])
        self.app.armed_until = time.monotonic() - 1
        self.final_turn("extend")
        self.assertEqual(self.requests, [])

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

    def test_named_voice_poses_use_distinct_safe_targets(self):
        cases = (("go to home position", "home"), ("retract", "home"),
                 ("extend", "extend"), ("dance", "dance"))
        for phrase, pose in cases:
            self.requests.clear()
            self.final_turn(f"Hey Arm {phrase}")
            self.assertIn(("http://127.0.0.1:8765/api/pose/start", {"pose": pose}),
                          self.requests)
            self.assertFalse(any(data == {"pose": "fold"} for _, data in self.requests))

    def test_open_and_close_hand_are_local_gripper_actions(self):
        for phrase in ("open the hand", "close the hand"):
            self.requests.clear()
            self.final_turn(f"Hey Arm {phrase}")
            self.assertTrue(any(url.endswith("/api/motor6/go") for url, _ in self.requests))
            self.assertEqual(self.app.command_backend, "Local exact phrase (fast path)")

    def test_pose_busy_or_fault_state(self):
        self.robot_state["pose"]["active"] = True
        self.final_turn("Hey Arm dance")
        self.assertFalse(any(url.endswith("/api/pose/start") for url, _ in self.requests))
        self.robot_state["pose"]["active"] = False
        self.requests.clear()
        self.robot_state["pose_enabled"] = False
        self.final_turn("Hey Arm extend")
        self.assertFalse(any(url.endswith("/api/pose/start") for url, _ in self.requests))

    def test_negated_questions_and_quoted_commands_do_not_move(self):
        self.app.openrouter_disabled = False
        with patch("pi_voice.classify_openrouter", return_value=("dance", 5)):
            for phrase in ("don't dance", "should I dance", 'I said "dance"', "maybe dance"):
                self.requests.clear()
                self.final_turn(f"Hey Arm {phrase}")
                self.assertFalse(any(url.endswith("/api/pose/start") for url, _ in self.requests))
        self.requests.clear()
        self.final_turn('Hey Arm "dance"')
        self.assertFalse(any(url.endswith("/api/pose/start") for url, _ in self.requests))

    def test_stop_and_cancel_require_wake(self):
        self.final_turn("stop")
        self.assertEqual(self.requests, [])
        self.final_turn("Hey Arm stop")
        self.assertTrue(any(url.endswith("/api/pose/stop") for url, _ in self.requests))
        self.requests.clear()
        self.final_turn("Hey Arm cancel")
        self.assertFalse(any(url.endswith("/api/pose/start") for url, _ in self.requests))


if __name__ == "__main__":
    unittest.main()
