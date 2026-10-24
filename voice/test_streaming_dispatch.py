"""No-network checks for streaming dispatch and voice wake gating."""

import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
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
        self.state_dir = tempfile.TemporaryDirectory()
        self.state_patch = patch.dict("os.environ", {
            "CARE_REACH_VOICE_LISTENING_STATE": str(Path(self.state_dir.name) / "listening.json")})
        self.state_patch.start()
        self.app = PiVoiceTest("fake", "cartesia-key", "openrouter-key", MODEL)
        self.app.streaming_active = True
        self.jev_patch = patch("pi_voice.classify_jev",
                               side_effect=lambda text, key, model: (self.app.local_command(text), 0))
        self.jev_mock = self.jev_patch.start()
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
        self.jev_patch.stop()
        self.rows_patch.stop()
        self.state_patch.stop()
        self.state_dir.cleanup()
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

    def test_no_wake_does_not_move_even_after_wake_only_turn(self):
        for phrase in ("dance", "open the hand", "extend", "stop", "cancel",
                       "Hey um dance", "Hey aam dance"):
            self.final_turn(phrase)
        self.assertEqual(self.requests, [])
        self.final_turn("Hey Arm dance")
        self.assertTrue(any(url.endswith("/api/pose/start") for url, _ in self.requests))
        self.requests.clear()
        self.final_turn("Hey Arm")
        self.assertEqual(self.requests, [])
        self.assertEqual(self.app.armed_until, 0)
        self.final_turn("dance")
        self.assertEqual(self.requests, [])
        self.requests.clear()
        self.final_turn("dance")
        self.assertEqual(self.requests, [])
        self.app.armed_until = time.monotonic() - 1
        self.final_turn("extend")
        self.assertEqual(self.requests, [])

    def test_wake_with_background_tail_does_not_arm_next_turn(self):
        self.final_turn("Others speaking hey arm and the room keeps chatting")
        self.assertEqual(self.requests, [])
        self.assertEqual(self.app.armed_until, 0)
        self.jev_mock.reset_mock()
        self.final_turn("retract")
        self.assertEqual(self.requests, [])
        self.jev_mock.assert_not_called()

    def test_paused_listening_closes_stream_and_sends_no_pcm(self):
        sent = []
        closed = []
        self.app.streaming_stt = types.SimpleNamespace(send=sent.append, close=lambda: closed.append(True))
        self.app.set_listening(False)
        self.app.handle_streaming_frame(bytes([7]) * FRAME_BYTES)
        self.assertEqual(sent, [])
        self.assertEqual(closed, [True])
        self.assertIsNone(self.app.streaming_stt)
        self.final_turn()
        self.assertEqual(self.requests, [])

    def test_listening_restart_opens_fresh_stream_and_ignores_old_callback(self):
        old_sent, closed, new_sent = [], [], []
        self.app.streaming_stt = types.SimpleNamespace(send=old_sent.append, close=lambda: closed.append(True))
        self.app.set_listening(False)
        class FakeStream:
            def __init__(self, key, turn, error, **kwargs):
                self.turn = turn
                self.send = new_sent.append
                self.start = lambda: None
                self.close = lambda: closed.append(True)
        with patch.dict(sys.modules, {"streaming_stt": types.SimpleNamespace(StreamingSTT=FakeStream)}):
            self.app.set_listening(True)
        self.app.handle_streaming_frame(bytes([3]) * FRAME_BYTES)
        self.assertEqual(old_sent, [])
        self.assertEqual(new_sent, [bytes([3]) * FRAME_BYTES])
        self.assertEqual(closed, [True])
        self.app._stream_turn(self.app.stream_generation - 1, "Hey Arm dance", {})
        self.assertTrue(self.app.transcript_queue.empty())

    def test_listening_off_survives_service_restart(self):
        self.app.streaming_stt = types.SimpleNamespace(close=lambda: None)
        self.app.set_listening(False)
        restarted = PiVoiceTest("fake", "cartesia-key", "openrouter-key", MODEL)
        self.assertFalse(restarted.listening)
        self.assertFalse(restarted.ui["listening"])
        self.assertFalse(restarted.streaming_active)

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

    def test_background_words_do_not_hide_nearby_wake_command(self):
        # These shapes occurred in live Cartesia turns: the wake phrase was
        # buried in conversation or followed by unrelated words.
        turns = (
            "The people over there are talking about something and hey arm retract",
            "It is still really noisy but hey arm retract and those folks keep talking",
            "We were discussing the camera hey arm retract while conversation continues",
            "Hey Arm retract now there are four extra words",
            "Hey Arm attract while the room keeps talking",
        )
        with patch("pi_voice.classify_jev", side_effect=lambda text, key, model: ("home", 7)) as classify:
            for turn in turns:
                self.requests.clear()
                self.final_turn(turn)
                self.assertIn(("http://127.0.0.1:8765/api/pose/start", {"pose": "home"}),
                              self.requests, turn)
        self.assertEqual([call.args[0] for call in classify.call_args_list],
                         ["retract", "retract", "retract", "retract", "attract"])

    def test_background_near_other_commands(self):
        for turn, pose in (
            ("Background speech hey arm extend yes that is what we discussed", "extend"),
            ("Others talking hey arm dance and this conversation goes on", "dance"),
        ):
            self.requests.clear()
            self.final_turn(turn)
            self.assertIn(("http://127.0.0.1:8765/api/pose/start", {"pose": pose}),
                          self.requests)
            self.assertEqual(self.jev_mock.call_args.args[0], pose)
        self.requests.clear()
        self.final_turn("Long noisy discussion hey arm open the hand there is other speech")
        self.assertTrue(any(url.endswith("/api/motor6/go") for url, _ in self.requests))
        self.assertEqual(self.jev_mock.call_args.args[0], "open the hand")

    def test_descriptive_speech_is_not_trimmed_into_a_command(self):
        for spoken, expected in (("Hey Arm I saw the robot dance", "I saw the robot dance"),
                                 ("Hey Arm we were talking about retract", "we were talking about retract"),
                                 ("Hey Arm should I dance", "should I dance")):
            self.requests.clear()
            with patch("pi_voice.classify_jev", return_value=("dance", 7)) as classify:
                self.final_turn(spoken)
            self.assertEqual(classify.call_args.args[0], expected)
            self.assertFalse(any(url.endswith("/api/pose/start") for url, _ in self.requests))

    def test_embedded_wake_still_rejects_negation_quotes_and_conflicts(self):
        for turn in (
            "People talking hey arm don't retract and keep talking",
            "People talking hey arm should I retract now",
            'People talking "hey arm retract" as an example',
            "People talking hey arm retract and then extend",
            "People talking hey arm retract not yet",
            "People talking retract with no wake phrase",
        ):
            self.requests.clear()
            self.final_turn(turn)
            self.assertFalse(any(url.endswith("/api/pose/start") for url, _ in self.requests),
                             turn)

    def test_model_gets_only_post_wake_text_for_unfamiliar_request(self):
        spoken = "People in the room are chatting. Hey Arm please bring the arm back toward me thanks"
        with patch("pi_voice.classify_jev", return_value=("home", 7)) as classify:
            self.final_turn(spoken)
        classify.assert_called_once_with("please bring the arm back toward me thanks",
                                         "openrouter-key", MODEL)
        self.assertIn(("http://127.0.0.1:8765/api/pose/start", {"pose": "home"}),
                      self.requests)

    def test_no_wake_never_calls_model_and_model_cannot_override_ambiguity(self):
        with patch("pi_voice.classify_jev", return_value=("home", 7)) as classify:
            self.app.armed_until = time.monotonic() + 12
            self.final_turn("The discussion mentions bringing the arm back toward me")
            classify.assert_not_called()
            self.assertEqual(self.requests, [])
            self.final_turn("Hey Arm should I bring the arm back toward me")
            self.assertFalse(any(url.endswith("/api/pose/start") for url, _ in self.requests))
            self.requests.clear()
            self.final_turn("Hey Arm bring the arm back or point it upright")
            self.assertFalse(any(url.endswith("/api/pose/start") for url, _ in self.requests))

    def test_open_and_close_hand_use_jev_for_gripper_actions(self):
        for phrase in ("open the hand", "close the hand"):
            self.requests.clear()
            self.final_turn(f"Hey Arm {phrase}")
            self.assertTrue(any(url.endswith("/api/motor6/go") for url, _ in self.requests))
            self.assertEqual(self.app.command_backend, "Jev")

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
        with patch("pi_voice.classify_jev", return_value=("dance", 5)):
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
