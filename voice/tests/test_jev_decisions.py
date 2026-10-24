"""Credential-free checks for the OpenRouter Jev Decisions API contract."""

import io
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jev_decisions import CRITERIA, JEV_MODEL, JEV_URL, classify_jev


def answer(choice="home", *, probability=0.96, confidence=0.90, alternative=0.02):
    probabilities = {name: 0.0 for name in CRITERIA}
    probabilities[choice] = probability
    if choice != "none":
        probabilities["none"] = alternative
    return {"answers": {"command": {
        "type": "choice", "choice": choice,
        "probabilities": probabilities, "confidence": confidence,
    }}}


class JevDecisionTests(unittest.TestCase):
    @staticmethod
    def mocked_response(payload):
        return io.BytesIO(json.dumps(payload).encode())

    def test_sends_only_supplied_post_wake_state_to_decisions_endpoint(self):
        post_wake = "please bring the arm back home"
        with patch("jev_decisions.urlopen", return_value=self.mocked_response(answer())) as opener:
            command, elapsed_ms = classify_jev(post_wake, "dummy-test-key")
        self.assertEqual(command, "home")
        self.assertGreaterEqual(elapsed_ms, 0)
        request = opener.call_args.args[0]
        self.assertEqual(request.full_url, JEV_URL)
        self.assertEqual(request.get_method(), "POST")
        self.assertTrue(request.has_header("Authorization"))
        self.assertEqual(opener.call_args.kwargs["timeout"], 8)
        body = json.loads(request.data)
        self.assertEqual(body["model"], JEV_MODEL)
        self.assertEqual(body["state"], post_wake)
        self.assertEqual(body["questions"]["command"]["type"], "choice")
        self.assertEqual(set(body["questions"]["command"]["criteria"]), set(CRITERIA))

    def test_returns_none_for_model_none_without_extra_action(self):
        with patch("jev_decisions.urlopen", return_value=self.mocked_response(answer("none"))):
            self.assertEqual(classify_jev("what time is it", "dummy-test-key")[0], "none")

    def test_low_probability_margin_or_confidence_returns_none(self):
        responses = [
            answer(probability=0.69),
            answer(probability=0.75, alternative=0.65),
            answer(confidence=0.59),
        ]
        for response in responses:
            with self.subTest(response=response), patch(
                "jev_decisions.urlopen", return_value=self.mocked_response(response)
            ):
                self.assertEqual(classify_jev("retract", "dummy-test-key")[0], "none")

    def test_malformed_or_unknown_answer_fails_closed_by_raising(self):
        malformed = [
            {},
            {"answers": {"command": {"type": "score"}}},
            answer("home") | {"answers": {"command": {
                **answer("home")["answers"]["command"], "choice": "unsafe_action"}}},
            {"answers": {"command": {
                **answer("home")["answers"]["command"],
                "probabilities": {"home": 1.0},
            }}},
            {"answers": {"command": {
                **answer("home")["answers"]["command"],
                "confidence": float("nan"),
            }}},
        ]
        for response in malformed:
            with self.subTest(response=response), patch(
                "jev_decisions.urlopen", return_value=self.mocked_response(response)
            ):
                with self.assertRaises(ValueError):
                    classify_jev("retract", "dummy-test-key")

    def test_network_error_propagates_and_empty_state_skips_network(self):
        with patch("jev_decisions.urlopen", side_effect=TimeoutError("offline")):
            with self.assertRaises(TimeoutError):
                classify_jev("retract", "dummy-test-key")
        with patch("jev_decisions.urlopen") as opener:
            self.assertEqual(classify_jev("  ", "dummy-test-key"), ("none", 0))
            opener.assert_not_called()


if __name__ == "__main__":
    unittest.main()
