"""Typed Jev command decision for text already isolated after the wake phrase.

This module never handles microphone audio or finds the wake phrase. The caller
must supply only the post-wake text, and must apply robot-state safety checks
before acting on a decision.
"""

from __future__ import annotations

import json
import math
import time
from urllib.request import Request, urlopen


JEV_URL = "https://openrouter.ai/api/alpha/decisions"
JEV_MODEL = "typesafe/jev-1.14"
CRITERIA = {
    "open_gripper": "Direct request to open or release the robot hand, gripper, claw, or fingers.",
    "close_gripper": "Direct request to close the robot hand, grip, or grasp with the fingers.",
    "home": "Direct request to retract the arm, bring it back, or move to its saved Home pose.",
    "extend": "Direct request to extend or raise the arm into its upright straight-up pose.",
    "dance": "Direct request for the robot arm to dance, jig, or perform its saved dance.",
    "stop": "Direct request to stop current robot motion immediately.",
    "cancel": "Direct request to cancel a pending robot request or wake window.",
    "none": (
        "No single clear immediate robot action. Includes unrelated speech, unsupported tasks, "
        "questions, negated or quoted commands, uncertainty, and multiple conflicting actions."
    ),
}
MIN_PROBABILITY = 0.70
MIN_MARGIN = 0.20
MIN_CONFIDENCE = 0.60


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Jev returned invalid {label}")
    number = float(value)
    if not math.isfinite(number) or not 0 <= number <= 1:
        raise ValueError(f"Jev returned invalid {label}")
    return number


def classify_jev(text: str, key: str, model: str = JEV_MODEL) -> tuple[str, int]:
    """Choose one supported action; uncertain decisions fail closed to ``none``.

    Return ``(command, elapsed_ms)``. Network and malformed-response errors
    propagate so the caller can log a sanitized error and decline to move.
    """
    if not isinstance(text, str) or not isinstance(key, str) or not isinstance(model, str):
        raise TypeError("Jev input, key, and model must be strings")
    if not text.strip():
        return "none", 0
    if not key.strip() or not model.strip():
        raise ValueError("Jev key and model must be configured")

    payload = {
        "model": model,
        "state": text,
        "questions": {
            "command": {
                "type": "choice",
                "instructions": (
                    "Which one immediate, unambiguous action does the speaker request from the "
                    "robot arm? Choose none if there is no supported direct request, or if the "
                    "speech is negated, quoted, hypothetical, or asks for multiple actions."
                ),
                "criteria": CRITERIA,
            },
        },
    }
    request = Request(
        JEV_URL,
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {key}",
            "Content-Type": "application/json",
            "HTTP-Referer": "https://github.com/0xABAN/care-reach",
            "X-Title": "Care Reach Voice",
        },
        method="POST",
    )
    started = time.monotonic()
    with urlopen(request, timeout=8) as response:
        result = json.load(response)
    elapsed_ms = round((time.monotonic() - started) * 1000)
    try:
        answer = result["answers"]["command"]
        if answer["type"] != "choice":
            raise ValueError("Jev returned a non-choice command answer")
        choice = answer["choice"]
        if not isinstance(choice, str) or choice not in CRITERIA:
            raise ValueError("Jev returned an unknown command")
        probabilities = answer["probabilities"]
        if not isinstance(probabilities, dict) or set(probabilities) != set(CRITERIA):
            raise ValueError("Jev returned incomplete command probabilities")
        scores = {name: _number(value, f"{name} probability")
                  for name, value in probabilities.items()}
        confidence = _number(answer["confidence"], "confidence")
    except (KeyError, TypeError, IndexError) as error:
        raise ValueError("Jev returned an invalid command answer") from error

    if choice == "none":
        return "none", elapsed_ms
    alternatives = max(value for name, value in scores.items() if name != choice)
    if (scores[choice] < MIN_PROBABILITY or scores[choice] - alternatives < MIN_MARGIN
            or confidence < MIN_CONFIDENCE):
        return "none", elapsed_ms
    return choice, elapsed_ms
