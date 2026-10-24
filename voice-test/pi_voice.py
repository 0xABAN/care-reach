#!/usr/bin/env python3
"""Pi USB microphone -> Cartesia STT -> OpenRouter command -> guarded dashboard API."""

from __future__ import annotations

import argparse
import json
import os
import re
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from voice_test import ROOT, VoiceTest, normalize


OPENROUTER_URL = "https://openrouter.ai/api/v1/chat/completions"
ROBOT_URL = "http://127.0.0.1:8765"
MODEL = "openai/gpt-4.1-mini"
MOTION_WINDOW_SECONDS = 60
COMMANDS = {"open_gripper", "stop", "cancel", "pick_up_can", "none"}


def request_json(url: str, data: dict | None = None, *, headers: dict | None = None,
                 timeout: float = 8) -> dict:
    body = json.dumps(data).encode() if data is not None else None
    request = Request(url, data=body, headers=headers or {"Content-Type": "application/json"})
    with urlopen(request, timeout=timeout) as response:
        result = json.load(response)
    if not isinstance(result, dict):
        raise ValueError("Expected an object response")
    return result


def classify_openrouter(text: str, key: str, model: str = MODEL) -> tuple[str, int]:
    prompt = (
        "Classify a short transcript for an assistive robot. Return only a JSON object "
        'with one key, "command", whose value is exactly one of: '
        '"open_gripper", "stop", "cancel", "pick_up_can", "none". '
        "Choose open_gripper only for an unambiguous request to open or release the gripper. "
        "Choose stop only for an unambiguous request to stop current motion. "
        "Choose cancel only for an unambiguous request to cancel a pending request. "
        "Choose pick_up_can for a request to pick up a can, even though this robot cannot yet do it. "
        "Choose none for all other requests, questions, negation, uncertainty, or quoted commands. "
        "Never create a motor target or suggest a new command."
    )
    payload = {
        "model": model,
        "temperature": 0,
        "max_tokens": 45,
        "response_format": {"type": "json_object"},
        "messages": [{"role": "system", "content": prompt},
                     {"role": "user", "content": text[:300]}],
    }
    start = time.monotonic()
    response = request_json(OPENROUTER_URL, payload, headers={
        "Authorization": f"Bearer {key}", "Content-Type": "application/json",
        "HTTP-Referer": "https://github.com/0xABAN/care-reach",
        "X-Title": "Care Reach Pi Voice Test",
    }, timeout=15)
    elapsed_ms = round((time.monotonic() - start) * 1000)
    content = response["choices"][0]["message"]["content"]
    if not isinstance(content, str):
        raise ValueError("OpenRouter returned no command text")
    command = json.loads(content).get("command")
    if command not in COMMANDS:
        raise ValueError("OpenRouter returned an invalid command")
    return command, elapsed_ms


class PiVoiceTest(VoiceTest):
    def __init__(self, device: str, cartesia_key: str, openrouter_key: str, model: str):
        super().__init__(device, cartesia_key, backend="alsa")
        self.openrouter_key = openrouter_key
        self.openrouter_model = model
        self.dashboard_file = ROOT / "pi_dashboard.html"
        self.example_phrase = "open the gripper"
        self.arm_enabled_until = 0.0
        self.command_latency_ms: int | None = None
        self.openrouter_disabled = os.environ.get("OPENROUTER_DISABLED") == "1"
        self.command_backend = ("Local exact phrases (OpenRouter key rejected)"
                                if self.openrouter_disabled else "OpenRouter (awaiting first command)")
        self.ui["model"] = f"Cartesia Ink-Whisper + {model}"
        self.ui["action"] = ""

    def extra_state(self) -> dict:
        return {
            "arm_seconds_left": max(0, round(self.arm_enabled_until - time.monotonic())),
            "command_latency_ms": self.command_latency_ms,
            "command_backend": self.command_backend,
        }

    def set_arm_enabled(self, enabled: bool) -> None:
        if enabled:
            state = request_json(f"{ROBOT_URL}/api/state", timeout=5)
            motors = state.get("motors", [])
            if not any(m.get("id") == 6 and m.get("motion_enabled") and not m.get("fault")
                       for m in motors):
                raise ValueError("Gripper is not ready")
            self.arm_enabled_until = time.monotonic() + MOTION_WINDOW_SECONDS
            self.print(f"[arm] Gripper opening enabled for {MOTION_WINDOW_SECONDS} seconds.")
        else:
            self.arm_enabled_until = 0.0
            self.print("[arm] Motion disabled. Voice decisions remain visible.")

    def choose_command(self, text: str) -> str | None:
        self.command_latency_ms = None
        command = None
        if not self.openrouter_disabled:
            try:
                command, self.command_latency_ms = classify_openrouter(
                    text, self.openrouter_key, self.openrouter_model)
                self.command_backend = "OpenRouter"
            except HTTPError as exc:
                if exc.code == 401:
                    self.openrouter_disabled = True
                    self.print("[error] OpenRouter key rejected (HTTP 401). Using local exact phrases until restart.")
                else:
                    self.print(f"[error] OpenRouter returned HTTP {exc.code}. Using local exact phrases.")
            except (URLError, TimeoutError, ValueError, KeyError, IndexError,
                    TypeError, json.JSONDecodeError) as exc:
                self.print(f"[error] OpenRouter command request failed ({type(exc).__name__}). Using local exact phrases.")
        if command is None:
            self.command_backend = ("Local exact phrases (OpenRouter key rejected)"
                                    if self.openrouter_disabled else "Local exact phrases")
            command = self.local_command(text)
        # An LLM result alone is never permission to move. Check the transcript too.
        words = normalize(text)
        if command == "open_gripper" and not (
            re.search(r"\b(open|release)\b", words)
            and re.search(r"\b(gripper|claw|hand|fingers)\b", words)
        ):
            self.print("[guard] Rejected gripper action: transcript did not ask to open it clearly.")
            return None
        if command == "stop" and not re.search(r"\b(stop|halt|freeze)\b", words):
            return None
        if command == "cancel" and not re.search(r"\b(cancel|never mind)\b", words):
            return None
        if command == "none":
            return None
        return command

    @staticmethod
    def local_command(text: str) -> str:
        words = normalize(text)
        if words in {"open the gripper", "open gripper", "release the gripper",
                     "open your hand", "open the claw"}:
            return "open_gripper"
        if words in {"stop", "stop now", "halt", "freeze", "emergency stop"}:
            return "stop"
        if words in {"cancel", "cancel that", "never mind"}:
            return "cancel"
        if words in {"pick up the can", "pick up a can", "grab the can",
                     "pick up that can"}:
            return "pick_up_can"
        return "none"

    def execute_command(self, command: str) -> str:
        if command == "pick_up_can":
            return "Not executed: can position and grasp plan are not available"
        if command == "cancel":
            self.arm_enabled_until = 0.0
            return "Pending gripper action disabled"
        if command == "stop":
            try:
                request_json(f"{ROBOT_URL}/api/hold/stop", {}, timeout=5)
                request_json(f"{ROBOT_URL}/api/pose/stop", {}, timeout=5)
                self.arm_enabled_until = 0.0
                return "Stop sent to arm controller"
            except (HTTPError, URLError, TimeoutError, ValueError) as exc:
                return f"Stop failed: {type(exc).__name__}"
        if command != "open_gripper":
            return "No motor action for this command"
        if time.monotonic() >= self.arm_enabled_until:
            return "Dry run: enable arm action in this page for a 60-second test"
        self.arm_enabled_until = 0.0  # One physical action per enable.
        try:
            state = request_json(f"{ROBOT_URL}/api/state", timeout=5)
            if state.get("holding") is not None or state.get("pose", {}).get("active"):
                return "Not executed: arm is already moving"
            gripper = next(m for m in state["motors"] if m["id"] == 6)
            if not gripper.get("motion_enabled") or gripper.get("fault"):
                return "Not executed: gripper is not ready"
            target = min(gripper["position"] + 120, gripper["effective_upper"])
            if target - gripper["position"] < 25:
                return "Not executed: gripper is already open near its limit"
            request_json(f"{ROBOT_URL}/api/motor6/go", {"target": target}, timeout=5)
            return f"Opening gripper to tick {target} (120 tick maximum)"
        except (HTTPError, URLError, TimeoutError, ValueError, KeyError, StopIteration) as exc:
            return f"Gripper action failed: {type(exc).__name__}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="hw:2,0", help="ALSA USB mic device")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--model", default=MODEL)
    args = parser.parse_args()
    cartesia_key = os.environ.get("CARTESIA_API_KEY", "").strip()
    openrouter_key = os.environ.get("OPENROUTER_API_KEY", "").strip()
    if not cartesia_key or not openrouter_key:
        parser.error("CARTESIA_API_KEY and OPENROUTER_API_KEY must be set")
    PiVoiceTest(args.device, cartesia_key, openrouter_key, args.model).run(args.port)


if __name__ == "__main__":
    main()
