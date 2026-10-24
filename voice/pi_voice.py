#!/usr/bin/env python3
"""Pi USB microphone -> Cartesia STT -> OpenRouter command -> guarded dashboard API."""

from __future__ import annotations

import argparse
import json
import os
import re
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from voice_test import ROOT, VoiceTest, normalize


OPENROUTER_URL = "https://openrouter.ai/api/v1/chat/completions"
ROBOT_URL = "http://127.0.0.1:8765"
MODEL = "openai/gpt-4.1-mini"
COMMANDS = {"open_gripper", "close_gripper", "home", "extend", "dance",
            "stop", "cancel", "pick_up_can", "none"}
POSE_COMMANDS = {"home", "extend", "dance"}
MOTION_COMMANDS = POSE_COMMANDS | {"open_gripper", "close_gripper"}


def grounded_request(text: str, command: str) -> bool:
    """Require a direct request even when a classifier proposes a command."""
    if any(mark in text for mark in ('"', '“', '”')):
        return False
    words = normalize(text)
    checked = words.replace("never mind", "") if command == "cancel" else words
    if re.search(r"\b(dont|do not|never|not|no|avoid|without|maybe|perhaps|"
                 r"possibly|should i|could i|can i|what if|instead of)\b", checked):
        return False
    prefix = r"(?:please |can you |could you |would you |go ahead and |i want you to )?"
    core = {
        "open_gripper": r"(?:open|release) (?:the |your )?(?:gripper|claw|hand|fingers)",
        "close_gripper": r"(?:close|shut) (?:the |your )?(?:gripper|claw|hand|fingers)",
        "home": r"(?:retract(?: the arm)?|go (?:to )?(?:the )?home(?: position)?|"
                r"return (?:to )?home|home the arm|move (?:the arm|it) home)",
        "extend": r"(?:extend(?: the arm)?|(?:raise|point|move) (?:the arm|it) straight up)",
        "dance": r"(?:dance(?: for me)?|do (?:a|the) dance|start (?:the )?dance)",
        "stop": r"(?:stop(?: now)?|halt|freeze|emergency stop)",
        "cancel": r"(?:cancel(?: that)?|never mind)",
    }.get(command)
    return bool(core and re.fullmatch(prefix + core + r"(?: please)?", words))


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
        '"open_gripper", "close_gripper", "home", "extend", "dance", '
        '"stop", "cancel", "pick_up_can", "none". '
        "Choose open_gripper only for an unambiguous request to open or release the gripper. "
        "Choose close_gripper only for an unambiguous request to close the gripper. "
        "Choose home for retract or return to the saved Home pose. "
        "Choose extend for a request to point the arm straight up. "
        "Choose dance only for an explicit request to dance. Never choose fold: "
        "the wrist camera makes the old folded pose unsafe. "
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
    def __init__(self, device: str, cartesia_key: str, openrouter_key: str, model: str,
                 stt_mode: str = "stream"):
        super().__init__(device, cartesia_key, backend="alsa")
        self.stt_mode = stt_mode
        self.openrouter_key = openrouter_key
        self.openrouter_model = model
        self.dashboard_file = ROOT / "pi_dashboard.html"
        self.example_phrase = "open the hand"
        self.segmenter.silence_frames = 18
        self.openrouter_disabled = os.environ.get("OPENROUTER_DISABLED") == "1"
        self.command_backend = ("Local exact phrases (OpenRouter key rejected)"
                                if self.openrouter_disabled else "OpenRouter (awaiting first command)")
        stt_model = "Ink-2 (streaming)" if stt_mode == "stream" else "Ink-Whisper (batch)"
        self.ui["model"] = f"Cartesia {stt_model} + {model}"
        self.ui["stt_mode"] = stt_mode
        self.ui["action"] = ""

    def parse_wake(self, text: str) -> tuple[bool, str]:
        words = normalize(text)
        if words == "hey arm":
            return True, ""
        if words.startswith("hey arm "):
            return True, words[len("hey arm "):]
        return False, words

    def command_after_wake(self, text: str) -> str:
        return re.sub(r"^\s*hey[\s,.!?-]+arm\b[\s,.!?-]*", "", text,
                      count=1, flags=re.IGNORECASE).strip()

    def extra_state(self) -> dict:
        return {
            "command_latency_ms": self.command_latency_ms,
            "command_backend": self.command_backend,
        }

    def choose_command(self, text: str) -> str | None:
        self.command_latency_ms = None
        command = self.local_command(text)
        if command != "none":
            self.command_latency_ms = 0
            self.command_backend = "Local exact phrase (fast path)"
        else:
            command = None
        if command is None and not self.openrouter_disabled:
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
        if command in MOTION_COMMANDS | {"stop", "cancel"} and not grounded_request(text, command):
            self.print(f"[guard] Rejected {command}: transcript is not a direct request.")
            return None
        if command == "none":
            return None
        return command

    @staticmethod
    def local_command(text: str) -> str:
        words = normalize(text)
        if words in {"open the gripper", "open gripper", "release the gripper",
                     "open your hand", "open the hand", "open the claw"}:
            return "open_gripper"
        if words in {"close the gripper", "close gripper", "shut the gripper",
                     "close your hand", "close the hand", "close the claw"}:
            return "close_gripper"
        if words in {"go home", "go to home", "go to home position", "go to the home position",
                     "return home", "retract", "retract the arm"}:
            return "home"
        if words in {"extend", "extend the arm", "point the arm straight up",
                     "raise the arm straight up"}:
            return "extend"
        if words in {"dance", "do a dance", "do the dance", "start the dance"}:
            return "dance"
        if words in {"stop", "stop now", "halt", "freeze", "emergency stop"}:
            return "stop"
        if words in {"cancel", "cancel that", "never mind"}:
            return "cancel"
        if words in {"pick up the can", "pick up a can", "grab the can",
                     "pick up that can"}:
            return "pick_up_can"
        for command in ("stop", "cancel", "open_gripper", "close_gripper", "home", "extend", "dance"):
            if grounded_request(text, command):
                return command
        return "none"

    def execute_command(self, command: str) -> str:
        if command == "pick_up_can":
            return "Not executed: can position and grasp plan are not available"
        if command == "cancel":
            self.armed_until = 0.0
            return "Pending wake window cancelled"
        if command == "stop":
            try:
                self.dashboard_request(f"{ROBOT_URL}/api/hold/stop", {}, timeout=5)
                self.dashboard_request(f"{ROBOT_URL}/api/pose/stop", {}, timeout=5)
                self.armed_until = 0.0
                return "Stop sent to arm controller"
            except (HTTPError, URLError, TimeoutError, ValueError) as exc:
                return f"Stop failed: {type(exc).__name__}"
        if command not in MOTION_COMMANDS:
            return "No motor action for this command"
        try:
            state = self.dashboard_request(f"{ROBOT_URL}/api/state", timeout=5)
            if state.get("holding") is not None or state.get("pose", {}).get("active"):
                return "Not executed: arm is already moving"
            if command in POSE_COMMANDS:
                if not state.get("pose_enabled"):
                    return "Not executed: a joint is not ready for a group pose"
                self.dashboard_request(f"{ROBOT_URL}/api/pose/start", {"pose": command}, timeout=5)
                return f"Moving to {command} pose"
            gripper = next(m for m in state["motors"] if m["id"] == 6)
            if not gripper.get("motion_enabled") or gripper.get("fault"):
                return "Not executed: gripper is not ready"
            if command == "open_gripper":
                target = min(gripper["position"] + 120, gripper["effective_upper"])
                distance = target - gripper["position"]
                direction = "Opening"
            else:
                target = max(gripper["position"] - 120, gripper["effective_lower"])
                distance = gripper["position"] - target
                direction = "Closing"
            if distance < 50:
                return "Not executed: gripper is already near that limit"
            self.dashboard_request(f"{ROBOT_URL}/api/motor6/go", {"target": target}, timeout=5)
            return f"{direction} gripper to tick {target} (120 tick maximum)"
        except (HTTPError, URLError, TimeoutError, ValueError, KeyError, StopIteration) as exc:
            return f"Gripper action failed: {type(exc).__name__}"

    def dashboard_request(self, url: str, data: dict | None = None, *, timeout: float = 8) -> dict:
        started = time.monotonic()
        try:
            return request_json(url, data, timeout=timeout)
        finally:
            self.dashboard_api_ms += round((time.monotonic() - started) * 1000)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="hw:2,0", help="ALSA USB mic device")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--model", default=MODEL)
    parser.add_argument("--stt-mode", choices=("stream", "batch"), default="stream",
                        help="Cartesia live turns or local batch utterance detection")
    args = parser.parse_args()
    cartesia_key = os.environ.get("CARTESIA_API_KEY", "").strip()
    openrouter_key = os.environ.get("OPENROUTER_API_KEY", "").strip()
    if not cartesia_key or not openrouter_key:
        parser.error("CARTESIA_API_KEY and OPENROUTER_API_KEY must be set")
    PiVoiceTest(args.device, cartesia_key, openrouter_key, args.model, args.stt_mode).run(args.port)


if __name__ == "__main__":
    main()
