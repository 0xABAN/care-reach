#!/usr/bin/env python3
"""Pi microphone -> Cartesia STT -> Jev command decision -> guarded dashboard API."""

from __future__ import annotations

import argparse
import json
import os
import re
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from jev_decisions import classify_jev
from voice_test import ROOT, VoiceTest, normalize


ROBOT_URL = "http://127.0.0.1:8765"
MODEL = "typesafe/jev-1.14"
TAUGHT_COMMANDS = {"lift_this", "press_button"}
COMMANDS = {"open_gripper", "close_gripper", "home", "extend", "dance",
            "stop", "cancel", "pick_up_can", "none"} | TAUGHT_COMMANDS
POSE_COMMANDS = {"home", "extend", "dance"} | TAUGHT_COMMANDS
MOTION_COMMANDS = POSE_COMMANDS | {"open_gripper", "close_gripper"}
DIRECT_ACTIONS = {"open_gripper", "close_gripper", "home", "extend", "dance"}
WAKE_PATTERN = re.compile(r"\bhey[\s,.!?-]+arm\b", re.IGNORECASE)

# Cartesia can put a clear request inside a longer final turn when other
# people are talking. Match the request near the wake word, then ignore
# unrelated words later in that same turn.
COMMAND_PATTERNS = {
    "open_gripper": r"(?:open|release) (?:(?:the|your) )?(?:gripper|claw|hand|fingers|it)",
    "close_gripper": r"(?:close|shut) (?:the |your )?(?:gripper|claw|hand|fingers)",
    "home": (r"(?:retract(?: the arm)?|attract|go (?:to )?(?:the )?home(?: position)?|"
             r"return (?:to )?home|home the arm|move (?:the arm|it) home|home)"),
    "extend": r"(?:extend(?: the arm)?|(?:raise|point|move) (?:the arm|it) straight up)",
    "dance": r"(?:dance(?: for me)?|do (?:a|the) dance|start (?:the )?dance)",
    "lift_this": (r"(?:(?:lift|pick up|grab) (?:the )?(?:this|that|it|object|thing)(?: up)?|"
                  r"pick (?:this|that|it) up)"),
    "press_button": r"(?:press|push|hit|click) (?:the |that |a )?button",
    "stop": r"(?:stop(?: now)?|halt|freeze|emergency stop)",
    "cancel": r"(?:cancel(?: that)?|never mind)",
}
MODEL_CUES = {
    "open_gripper": r"\b(?:open|release|unclench|let go of)\b.{0,35}\b(?:hand|gripper|claw|fingers|it)\b",
    "close_gripper": r"\b(?:close|shut|clench|grip)\b.{0,35}\b(?:hand|gripper|claw|fingers|it)\b",
    "home": r"\b(?:retract|attract|home|pull (?:the )?arm back|bring (?:the )?arm back|draw (?:the )?arm back)\b",
    "extend": r"\b(?:extend|upright|straight up|raise (?:the )?arm|lift (?:the )?arm|point (?:the )?arm up)\b",
    "dance": r"\b(?:dance|jig|wiggle)\b",
    "lift_this": r"\b(?:lift|pick up|grab)\b.{0,20}\b(?:this|that|it|object|thing)\b",
    "press_button": r"\b(?:press|push|hit|click)\b.{0,15}\bbutton\b",
    "stop": r"\b(?:stop|halt|freeze)\b",
    "cancel": r"\b(?:cancel|never mind|forget that)\b",
}
DIRECT_PREFIX = re.compile(
    r"(?:(?:please|uh|um|okay|alright|well|so) )*"
    r"(?:(?:(?:can|could|would) you|go ahead and|i want you to) )?"
    r"(?:(?:please|uh|um) )*"
)
DESCRIPTIVE_CONTEXT = re.compile(
    r"\b(?:i saw|we saw|they saw|we were talking|talking about|discussing|"
    r"mentioned|you said|i said|someone said|he said|she said|they said|"
    r"heard someone)\b"
)


def quoted_span(text: str, start: int, end: int) -> bool:
    """Whether a wake phrase or command is within quoted speech."""
    marks = {'"', '“', '”'}
    return (sum(char in marks for char in text[:start]) % 2 == 1
            or any(char in marks for char in text[start:end]))


def local_request(text: str) -> str | None:
    """Return one unambiguous command near the start of a post-wake turn."""
    words = normalize(text)
    tokens = words.split()
    if not tokens:
        return None
    raw_tokens = list(re.finditer(r"[a-z0-9]+", text, flags=re.IGNORECASE))
    hits: list[tuple[str, int, int]] = []
    for command, pattern in COMMAND_PATTERNS.items():
        for match in re.finditer(r"\b" + pattern + r"\b", words):
            start = len(words[:match.start()].split())
            end = len(words[:match.end()].split())
            if not quoted_span(text, raw_tokens[start].start(), raw_tokens[end - 1].end()):
                hits.append((command, start, end))
    nearby_hits = [hit for hit in hits if hit[1] <= 5
                   and DIRECT_PREFIX.fullmatch(" ".join(tokens[:hit[1]]) + (" " if hit[1] else ""))]
    if not nearby_hits:
        return None
    # A nearby second request is ambiguous, even if an LLM picks one.
    nearby = {command for command, start, _ in hits if start <= 8}
    if len(nearby) != 1:
        return None
    command, start, end = min(nearby_hits, key=lambda hit: hit[1])
    before = " ".join(tokens[max(0, start - 4):start])
    after = " ".join(tokens[end:end + 2])
    if re.search(r"\b(?:dont|don t|do not|never|not|no|avoid|without|maybe|perhaps|"
                 r"possibly|should i|could i|can i|what if|instead of)\b", before):
        return None
    if re.search(r"\b(?:not|no|never|dont|don t)\b", after):
        return None
    return command


def jev_state(text: str) -> str:
    """Use a clear spoken command clause, leaving unclear requests intact."""
    command = local_request(text)
    if command is None:
        return text
    words = normalize(text)
    raw_tokens = list(re.finditer(r"[a-z0-9]+", text, flags=re.IGNORECASE))
    for match in re.finditer(r"\b" + COMMAND_PATTERNS[command] + r"\b", words):
        start = len(words[:match.start()].split())
        end = len(words[:match.end()].split())
        if start <= 5:
            return text[raw_tokens[start].start():raw_tokens[end - 1].end()]
    return text


def grounded_request(text: str, command: str) -> bool:
    """Reject obvious ambiguity and non-commands after Jev's intent decision."""
    direct = local_request(text)
    if direct:
        return direct == command
    if command not in MODEL_CUES:
        return False
    words = normalize(text)
    tokens = words.split()
    early = " ".join(tokens[:12])
    if any(mark in text for mark in ('"', '“', '”')):
        return False
    if re.search(r"\b(?:dont|don t|do not|never|not|no|avoid|without|maybe|perhaps|"
                 r"possibly|should i|could i|can i|what if|instead of|i think)\b", early):
        return False
    if re.match(r"(?:what|why|how|when|where|is|are|does|do)\b", early):
        return False
    if DESCRIPTIVE_CONTEXT.search(early):
        return False
    raw_tokens = list(re.finditer(r"[a-z0-9]+", text, flags=re.IGNORECASE))
    cues: set[str] = set()
    for candidate, pattern in MODEL_CUES.items():
        for match in re.finditer(pattern, words):
            start = len(words[:match.start()].split())
            end = len(words[:match.end()].split())
            if start <= 8 and not quoted_span(text, raw_tokens[start].start(),
                                             raw_tokens[end - 1].end()):
                cues.add(candidate)
    if cues and cues != {command}:
        return False
    return True


def request_json(url: str, data: dict | None = None, *, headers: dict | None = None,
                 timeout: float = 8) -> dict:
    body = json.dumps(data).encode() if data is not None else None
    request = Request(url, data=body, headers=headers or {"Content-Type": "application/json"})
    with urlopen(request, timeout=timeout) as response:
        result = json.load(response)
    if not isinstance(result, dict):
        raise ValueError("Expected an object response")
    return result


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
        self.command_backend = "Jev (awaiting first command)"
        stt_model = "Ink-2 (streaming)" if stt_mode == "stream" else "Ink-Whisper (batch)"
        self.ui["model"] = f"Cartesia {stt_model} + {model}"
        self.ui["stt_mode"] = stt_mode
        self.ui["action"] = ""
        self.toggle_lock = threading.Lock()
        self.listening = False
        self.ui["listening"] = False

    def set_listening(self, enabled: bool) -> None:
        with self.toggle_lock:
            super().set_listening(enabled)

    def parse_wake(self, text: str) -> tuple[bool, str]:
        words = normalize(text)
        match = next((candidate for candidate in WAKE_PATTERN.finditer(text)
                      if not quoted_span(text, *candidate.span())), None)
        return (True, normalize(text[match.end():])) if match else (False, words)

    def allow_followup_without_wake(self) -> bool:
        return False

    def command_after_wake(self, text: str) -> str:
        match = next((candidate for candidate in WAKE_PATTERN.finditer(text)
                      if not quoted_span(text, *candidate.span())), None)
        return text[match.end():].strip() if match else text.strip()

    def extra_state(self) -> dict:
        return {
            "command_latency_ms": self.command_latency_ms,
            "command_backend": self.command_backend,
        }

    def choose_command(self, text: str) -> str | None:
        self.command_latency_ms = None
        direct = local_request(text)
        if direct in DIRECT_ACTIONS:
            self.command_backend = "Local"
            self.command_latency_ms = 0
            return direct
        try:
            command, self.command_latency_ms = classify_jev(
                jev_state(text), self.openrouter_key, self.openrouter_model)
            self.command_backend = "Jev"
        except (HTTPError, URLError, TimeoutError, ValueError, KeyError, IndexError,
                TypeError, json.JSONDecodeError, RuntimeError) as exc:
            self.command_backend = "Jev unavailable"
            self.print(f"[error] Jev command decision failed ({type(exc).__name__}); no motion sent.")
            return None
        if command not in COMMANDS:
            self.print("[guard] Jev returned an unsupported command.")
            return None
        if command in MOTION_COMMANDS | {"stop", "cancel"} and not grounded_request(text, command):
            self.print(f"[guard] Rejected {command}: transcript is not a direct request.")
            return None
        if command == "none":
            return None
        return command

    @staticmethod
    def local_command(text: str) -> str:
        command = local_request(text)
        if command:
            return command
        words = normalize(text)
        if words in {"pick up the can", "pick up a can", "grab the can",
                     "pick up that can"}:
            return "pick_up_can"
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
                    if state.get("movement_mode") in {"shoulder_isolated", "single_joint"}:
                        return "Not executed: select coordinated mode for this pose"
                    return "Not executed: a joint is not ready for a group pose"
                if command in TAUGHT_COMMANDS and not state.get("taught_moves", {}).get(command):
                    return f"Not executed: {command} has not been taught on the dashboard yet"
                self.dashboard_request(f"{ROBOT_URL}/api/pose/start", {"pose": command}, timeout=5)
                if command in TAUGHT_COMMANDS:
                    return f"Playing taught move {command.replace('_', ' ')}"
                return f"Moving to {command} pose"
            gripper = next(m for m in state["motors"] if m["id"] == 6)
            if not gripper.get("motion_enabled") or gripper.get("fault"):
                return "Not executed: gripper is not ready"
            if command == "open_gripper":
                target = gripper["effective_upper"]
                distance = target - gripper["position"]
                direction = "Opening"
            else:
                target = gripper["effective_lower"]
                distance = gripper["position"] - target
                direction = "Closing"
            if distance <= 0:
                return "Not executed: gripper is already near that limit"
            self.dashboard_request(f"{ROBOT_URL}/api/motor6/go", {"target": target}, timeout=5)
            return f"{direction} gripper to tick {target}"
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
