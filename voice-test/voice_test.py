#!/usr/bin/env python3
"""USB microphone voice command evaluator using Cartesia STT. No robot control."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import re
import statistics
import subprocess
import sys
import tempfile
import threading
import time
import wave
from array import array
from collections import Counter, deque
from dataclasses import dataclass
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from cartesia import Cartesia


ROOT = Path(__file__).resolve().parent
MODEL_ID = "cartesia-ink-whisper"
MODEL_NAME = "Cartesia Ink-Whisper (cloud)"
REPORTS = ROOT / "reports"
SAMPLE_RATE = 16000
FRAME_MS = 20
FRAME_BYTES = SAMPLE_RATE * FRAME_MS // 1000 * 2
WAKE_SECONDS = 12
TRIAL_SECONDS = 90
EVENT_FIELDS = ["time", "trial_id", "raw_text", "wake_detected", "decision", "command", "latency_ms"]
TRIAL_FIELDS = ["time", "trial_id", "expected", "observed", "wake_detected", "outcome", "latency_ms"]


def normalize(text: str) -> str:
    return re.sub(r"\s+", " ", re.sub(r"[^a-z0-9 ]", " ", text.lower())).strip()


def classify(text: str) -> str | None:
    words = normalize(text)
    if words in {"stop", "stop now", "emergency stop"}:
        return "stop"
    if words in {"cancel", "cancel that", "never mind"}:
        return "cancel"
    if words in {
        "pick up the can", "pick up that can", "pick up a can",
        "grab the can", "grab that can", "get the can",
    }:
        return "pick_up_can"
    return None


def split_wake(text: str) -> tuple[bool, str]:
    words = normalize(text)
    for phrase in ("hey arm", "hey um"):
        if words == phrase:
            return True, ""
        if words.startswith(phrase + " "):
            return True, words[len(phrase) + 1:]
    return False, words


def write_row(path: Path, fields: list[str], row: dict) -> None:
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        if handle.tell() == 0:
            writer.writeheader()
        writer.writerow(row)


def report() -> Path:
    trials_path = REPORTS / f"trials-{MODEL_ID}.csv"
    trials = []
    if trials_path.exists():
        with trials_path.open(newline="", encoding="utf-8") as handle:
            trials = list(csv.DictReader(handle))
    scored = [row for row in trials if row["outcome"] != "incomplete"]
    counts = Counter(row["outcome"] for row in scored)
    positives = [row for row in scored if row["expected"] != "none"]
    negatives = [row for row in scored if row["expected"] == "none"]
    accepted_latencies = [int(row["latency_ms"]) for row in scored if row["outcome"] == "correct" and row["latency_ms"]]
    lines = [
        "# Cartesia voice test report", "",
        f"Generated: {datetime.now().astimezone().isoformat(timespec='seconds')}", "",
        f"Model: {MODEL_NAME}", "",
        f"Scored command trials: {len(positives)}",
        f"Incomplete trials: {len(trials) - len(scored)}",
        f"Correct commands: {counts['correct']}",
        f"Missed wake words: {counts['missed_trigger']}",
        f"Missed or rejected commands after wake: {counts['missed_command']}",
        f"Wrong accepted commands: {counts['wrong_command']}",
        f"Negative trials: {len(negatives)}",
        f"Unwanted command activations: {counts['unwanted_activation']}",
        f"False wake detections without a command: {counts['false_wake']}",
        f"Median speech-end to decision: {round(statistics.median(accepted_latencies)) if accepted_latencies else 'n/a'} ms",
        "",
        "These counts cover only completed labeled trials. Incomplete trials stay in the table but do not affect scores. These results do not prove safe operation.",
        "",
        "| Trial | Expected | Observed | Outcome | Latency |", "|---|---|---|---|---:|",
    ]
    for row in trials:
        lines.append(f"| {row['trial_id']} | {row['expected']} | {row['observed']} | {row['outcome']} | {row['latency_ms'] or '-'} ms |")
    output = REPORTS / "latest.md"
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output


@dataclass
class Trial:
    id: int
    expected: str
    deadline: float
    wake_seen: bool = False
    observed: str = ""
    latency_ms: int | None = None


class VoiceTest:
    def __init__(self, device: str, api_key: str, *, backend: str = "avfoundation"):
        self.device = device
        self.backend = backend
        self.dashboard_file = ROOT / "dashboard.html"
        self.example_phrase = "pick up the can"
        self.client = Cartesia(api_key=api_key)
        self.trials_path = REPORTS / f"trials-{MODEL_ID}.csv"
        self.events_path = REPORTS / f"events-{MODEL_ID}.csv"
        self.frame_queue: queue.Queue[bytes] = queue.Queue(maxsize=1000)
        self.input_queue: queue.Queue[str] = queue.Queue()
        self.stop = threading.Event()
        self.listening = True
        self.capture: subprocess.Popen | None = None
        self.trial: Trial | None = None
        self.next_trial_id = 1
        self.armed_until = 0.0
        self.noise_floor = 20.0
        self.pre_roll: deque[bytes] = deque(maxlen=40)
        self.speech: list[bytes] = []
        self.speech_frames = 0
        self.quiet_frames = 0
        self.hot_frames = 0
        self.ui_lock = threading.Lock()
        self.ui = {"status": "Starting", "model": MODEL_NAME, "listening": True, "raw": "", "decision": "", "command": "", "latency_ms": None,
                   "trial": "", "mic_level": 0, "trigger_level": 0, "audio_sequence": 0}
        self.messages: deque[str] = deque(maxlen=30)
        self.web: ThreadingHTTPServer | None = None
        self.save_next_audio = False
        self.level_frames = 0

    def print(self, message: str) -> None:
        print(message, flush=True)
        with self.ui_lock:
            self.messages.append(message)
            if message.startswith("[status] "):
                self.ui["status"] = message[len("[status] "):]

    def set_listening(self, enabled: bool) -> None:
        self.listening = enabled
        with self.ui_lock:
            self.ui["listening"] = enabled
        self.print("[status] Waiting for 'Hey Arm'..." if enabled else "[status] Paused. No audio sent to Cartesia.")

    def start_web(self, port: int) -> None:
        app = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args) -> None:
                pass

            def respond(self, status: int, body: bytes, content_type: str) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def trusted_request(self) -> bool:
                host = self.headers.get("Host", "")
                if host not in {f"127.0.0.1:{port}", f"localhost:{port}"}:
                    self.respond(403, b"Invalid host", "text/plain")
                    return False
                origin = self.headers.get("Origin")
                if origin and origin != f"http://{host}":
                    self.respond(403, b"Invalid origin", "text/plain")
                    return False
                return True

            def do_GET(self) -> None:
                if not self.trusted_request():
                    return
                if self.path == "/":
                    self.respond(200, app.dashboard_file.read_bytes(), "text/html; charset=utf-8")
                elif self.path == "/state":
                    with app.ui_lock:
                        state = dict(app.ui)
                        state["messages"] = list(app.messages)
                    state.update(app.extra_state())
                    path = REPORTS / "latest.md"
                    state["report"] = path.read_text(encoding="utf-8") if path.exists() else "No trials yet."
                    self.respond(200, json.dumps(state).encode(), "application/json")
                elif self.path == "/last-utterance.wav":
                    path = REPORTS / "last-utterance.wav"
                    if path.exists():
                        self.respond(200, path.read_bytes(), "audio/wav")
                    else:
                        self.respond(404, b"No saved audio", "text/plain")
                else:
                    self.respond(404, b"Not found", "text/plain")

            def do_POST(self) -> None:
                if not self.trusted_request():
                    return
                if self.path == "/arm":
                    try:
                        if self.headers.get("Content-Type", "").split(";", 1)[0] != "application/json":
                            raise ValueError("JSON required")
                        length = min(int(self.headers.get("Content-Length", "0")), 1000)
                        enabled = json.loads(self.rfile.read(length))["enabled"]
                        if not isinstance(enabled, bool):
                            raise ValueError("enabled must be boolean")
                        app.set_arm_enabled(enabled)
                    except (ValueError, KeyError, TypeError, NotImplementedError):
                        self.respond(400, b"Arm control unavailable", "text/plain")
                        return
                    self.respond(200, b"OK", "text/plain")
                    return
                if self.path == "/listen":
                    length = min(int(self.headers.get("Content-Length", "0")), 1000)
                    try:
                        enabled = json.loads(self.rfile.read(length))["enabled"]
                        if not isinstance(enabled, bool):
                            raise ValueError("enabled must be boolean")
                    except (ValueError, KeyError, TypeError):
                        self.respond(400, b"Invalid request", "text/plain")
                        return
                    app.set_listening(enabled)
                    self.respond(200, b"OK", "text/plain")
                    return
                if self.path == "/diagnostic":
                    if not app.listening:
                        app.set_listening(True)
                    app.save_next_audio = True
                    app.print("[diagnostic] The next utterance will be saved locally for replay.")
                    self.respond(200, b"OK", "text/plain")
                    return
                if self.path != "/trial":
                    self.respond(404, b"Not found", "text/plain")
                    return
                length = min(int(self.headers.get("Content-Length", "0")), 1000)
                try:
                    data = json.loads(self.rfile.read(length))
                    expected = data["expected"]
                except (ValueError, KeyError, TypeError):
                    self.respond(400, b"Invalid request", "text/plain")
                    return
                if expected not in {"pick_up_can", "open_gripper", "cancel", "stop", "none"}:
                    self.respond(400, b"Invalid trial", "text/plain")
                    return
                if not app.listening:
                    app.set_listening(True)
                app.input_queue.put(f"test {expected}")
                self.respond(200, b"OK", "text/plain")

        self.web = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        threading.Thread(target=self.web.serve_forever, daemon=True).start()
        self.print(f"[dashboard] Open http://127.0.0.1:{port}")

    def start_capture(self) -> None:
        command = ["ffmpeg", "-nostdin", "-loglevel", "error", "-f", self.backend]
        if self.backend == "alsa":
            command.extend(["-channels", "1", "-sample_rate", "44100"])
        command.extend([
            "-i", f":{self.device}" if self.backend == "avfoundation" else self.device,
            "-ac", "1", "-ar", str(SAMPLE_RATE), "-f", "s16le", "pipe:1",
        ])
        self.capture = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        threading.Thread(target=self.read_audio, daemon=True).start()
        threading.Thread(target=self.read_commands, daemon=True).start()

    def read_audio(self) -> None:
        assert self.capture and self.capture.stdout
        while not self.stop.is_set():
            frame = self.capture.stdout.read(FRAME_BYTES)
            if len(frame) != FRAME_BYTES:
                self.stop.set()
                break
            try:
                self.frame_queue.put_nowait(frame)
            except queue.Full:
                try:
                    self.frame_queue.get_nowait()
                    self.frame_queue.put_nowait(frame)
                except queue.Empty:
                    pass

    def read_commands(self) -> None:
        for line in sys.stdin:
            self.input_queue.put(line.strip())
            if line.strip().lower() in {"quit", "exit"}:
                break

    def handle_console(self) -> None:
        while True:
            try:
                line = self.input_queue.get_nowait()
            except queue.Empty:
                return
            parts = line.lower().split()
            if not parts:
                continue
            if parts[0] in {"quit", "exit"}:
                self.stop.set()
            elif parts[0] == "report":
                self.print(f"[report] {report()}")
            elif parts[0] == "test" and len(parts) == 2 and parts[1] in {"pick_up_can", "open_gripper", "cancel", "stop", "none"}:
                if self.trial:
                    self.finish_trial("replaced")
                self.trial = Trial(self.next_trial_id, parts[1], time.monotonic() + TRIAL_SECONDS)
                self.next_trial_id += 1
                self.armed_until = 0.0
                with self.ui_lock:
                    self.ui.update(trial=f"Trial {self.trial.id}: {parts[1]} ({TRIAL_SECONDS} seconds)", raw="", decision="", command="", latency_ms=None)
                self.print(f"[test {self.trial.id}] Expected: {parts[1]}. Speak now; trial lasts {TRIAL_SECONDS} seconds.")
            else:
                self.print("[help] Type: test open_gripper | test pick_up_can | test cancel | test stop | test none | report | quit")

    def finish_trial(self, reason: str = "timeout") -> None:
        trial = self.trial
        if not trial:
            return
        if reason == "replaced":
            outcome = "incomplete"
        elif trial.expected == "none":
            outcome = "unwanted_activation" if trial.observed else ("false_wake" if trial.wake_seen else "correct_rejection")
        elif trial.observed == trial.expected:
            outcome = "correct"
        elif trial.observed:
            outcome = "wrong_command"
        else:
            outcome = "missed_command" if trial.wake_seen else "missed_trigger"
        write_row(self.trials_path, TRIAL_FIELDS, {
            "time": datetime.now().astimezone().isoformat(timespec="seconds"),
            "trial_id": trial.id, "expected": trial.expected, "observed": trial.observed or "-",
            "wake_detected": trial.wake_seen, "outcome": outcome,
            "latency_ms": trial.latency_ms if trial.latency_ms is not None else "",
        })
        self.print(f"[test {trial.id}] {outcome}; observed={trial.observed or '-'}")
        self.print(f"[report] {report()}")
        self.trial = None
        self.armed_until = 0.0
        with self.ui_lock:
            self.ui["trial"] = ""

    @staticmethod
    def rms(frame: bytes) -> float:
        samples = array("h")
        samples.frombytes(frame)
        return math.sqrt(sum(value * value for value in samples) / len(samples))

    def transcribe(self, pcm: bytes) -> tuple[str, int]:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            path = Path(tmp.name)
        try:
            with wave.open(str(path), "wb") as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(SAMPLE_RATE)
                wav.writeframes(pcm)
            start = time.monotonic()
            with path.open("rb") as audio_file:
                result = self.client.stt.transcribe(file=audio_file, model="ink-whisper", language="en")
            latency_ms = round((time.monotonic() - start) * 1000)
            return " ".join(result.text.split()).strip(), latency_ms
        except Exception as error:
            self.print(f"[error] Cartesia transcription failed ({type(error).__name__}).")
            return "", 0
        finally:
            path.unlink(missing_ok=True)

    def process_utterance(self, pcm: bytes) -> None:
        if not self.listening:
            return
        decision_started = time.monotonic()
        if self.save_next_audio:
            path = REPORTS / "last-utterance.wav"
            with wave.open(str(path), "wb") as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(SAMPLE_RATE)
                wav.writeframes(pcm)
            self.save_next_audio = False
            with self.ui_lock:
                self.ui["audio_sequence"] += 1
            self.print("[diagnostic] Saved the last utterance locally. Use Replay in the dashboard.")
        self.print("[status] Sending utterance to Cartesia...")
        raw, latency_ms = self.transcribe(pcm)
        wake, remainder = split_wake(raw)
        armed = time.monotonic() < self.armed_until
        decision = "rejected"
        command = ""
        if wake:
            self.armed_until = time.monotonic() + WAKE_SECONDS
            if self.trial:
                self.trial.wake_seen = True
            decision = "wake_detected"
            match = "Approximate Hey Arm match" if normalize(raw).startswith("hey um") else "Hey Arm detected"
            self.print(f"[wake] {match}. Next command within {WAKE_SECONDS}s.")
        if wake or armed:
            candidate = remainder if wake else normalize(raw)
            if candidate:
                command = self.choose_command(candidate) or ""
                decision = "accepted" if command else "rejected"
                self.armed_until = 0.0
        action = self.execute_command(command) if command else ""
        total_latency_ms = round((time.monotonic() - decision_started) * 1000)
        self.print(f"[heard] {raw or '(nothing)'}")
        self.print(f"[decision] {decision}" + (f": {command}" if command else "") + f" | response {total_latency_ms} ms" + (f" | action: {action}" if action else ""))
        with self.ui_lock:
            self.ui.update(raw=raw or "(nothing)", decision=decision, command=command,
                           latency_ms=latency_ms, total_latency_ms=total_latency_ms, action=action)
        write_row(self.events_path, EVENT_FIELDS, {
            "time": datetime.now().astimezone().isoformat(timespec="seconds"),
            "trial_id": self.trial.id if self.trial else "", "raw_text": raw,
            "wake_detected": wake, "decision": decision, "command": command,
            "latency_ms": total_latency_ms,
        })
        if self.trial and command:
            self.trial.observed = command
            self.trial.latency_ms = total_latency_ms
            if self.trial.expected != "none":
                self.finish_trial()
        if not self.listening:
            self.print("[status] Paused. No audio sent to Cartesia.")
        elif self.armed_until:
            self.print(f"[status] Wake heard. Say '{self.example_phrase}' now...")
        else:
            self.print("[status] Waiting for 'Hey Arm'...")

    def choose_command(self, text: str) -> str | None:
        return classify(text)

    def execute_command(self, command: str) -> str:
        return "Test only; no arm connected"

    def extra_state(self) -> dict:
        return {}

    def set_arm_enabled(self, enabled: bool) -> None:
        raise NotImplementedError

    def run(self, port: int) -> None:
        REPORTS.mkdir(exist_ok=True)
        report()
        self.start_web(port)
        self.start_capture()
        self.print(f"[status] Starting USB microphone ({self.backend} device {self.device}); speech is sent to Cartesia")
        self.print(f"[status] Waiting for 'Hey Arm'... Speak 'Hey Arm, {self.example_phrase}'.")
        self.print("[help] For scored trials type: test open_gripper | test pick_up_can | test cancel | test stop | test none")
        self.print("[help] Type report for counts, quit to stop.")
        try:
            while not self.stop.is_set():
                self.handle_console()
                if self.trial and time.monotonic() >= self.trial.deadline:
                    self.finish_trial()
                if self.armed_until and time.monotonic() >= self.armed_until:
                    self.armed_until = 0.0
                    self.print("[status] Command window expired. Waiting for 'Hey Arm'...")
                try:
                    frame = self.frame_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                level = self.rms(frame)
                threshold = max(100.0, self.noise_floor * 3.0)
                self.level_frames += 1
                if self.level_frames >= 10:
                    with self.ui_lock:
                        self.ui["mic_level"] = round(level)
                        self.ui["trigger_level"] = round(threshold)
                    self.level_frames = 0
                if not self.listening:
                    self.speech = []
                    self.pre_roll.clear()
                    self.hot_frames = 0
                    self.armed_until = 0.0
                    continue
                hot = level >= threshold
                if not self.speech:
                    if not hot:
                        self.noise_floor = 0.995 * self.noise_floor + 0.005 * level
                    self.hot_frames = self.hot_frames + 1 if hot else 0
                    self.pre_roll.append(frame)
                    if self.hot_frames >= 2:
                        self.speech = list(self.pre_roll)
                        self.speech_frames = 2
                        self.quiet_frames = 0
                        self.print("[status] Speech detected...")
                else:
                    self.speech.append(frame)
                    self.speech_frames += 1
                    self.quiet_frames = 0 if hot else self.quiet_frames + 1
                    if self.quiet_frames >= 45 or self.speech_frames >= 400:
                        pcm = b"".join(self.speech)
                        self.speech = []
                        self.pre_roll.clear()
                        self.hot_frames = 0
                        if self.speech_frames >= 15:
                            self.process_utterance(pcm)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()
            if self.web:
                self.web.shutdown()
                self.web.server_close()
            if self.capture:
                self.capture.terminate()
                try:
                    self.capture.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.capture.kill()
                if self.capture.stderr:
                    error = self.capture.stderr.read().decode(errors="replace").strip()
                    if error and self.capture.returncode not in {0, -15}:
                        self.print(f"[audio] {error[-600:]}")
            self.print(f"[report] {report()}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="1", help="FFmpeg AVFoundation audio index; USB mic was index 1 during setup")
    parser.add_argument("--port", type=int, default=8765, help="local dashboard port")
    args = parser.parse_args()
    for executable in ("ffmpeg",):
        if not any((Path(folder) / executable).is_file() for folder in os.environ.get("PATH", "").split(os.pathsep)):
            parser.error(f"{executable} is missing from PATH")
    api_key = os.environ.get("CARTESIA_API_KEY", "").strip()
    if not api_key and sys.platform == "darwin":
        result = subprocess.run(
            ["security", "find-generic-password", "-a", "calhacks-voice", "-s", "calhacks-cartesia", "-w"],
            capture_output=True, text=True,
        )
        if result.returncode == 0:
            api_key = result.stdout.strip()
    if not api_key:
        parser.error("Cartesia API key missing. Set CARTESIA_API_KEY or add the Mac Keychain item.")
    VoiceTest(args.device, api_key).run(args.port)


if __name__ == "__main__":
    main()
