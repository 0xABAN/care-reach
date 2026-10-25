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


class LevelSegmenter:
    """RMS endpoint detector with a short ambient calibration and hysteresis.

    Keeping the ambient estimate current while idle prevents a steady loud mic
    floor from looking like continuous speech. During speech it stays fixed so
    a long phrase cannot teach the detector that the speaker is background.
    """

    def __init__(self, *, silence_frames: int = 30, calibration_frames: int = 20,
                 ambient_frames: int = 150, onset_ratio: float = 1.8,
                 release_ratio: float = 1.35, min_level: float = 100.0,
                 max_frames: int = 400):
        self.silence_frames = silence_frames
        self.calibration_frames = calibration_frames
        self.onset_ratio = onset_ratio
        self.release_ratio = release_ratio
        self.min_level = min_level
        self.max_frames = max_frames
        self.ambient: deque[float] = deque(maxlen=ambient_frames)
        self.active = False
        self.hot_frames = 0
        self.quiet_frames = 0
        self.speech_frames = 0
        self.last_endpoint_frames = 0
        self.noise_floor = 20.0

    @property
    def threshold(self) -> float:
        return max(self.min_level, self.noise_floor * self.onset_ratio)

    def reset(self) -> None:
        self.active = False
        self.hot_frames = self.quiet_frames = self.speech_frames = 0

    def feed(self, level: float) -> str | None:
        if not self.active:
            self.ambient.append(level)
            self.noise_floor = statistics.median(self.ambient)
            if len(self.ambient) < self.calibration_frames:
                return None
            self.hot_frames = self.hot_frames + 1 if level >= self.threshold else 0
            if self.hot_frames >= 2:
                self.active = True
                self.speech_frames = self.hot_frames
                self.quiet_frames = 0
                return "start"
            return None
        self.speech_frames += 1
        release_level = max(self.min_level, self.noise_floor * self.release_ratio)
        self.quiet_frames = self.quiet_frames + 1 if level < release_level else 0
        if self.quiet_frames >= self.silence_frames or self.speech_frames >= self.max_frames:
            self.last_endpoint_frames = self.quiet_frames
            self.reset()
            return "end"
        return None


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
    for phrase in ("hey arm", "hey um", "hey aam", "hey ahm", "hey im", "hey i m"):
        if words == phrase:
            return True, ""
        if words.startswith(phrase + " "):
            return True, words[len(phrase) + 1:]
    return False, words


def raw_command_after_wake(text: str) -> str:
    """Remove a spoken wake prefix without erasing quotes or negation punctuation."""
    return re.sub(r"^\s*hey[\s,.!?]+(?:arm|um|aam|ahm|im|i\s+m)\b[\s,.!?]*",
                  "", text, count=1, flags=re.IGNORECASE).strip()


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
        from cartesia import Cartesia

        self.device = device
        self.api_key = api_key
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
        self.segmenter = LevelSegmenter()
        self.pre_roll: deque[bytes] = deque(maxlen=40)
        self.speech: list[bytes] = []
        self.command_latency_ms: int | None = None
        self.ui_lock = threading.Lock()
        self.ui = {"status": "Starting", "model": MODEL_NAME, "listening": True, "raw": "", "decision": "", "command": "", "latency_ms": None,
                   "trial": "", "mic_level": 0, "trigger_level": 0, "audio_sequence": 0,
                   "last_command_raw": "", "last_command": "", "action": "", "last_command_time": ""}
        self.messages: deque[str] = deque(maxlen=30)
        self.web: ThreadingHTTPServer | None = None
        self.save_next_audio = False
        self.level_frames = 0
        self.last_endpoint_ms = 0
        self.last_utterance_ms = 0
        self.pre_roll_frames = 0
        self.dashboard_api_ms = 0
        self.stt_mode = "batch"
        self.streaming_stt = None
        self.streaming_active = False
        self.stream_close_pending = False
        self.stream_lock = threading.RLock()
        self.stream_generation = 0
        self.transcript_queue: queue.Queue[tuple[str, dict]] = queue.Queue()
        self.diagnostic_audio: deque[bytes] = deque(maxlen=400)
        self.ui["stt_mode"] = "batch"

    def print(self, message: str) -> None:
        print(message, flush=True)
        with self.ui_lock:
            self.messages.append(message)
            if message.startswith("[status] "):
                self.ui["status"] = message[len("[status] "):]

    def set_listening(self, enabled: bool) -> None:
        old_stream = None
        with self.stream_lock:
            self.listening = enabled
            self.armed_until = 0.0
            if not enabled:
                self.stream_generation += 1
                old_stream = self.streaming_stt
                self.streaming_stt = None
                self.streaming_active = False
                self.stream_close_pending = False
                self.speech = []
                self.pre_roll.clear()
                self.segmenter.reset()
                while not self.frame_queue.empty():
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        break
                while not self.transcript_queue.empty():
                    try:
                        self.transcript_queue.get_nowait()
                    except queue.Empty:
                        break
        with self.ui_lock:
            self.ui["listening"] = enabled
        if old_stream:
            old_stream.close()
        if enabled and self.stt_mode == "stream" and not self.streaming_active:
            try:
                self._start_stream()
            except Exception as error:
                with self.stream_lock:
                    failed = self.streaming_stt
                    self.streaming_stt = None
                    self.streaming_active = False
                if failed:
                    failed.close()
                self.print(f"[error] Streaming STT restart failed ({type(error).__name__}).")
        self.print("[status] Waiting for 'Hey Arm'..." if enabled else "[status] Paused. No audio sent to Cartesia.")

    def _start_stream(self) -> None:
        from streaming_stt import StreamingSTT
        with self.stream_lock:
            if not self.listening:
                return
            self.stream_generation += 1
            generation = self.stream_generation
            stream = StreamingSTT(
                self.api_key,
                lambda raw, timing: self._stream_turn(generation, raw, timing),
                lambda error: self._stream_error(generation, error),
                sample_rate=SAMPLE_RATE, turn_end_timeout_ms=640)
            self.streaming_stt = stream
            stream.start()
            self.streaming_active = True
        with self.ui_lock:
            self.ui["stt_mode"] = "stream"
        self.print("[status] Streaming microphone audio to Cartesia")

    def _stream_turn(self, generation: int, raw: str, timing: dict) -> None:
        with self.stream_lock:
            if generation != self.stream_generation:
                return
            self.on_streaming_turn(raw, timing)

    def _stream_error(self, generation: int, error: Exception) -> None:
        with self.stream_lock:
            if generation != self.stream_generation:
                return
            self.on_streaming_error(error)

    def parse_wake(self, text: str) -> tuple[bool, str]:
        return split_wake(text)

    def command_after_wake(self, text: str) -> str:
        return raw_command_after_wake(text)

    def allow_followup_without_wake(self) -> bool:
        return True

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
                if self.path == "/text" and hasattr(app, "submit_text"):
                    if self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
                        self.respond(400, b'{"error":"Expected JSON"}', "application/json")
                        return
                    try:
                        length = int(self.headers.get("Content-Length", "0"))
                        if not 0 < length <= 1024:
                            raise ValueError("Invalid request size")
                        data = json.loads(self.rfile.read(length))
                        if not isinstance(data, dict) or set(data) != {"text"}:
                            raise ValueError("Expected text")
                        result = app.submit_text(data["text"])
                    except (ValueError, TypeError, UnicodeDecodeError):
                        self.respond(400, b'{"error":"Invalid text request"}', "application/json")
                        return
                    except RuntimeError:
                        self.respond(503, b'{"error":"Jev unavailable"}', "application/json")
                        return
                    self.respond(200, json.dumps(result).encode(), "application/json")
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
                if expected not in {"pick_up_can", "open_gripper", "close_gripper", "cancel", "stop", "none"}:
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
        if self.stt_mode == "stream":
            try:
                if self.listening:
                    self._start_stream()
            except Exception as error:
                if self.streaming_stt:
                    try:
                        self.streaming_stt.close()
                    except Exception:
                        pass
                self.print(f"[error] Streaming STT unavailable ({type(error).__name__}); using batch mode.")
                self.streaming_active = False
                self.stt_mode = "batch"
                with self.ui_lock:
                    self.ui["stt_mode"] = "batch"
                    self.ui["model"] = self.ui["model"].replace("Ink-2 (streaming)", "Ink-Whisper (batch)")
                self.print("[status] Using batch microphone detection.")
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
            if not self.listening:
                continue
            if self.streaming_active:
                self.handle_streaming_frame(frame)
                continue
            if self.stream_close_pending:
                continue  # Main loop closes the failed stream before batch capture.
            try:
                self.frame_queue.put_nowait(frame)
            except queue.Full:
                try:
                    self.frame_queue.get_nowait()
                    self.frame_queue.put_nowait(frame)
                except queue.Empty:
                    pass

    def handle_streaming_frame(self, frame: bytes) -> None:
        self.level_frames += 1
        if self.level_frames >= 10:
            with self.ui_lock:
                self.ui["mic_level"] = round(self.rms(frame))
                self.ui["trigger_level"] = 0  # Server determines streaming turn boundaries.
            self.level_frames = 0
        with self.stream_lock:
            if not self.listening or not self.streaming_active or self.streaming_stt is None:
                return
            self.diagnostic_audio.append(frame)
            try:
                self.streaming_stt.send(frame)
            except Exception as error:
                self.on_streaming_error(error)

    def on_streaming_turn(self, raw: str, timing: dict) -> None:
        if self.streaming_active and self.listening:
            self.transcript_queue.put((raw, timing))

    def on_streaming_error(self, error: Exception) -> None:
        if self.streaming_active:
            self.streaming_active = False
            self.stream_close_pending = True
            self.stt_mode = "batch"
            with self.ui_lock:
                self.ui["stt_mode"] = "batch"
                self.ui["model"] = self.ui["model"].replace("Ink-2 (streaming)", "Ink-Whisper (batch)")
            while not self.transcript_queue.empty():
                try:
                    self.transcript_queue.get_nowait()
                except queue.Empty:
                    break
            self.print(f"[error] Streaming STT failed ({type(error).__name__}); using batch mode.")
            self.print("[status] Using batch microphone detection.")

    def handle_streaming_turns(self) -> None:
        while True:
            try:
                raw, timing = self.transcript_queue.get_nowait()
            except queue.Empty:
                return
            if not self.listening or self.stt_mode != "stream":
                continue
            if self.save_next_audio:
                path = REPORTS / "last-utterance.wav"
                with wave.open(str(path), "wb") as wav:
                    wav.setnchannels(1)
                    wav.setsampwidth(2)
                    wav.setframerate(SAMPLE_RATE)
                    wav.writeframes(b"".join(self.diagnostic_audio))
                self.save_next_audio = False
                with self.ui_lock:
                    self.ui["audio_sequence"] += 1
                self.print("[diagnostic] Saved recent microphone audio locally for replay.")
            self.diagnostic_audio.clear()
            self.process_transcript(raw, int(timing.get("stt_ms", 0)), 0, 0, 0,
                                    stream_timing=timing)

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
            elif parts[0] == "test" and len(parts) == 2 and parts[1] in {"pick_up_can", "open_gripper", "close_gripper", "cancel", "stop", "none"}:
                if self.trial:
                    self.finish_trial("replaced")
                self.trial = Trial(self.next_trial_id, parts[1], time.monotonic() + TRIAL_SECONDS)
                self.next_trial_id += 1
                self.armed_until = 0.0
                with self.ui_lock:
                    self.ui.update(trial=f"Trial {self.trial.id}: {parts[1]} ({TRIAL_SECONDS} seconds)", raw="", decision="", command="", latency_ms=None)
                self.print(f"[test {self.trial.id}] Expected: {parts[1]}. Speak now; trial lasts {TRIAL_SECONDS} seconds.")
            else:
                self.print("[help] Type: test open_gripper | test close_gripper | test pick_up_can | test cancel | test stop | test none | report | quit")

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
        self.command_latency_ms = None
        decision_started = time.monotonic()
        capture_ms = round(len(pcm) / (SAMPLE_RATE * 2) * 1000)
        utterance_ms = self.last_utterance_ms
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
        self.process_transcript(raw, latency_ms, capture_ms, utterance_ms, self.last_endpoint_ms,
                                decision_started)

    def process_transcript(self, raw: str, latency_ms: int, capture_ms: int,
                           utterance_ms: int, endpoint_ms: int,
                           decision_started: float | None = None,
                           stream_timing: dict | None = None) -> None:
        if not self.listening:
            return
        if decision_started is None:
            decision_started = time.monotonic()
        wake, _ = self.parse_wake(raw)
        armed = self.allow_followup_without_wake() and time.monotonic() < self.armed_until
        decision = "rejected"
        command = ""
        if wake:
            self.armed_until = (time.monotonic() + WAKE_SECONDS
                                if self.allow_followup_without_wake() else 0.0)
            if self.trial:
                self.trial.wake_seen = True
            decision = "wake_detected"
            match = ("Hey Arm detected" if normalize(raw).startswith("hey arm")
                     else "Approximate Hey Arm match")
            self.print(f"[wake] {match}.")
        classifier_ms = 0
        if wake or armed:
            # Preserve the original punctuation for the Pi's grounding guard.
            candidate = self.command_after_wake(raw) if wake else raw.strip()
            if candidate:
                classifier_started = time.monotonic()
                command = self.choose_command(candidate) or ""
                classifier_ms = round((time.monotonic() - classifier_started) * 1000)
                decision = "accepted" if command else "rejected"
                self.armed_until = 0.0
        self.dashboard_api_ms = 0
        action = self.execute_command(command) if command else ""
        total_latency_ms = round((time.monotonic() - decision_started) * 1000)
        self.print(f"[heard] {raw or '(nothing)'}")
        self.print(f"[decision] {decision}" + (f": {command}" if command else "") + f" | response {total_latency_ms} ms" + (f" | action: {action}" if action else ""))
        if stream_timing is None:
            timing_values = {"capture_ms": capture_ms, "utterance_ms": utterance_ms,
                             "endpoint_ms": endpoint_ms, "stt_ms": latency_ms}
            timing_line = (f"capture={capture_ms}ms utterance≈{utterance_ms}ms "
                           f"endpoint={endpoint_ms}ms stt={latency_ms}ms")
        else:
            timing_values = {"turn_ms": stream_timing.get("turn_ms"),
                             "first_partial_ms": stream_timing.get("first_partial_ms"),
                             "finalize_ms": stream_timing.get("finalize_ms") or None}
            finalize_text = (f"{timing_values['finalize_ms']}ms"
                             if timing_values["finalize_ms"] is not None else "n/a")
            timing_line = (f"model_turn={timing_values['turn_ms']}ms "
                           f"first_partial={timing_values['first_partial_ms']}ms "
                           f"eager_to_final={finalize_text}")
        timing_values.update(classifier_ms=classifier_ms, dashboard_api_ms=self.dashboard_api_ms)
        self.print(f"[timing] {timing_line} classifier={classifier_ms}ms "
                   f"dashboard_api={self.dashboard_api_ms}ms")
        with self.ui_lock:
            self.ui.update(raw=raw or "(nothing)", decision=decision, command=command,
                           latency_ms=latency_ms if stream_timing is None else None,
                           total_latency_ms=total_latency_ms,
                           timing=timing_values)
            if command:
                self.ui.update(last_command_raw=raw, last_command=command, action=action,
                               last_command_time=datetime.now().astimezone().isoformat(timespec="seconds"))
        if command:
            write_row(REPORTS / "actions.csv", ["time", "raw_text", "command", "action", "latency_ms"], {
                "time": datetime.now().astimezone().isoformat(timespec="seconds"),
                "raw_text": raw, "command": command, "action": action, "latency_ms": total_latency_ms,
            })
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

    def run(self, port: int) -> None:
        REPORTS.mkdir(exist_ok=True)
        report()
        self.start_web(port)
        self.start_capture()
        if self.listening:
            self.print(f"[status] Starting USB microphone ({self.backend} device {self.device}); speech is sent to Cartesia")
            self.print(f"[status] Waiting for 'Hey Arm'... Speak 'Hey Arm, {self.example_phrase}'.")
        else:
            self.print("[status] Transcription paused. No audio sent to Cartesia; enable listening in the dashboard.")
        self.print("[help] For scored trials type: test open_gripper | test close_gripper | test pick_up_can | test cancel | test stop | test none")
        self.print("[help] Type report for counts, quit to stop.")
        try:
            while not self.stop.is_set():
                with self.stream_lock:
                    failed_stream = self.streaming_stt if self.stream_close_pending else None
                    if self.stream_close_pending:
                        self.streaming_stt = None
                        self.stream_close_pending = False
                if failed_stream:
                    try:
                        failed_stream.close()
                    except Exception as error:
                        self.print(f"[error] Streaming cleanup failed ({type(error).__name__}).")
                self.handle_console()
                self.handle_streaming_turns()
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
                event = self.segmenter.feed(level) if self.listening else None
                threshold = self.segmenter.threshold
                self.level_frames += 1
                if self.level_frames >= 10:
                    with self.ui_lock:
                        self.ui["mic_level"] = round(level)
                        self.ui["trigger_level"] = round(threshold)
                    self.level_frames = 0
                if not self.listening:
                    self.speech = []
                    self.pre_roll.clear()
                    self.segmenter.reset()
                    self.armed_until = 0.0
                    continue
                if not self.speech:
                    self.pre_roll.append(frame)
                    if event == "start":
                        self.speech = list(self.pre_roll)
                        self.pre_roll_frames = len(self.pre_roll)
                        self.print("[status] Speech detected...")
                else:
                    self.speech.append(frame)
                    if event == "end":
                        pcm = b"".join(self.speech)
                        self.last_endpoint_ms = self.segmenter.last_endpoint_frames * FRAME_MS
                        self.last_utterance_ms = max(0, (len(self.speech) - self.pre_roll_frames
                                                         - self.segmenter.last_endpoint_frames) * FRAME_MS)
                        self.speech = []
                        self.pre_roll.clear()
                        if len(pcm) >= 15 * FRAME_BYTES:
                            self.process_utterance(pcm)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()
            if self.streaming_stt:
                self.streaming_stt.close()
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
