#!/usr/bin/env python3
"""Read the USB Arduino ranger and expose live distance on Pi localhost."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import statistics
import threading
import time
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import serial
from serial.tools import list_ports


BAUD = 115200
STALE_AFTER_S = 1.5
RANGE_MIN_CM = 2.0
RANGE_MAX_CM = 400.0
KNOWN_ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86, 0x10C4, 0x0403}
URT_VID_PID = (0x1A86, 0x55D3)
NUMBER = re.compile(rb"(?:\d+(?:\.\d*)?|\.\d+)\Z")


def find_arduino_port() -> str | None:
    """Prefer Arduino-style USB serial devices and never claim the URT-2."""
    ports = list(list_ports.comports())
    configured = os.environ.get("CARE_REACH_ULTRASONIC_PORT", "").strip()
    if configured:
        if not configured.startswith("/dev/"):
            raise ValueError("CARE_REACH_ULTRASONIC_PORT must be a /dev path")
        if any((port.vid, port.pid) == URT_VID_PID
               and os.path.realpath(port.device) == os.path.realpath(configured)
               for port in ports):
            raise ValueError("Refusing the URT-2 controller as ultrasonic port")
        return configured
    candidates = []
    for port in ports:
        if (port.vid, port.pid) == URT_VID_PID:
            continue
        if port.vid in KNOWN_ARDUINO_VIDS:
            rank = 0 if "arduino" in (port.description or "").lower() else 1
            candidates.append((rank, port.device))
    candidates.sort()
    return candidates[0][1] if candidates else None


def parse_measurement(raw: bytes) -> float | None:
    """Accept only one finite centimeter number in the sketch's range."""
    raw = raw.strip()
    if raw == b"NA":
        return None
    if not NUMBER.fullmatch(raw):
        raise ValueError("Invalid ranger frame")
    value = float(raw)
    if not math.isfinite(value) or not RANGE_MIN_CM <= value <= RANGE_MAX_CM:
        raise ValueError("Ranger reading outside supported range")
    return value


class Ranger:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.samples: deque[float] = deque(maxlen=3)
        self.last_valid_monotonic: float | None = None
        self.updated_at: str | None = None
        self.connected = False
        self.error = False

    def snapshot(self) -> dict:
        with self.lock:
            age_ms = (round((time.monotonic() - self.last_valid_monotonic) * 1000)
                      if self.last_valid_monotonic is not None else None)
            if self.error:
                status = "error"
            elif not self.connected:
                status = "waiting"
            elif age_ms is None or age_ms > STALE_AFTER_S * 1000:
                status = "stale"
            else:
                status = "ok"
            return {
                "status": status,
                "distance_cm": round(statistics.median(self.samples), 1) if status == "ok" else None,
                "sample_age_ms": age_ms,
                "updated_at": self.updated_at,
                "message": {
                    "ok": "Live range reading",
                    "waiting": "Waiting for Arduino USB connection",
                    "stale": "No recent range reading",
                    "error": "Arduino serial connection failed; retrying",
                }[status],
            }

    def accept(self, raw: bytes) -> None:
        try:
            value = parse_measurement(raw)
        except ValueError:
            return
        if value is None:
            return
        with self.lock:
            self.samples.append(value)
            self.last_valid_monotonic = time.monotonic()
            self.updated_at = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            self.error = False

    def run(self) -> None:
        while not self.stop.is_set():
            try:
                path = find_arduino_port()
            except ValueError:
                path = None
                with self.lock:
                    self.error = True
            if path is None:
                with self.lock:
                    self.connected = False
                    self.samples.clear()
                    self.last_valid_monotonic = None
                    self.updated_at = None
                self.stop.wait(1)
                continue
            try:
                with serial.Serial(path, BAUD, timeout=1) as conn:
                    with self.lock:
                        self.connected = True
                        self.error = False
                        self.samples.clear()
                        self.last_valid_monotonic = None
                        self.updated_at = None
                    # Opening USB serial resets many Arduino boards.
                    self.stop.wait(2)
                    while not self.stop.is_set():
                        raw = conn.readline(32)
                        if raw:
                            self.accept(raw)
            except (OSError, serial.SerialException):
                with self.lock:
                    self.connected = False
                    self.error = True
                    self.samples.clear()
                    self.last_valid_monotonic = None
                    self.updated_at = None
                self.stop.wait(1)


def handler_for(ranger: Ranger, port: int):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            host = self.headers.get("Host", "")
            origin = self.headers.get("Origin")
            if host not in {f"127.0.0.1:{port}", f"localhost:{port}"} or (
                origin and origin != f"http://{host}"
            ):
                self.send_error(403)
                return
            if urlsplit(self.path).path != "/state":
                self.send_error(404)
                return
            body = json.dumps(ranger.snapshot()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt: str, *args) -> None:
            pass

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8767)
    args = parser.parse_args()
    ranger = Ranger()
    worker = threading.Thread(target=ranger.run, name="ultrasonic-serial", daemon=True)
    worker.start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(ranger, args.port))

    def shutdown(_signum, _frame):
        ranger.stop.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        ranger.stop.set()
        server.server_close()
        worker.join(timeout=2)


if __name__ == "__main__":
    main()
