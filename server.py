#!/usr/bin/env python3
"""Local, torque-preserving SO-101 follower control panel.

This module is the HTTP/camera adapter and process entry point. Serial bus
ownership and all movement logic live in ``arm_control``; ``Arm`` and its
tuning constants are re-exported here for existing callers and tests.
"""

from __future__ import annotations

import argparse
import json
import math
import logging
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from arm_control import Arm  # noqa: F401  (re-exported for tests and old callers)
from arm_control.arm import (  # noqa: F401  (re-exported for tests and old callers)
    GRIP_HOLD_LEAD,
    HOME_TOLERANCE,
    POSE_TOLERANCE,
    POSE_NAMES,
    SCALES,
    STOP_SETTLE_LEAD,
    find_port,
)


ROOT = Path(__file__).resolve().parent


def motor_id(value):
    if type(value) is not int or not 1 <= value <= 6:
        raise ValueError("motor must be an integer from 1 to 6")
    return value


ACCEPTED_HOSTNAMES = ("localhost", "127.0.0.1", "pi", "pi.local")
VOICE_URL = "http://127.0.0.1:8770"
ULTRASONIC_URL = "http://127.0.0.1:8767/state"


def voice_state():
    """Expose only the voice switch state, not transcripts or room audio."""
    with urlopen(f"{VOICE_URL}/state", timeout=3) as response:
        state = json.load(response)
    return {"enabled": state["listening"], "stt_mode": state.get("stt_mode", "unknown")}


def set_voice_enabled(enabled):
    payload = json.dumps({"enabled": enabled}).encode()
    request = Request(f"{VOICE_URL}/listen", data=payload,
                      headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(request, timeout=3) as response:
        response.read()
    return {"enabled": enabled}


def ultrasonic_state():
    """Read the separate sensor service without opening the arm's serial bus."""
    with urlopen(ULTRASONIC_URL, timeout=1) as response:
        state = json.loads(response.read(4096))
    if not isinstance(state, dict) or state.get("status") not in {"ok", "waiting", "stale", "error"}:
        raise ValueError("Invalid ultrasonic status")
    distance = state.get("distance_cm")
    if distance is not None:
        if (isinstance(distance, bool) or not isinstance(distance, (int, float))
                or not math.isfinite(distance) or not 0 <= distance <= 1000):
            raise ValueError("Invalid ultrasonic distance")
        distance = float(distance)
    age = state.get("sample_age_ms")
    if age is not None and (type(age) is not int or not 0 <= age <= 86_400_000):
        raise ValueError("Invalid ultrasonic sample age")
    updated = state.get("updated_at")
    if updated is not None and (not isinstance(updated, str) or len(updated) > 64):
        raise ValueError("Invalid ultrasonic sample time")
    return {"status": state["status"], "distance_cm": distance,
            "sample_age_ms": age, "updated_at": updated}


def handler_for(arm, port):
    class Handler(BaseHTTPRequestHandler):
        # Keep-alive: every finite route below sends an exact Content-Length,
        # and the unbounded MJPEG streams explicitly close the connection.
        protocol_version = "HTTP/1.1"

        def reply(self, code, data):
            body = json.dumps(data).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _accepted_host(self, raw):
            try:
                parts = urlsplit("//" + raw)
                hostname, host_port = parts.hostname, parts.port
            except ValueError:
                return False
            if hostname is None or host_port != port:
                return False
            hostname = hostname.lower()
            if hostname in ACCEPTED_HOSTNAMES:
                return True
            return hostname == self.connection.getsockname()[0].lower()

        def allowed(self):
            host = self.headers.get("Host")
            if not host or not self._accepted_host(host):
                self.reply(403, {"error": "Invalid host"})
                return False
            origin = self.headers.get("Origin")
            if origin:
                parts = urlsplit(origin)
                if parts.scheme != "http" or parts.netloc != host:
                    self.reply(403, {"error": "Invalid origin"})
                    return False
            return True

        def do_GET(self):
            if not self.allowed():
                return
            path = urlsplit(self.path).path
            if path in ("/camera/stream", "/camera/snapshot", "/camera2/stream", "/camera2/snapshot"):
                upstream_path = "/stream" if path.endswith("stream") else "/snapshot"
                camera_port = 8768 if path.startswith("/camera2/") else 8766
                try:
                    upstream = urlopen(f"http://127.0.0.1:{camera_port}{upstream_path}", timeout=5)
                except (OSError, URLError) as exc:
                    logging.warning("Camera unavailable: %s", exc)
                    self.reply(503, {"error": "Camera unavailable"})
                    return
                with upstream:
                    if upstream_path == "/snapshot":
                        # A snapshot is one finite JPEG: buffer it so the
                        # response always carries an accurate Content-Length,
                        # which HTTP/1.1 keep-alive needs even when the
                        # upstream omits it.
                        body = upstream.read()
                        self.send_response(200)
                        self.send_header("Content-Type", upstream.headers.get("Content-Type", "image/jpeg"))
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Content-Length", str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)
                    else:
                        # The MJPEG stream is unbounded and close-delimited,
                        # so this response cannot be reused by another request.
                        self.send_response(200)
                        self.send_header("Content-Type", upstream.headers.get("Content-Type", "image/jpeg"))
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Connection", "close")
                        self.close_connection = True
                        self.end_headers()
                        try:
                            while chunk := upstream.read1(16 * 1024):
                                self.wfile.write(chunk)
                        except (BrokenPipeError, ConnectionResetError, TimeoutError):
                            pass  # Viewer closed the tab or disconnected.
            elif path == "/":
                body = (ROOT / "static" / "index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/api/state":
                try:
                    self.reply(200, arm.state())
                except Exception as exc:
                    logging.exception("State read failed")
                    self.reply(503, {"error": f"Motor communication error: {exc}"})
            elif path == "/api/transcription":
                try:
                    self.reply(200, voice_state())
                except (OSError, URLError, ValueError, KeyError, TypeError) as exc:
                    logging.warning("Voice service unavailable: %s", type(exc).__name__)
                    self.reply(503, {"error": "Transcription service unavailable"})
            elif path == "/api/ultrasonic":
                try:
                    self.reply(200, ultrasonic_state())
                except (OSError, URLError, ValueError, KeyError, TypeError) as exc:
                    logging.warning("Ultrasonic service unavailable: %s", type(exc).__name__)
                    self.reply(503, {"error": "Ultrasonic service unavailable"})
            elif path == "/api/diagnostics":
                try:
                    self.reply(200, arm.diagnostics())
                except Exception as exc:
                    logging.exception("Diagnostics read failed")
                    self.reply(503, {"error": f"Motor communication error: {exc}"})
            else:
                self.reply(404, {"error": "Not found"})

        def do_POST(self):
            if not self.allowed():
                return
            try:
                if self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
                    raise ValueError("Content-Type must be application/json")
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 1024:
                    raise ValueError("Invalid request size")
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError("JSON body must be an object")
                path = urlsplit(self.path).path
                if path == "/api/transcription":
                    if set(data) != {"enabled"} or type(data["enabled"]) is not bool:
                        raise ValueError("enabled must be boolean")
                    try:
                        result = set_voice_enabled(data["enabled"])
                    except (OSError, URLError, ValueError) as exc:
                        logging.warning("Voice service unavailable: %s", type(exc).__name__)
                        self.reply(503, {"error": "Transcription service unavailable"})
                        return
                elif path == "/api/hold/start":
                    motor = motor_id(data.get("motor"))
                    direction = data.get("direction")
                    if type(direction) is not int or direction not in (-1, 1):
                        raise ValueError("direction must be -1 or 1")
                    result = {"token": arm.start(motor, direction)}
                elif path == "/api/motor6/go":
                    result = {"token": arm.go(data.get("target"))}
                elif path == "/api/hold/heartbeat":
                    token = data.get("token")
                    if not isinstance(token, str):
                        raise ValueError("token is required")
                    arm.heartbeat(token)
                    result = {"ok": True}
                elif path == "/api/hold/stop":
                    token = data.get("token")
                    if token is not None and not isinstance(token, str):
                        raise ValueError("token must be a string")
                    arm.stop(token)
                    result = {"ok": True}
                elif path == "/api/pose/start":
                    pose = data.get("pose")
                    if pose not in POSE_NAMES:
                        raise ValueError(f"pose must be one of {', '.join(POSE_NAMES)}; folded pose is unavailable with the wrist camera")
                    result = arm.pose_start(pose)
                elif path == "/api/teach/add":
                    result = arm.teach_add(data.get("move"))
                elif path == "/api/teach/clear":
                    result = arm.teach_clear(data.get("move"))
                elif path == "/api/pose/stop":
                    result = arm.pose_stop()
                elif path == "/api/scale":
                    motor = motor_id(data.get("motor"))
                    scale = data.get("scale")
                    if type(scale) is not int or scale not in SCALES:
                        raise ValueError("scale must be 1, 2, 4, or 8")
                    result = arm.scale(motor, scale)
                elif path == "/api/release":
                    arm.release()
                    result = {"ok": True}
                elif path == "/api/hold-all":
                    result = arm.hold_all()
                else:
                    self.reply(404, {"error": "Not found"})
                    return
                self.reply(200, result)
            except (ValueError, json.JSONDecodeError) as exc:
                self.reply(400, {"error": str(exc)})
            except Exception as exc:
                logging.exception("Request failed")
                self.reply(503, {"error": f"Motor communication error: {exc}"})

        def log_message(self, fmt, *args):
            logging.info("%s %s", self.address_string(), fmt % args)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="URT-2 serial port; detected automatically by default")
    parser.add_argument("--http-host", default="127.0.0.1",
                        help="HTTP bind address; use 0.0.0.0 on the Pi for LAN access")
    parser.add_argument("--http-port", type=int, default=8765)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    arm = Arm(args.port or find_port())
    try:
        server = ThreadingHTTPServer((args.http_host, args.http_port), handler_for(arm, args.http_port))
    except Exception:
        arm.close()
        raise
    server.daemon_threads = True

    def shutdown(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    if args.http_host == "0.0.0.0":
        target = f"http://pi.local:{args.http_port} (or the Pi's LAN IPv4 address)"
    else:
        target = f"http://{args.http_host}:{args.http_port}"
    print(f"SO-101 control panel: {target}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        server.server_close()
        arm.close()


if __name__ == "__main__":
    main()
