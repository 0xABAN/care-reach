"""Local, torque-preserving SO-101 follower control panel.

Run on the robot host from the repository root:

    python -m care_reach.dashboard.server --port <URT-2 serial path>

It binds 127.0.0.1 by default. The Pi service passes --http-host 0.0.0.0 so the
panel is reachable at http://pi.local:8765 on the local network. The panel has
no login, so network access grants control of the arm.
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from care_reach.dashboard.panel import POSES, PanelController
from care_reach.hardware.bus import find_port
from care_reach.settings import SPEED_SCALES


STATIC_DIR = Path(__file__).resolve().parent / "static"

# Host header names accepted besides the address that received the connection.
ACCEPTED_HOSTNAMES = ("localhost", "127.0.0.1", "pi", "pi.local")


def motor_id(value):
    if type(value) is not int or not 1 <= value <= 6:
        raise ValueError("motor must be an integer from 1 to 6")
    return value


def handler_for(arm, port):
    class Handler(BaseHTTPRequestHandler):
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
            """Reject DNS-rebinding and cross-site requests before any action."""
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
            if path == "/":
                body = (STATIC_DIR / "index.html").read_bytes()
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
                if path == "/api/hold/start":
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
                    if pose not in POSES:
                        raise ValueError("pose must be 'home', 'fold', or 'dance'")
                    result = arm.pose_start(pose)
                elif path == "/api/pose/stop":
                    result = arm.pose_stop()
                elif path == "/api/scale":
                    motor = motor_id(data.get("motor"))
                    scale = data.get("scale")
                    if type(scale) is not int or scale not in SPEED_SCALES:
                        raise ValueError("scale must be 1, 2, 4, or 8")
                    result = arm.scale(motor, scale)
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
    arm = PanelController(args.port or find_port())
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
