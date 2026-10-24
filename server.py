#!/usr/bin/env python3
"""Local, torque-preserving SO-101 follower control panel."""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from serial.tools import list_ports
from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus

from motion_core import JOINT_NAMES, SafetyBounds
from park import folded_target


ROOT = Path(__file__).resolve().parent
SETTINGS = ROOT / "joint_settings.json"
LABELS = ("Shoulder pan", "Shoulder lift", "Elbow flex", "Wrist flex", "Wrist roll", "Gripper")
SCALES = (1, 2, 4, 8)
BASE_TICKS_PER_SECOND = 25
MAX_COMMAND_SPEED = (180, 180, 180, 180, 200, 180)
MAX_COMMAND_ACCELERATION = (300, 300, 300, 300, 360, 300)
MAX_TARGET_LEAD = 80
RECOVERY_MARGIN = 40  # Allow a small encoder drift past the saved fold, inward only.
STEP_SECONDS = 0.04
LEASE_SECONDS = 0.75
MIN_VOLTAGE_RAW = 48
MAX_LOAD = 900
POSITION_TOLERANCE = 15


def find_port():
    ports = [p.device for p in list_ports.comports() if p.vid == 0x1A86 and p.pid == 0x55D3]
    if len(ports) != 1:
        raise RuntimeError(f"Expected one URT-2 (USB 1A86:55D3), found {len(ports)}; use --port")
    return ports[0]


def load_settings(path: Path = SETTINGS):
    result = {str(i): {"lower": None, "upper": None, "scale": 1} for i in range(1, 7)}
    if path.exists():
        saved = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(saved, dict):
            raise ValueError("joint_settings.json must be an object")
        for key, entry in saved.items():
            if key not in result or not isinstance(entry, dict):
                raise ValueError(f"Invalid settings entry {key}")
            lower, upper, scale = entry.get("lower"), entry.get("upper"), entry.get("scale", 1)
            if any(value is not None and (type(value) is not int or not 0 <= value <= 4095)
                   for value in (lower, upper)):
                raise ValueError(f"Invalid limit for motor {key}")
            if lower is not None and upper is not None and lower >= upper:
                raise ValueError(f"Lower limit must be less than upper limit for motor {key}")
            if type(scale) is not int or scale not in SCALES:
                raise ValueError(f"Invalid scale for motor {key}")
            result[key] = {"lower": lower, "upper": upper, "scale": scale}
    return result


class Arm:
    """One owner of the serial bus. Read-only on connect; every motion is explicit."""

    def __init__(self, port, *, bus_factory=None, settings_path: Path = SETTINGS,
                 calibration_path: Path | None = None, pose_path: Path | None = None):
        self.lock = threading.RLock()
        self.settings_path = Path(settings_path)
        self.calibration_path = calibration_path
        self.settings = load_settings(self.settings_path)
        self.folded = folded_target(pose_path) if pose_path is not None else folded_target()
        self.bounds = SafetyBounds.from_files(self.settings_path, self.calibration_path)
        self.park_bounds = self._fold_envelope(self.bounds)
        self.hold = None
        self.error = None
        self.closed = False
        self.bus = (bus_factory or self._real_bus)(port)
        self.bus.connect(handshake=True)
        try:
            self.firmware_min = self._read("Min_Position_Limit")
            self.firmware_max = self._read("Max_Position_Limit")
            modes = self._read("Operating_Mode")
            if any(modes[name] != 0 for name in JOINT_NAMES):
                raise RuntimeError("All motors must be in position mode 0; no modes were changed")
            self._snapshot()
        except Exception:
            self.bus.disconnect(disable_torque=False)
            raise
        self.worker = threading.Thread(target=self._worker, name="so101-panel-jog", daemon=True)
        self.worker.start()

    @staticmethod
    def _real_bus(port):
        motors = {
            name: Motor(index, "sts3215", MotorNormMode.RANGE_0_100 if index == 6 else MotorNormMode.DEGREES)
            for index, name in enumerate(JOINT_NAMES, 1)
        }
        return SerialMotorsBus(port=port, motors=motors)

    def _read(self, register):
        values = self.bus.sync_read(register, normalize=False, num_retry=2)
        if set(values) != set(JOINT_NAMES):
            raise RuntimeError(f"Incomplete {register} read")
        return {name: int(values[name]) for name in JOINT_NAMES}

    def _snapshot(self):
        names = (
            ("position", "Present_Position"),
            ("goal", "Goal_Position"),
            ("status", "Status"),
            ("torque", "Torque_Enable"),
            ("voltage", "Present_Voltage"),
            ("temperature", "Present_Temperature"),
            ("load", "Present_Load"),
        )
        fields = {label: self._read(register) for label, register in names}
        return {
            name: {label: fields[label][name] for label, _ in names}
            for name in JOINT_NAMES
        }

    def _refresh_bounds(self):
        self.bounds = SafetyBounds.from_files(self.settings_path, self.calibration_path)
        self.park_bounds = self._fold_envelope(self.bounds)

    def _fold_envelope(self, bounds):
        # The manually observed endpoint only permits a way back inside a
        # tighter saved bound. It is never an outward jog destination.
        return SafetyBounds(
            tuple(min(lo, tick) for lo, tick in zip(bounds.lower, self.folded, strict=True)),
            tuple(max(hi, tick) for hi, tick in zip(bounds.upper, self.folded, strict=True)),
        )

    def _safe_span(self, index, position):
        """Ordinary limits, plus an inward-only exit from the recorded fold."""
        low, high = self.bounds.lower[index], self.bounds.upper[index]
        park_low, park_high = self.park_bounds.lower[index], self.park_bounds.upper[index]
        if position < low:
            if position < park_low - RECOVERY_MARGIN:
                raise ValueError(f"Motor {index + 1} is beyond its recorded folded bound")
            return position, high, False, True
        if position > high:
            if position > park_high + RECOVERY_MARGIN:
                raise ValueError(f"Motor {index + 1} is beyond its recorded folded bound")
            return low, position, True, False
        return low, high, position > low, position < high

    def _check_snapshot(self, snapshot, *, need_torque=True):
        for index, name in enumerate(JOINT_NAMES):
            joint = snapshot[name]
            if joint["status"] != 0:
                raise RuntimeError(f"Motor {index + 1} servo fault {joint['status']}")
            if need_torque and joint["torque"] != 1:
                raise RuntimeError(f"Motor {index + 1} torque is off; use Hold all joints first")
            if joint["voltage"] < MIN_VOLTAGE_RAW:
                if self._read("Present_Voltage")[name] < MIN_VOLTAGE_RAW:
                    raise RuntimeError(f"Motor {index + 1} voltage stayed low")
            if joint["temperature"] > 55:
                raise RuntimeError(f"Motor {index + 1} is too hot")
            position = joint["position"]
            if not self.firmware_min[name] <= position <= self.firmware_max[name]:
                raise RuntimeError(f"Motor {index + 1} exceeds firmware position bounds")
            self._safe_span(index, position)

    def _write_goal(self, motor, tick):
        name = JOINT_NAMES[motor - 1]
        if type(tick) is not int or not self.firmware_min[name] <= tick <= self.firmware_max[name]:
            raise ValueError(f"Motor {motor} target is outside firmware bounds")
        self.bus.write("Goal_Position", name, tick, normalize=False, num_retry=2)

    def _hold_measured(self, motor):
        name = JOINT_NAMES[motor - 1]
        if self._read("Torque_Enable")[name] != 1:
            return
        position = self._read("Present_Position")[name]
        self._write_goal(motor, position)

    def _stop(self):
        previous = self.hold
        self.hold = None
        if previous:
            self._hold_measured(previous["motor"])

    def state(self):
        with self.lock:
            snapshot = self._snapshot()
            all_powered = all(snapshot[name]["torque"] == 1 for name in JOINT_NAMES)
            motors = []
            for index, (name, label) in enumerate(zip(JOINT_NAMES, LABELS, strict=True)):
                joint = snapshot[name]
                try:
                    low, high, backward, forward = self._safe_span(index, joint["position"])
                except ValueError:
                    low, high, backward, forward = self.bounds.lower[index], self.bounds.upper[index], False, False
                fault = f"Servo fault {joint['status']}" if joint["status"] else None
                enabled = all_powered and fault is None and joint["voltage"] >= MIN_VOLTAGE_RAW
                setting = self.settings[str(index + 1)]
                motors.append({
                    "id": index + 1, "name": label, "position": joint["position"],
                    "goal": joint["goal"], "voltage": joint["voltage"] / 10,
                    "temperature": joint["temperature"], "torque": bool(joint["torque"]),
                    "status": joint["status"], "fault": fault,
                    "firmware_lower": self.firmware_min[name],
                    "firmware_upper": self.firmware_max[name],
                    "effective_lower": low, "effective_upper": high,
                    "can_forward": enabled and forward,
                    "can_backward": enabled and backward,
                    "motion_enabled": enabled,
                    **setting,
                })
            return {
                "motors": motors, "holding": self.hold["motor"] if self.hold else None,
                "error": self.error, "motion_enabled": all_powered,
                "hold_all_enabled": not all_powered and not any(m["fault"] for m in motors),
                "folded_ticks": dict(zip(JOINT_NAMES, self.folded, strict=True)),
            }

    def diagnostics(self):
        with self.lock:
            if self.hold:
                raise RuntimeError("Diagnostics are paused during movement")
            registers = (
                "Present_Position", "Goal_Position", "Present_Load", "Present_Current",
                "Present_Voltage", "Present_Temperature", "Status", "Moving",
                "Torque_Enable", "Operating_Mode", "Goal_Speed", "Acceleration",
            )
            return {
                str(index): {
                    register: int(self.bus.read(register, name, normalize=False, num_retry=2))
                    for register in registers
                }
                for index, name in enumerate(JOINT_NAMES, 1)
            }

    def start(self, motor, direction):
        with self.lock:
            self._stop()
            snapshot = self._snapshot()
            self._check_snapshot(snapshot)
            name = JOINT_NAMES[motor - 1]
            position = snapshot[name]["position"]
            _, _, backward, forward = self._safe_span(motor - 1, position)
            if not (forward if direction > 0 else backward):
                raise ValueError(f"Motor {motor} is at that limit; move in the other direction")
            self.hold = self._new_hold(motor, "jog", position, direction=direction)
            self.error = None
            return self.hold["token"]

    def go(self, target):
        with self.lock:
            if type(target) is not int:
                raise ValueError("Target must be an integer tick")
            self._stop()
            snapshot = self._snapshot()
            self._check_snapshot(snapshot)
            position = snapshot["gripper"]["position"]
            low, high, backward, forward = self._safe_span(5, position)
            if not low <= target <= high or (target > position and not forward) or (target < position and not backward):
                raise ValueError(f"Gripper target must be within {low}–{high} and toward an allowed direction")
            self.hold = self._new_hold(6, "goto", position, target=target)
            self.error = None
            return self.hold["token"]

    def _new_hold(self, motor, mode, position, *, direction=0, target=None):
        now = time.monotonic()
        return {
            "token": uuid.uuid4().hex, "motor": motor, "mode": mode,
            "direction": direction, "target": position if target is None else target,
            "command": float(position), "velocity": 0.0, "last_goal": position,
            "last_position": position, "progress_at": now, "last_update": now,
            "expires": now + LEASE_SECONDS if mode == "jog" else now + 35,
            "high_load_count": 0,
            "next": now,
        }

    def heartbeat(self, token):
        with self.lock:
            if not self.hold or self.hold["token"] != token or self.hold["mode"] != "jog":
                raise ValueError("Hold ended")
            self.hold["expires"] = time.monotonic() + LEASE_SECONDS

    def stop(self, token=None):
        with self.lock:
            if self.hold and (token is None or token == self.hold["token"]):
                self._stop()

    def _advance(self, now):
        hold = self.hold
        if hold is None or now < hold["next"]:
            return
        if now >= hold["expires"]:
            self._stop()
            self.error = "Movement timed out and stopped"
            return
        snapshot = self._snapshot()
        self._check_snapshot(snapshot)
        index = hold["motor"] - 1
        name = JOINT_NAMES[index]
        position = snapshot[name]["position"]
        load = abs(snapshot[name]["load"])
        hold["high_load_count"] = hold["high_load_count"] + 1 if load >= MAX_LOAD else 0
        if hold["high_load_count"] >= 3:
            raise RuntimeError(f"Motor {index + 1} sustained high load")
        low, high, backward, forward = self._safe_span(index, position)
        dt = max(0.001, min(0.1, now - hold["last_update"]))
        hold["last_update"] = now
        if hold["mode"] == "jog":
            if not (forward if hold["direction"] > 0 else backward):
                self._stop()
                return
            hold["target"] += hold["direction"] * BASE_TICKS_PER_SECOND * self.settings[str(index + 1)]["scale"] * dt
            desired = hold["target"]
        else:
            desired = hold["target"]
            if abs(position - desired) <= POSITION_TOLERANCE:
                self._stop()
                return
        desired = max(low, min(high, desired))
        desired = max(position - MAX_TARGET_LEAD, min(position + MAX_TARGET_LEAD, desired))
        requested_speed = min(
            MAX_COMMAND_SPEED[index],
            BASE_TICKS_PER_SECOND * self.settings[str(index + 1)]["scale"],
        )
        delta = desired - hold["command"]
        desired_velocity = max(-requested_speed, min(requested_speed, delta / dt))
        velocity = max(
            hold["velocity"] - MAX_COMMAND_ACCELERATION[index] * dt,
            min(hold["velocity"] + MAX_COMMAND_ACCELERATION[index] * dt, desired_velocity),
        )
        step = velocity * dt
        if abs(step) > abs(delta) and step * delta >= 0:
            step = delta
        command = max(low, min(high, hold["command"] + step))
        command = max(position - MAX_TARGET_LEAD, min(position + MAX_TARGET_LEAD, command))
        hold["velocity"] = (command - hold["command"]) / dt
        hold["command"] = command
        tick = round(command)
        if tick != hold["last_goal"]:
            self._write_goal(hold["motor"], tick)
            hold["last_goal"] = tick
        if abs(position - hold["last_position"]) >= 4:
            hold["last_position"] = position
            hold["progress_at"] = now
        elif abs(hold["last_goal"] - position) > 35 and now - hold["progress_at"] > 0.8:
            raise RuntimeError(f"Motor {index + 1} stalled at {position}")
        hold["next"] = now + STEP_SECONDS

    def _worker(self):
        while not self.closed:
            time.sleep(0.02)
            with self.lock:
                if self.hold is None:
                    continue
                try:
                    self._advance(time.monotonic())
                except Exception as exc:
                    logging.exception("Panel movement stopped")
                    self.error = str(exc)
                    try:
                        self._stop()
                    except Exception:
                        logging.exception("Could not hold measured position")
                        self.hold = None

    def limit(self, motor, bound, clear):
        with self.lock:
            self._stop()
            value = None if clear else self._read("Present_Position")[JOINT_NAMES[motor - 1]]
            entry = self.settings[str(motor)]
            other = entry["upper" if bound == "lower" else "lower"]
            if value is not None and other is not None:
                if (bound == "lower" and value >= other) or (bound == "upper" and value <= other):
                    raise ValueError("Lower limit must be below upper limit")
            old = entry[bound]
            entry[bound] = value
            try:
                self._save()
                self._refresh_bounds()
            except Exception:
                entry[bound] = old
                self._save()
                self._refresh_bounds()
                raise
            return {"motor": motor, "bound": bound, "value": value}

    def scale(self, motor, scale):
        with self.lock:
            self.settings[str(motor)]["scale"] = scale
            self._save()
            return {"motor": motor, "scale": scale}

    def _save(self):
        tmp = self.settings_path.with_suffix(".json.tmp")
        with tmp.open("w", encoding="utf-8") as stream:
            json.dump(self.settings, stream, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        tmp.replace(self.settings_path)

    def hold_all(self):
        with self.lock:
            self._stop()
            snapshot = self._snapshot()
            self._check_snapshot(snapshot, need_torque=False)
            positions = {name: snapshot[name]["position"] for name in JOINT_NAMES}
            # Goal_Position may enable torque on some STS3215 firmware; this
            # entire operation is only reached after the explicit Hold click.
            for index, name in enumerate(JOINT_NAMES):
                self._write_goal(index + 1, positions[name])
            self.bus.sync_write("Torque_Enable", {name: 1 for name in JOINT_NAMES}, normalize=False)
            if any(value != 1 for value in self._read("Torque_Enable").values()):
                raise RuntimeError("One or more motors did not enable torque")
            return {"ok": True}

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            try:
                self._stop()
            finally:
                self.bus.disconnect(disable_torque=False)
        self.worker.join(timeout=1)


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

        def allowed(self):
            if self.headers.get("Host") not in (f"127.0.0.1:{port}", f"localhost:{port}"):
                self.reply(403, {"error": "Invalid host"})
                return False
            origin = self.headers.get("Origin")
            if origin and origin not in (f"http://127.0.0.1:{port}", f"http://localhost:{port}"):
                self.reply(403, {"error": "Invalid origin"})
                return False
            return True

        def do_GET(self):
            if not self.allowed():
                return
            path = urlsplit(self.path).path
            if path == "/":
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
                elif path == "/api/limit":
                    motor = motor_id(data.get("motor"))
                    bound = data.get("bound")
                    if bound not in ("lower", "upper") or type(data.get("clear", False)) is not bool:
                        raise ValueError("bound must be lower/upper and clear must be boolean")
                    result = arm.limit(motor, bound, data.get("clear", False))
                elif path == "/api/scale":
                    motor = motor_id(data.get("motor"))
                    scale = data.get("scale")
                    if type(scale) is not int or scale not in SCALES:
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
    parser.add_argument("--http-port", type=int, default=8765)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    arm = Arm(args.port or find_port())
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.http_port), handler_for(arm, args.http_port))
    except Exception:
        arm.close()
        raise
    server.daemon_threads = True

    def shutdown(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    print(f"SO-101 control panel: http://127.0.0.1:{args.http_port}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        server.server_close()
        arm.close()


if __name__ == "__main__":
    main()
