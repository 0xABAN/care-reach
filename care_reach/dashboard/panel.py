"""Bus-owning controller behind the dashboard's jog and limit controls.

Connecting is read-only. Every movement is an explicit, leased hold that the
browser must keep renewing; torque stays enabled when a hold ends.
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from pathlib import Path

from care_reach.hardware.bus import (
    MIN_VOLTAGE_RAW,
    STALL_ERROR_TICKS,
    STALL_PROGRESS_TICKS,
    STALL_TIME_S,
    ArmSafetyError,
    LoadWatch,
    check_voltage,
    make_bus,
    read_firmware_limits,
    read_joints,
    require_position_mode,
)
from care_reach.motion.bounds import SafetyBounds
from care_reach.robot import (
    CALIBRATION_PATH,
    FOLDED_POSE_PATH,
    JOINT_LABELS,
    JOINT_NAMES,
    JOINT_SETTINGS_PATH,
    load_folded_pose,
)
from care_reach.settings import load_joint_settings, save_joint_settings


BASE_TICKS_PER_SECOND = 25
MAX_COMMAND_SPEED = (180, 180, 180, 180, 200, 180)
MAX_COMMAND_ACCELERATION = (300, 300, 300, 300, 360, 300)
MAX_TARGET_LEAD = 80
RECOVERY_MARGIN = 40  # Allow a small encoder drift past the saved fold, inward only.
STEP_SECONDS = 0.04
LEASE_SECONDS = 0.75
GOTO_TIMEOUT_SECONDS = 35
POSITION_TOLERANCE = 15
MAX_TEMPERATURE_C = 55


class PanelController:
    """One owner of the serial bus. Read-only on connect; every motion is explicit."""

    def __init__(self, port, *, bus_factory=None, settings_path: Path = JOINT_SETTINGS_PATH,
                 calibration_path: Path = CALIBRATION_PATH, pose_path: Path = FOLDED_POSE_PATH):
        self.lock = threading.RLock()
        self.settings_path = Path(settings_path)
        self.calibration_path = calibration_path
        self.settings = load_joint_settings(self.settings_path)
        self.folded = load_folded_pose(pose_path)
        self.bounds = SafetyBounds.from_files(self.settings_path, self.calibration_path)
        self.park_bounds = self._fold_envelope(self.bounds)
        self.hold = None
        self.error = None
        self.closed = False
        self.bus = (bus_factory or make_bus)(port)
        self.bus.connect(handshake=True)
        try:
            self.firmware_min, self.firmware_max = read_firmware_limits(self.bus)
            require_position_mode(self.bus)
            self._snapshot()
        except Exception:
            self.bus.disconnect(disable_torque=False)
            raise
        self.worker = threading.Thread(target=self._worker, name="so101-panel-jog", daemon=True)
        self.worker.start()

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
        fields = {label: read_joints(self.bus, register) for label, register in names}
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
                raise ArmSafetyError(f"Motor {index + 1} servo fault {joint['status']}")
            if need_torque and joint["torque"] != 1:
                raise ArmSafetyError(f"Motor {index + 1} torque is off; use Hold all joints first")
            check_voltage(self.bus, name, joint["voltage"])
            if joint["temperature"] > MAX_TEMPERATURE_C:
                raise ArmSafetyError(f"Motor {index + 1} is too hot")
            position = joint["position"]
            if not self.firmware_min[name] <= position <= self.firmware_max[name]:
                raise ArmSafetyError(f"Motor {index + 1} exceeds firmware position bounds")
            self._safe_span(index, position)

    def _write_goal(self, motor, tick):
        name = JOINT_NAMES[motor - 1]
        if type(tick) is not int or not self.firmware_min[name] <= tick <= self.firmware_max[name]:
            raise ValueError(f"Motor {motor} target is outside firmware bounds")
        self.bus.write("Goal_Position", name, tick, normalize=False, num_retry=2)

    def _hold_measured(self, motor):
        name = JOINT_NAMES[motor - 1]
        if read_joints(self.bus, "Torque_Enable")[name] != 1:
            return
        position = read_joints(self.bus, "Present_Position")[name]
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
            for index, (name, label) in enumerate(zip(JOINT_NAMES, JOINT_LABELS, strict=True)):
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
            "expires": now + (LEASE_SECONDS if mode == "jog" else GOTO_TIMEOUT_SECONDS),
            "load_watch": LoadWatch(),
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
        hold["load_watch"].check(name, snapshot[name]["load"])
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
        if abs(position - hold["last_position"]) >= STALL_PROGRESS_TICKS:
            hold["last_position"] = position
            hold["progress_at"] = now
        elif abs(hold["last_goal"] - position) > STALL_ERROR_TICKS and now - hold["progress_at"] > STALL_TIME_S:
            raise ArmSafetyError(f"Motor {index + 1} stalled at {position}")
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
            value = None if clear else read_joints(self.bus, "Present_Position")[JOINT_NAMES[motor - 1]]
            entry = self.settings[str(motor)]
            other = entry["upper" if bound == "lower" else "lower"]
            if value is not None and other is not None:
                if (bound == "lower" and value >= other) or (bound == "upper" and value <= other):
                    raise ValueError("Lower limit must be below upper limit")
            old = entry[bound]
            entry[bound] = value
            try:
                save_joint_settings(self.settings_path, self.settings)
                self._refresh_bounds()
            except Exception:
                entry[bound] = old
                save_joint_settings(self.settings_path, self.settings)
                self._refresh_bounds()
                raise
            return {"motor": motor, "bound": bound, "value": value}

    def scale(self, motor, scale):
        with self.lock:
            self.settings[str(motor)]["scale"] = scale
            save_joint_settings(self.settings_path, self.settings)
            return {"motor": motor, "scale": scale}

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
            if any(value != 1 for value in read_joints(self.bus, "Torque_Enable").values()):
                raise ArmSafetyError("One or more motors did not enable torque")
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
