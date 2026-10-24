"""Bus-owning controller behind the dashboard's jog and pose controls.

Connecting is read-only. A jog is an explicit, leased hold that the browser
must keep renewing. Home, Fold, and Dance are whole-arm pose moves driven from
measured feedback. Torque stays enabled whenever a hold or pose move ends.
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
from care_reach.motion.bounds import SafetyBounds, validate_pose
from care_reach.robot import (
    CALIBRATION_PATH,
    FOLDED_POSE_PATH,
    HOME_POSE,
    JOINT_LABELS,
    JOINT_NAMES,
    JOINT_SETTINGS_PATH,
    load_folded_pose,
)
from care_reach.routines.dance import build_plan as build_dance_plan
from care_reach.routines.park import folded_bounds
from care_reach.settings import load_joint_settings, save_joint_settings


BASE_TICKS_PER_SECOND = 25
MAX_COMMAND_SPEED = (180, 180, 180, 180, 200, 180)
MAX_COMMAND_ACCELERATION = (300, 300, 300, 300, 360, 300)
MAX_TARGET_LEAD = 80
STOP_SETTLE_LEAD = 50  # Measured wrist coast reached 46 ticks at 8x speed.
RECOVERY_MARGIN = 40  # Allow a small encoder drift past the saved fold, inward only.
STEP_SECONDS = 0.04
LEASE_SECONDS = 0.75
GOTO_TIMEOUT_SECONDS = 35
POSITION_TOLERANCE = 15
MAX_TEMPERATURE_C = 55
VOLTAGE_ALARM_STATUS = 1  # STS3215 status bit 0; see docs/POSES.md.

# Whole-arm pose moves (Home, Fold, and the Home legs of Dance).
POSES = ("home", "fold", "dance")
POSE_STEP_SECONDS = 0.06
POSE_MAX_LEAD = 80
POSE_TOLERANCE = 35
HOME_TOLERANCE = 15
HOME_TRIM_OFFSET = 40
HOME_TRIM_PERIOD = 0.15
POSE_TIMEOUT_SECONDS = 40
POSE_STALL_SECONDS = 2.0
POSE_STALL_TICKS = 3
POSE_MEASURED_OVERSHOOT = 5

# Dance playback.
DANCE_FPS = 20
DANCE_SCALE = 1.0
DANCE_STALL_SECONDS = 2.0
DANCE_STALL_ERROR_TICKS = 45


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

    def _check_snapshot(self, snapshot, *, need_torque=True, bounds=None, overshoot=0):
        """Raise on any fault. ``bounds`` replaces the jog span for pose moves,
        allowing ``overshoot`` ticks of measured drift past either end."""
        for index, name in enumerate(JOINT_NAMES):
            joint = snapshot[name]
            if joint["status"] == VOLTAGE_ALARM_STATUS:
                raise ArmSafetyError(f"Motor {index + 1} voltage alarm (servo status 1)")
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
            if bounds is not None:
                low, high = bounds.lower[index], bounds.upper[index]
                if not low - overshoot <= position <= high + overshoot:
                    raise ArmSafetyError(f"Motor {index + 1} measured position {position} exceeds [{low}, {high}]")
            else:
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

    def _settle_goal(self, hold, name, position):
        """Coast at most STOP_SETTLE_LEAD ticks in the jog direction, never past
        the last commanded goal or outside the safe span and firmware range;
        if measured already overshot the last goal, hold measured."""
        index = hold["motor"] - 1
        last_goal = hold["last_goal"]
        if hold["direction"] > 0:
            if position >= last_goal:
                return position
            target = min(position + STOP_SETTLE_LEAD, last_goal)
        else:
            if position <= last_goal:
                return position
            target = max(position - STOP_SETTLE_LEAD, last_goal)
        try:
            low, high, _, _ = self._safe_span(index, position)
        except ValueError:
            return position
        target = min(high, max(low, target))
        target = min(self.firmware_max[name], max(self.firmware_min[name], target))
        if (hold["direction"] > 0 and target < position) or (hold["direction"] < 0 and target > position):
            return position
        return target

    def _stop(self, *, settle=False):
        """End the active hold. ``settle`` lets a released jog coast forward
        instead of snapping back to a lagging encoder reading."""
        previous = self.hold
        self.hold = None
        if not previous:
            return
        if previous["mode"] == "pose":
            self._hold_measured_all()
            return
        if settle and previous["mode"] == "jog":
            name = JOINT_NAMES[previous["motor"] - 1]
            if read_joints(self.bus, "Torque_Enable")[name] != 1:
                return
            position = read_joints(self.bus, "Present_Position")[name]
            self._write_goal(previous["motor"], self._settle_goal(previous, name, position))
        else:
            self._hold_measured(previous["motor"])

    def _hold_measured_all(self):
        """Command every enabled joint to its measured position; never torque off."""
        torque = read_joints(self.bus, "Torque_Enable")
        if not any(value == 1 for value in torque.values()):
            return
        positions = read_joints(self.bus, "Present_Position")
        goals = {}
        for index, name in enumerate(JOINT_NAMES):
            if torque[name] != 1:
                continue
            position = positions[name]
            goal = max(self.park_bounds.lower[index], min(self.park_bounds.upper[index], position))
            goal = max(self.firmware_min[name], min(self.firmware_max[name], goal))
            goals[name] = int(goal)
        if goals:
            self.bus.sync_write("Goal_Position", goals, normalize=False)

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
                fault = ("Voltage alarm (status 1)" if joint["status"] == VOLTAGE_ALARM_STATUS else
                         f"Servo fault {joint['status']}" if joint["status"] else None)
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
            pose = self.hold if self.hold is not None and self.hold["mode"] == "pose" else None
            return {
                "motors": motors, "holding": self.hold["motor"] if self.hold else None,
                "error": self.error, "motion_enabled": all_powered,
                "hold_all_enabled": not all_powered and not any(m["fault"] for m in motors),
                "folded_ticks": dict(zip(JOINT_NAMES, self.folded, strict=True)),
                "pose": {
                    "active": pose is not None,
                    "name": pose["pose"] if pose else None,
                    "progress": pose["progress"] if pose else 0.0,
                    "target": dict(zip(JOINT_NAMES, pose["target"], strict=True)) if pose else None,
                    "positions": pose["measured"] if pose else None,
                },
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
            if self.hold is not None and self.hold["mode"] == "pose":
                raise ValueError("A pose move is in progress; cancel it before jogging")
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
            if self.hold is not None and self.hold["mode"] == "pose":
                raise ValueError("A pose move is in progress; cancel it before using go-to-tick")
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

    def pose_start(self, pose):
        """Start Home, Fold, or Dance from the measured pose. Dance homes first."""
        with self.lock:
            if self.hold is not None:
                raise ValueError("A movement is already active; cancel it before starting a pose move")
            if pose not in POSES:
                raise ValueError("pose must be 'home', 'fold', or 'dance'")
            dance_frames = None
            if pose == "dance":
                _, dance_frames = build_dance_plan(self.bounds, DANCE_SCALE, DANCE_FPS)
                for index, name in enumerate(JOINT_NAMES):
                    low = min(frame[index] for frame in dance_frames)
                    high = max(frame[index] for frame in dance_frames)
                    if low < self.firmware_min[name] or high > self.firmware_max[name]:
                        raise ArmSafetyError(f"Motor {index + 1} dance exceeds firmware bounds")
            target = self.folded if pose == "fold" else HOME_POSE
            envelope = folded_bounds(self.bounds, self.folded)
            validate_pose(target, envelope)
            require_position_mode(self.bus)
            for index, name in enumerate(JOINT_NAMES):
                if not self.firmware_min[name] <= target[index] <= self.firmware_max[name]:
                    raise ArmSafetyError(f"Motor {index + 1} target is outside firmware bounds")
            snapshot = self._snapshot()
            self._check_snapshot(snapshot, bounds=envelope, overshoot=POSE_MEASURED_OVERSHOOT)
            self.error = None
            now = time.monotonic()
            self.hold = {
                "token": uuid.uuid4().hex, "motor": None, "mode": "pose", "pose": pose,
                "stage": "home" if pose == "dance" else None,
                "dance_frames": dance_frames,
                "target": target, "bounds": envelope,
                "start_positions": {name: snapshot[name]["position"] for name in JOINT_NAMES},
                "measured": {name: snapshot[name]["position"] for name in JOINT_NAMES},
                "progress_position": {name: snapshot[name]["position"] for name in JOINT_NAMES},
                "progress_time": {name: now for name in JOINT_NAMES},
                "load_watch": LoadWatch(),
                "trim_pending": set(),
                "writes": 0, "progress": 0.0,
                "next": now, "last_update": now, "expires": now + POSE_TIMEOUT_SECONDS,
            }
            return {"token": self.hold["token"], "pose": pose}

    def pose_stop(self):
        with self.lock:
            if self.hold is not None and self.hold["mode"] == "pose":
                self._stop()
            return {"ok": True}

    def _advance_pose(self, now):
        hold = self.hold
        if hold["pose"] == "dance" and hold["stage"] == "playback":
            self._advance_dance(now)
            return
        snapshot = self._snapshot()
        self._check_snapshot(snapshot, bounds=hold["bounds"], overshoot=POSE_MEASURED_OVERSHOOT)
        targets = hold["target"]
        envelope = hold["bounds"]
        positions = {name: snapshot[name]["position"] for name in JOINT_NAMES}
        hold["measured"] = positions
        errors = {name: targets[index] - positions[name] for index, name in enumerate(JOINT_NAMES)}
        remaining = sum(abs(errors[name]) for name in JOINT_NAMES)
        started = sum(abs(targets[index] - hold["start_positions"][name])
                      for index, name in enumerate(JOINT_NAMES))
        stage_progress = 1.0 if not started else max(0.0, min(1.0, 1.0 - remaining / started))
        if hold["pose"] == "dance":
            hold["progress"] = (.95 + .05 * stage_progress) if hold["stage"] == "finish" else .05 * stage_progress
        else:
            hold["progress"] = stage_progress
        tolerance = HOME_TOLERANCE if hold["pose"] in ("home", "dance") else POSE_TOLERANCE
        if all(abs(errors[name]) <= tolerance for name in JOINT_NAMES):
            # The previous bounded goal is already near this endpoint. Keep
            # the exact pose as the holding goal so completion does not snap
            # the target backward to a still-moving encoder reading.
            self.bus.sync_write("Goal_Position", dict(zip(JOINT_NAMES, targets, strict=True)), normalize=False)
            if hold["pose"] == "dance" and hold["stage"] == "home":
                hold["stage"] = "playback"
                hold["bounds"] = self.bounds
                hold["frame_index"] = 0
                hold["last_goals"] = dict(zip(JOINT_NAMES, HOME_POSE, strict=True))
                hold["temperatures"] = {name: snapshot[name]["temperature"] for name in JOINT_NAMES}
                hold["temperature_at"] = now
                hold["progress"] = .05
                hold["next"] = now + 1 / DANCE_FPS
                hold["expires"] = now + max(180, len(hold["dance_frames"]) * .15)
                return
            self.hold = None
            return
        goals = {}
        trimming = False
        for index, name in enumerate(JOINT_NAMES):
            hold["load_watch"].check(name, snapshot[name]["load"])
            position = positions[name]
            if abs(position - hold["progress_position"][name]) >= POSE_STALL_TICKS:
                hold["progress_position"][name] = position
                hold["progress_time"][name] = now
            elif abs(errors[name]) > tolerance and now - hold["progress_time"][name] >= POSE_STALL_SECONDS:
                raise ArmSafetyError(f"Motor {index + 1} stalled at {position}")
            direction = 1 if errors[name] > 0 else -1
            if hold["pose"] in ("home", "dance") and tolerance < abs(errors[name]) <= HOME_TRIM_OFFSET:
                # Overcome the servo's static-friction deadband by briefly
                # aiming past the target, then return to the exact target.
                trimming = True
                if name in hold["trim_pending"]:
                    goal = targets[index]
                    hold["trim_pending"].remove(name)
                else:
                    goal = targets[index] + direction * HOME_TRIM_OFFSET
                    hold["trim_pending"].add(name)
            else:
                hold["trim_pending"].discard(name)
                goal = position + direction * min(abs(errors[name]), POSE_MAX_LEAD)
            goal = max(envelope.lower[index], min(envelope.upper[index], goal))
            goal = max(self.firmware_min[name], min(self.firmware_max[name], goal))
            if (direction > 0 and goal < position) or (direction < 0 and goal > position):
                goal = position
            goals[name] = int(goal)
        self.bus.sync_write("Goal_Position", goals, normalize=False)
        hold["writes"] += 1
        hold["last_update"] = now
        hold["next"] = now + (HOME_TRIM_PERIOD if trimming else POSE_STEP_SECONDS)

    def _advance_dance(self, now):
        """Play one dance frame, holding each goal within POSE_MAX_LEAD of the
        encoder and waiting rather than skipping ahead when a joint lags."""
        hold = self.hold
        fields = {label: read_joints(self.bus, register) for label, register in (
            ("position", "Present_Position"), ("status", "Status"),
            ("torque", "Torque_Enable"), ("voltage", "Present_Voltage"),
            ("load", "Present_Load"),
        )}
        # Temperature changes slowly; reading it once a second keeps the
        # 20 Hz loop's bus traffic down.
        if now - hold["temperature_at"] >= 1.0:
            hold["temperatures"] = read_joints(self.bus, "Present_Temperature")
            hold["temperature_at"] = now
        snapshot = {
            name: {**{label: values[name] for label, values in fields.items()},
                   "temperature": hold["temperatures"][name]}
            for name in JOINT_NAMES
        }
        self._check_snapshot(snapshot, bounds=self.bounds, overshoot=POSE_MEASURED_OVERSHOOT)
        positions = {name: snapshot[name]["position"] for name in JOINT_NAMES}
        hold["measured"] = positions
        for name in JOINT_NAMES:
            hold["load_watch"].check(name, snapshot[name]["load"])
            if abs(positions[name] - hold["progress_position"][name]) >= POSE_STALL_TICKS:
                hold["progress_position"][name] = positions[name]
                hold["progress_time"][name] = now
            elif (abs(hold["last_goals"][name] - positions[name]) > DANCE_STALL_ERROR_TICKS
                  and now - hold["progress_time"][name] >= DANCE_STALL_SECONDS):
                raise ArmSafetyError(f"{name} stalled at {positions[name]}")
        frame = hold["dance_frames"][hold["frame_index"]]
        hold["target"] = frame
        goals = {}
        behind = False
        for index, name in enumerate(JOINT_NAMES):
            distance = frame[index] - positions[name]
            behind |= abs(distance) > POSE_MAX_LEAD
            goal = positions[name] + max(-POSE_MAX_LEAD, min(POSE_MAX_LEAD, distance))
            goals[name] = int(goal)
        validate_pose(tuple(goals[name] for name in JOINT_NAMES), self.bounds)
        self.bus.sync_write("Goal_Position", goals, normalize=False)
        hold["last_goals"] = goals
        if not behind:
            hold["frame_index"] += 1
        hold["progress"] = .05 + .90 * hold["frame_index"] / len(hold["dance_frames"])
        hold["next"] = now + 1 / DANCE_FPS
        if hold["frame_index"] == len(hold["dance_frames"]):
            hold["stage"] = "finish"
            hold["target"] = HOME_POSE
            hold["start_positions"] = positions
            hold["progress_position"] = positions.copy()
            hold["progress_time"] = {name: now for name in JOINT_NAMES}
            hold["trim_pending"] = set()
            hold["expires"] = now + POSE_TIMEOUT_SECONDS
            hold["next"] = now + POSE_STEP_SECONDS

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
                self._stop(settle=True)

    def _advance(self, now):
        hold = self.hold
        if hold is None or now < hold["next"]:
            return
        if now >= hold["expires"]:
            self._stop()
            self.error = "Movement timed out and stopped"
            return
        if hold["mode"] == "pose":
            self._advance_pose(now)
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
                self._stop(settle=True)
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
            # Dance playback runs at DANCE_FPS, so poll faster while it plays.
            active = self.hold
            time.sleep(0.005 if active is not None and active.get("stage") == "playback" else 0.02)
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

    def scale(self, motor, scale):
        with self.lock:
            self.settings[str(motor)]["scale"] = scale
            save_joint_settings(self.settings_path, self.settings)
            return {"motor": motor, "scale": scale}

    def hold_all(self):
        with self.lock:
            if self.hold is not None and self.hold["mode"] == "pose":
                raise ValueError("A pose move is in progress; cancel it before holding all joints")
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
