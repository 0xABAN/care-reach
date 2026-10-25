"""Movement and hardware ownership for the SO-101 follower.

This module is the importable movement core: it owns the URT-2 serial bus,
enforces every safety limit, and drives jog, go-to-tick, Home, Extend, Dance, and taught moves.
``server.py`` is only the HTTP/camera adapter around it. ``motion_core`` stays
pure offline planning and is reused here unchanged.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
import uuid
from pathlib import Path

from serial.tools import list_ports

from .bus import real_bus
from .bus_lock import BusLock
from .movement_policy import MovementMode, parse_movement_mode, validate_movement_path
from .timing import MonotonicClock, MotionTiming, TimedBus
from dance_demo import HOME as DANCE_CENTER, build_plan as build_dance_plan
from extended_pose import extended_target
from home_pose import home_bounds, home_target
from motion_core import JOINT_NAMES, SafetyBounds, quintic_segment, validate_pose
from park import folded_bounds, folded_target
from telemetry import DEFAULT_PATH, TelemetryLog


ROOT = Path(__file__).resolve().parent.parent
SETTINGS = ROOT / "joint_settings.json"
LABELS = ("Shoulder pan", "Shoulder lift", "Elbow flex", "Wrist flex", "Wrist roll", "Gripper")
SCALES = (1, 2, 4, 8)
BASE_TICKS_PER_SECOND = 25
MAX_COMMAND_SPEED = (180, 180, 180, 180, 200, 180)
MAX_COMMAND_ACCELERATION = (300, 300, 300, 300, 360, 300)
MAX_TARGET_LEAD = 80
STOP_SETTLE_LEAD = 50  # Measured wrist coast reached 46 ticks at 8x speed.
RECOVERY_MARGIN = 40  # Allow a small encoder drift past the saved fold, inward only.
STEP_SECONDS = 0.04
LEASE_SECONDS = 1.5
GRIP_HOLD_LEAD = 30  # Small closing preload after a normal release or lost heartbeat.
MIN_VOLTAGE_RAW = 48
MAX_LOAD = 900
POSITION_TOLERANCE = 15
SINGLE_MOTOR_GRIP_HOLD_TOLERANCE = GRIP_HOLD_LEAD + 5
SINGLE_MOTOR_SETTLE_SAMPLE_SECONDS = 0.05
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
DANCE_FPS = 20
DANCE_SCALE = 1.0
DANCE_STALL_SECONDS = 2.0
EXTEND_FPS = 20
EXTEND_MAX_LEAD = 55
EXTEND_VMAX = (180, 90, 200, 90, 100, 100)
EXTEND_AMAX = (300, 180, 260, 180, 200, 200)
TAUGHT_MOVES = ("press_button", "lift_this", "bring_to_mouth")
# Moves that start with the object already gripped. They always lift one heavy
# joint at a time and run only at 1x, because they end near a person's face.
TAUGHT_CARRYING = frozenset({"bring_to_mouth"})
TAUGHT_PATH = ROOT / "taught_moves.json"
TAUGHT_MAX_STEPS = 12
TAUGHT_VMAX = (120, 90, 120, 90, 100, 120)
TAUGHT_AMAX = (240, 180, 240, 180, 200, 240)
# Per-move playback speed, chosen on the dashboard. Capped by MAX_COMMAND_*.
TAUGHT_SPEEDS = (1.0, 1.5, 2.0)
TAUGHT_MOVE_SPEEDS = {"bring_to_mouth": (1.0,)}
TAUGHT_MAX_LEAD = (POSE_MAX_LEAD, 120, 120, POSE_MAX_LEAD, POSE_MAX_LEAD, POSE_MAX_LEAD)
TAUGHT_SETTLE_TOLERANCE = 60
# Shared supply: pushing harder draws more current and sags every servo. In
# taught and staged moves, ease the push when any servo drops 0.1 V below the
# resting voltage measured at the start of the move (at most 5.2 V), so the bus
# stays near 5.1-5.2 V like a manual jog, far from the 4.8 V stop. The
# threshold follows the supply: a 5.2 V rest must not ease at every small load.
VOLTAGE_EASE_RAW = 52
VOLTAGE_EASE_DROP = 1
# Home must finish from any pose. Folding the elbow against gravity alone
# pulls a weak supply to about 5.0 V, and easing at 5.1 V left it crawling
# (125 of 725 ticks in 43 s), so Home eases only below 5.0 V, still 0.2 V
# above the hard stop.
HOME_EASE_RAW = 50
TAUGHT_EASE_MIN = 0.25
TAUGHT_EASE_RECOVERY = 0.02  # Per control step; about 2.5 s back to full push.
POSE_NAMES = ("home", "extend", "dance") + TAUGHT_MOVES
TIMING_SAMPLE_INTERVAL_S = 5.0
CURRENT_SAMPLE_INTERVAL_S = 0.5


def load_taught(path: Path) -> dict[str, list[tuple[int, ...]]]:
    """Saved waypoints for each taught move; missing or empty moves have none."""
    moves = {name: [] for name in TAUGHT_MOVES}
    if not path.is_file():
        return moves
    data = json.loads(path.read_text(encoding="utf-8"))
    for name in TAUGHT_MOVES:
        steps = data.get(name, [])
        if not isinstance(steps, list) or len(steps) > TAUGHT_MAX_STEPS:
            raise ValueError(f"{path}: {name} must be a list of at most {TAUGHT_MAX_STEPS} steps")
        for step in steps:
            if (not isinstance(step, list) or len(step) != 6
                    or any(type(tick) is not int for tick in step)):
                raise ValueError(f"{path}: each {name} step must be six integer ticks")
        moves[name] = [tuple(step) for step in steps]
    return moves

def load_taught_speeds(path: Path) -> dict[str, float]:
    """Saved playback speed per taught move; missing or unknown values are 1x."""
    speeds = {name: 1.0 for name in TAUGHT_MOVES}
    if not path.is_file():
        return speeds
    saved = json.loads(path.read_text(encoding="utf-8")).get("speeds", {})
    if isinstance(saved, dict):
        for name in TAUGHT_MOVES:
            value = saved.get(name)
            if type(value) in (int, float) and float(value) in TAUGHT_MOVE_SPEEDS.get(name, TAUGHT_SPEEDS):
                speeds[name] = float(value)
    return speeds


def taught_limits(speed: float) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Speed and acceleration limits for a taught move at ``speed``."""
    vmax = tuple(min(v * speed, cap) for v, cap in zip(TAUGHT_VMAX, MAX_COMMAND_SPEED))
    amax = tuple(min(a * speed, cap) for a, cap in zip(TAUGHT_AMAX, MAX_COMMAND_ACCELERATION))
    return vmax, amax


# While carrying, lift one heavy joint at a time: elbow, then shoulder, then
# the rest. Moving both together can sag the supply below MIN_VOLTAGE_RAW.
CARRY_ORDER = ((2,), (1,), (0, 3, 4))


def taught_segments(start, steps, carrying_move=False):
    """Waypoint pairs to replay; loaded segments are split joint by joint.

    ``carrying_move`` means the object is already gripped when the move starts,
    so every segment that keeps the gripper still is a loaded segment.
    """
    grips = [step[5] for step in steps]
    closes = any(later < earlier - POSE_TOLERANCE for earlier, later in zip(grips, grips[1:]))
    closed = min(grips)
    segments = []
    for begin, end in zip((tuple(start), *steps), steps):
        steady = abs(end[5] - begin[5]) <= POSE_TOLERANCE
        carrying = steady and (carrying_move or (closes and begin[5] <= closed + POSE_TOLERANCE))
        if not carrying:
            segments.append((tuple(begin), tuple(end)))
            continue
        segments += staged_segments(begin, end, CARRY_ORDER)
    return segments


def staged_segments(begin, end, order):
    """Split one move into consecutive moves of the joint groups in ``order``."""
    segments = []
    current = list(begin)
    for joints in order:
        following = list(current)
        for index in joints:
            following[index] = end[index]
        if following != current:
            segments.append((tuple(current), tuple(following)))
            current = following
    return segments


# Home folds the elbow, then lifts the shoulder, then the wrist, then the rest,
# one at a time from any start. Pulling several loaded joints at once sagged the
# supply below MIN_VOLTAGE_RAW. Elbow before shoulder also respects the
# Extend->Home rule that the shoulder must not lead the elbow.
HOME_ORDER = ((2,), (1,), (3,), (0,), (4,), (5,))  # Strictly one motor at a time.


def needs_staged_home(start, home):
    """Whether the shoulder, elbow, or wrist has a real distance to travel."""
    return any(abs(start[index] - home[index]) > POSE_TOLERANCE for index in (1, 2, 3))


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
                 calibration_path: Path | None = None, pose_path: Path | None = None,
                 telemetry_path: Path = DEFAULT_PATH, taught_path: Path = TAUGHT_PATH,
                 bus_lock: BusLock | None = None,
                 clock: MonotonicClock | None = None):
        self.lock = threading.RLock()
        self.settings_path = Path(settings_path)
        self.calibration_path = calibration_path
        self.pose_path = pose_path
        self.settings = load_settings(self.settings_path)
        self.folded = folded_target(pose_path) if pose_path is not None else folded_target()
        self.home = home_target()
        self.extended = extended_target()
        self.taught_path = Path(taught_path)
        self.taught = load_taught(self.taught_path)
        self.taught_speed = load_taught_speeds(self.taught_path)
        self.bounds = SafetyBounds.from_files(self.settings_path, self.calibration_path)
        self.park_bounds = folded_bounds(home_bounds(self.bounds, self.home), self.folded)
        self.hold = None
        self.movement_mode = MovementMode.COORDINATED
        self.error = None
        self.closed = False
        self.clock = clock if clock is not None else MonotonicClock()
        self.timing = MotionTiming()
        self.telemetry = TelemetryLog(telemetry_path)
        self._last_timing_emit = self.clock.monotonic()
        self.last_snapshot = None
        self.last_logged_at = 0.0
        self._last_current_at = float("-inf")
        self._last_current_token = None
        self.last_statuses = None
        self.low_voltage = False
        self.healthy_since = None
        self.next_idle_sample = 0.0
        # An explicit BusLock is always honored; the default lock is only taken
        # by real-bus ownership so injected fake buses never touch it.
        self._bus_lock = bus_lock if bus_lock is not None else (BusLock() if bus_factory is None else None)
        if self._bus_lock is not None:
            try:
                self._bus_lock.acquire(port)
            except Exception:
                self.telemetry.close()
                raise
        self.bus = None
        try:
            self.bus = (bus_factory or self._real_bus)(port)
            self.bus.connect(handshake=True)
            self.firmware_min = self._read("Min_Position_Limit")
            self.firmware_max = self._read("Max_Position_Limit")
            modes = self._read("Operating_Mode")
            if any(modes[name] != 0 for name in JOINT_NAMES):
                raise RuntimeError("All motors must be in position mode 0; no modes were changed")
            self._snapshot()
            self.telemetry.write("controller_started", snapshot=self.last_snapshot)
        except Exception:
            if self.bus is not None:
                try:
                    self.bus.disconnect(disable_torque=False)
                except Exception:
                    pass
            if self._bus_lock is not None:
                self._bus_lock.release()
            self.telemetry.close()
            raise
        self.bus = TimedBus(self.bus, self.clock, self.timing)
        self.worker = threading.Thread(target=self._worker, name="so101-panel-jog", daemon=True)
        try:
            self.worker.start()
        except Exception:
            try:
                self.bus.disconnect(disable_torque=False)
            except Exception:
                logging.exception("Could not close motor bus after worker startup failure")
            finally:
                if self._bus_lock is not None:
                    self._bus_lock.release()
                self.telemetry.close()
            raise

    @staticmethod
    def _real_bus(port):
        return real_bus(port)

    @property
    def single_motor_mode(self):
        """Compatibility alias for the older two-mode dashboard clients."""
        return self.movement_mode is MovementMode.SINGLE_JOINT

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
        snapshot = {
            name: {label: fields[label][name] for label, _ in names}
            for name in JOINT_NAMES
        }
        self._record_snapshot(snapshot)
        return snapshot

    def _record_snapshot(self, snapshot):
        now = self.clock.monotonic()
        statuses = {name: joint["status"] for name, joint in snapshot.items()}
        low = any(joint["voltage"] < MIN_VOLTAGE_RAW for joint in snapshot.values())
        fault_details = None
        if (self.last_statuses is not None and statuses != self.last_statuses) or low != self.low_voltage:
            try:
                fault_details = {"current_raw": self._read("Present_Current")}
            except Exception as exc:
                fault_details = {"current_read_error": str(exc)}
        if self.last_statuses is not None and statuses != self.last_statuses:
            self.telemetry.write("status_changed", previous=self.last_statuses,
                                 current=statuses, snapshot=snapshot, **fault_details)
        if low != self.low_voltage:
            self.telemetry.write("voltage_threshold_changed", low=low,
                                 snapshot=snapshot, **fault_details)
        self.last_statuses = statuses
        self.low_voltage = low
        self.last_snapshot = snapshot
        if not low and all(status == 0 for status in statuses.values()):
            if self.healthy_since is None:
                self.healthy_since = now
        else:
            self.healthy_since = None
        hold = self.hold
        token = hold["token"] if hold is not None else None
        first_motion_sample = token is not None and token != self._last_current_token
        interval = 0.1 if hold is not None else 1.0
        if first_motion_sample or now - self.last_logged_at >= interval:
            details = {"motion": hold["mode"] if hold else None, "snapshot": snapshot}
            if hold is not None:
                details["token"] = token
                if first_motion_sample or now - self._last_current_at >= CURRENT_SAMPLE_INTERVAL_S:
                    try:
                        # This is the servo's raw phase-current register, not
                        # the total current drawn from the shared power supply.
                        details["current_raw"] = self._read("Present_Current")
                    except Exception as exc:
                        details["current_read_error"] = str(exc)
                    self._last_current_at = now
                    self._last_current_token = token
            self.telemetry.write("sample", **details)
            self.last_logged_at = now
        if self.hold is not None and now - self._last_timing_emit >= TIMING_SAMPLE_INTERVAL_S:
            self.telemetry.write("timing_sample", token=self.hold["token"],
                                 mode=self.hold["mode"], timing=self.timing.summary())
            self._last_timing_emit = now

    def _set_error(self, message, *, token=None, timing=None):
        self.error = message
        hold = self.hold
        self.telemetry.write("motion_error", message=message,
                             motion=hold["mode"] if hold else None,
                             token=token if token is not None else (hold["token"] if hold else None),
                             timing=timing if timing is not None else (self.timing.summary() if hold else None),
                             snapshot=self.last_snapshot)

    def _fold_envelope(self, bounds):
        # The manually observed endpoint only permits a way back inside a
        # tighter saved bound. It is never an outward jog destination.
        return folded_bounds(home_bounds(bounds, self.home), self.folded)

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

    def _check_snapshot(self, snapshot, *, need_torque=True, bounds=None, overshoot=0,
                        moving_motor=None, previous_statuses=None):
        for index, name in enumerate(JOINT_NAMES):
            joint = snapshot[name]
            # Voltage is shared across the bus: a sag on any servo must still
            # stop motion. Other faults are local to a servo, so a single-joint
            # jog does not command or depend on a different faulted motor.
            if joint["voltage"] < MIN_VOLTAGE_RAW:
                if self._read("Present_Voltage")[name] < MIN_VOLTAGE_RAW:
                    raise RuntimeError(f"Motor {index + 1} voltage stayed low")
            if joint["status"] & 1:
                # An alarm already present on a different servo may be a
                # latched, local event. Current bus voltage must be healthy;
                # a new alarm during this move still stops it immediately.
                inactive = moving_motor is not None and index + 1 != moving_motor
                already_present = previous_statuses is None or bool(previous_statuses[name] & 1)
                if not inactive or not already_present:
                    raise RuntimeError(f"Motor {index + 1} voltage alarm (servo status {joint['status']})")
            if moving_motor is not None and index + 1 != moving_motor:
                continue
            if joint["status"] != 0:
                raise RuntimeError(f"Motor {index + 1} servo fault {joint['status']}")
            if need_torque and joint["torque"] != 1:
                raise RuntimeError(f"Motor {index + 1} torque is off; use Hold all joints first")
            if joint["temperature"] > 55:
                # A single garbled reading (32 -> 59 -> 32 C within a second was
                # seen during a supply dip) must not stop motion; a real
                # overheat persists on a fresh read.
                if self._read("Present_Temperature")[name] > 55:
                    raise RuntimeError(f"Motor {index + 1} is too hot")
            position = joint["position"]
            if not self.firmware_min[name] <= position <= self.firmware_max[name]:
                raise RuntimeError(f"Motor {index + 1} exceeds firmware position bounds")
            if bounds is not None:
                low, high = bounds.lower[index], bounds.upper[index]
                if not low - overshoot <= position <= high + overshoot:
                    raise RuntimeError(f"Motor {index + 1} measured position {position} exceeds [{low}, {high}]")
            else:
                self._safe_span(index, position)

    def _write_goal(self, motor, tick):
        name = JOINT_NAMES[motor - 1]
        if type(tick) is not int or not self.firmware_min[name] <= tick <= self.firmware_max[name]:
            raise ValueError(f"Motor {motor} target is outside firmware bounds")
        self._mark_first_goal_write()
        self.bus.write("Goal_Position", name, tick, normalize=False, num_retry=2)

    def _mark_first_goal_write(self):
        """Measure start-to-first-goal submission, not motor response latency."""
        if self.hold is not None and self.timing.first_command_ms is None:
            self.timing.record_first_command_ns(
                self.clock.perf_ns() - self.hold["started_perf_ns"]
            )

    def _write_pose_goals(self, goals):
        """Write only joints admitted by the active pose's movement policy."""
        if self.hold is not None and self.movement_mode is MovementMode.SHOULDER_ISOLATED:
            admitted = self.hold["policy_joints"]
            goals = {name: tick for name, tick in goals.items()
                     if JOINT_NAMES.index(name) + 1 in admitted}
        if goals:
            self._mark_first_goal_write()
            self.bus.sync_write("Goal_Position", goals, normalize=False)

    def _hold_measured(self, motor):
        name = JOINT_NAMES[motor - 1]
        if self._read("Torque_Enable")[name] != 1:
            return
        position = self._read("Present_Position")[name]
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

    def _grip_hold_goal(self, hold, position):
        """Keep a closing gripper's measured contact without a large stall gap."""
        low, high, _, _ = self._safe_span(5, position)
        command_goal = hold["target"] if hold["mode"] == "goto" else hold["last_goal"]
        target = max(position - GRIP_HOLD_LEAD, min(position, command_goal))
        return max(low, min(high, target))

    def _stop(self, *, settle=False):
        previous = self.hold
        self.hold = None
        if not previous:
            return
        stop_start_ns = self.clock.perf_ns()
        try:
            if previous["mode"] == "pose":
                self._hold_measured_all(joints=previous.get("policy_joints"))
            elif settle and previous["mode"] in ("jog", "goto"):
                name = JOINT_NAMES[previous["motor"] - 1]
                if self._read("Torque_Enable")[name] == 1:
                    position = self._read("Present_Position")[name]
                    closing_gripper = previous["motor"] == 6 and (
                        previous["direction"] < 0 or
                        (previous["mode"] == "goto" and previous["target"] < previous["start_position"])
                    )
                    if closing_gripper:
                        # A measured-position hold releases the jaws as soon as the
                        # closing command ends. Preserve a modest closing preload,
                        # but never push through a servo fault.
                        healthy = self._read("Status")[name] == 0
                        goal = self._grip_hold_goal(previous, position) if healthy else position
                    elif previous["mode"] == "jog":
                        goal = self._settle_goal(previous, name, position)
                    else:
                        goal = position
                    self._write_goal(previous["motor"], goal)
                    self.telemetry.write("hold_goal", motor=previous["motor"], goal=goal,
                                         measured=position, snapshot=self.last_snapshot)
            else:
                self._hold_measured(previous["motor"])
        finally:
            timing = self.timing.summary()
            timing["stop_command_ms"] = round((self.clock.perf_ns() - stop_start_ns) / 1e6, 3)
            timing["motion_elapsed_ms"] = round(
                (self.clock.perf_ns() - previous["started_perf_ns"]) / 1e6, 3
            )
            self.telemetry.write("motion_stopped", mode=previous["mode"],
                                 motor=previous["motor"], pose=previous.get("pose"),
                                 settled=settle, heartbeats=previous.get("heartbeats"),
                                 heartbeat_age=(self.clock.monotonic() - previous["last_heartbeat"])
                                 if "last_heartbeat" in previous else None,
                                 token=previous.get("token"), timing=timing,
                                 snapshot=self.last_snapshot)

    def _hold_measured_all(self, *, joints=None):
        """Hold selected enabled joints at measured position; keep torque."""
        torque = self._read("Torque_Enable")
        if not any(value == 1 for value in torque.values()):
            return
        positions = self._read("Present_Position")
        goals = {}
        for index, name in enumerate(JOINT_NAMES):
            if joints is not None and index + 1 not in joints:
                continue
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
            if (self.error and "voltage" in self.error.lower() and self.healthy_since is not None
                    and self.clock.monotonic() - self.healthy_since >= 2.0 and self.hold is None):
                self.telemetry.write("voltage_recovered", previous_error=self.error, snapshot=snapshot)
                self.error = None
            motors = []
            for index, (name, label) in enumerate(zip(JOINT_NAMES, LABELS, strict=True)):
                joint = snapshot[name]
                try:
                    low, high, backward, forward = self._safe_span(index, joint["position"])
                except ValueError:
                    low, high, backward, forward = self.bounds.lower[index], self.bounds.upper[index], False, False
                fault = (f"Voltage alarm (status {joint['status']})" if joint["status"] & 1 else
                         "Overload protection (status 32)" if joint["status"] == 32 else
                         f"Servo fault {joint['status']}" if joint["status"] else None)
                enabled = joint["torque"] == 1 and fault is None and joint["voltage"] >= MIN_VOLTAGE_RAW
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
            ready = all(m["motion_enabled"] for m in motors)
            available = {name: self.movement_mode is MovementMode.COORDINATED and ready
                         for name in POSE_NAMES}
            if self.movement_mode is MovementMode.SHOULDER_ISOLATED and ready:
                start_ticks = tuple(snapshot[name]["position"] for name in JOINT_NAMES)
                for name in TAUGHT_MOVES:
                    try:
                        validate_movement_path(self.movement_mode, start_ticks, self.taught[name])
                    except ValueError:
                        pass
                    else:
                        available[name] = bool(self.taught[name])
            return {
                "motors": motors, "holding": self.hold["motor"] if self.hold else None,
                "error": self.error, "motion_enabled": any(m["motion_enabled"] for m in motors),
                "recent_events": list(self.telemetry.recent)[-8:],
                "movement_mode": self.movement_mode.value,
                "single_motor_mode": self.single_motor_mode,
                "pose_enabled": self.movement_mode is MovementMode.COORDINATED and ready,
                "pose_available": available,
                "hold_all_enabled": self.movement_mode is MovementMode.COORDINATED and not all(m["torque"] for m in motors)
                                    and not any(m["fault"] for m in motors),
                "extended_ticks": dict(zip(JOINT_NAMES, self.extended, strict=True)),
                "taught_moves": {name: len(steps) for name, steps in self.taught.items()},
                "taught_speeds": dict(self.taught_speed),
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

    def _require_stopped_motors(self, *, except_motor=None, only_motors=None):
        """Reject a new owner while another servo has pending or reported motion.

        A gripper holding an object can retain its bounded closing preload even
        while stationary. Other joints must be near their last commanded goals.
        This check only reads registers and never changes torque or goals.
        """
        first_positions = None
        for sample in range(2):
            moving = self._read("Moving")
            positions = self._read("Present_Position")
            goals = self._read("Goal_Position")
            for motor, name in enumerate(JOINT_NAMES, 1):
                if motor == except_motor or (only_motors is not None and motor not in only_motors):
                    continue
                tolerance = (SINGLE_MOTOR_GRIP_HOLD_TOLERANCE if motor == 6
                             else POSITION_TOLERANCE)
                if (moving[name] != 0 or abs(goals[name] - positions[name]) > tolerance
                        or (first_positions is not None
                            and abs(positions[name] - first_positions[name]) > 1)):
                    raise ValueError(
                        f"Motor {motor} is still moving or has an unsettled goal; "
                        "wait for it to stop before moving another motor"
                    )
            if sample == 0:
                first_positions = positions
                time.sleep(SINGLE_MOTOR_SETTLE_SAMPLE_SECONDS)

    def set_movement_mode(self, mode: str):
        """Select the movement policy without commanding a motor."""
        selected = parse_movement_mode(mode)
        with self.lock:
            if self.hold is not None:
                raise ValueError("Stop the active movement before changing movement mode")
            if selected is not self.movement_mode:
                self._require_stopped_motors()
            self.movement_mode = selected
            self.telemetry.write("movement_mode_changed", mode=selected.value,
                                 snapshot=self.last_snapshot)
            return {"movement_mode": selected.value}

    def set_single_motor_mode(self, enabled: bool):
        """Legacy boolean route: True selects single-joint, False coordinated."""
        if type(enabled) is not bool:
            raise ValueError("single_motor_mode must be a boolean")
        self.set_movement_mode("single_joint" if enabled else "coordinated")
        return {"single_motor_mode": enabled}

    def _check_jog_policy(self, motor):
        if self.movement_mode is MovementMode.SINGLE_JOINT:
            self._require_stopped_motors(except_motor=motor)
        elif self.movement_mode is MovementMode.SHOULDER_ISOLATED:
            if motor == 2:
                self._require_stopped_motors(except_motor=2)
            else:
                self._require_stopped_motors(only_motors={2})

    def start(self, motor, direction):
        with self.lock:
            if self.hold is not None and self.hold["mode"] == "pose":
                raise ValueError("A pose move is in progress; cancel it before jogging")
            self._stop()
            self._check_jog_policy(motor)
            snapshot = self._snapshot()
            self._check_snapshot(snapshot, moving_motor=motor)
            name = JOINT_NAMES[motor - 1]
            position = snapshot[name]["position"]
            _, _, backward, forward = self._safe_span(motor - 1, position)
            if not (forward if direction > 0 else backward):
                raise ValueError(f"Motor {motor} is at that limit; move in the other direction")
            self.hold = self._new_hold(motor, "jog", position, direction=direction)
            self.hold["statuses"] = {name: joint["status"] for name, joint in snapshot.items()}
            self.error = None
            self.telemetry.write("motion_started", mode="jog", motor=motor,
                                 direction=direction, token=self.hold["token"], snapshot=snapshot)
            return self.hold["token"]

    def go(self, target):
        with self.lock:
            if self.hold is not None and self.hold["mode"] == "pose":
                raise ValueError("A pose move is in progress; cancel it before using go-to-tick")
            if type(target) is not int:
                raise ValueError("Target must be an integer tick")
            self._stop()
            self._check_jog_policy(6)
            snapshot = self._snapshot()
            self._check_snapshot(snapshot, moving_motor=6)
            position = snapshot["gripper"]["position"]
            low, high, backward, forward = self._safe_span(5, position)
            if not low <= target <= high or (target > position and not forward) or (target < position and not backward):
                raise ValueError(f"Gripper target must be within {low}–{high} and toward an allowed direction")
            self.hold = self._new_hold(6, "goto", position, target=target)
            self.hold["statuses"] = {name: joint["status"] for name, joint in snapshot.items()}
            self.error = None
            self.telemetry.write("motion_started", mode="goto", motor=6,
                                 target=target, token=self.hold["token"], snapshot=snapshot)
            return self.hold["token"]

    def pose_start(self, pose):
        with self.lock:
            if self.movement_mode is MovementMode.SINGLE_JOINT:
                raise ValueError("Multi-joint poses are unavailable in single-motor mode; turn it off first")
            if self.hold is not None:
                raise ValueError("A movement is already active; cancel it before starting a pose move")
            if pose not in POSE_NAMES:
                raise ValueError(f"pose must be one of {', '.join(POSE_NAMES)}; folded pose is unavailable with the wrist camera")
            if self.movement_mode is MovementMode.SHOULDER_ISOLATED and pose in ("home", "extend", "dance"):
                raise ValueError(f"{pose} needs a coordinated path; select coordinated mode first")
            taught = pose in TAUGHT_MOVES
            if taught and not self.taught[pose]:
                raise ValueError(f"{pose} has no saved positions yet; teach it first")
            dance_frames = None
            if pose == "dance":
                _, dance_frames = build_dance_plan(self.bounds, DANCE_SCALE, DANCE_FPS)
                for index, name in enumerate(JOINT_NAMES):
                    low = min(frame[index] for frame in dance_frames)
                    high = max(frame[index] for frame in dance_frames)
                    if low < self.firmware_min[name] or high > self.firmware_max[name]:
                        raise RuntimeError(f"Motor {index + 1} dance exceeds firmware bounds")
            target = (self.taught[pose][-1] if taught else
                      self.extended if pose == "extend" else self.home)
            envelope = self._fold_envelope(self.bounds)
            for step in (self.taught[pose] if taught else (target,)):
                validate_pose(step, envelope)
            modes = self._read("Operating_Mode")
            if any(modes[name] != 0 for name in JOINT_NAMES):
                raise RuntimeError("All motors must be in position mode 0; no modes were changed")
            for index, name in enumerate(JOINT_NAMES):
                if not self.firmware_min[name] <= target[index] <= self.firmware_max[name]:
                    raise RuntimeError(f"Motor {index + 1} target is outside firmware bounds")
            snapshot = self._snapshot()
            self._check_snapshot(snapshot, bounds=envelope, overshoot=POSE_MEASURED_OVERSHOOT)
            self.error = None
            now = self.clock.monotonic()
            start_ticks = tuple(snapshot[name]["position"] for name in JOINT_NAMES)
            # The Home↔Extend corridor must remain synchronized. Moving the
            # shoulder ahead of the elbow can dip the wrist camera below Home.
            coordinated = pose == "extend" or (
                pose in ("home", "dance") and self._on_upright_corridor(start_ticks)
            )
            staged = pose == "home" and needs_staged_home(start_ticks, target)
            if taught:
                trajectory = []
                vmax, amax = taught_limits(self.taught_speed[pose])
                for begin, end in taught_segments(start_ticks, self.taught[pose],
                                                  carrying_move=pose in TAUGHT_CARRYING):
                    trajectory += quintic_segment(begin, end, vmax, amax, EXTEND_FPS)
            elif staged:
                trajectory = []
                for begin, end in staged_segments(start_ticks, target, HOME_ORDER):
                    trajectory += quintic_segment(begin, end, TAUGHT_VMAX, TAUGHT_AMAX, EXTEND_FPS)
            elif coordinated:
                trajectory = quintic_segment(start_ticks, target, EXTEND_VMAX, EXTEND_AMAX, EXTEND_FPS)
            else:
                trajectory = None
            if trajectory is not None:
                for frame in trajectory:
                    validate_pose(frame, envelope)
                    if any(not self.firmware_min[name] <= frame[index] <= self.firmware_max[name]
                           for index, name in enumerate(JOINT_NAMES)):
                        raise RuntimeError(f"{pose} trajectory exceeds a firmware limit")
            policy_joints = frozenset(range(1, 7))
            if self.movement_mode is MovementMode.SHOULDER_ISOLATED:
                policy_joints = validate_movement_path(
                    self.movement_mode, start_ticks, trajectory if trajectory is not None else (target,)
                )
                self._require_stopped_motors(only_motors=set(range(1, 7)) - policy_joints)
            self.hold = {
                "token": uuid.uuid4().hex, "motor": None, "mode": "pose", "pose": pose,
                "stage": "home" if pose == "dance" else None,
                "dance_frames": dance_frames,
                "trajectory": trajectory, "frame_index": 0, "staged": staged,
                "policy_joints": policy_joints,
                "target": target, "bounds": envelope,
                "start_positions": {name: snapshot[name]["position"] for name in JOINT_NAMES},
                "measured": {name: snapshot[name]["position"] for name in JOINT_NAMES},
                "progress_position": {name: snapshot[name]["position"] for name in JOINT_NAMES},
                "progress_time": {name: now for name in JOINT_NAMES},
                "high_load": {name: 0 for name in JOINT_NAMES},
                "trim_pending": set(),
                "writes": 0, "progress": 0.0,
                "started_perf_ns": self.clock.perf_ns(),
                "next": now, "last_update": now,
                "expires": now + max(POSE_TIMEOUT_SECONDS,
                                     len(trajectory) / EXTEND_FPS * (3 if taught or staged else 1) + 30
                                     if trajectory else 0),
            }
            self.timing.reset()
            self.telemetry.write("motion_started", mode="pose", pose=pose,
                                 token=self.hold["token"], target=list(target), snapshot=snapshot)
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
        if hold.get("trajectory") is not None:
            self._advance_upright_trajectory(now)
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
            self._write_pose_goals(dict(zip(JOINT_NAMES, targets, strict=True)))
            if hold["pose"] == "dance" and hold["stage"] == "home":
                hold["stage"] = "entry"
                hold["target"] = DANCE_CENTER
                hold["start_positions"] = positions
                hold["progress_position"] = positions.copy()
                hold["progress_time"] = {name: now for name in JOINT_NAMES}
                hold["trim_pending"] = set()
                hold["next"] = now + POSE_STEP_SECONDS
                hold["expires"] = now + 60
                return
            if hold["pose"] == "dance" and hold["stage"] == "entry":
                hold["stage"] = "playback"
                hold["bounds"] = self.bounds
                hold["frame_index"] = 0
                hold["last_goals"] = dict(zip(JOINT_NAMES, DANCE_CENTER, strict=True))
                hold["temperatures"] = {name: snapshot[name]["temperature"] for name in JOINT_NAMES}
                hold["temperature_at"] = now
                hold["progress"] = .05
                hold["next"] = now + 1 / DANCE_FPS
                hold["expires"] = now + max(180, len(hold["dance_frames"]) * .15)
                return
            self._complete_pose(snapshot)
            return
        goals = {}
        trimming = False
        for index, name in enumerate(JOINT_NAMES):
            load = abs(snapshot[name]["load"])
            hold["high_load"][name] = hold["high_load"][name] + 1 if load >= MAX_LOAD else 0
            if hold["high_load"][name] >= 3:
                raise RuntimeError(f"Motor {index + 1} sustained high load {load}")
            position = positions[name]
            if abs(position - hold["progress_position"][name]) >= POSE_STALL_TICKS:
                hold["progress_position"][name] = position
                hold["progress_time"][name] = now
            elif abs(errors[name]) > tolerance and now - hold["progress_time"][name] >= POSE_STALL_SECONDS:
                raise RuntimeError(f"Motor {index + 1} stalled at {position}")
            direction = 1 if errors[name] > 0 else -1
            if hold["pose"] in ("home", "dance") and tolerance < abs(errors[name]) <= HOME_TRIM_OFFSET:
                trimming = True
                if name in hold["trim_pending"]:
                    goal = targets[index]
                    hold["trim_pending"].remove(name)
                else:
                    goal = targets[index] + direction * HOME_TRIM_OFFSET
                    if name == "elbow_flex":
                        goal = min(goal, self.home[index])
                    hold["trim_pending"].add(name)
            else:
                hold["trim_pending"].discard(name)
                goal = position + direction * min(abs(errors[name]), POSE_MAX_LEAD)
            goal = max(envelope.lower[index], min(envelope.upper[index], goal))
            goal = max(self.firmware_min[name], min(self.firmware_max[name], goal))
            if (direction > 0 and goal < position) or (direction < 0 and goal > position):
                goal = position
            goals[name] = int(goal)
        self._write_pose_goals(goals)
        hold["writes"] += 1
        hold["last_update"] = now
        hold["next"] = now + (HOME_TRIM_PERIOD if trimming else POSE_STEP_SECONDS)

    def _advance_dance(self, now):
        hold = self.hold
        fields = {label: self._read(register) for label, register in (
            ("position", "Present_Position"), ("status", "Status"),
            ("torque", "Torque_Enable"), ("voltage", "Present_Voltage"),
            ("load", "Present_Load"),
        )}
        if now - hold["temperature_at"] >= 1.0:
            hold["temperatures"] = self._read("Present_Temperature")
            hold["temperature_at"] = now
        snapshot = {
            name: {**{label: values[name] for label, values in fields.items()},
                   "temperature": hold["temperatures"][name]}
            for name in JOINT_NAMES
        }
        self._record_snapshot(snapshot)
        self._check_snapshot(snapshot, bounds=self.bounds, overshoot=POSE_MEASURED_OVERSHOOT)
        positions = {name: snapshot[name]["position"] for name in JOINT_NAMES}
        hold["measured"] = positions
        for name in JOINT_NAMES:
            load = abs(snapshot[name]["load"])
            hold["high_load"][name] = hold["high_load"][name] + 1 if load >= MAX_LOAD else 0
            if hold["high_load"][name] >= 3:
                raise RuntimeError(f"{name} sustained high load {load}")
            if abs(positions[name] - hold["progress_position"][name]) >= POSE_STALL_TICKS:
                hold["progress_position"][name] = positions[name]
                hold["progress_time"][name] = now
            elif (abs(hold["last_goals"][name] - positions[name]) > 45
                  and now - hold["progress_time"][name] >= DANCE_STALL_SECONDS):
                raise RuntimeError(f"{name} stalled at {positions[name]}")
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
        self._write_pose_goals(goals)
        hold["last_goals"] = goals
        if not behind:
            hold["frame_index"] += 1
        hold["progress"] = .05 + .90 * hold["frame_index"] / len(hold["dance_frames"])
        hold["next"] = now + 1 / DANCE_FPS
        if hold["frame_index"] == len(hold["dance_frames"]):
            hold["stage"] = "finish"
            hold["target"] = self.home
            hold["bounds"] = self._fold_envelope(self.bounds)
            hold["start_positions"] = positions
            hold["progress_position"] = positions.copy()
            hold["progress_time"] = {name: now for name in JOINT_NAMES}
            hold["trim_pending"] = set()
            hold["expires"] = now + POSE_TIMEOUT_SECONDS
            hold["next"] = now + POSE_STEP_SECONDS

    def _new_hold(self, motor, mode, position, *, direction=0, target=None):
        now = self.clock.monotonic()
        self.timing.reset()
        return {
            "token": uuid.uuid4().hex, "motor": motor, "mode": mode,
            "direction": direction, "target": position if target is None else target,
            "start_position": position,
            "command": float(position), "velocity": 0.0, "last_goal": position,
            "last_position": position, "progress_at": now, "last_update": now,
            "expires": now + LEASE_SECONDS if mode == "jog" else now + 35,
            "high_load_count": 0,
            "heartbeats": 0, "last_heartbeat": now,
            "next": now,
            "started_perf_ns": self.clock.perf_ns(),
        }

    def heartbeat(self, token):
        with self.lock:
            if not self.hold or self.hold["token"] != token or self.hold["mode"] != "jog":
                raise ValueError("Hold ended")
            now = self.clock.monotonic()
            self.hold["expires"] = now + LEASE_SECONDS
            self.hold["last_heartbeat"] = now
            self.hold["heartbeats"] += 1

    def stop(self, token=None):
        with self.lock:
            if self.hold and (token is None or token == self.hold["token"]):
                self._stop(settle=True)

    def _advance(self, now):
        hold = self.hold
        if hold is None or now < hold["next"]:
            return
        cycle_start_ns = self.clock.perf_ns()
        read_start_ns = self.timing.read.total_ns
        write_start_ns = self.timing.write.total_ns
        self.timing.record_lag_s(now - hold["next"])
        try:
            if now >= hold["expires"]:
                timing = self.timing.summary()
                token = hold["token"]
                self._stop(settle=hold["mode"] == "jog" and hold["motor"] == 6
                           and hold["direction"] < 0)
                self._set_error("Movement timed out and stopped", token=token, timing=timing)
                return
            if hold["mode"] == "pose":
                self._advance_pose(now)
                return
            snapshot = self._snapshot()
            self._check_snapshot(snapshot, moving_motor=hold["motor"],
                                 previous_statuses=hold["statuses"])
            hold["statuses"] = {name: joint["status"] for name, joint in snapshot.items()}
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
                    self._stop(settle=True)
                    return
                hold["target"] += hold["direction"] * BASE_TICKS_PER_SECOND * self.settings[str(index + 1)]["scale"] * dt
                desired = hold["target"]
            else:
                desired = hold["target"]
                if abs(position - desired) <= POSITION_TOLERANCE:
                    self._stop(settle=hold["motor"] == 6
                               and hold["target"] < hold["start_position"])
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
        finally:
            end_ns = self.clock.perf_ns()
            cycle_ns = max(0, end_ns - cycle_start_ns)
            bus_ns = (self.timing.read.total_ns - read_start_ns) + (
                self.timing.write.total_ns - write_start_ns
            )
            self.timing.record_cycle_ns(cycle_ns)
            self.timing.record_compute_ns(max(0, cycle_ns - bus_ns))

    def _worker(self):
        while not self.closed:
            active = self.hold
            time.sleep(0.005 if active is not None and active.get("stage") == "playback" else 0.02)
            with self.lock:
                if self.hold is None:
                    now = self.clock.monotonic()
                    if now >= self.next_idle_sample:
                        self.next_idle_sample = now + 1.0
                        try:
                            self._snapshot()
                        except Exception as exc:
                            self.telemetry.write("read_error", message=str(exc))
                            logging.exception("Idle motor telemetry read failed")
                    continue
                try:
                    self._advance(self.clock.monotonic())
                except Exception as exc:
                    logging.exception("Panel movement stopped")
                    self._set_error(str(exc))
                    try:
                        self._stop()
                    except Exception:
                        logging.exception("Could not hold measured position")
                        self.hold = None

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
            if self.movement_mode is MovementMode.SINGLE_JOINT:
                raise ValueError("Hold all joints is unavailable in single-motor mode; turn it off first")
            if self.movement_mode is MovementMode.SHOULDER_ISOLATED:
                raise ValueError("Hold all joints requires coordinated mode")
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
            if any(value != 1 for value in self._read("Torque_Enable").values()):
                raise RuntimeError("One or more motors did not enable torque")
            return {"ok": True}

    def release(self):
        raise ValueError("Torque release is unavailable in this dashboard")

    def close(self):
        did_close = False
        try:
            with self.lock:
                if self.closed:
                    return
                self.closed = True
                did_close = True
                try:
                    self._stop()
                finally:
                    try:
                        self.bus.disconnect(disable_torque=False)
                    finally:
                        if self._bus_lock is not None:
                            self._bus_lock.release()
        finally:
            if did_close:
                try:
                    self.worker.join(timeout=1)
                finally:
                    try:
                        self.telemetry.write("controller_stopped", snapshot=self.last_snapshot)
                    finally:
                        self.telemetry.close()

    def _on_upright_corridor(self, ticks):
        fractions = []
        for index in (1, 2):
            span = self.extended[index] - self.home[index]
            fractions.append((ticks[index] - self.home[index]) / span)
        return all(-0.08 <= fraction <= 1.08 for fraction in fractions) and abs(fractions[0] - fractions[1]) <= 0.15

    def teach_add(self, move):
        """Save the arm's measured position as the next step of a taught move."""
        with self.lock:
            if move not in TAUGHT_MOVES:
                raise ValueError(f"move must be one of {', '.join(TAUGHT_MOVES)}")
            if self.hold is not None:
                raise ValueError("Wait for the arm to stop before saving a position")
            if len(self.taught[move]) >= TAUGHT_MAX_STEPS:
                raise ValueError(f"{move} already has {TAUGHT_MAX_STEPS} steps; clear it to start over")
            envelope = self._fold_envelope(self.bounds)
            snapshot = self._snapshot()
            self._check_snapshot(snapshot, bounds=envelope)
            step = tuple(int(snapshot[name]["position"]) for name in JOINT_NAMES)
            validate_pose(step, envelope)
            self.taught[move].append(step)
            self._save_taught()
            self.telemetry.write("taught_step_saved", move=move, step=list(step),
                                 steps=len(self.taught[move]))
            return {"move": move, "steps": len(self.taught[move]), "saved": list(step)}

    def teach_clear(self, move):
        with self.lock:
            if move not in TAUGHT_MOVES:
                raise ValueError(f"move must be one of {', '.join(TAUGHT_MOVES)}")
            if self.hold is not None and self.hold.get("pose") == move:
                raise ValueError("Stop this move before clearing it")
            self.taught[move] = []
            self._save_taught()
            self.telemetry.write("taught_move_cleared", move=move)
            return {"move": move, "steps": 0}

    def teach_speed(self, move, speed):
        """Choose the playback speed of a taught move."""
        with self.lock:
            if move not in TAUGHT_MOVES:
                raise ValueError(f"move must be one of {', '.join(TAUGHT_MOVES)}")
            allowed = TAUGHT_MOVE_SPEEDS.get(move, TAUGHT_SPEEDS)
            if type(speed) not in (int, float) or float(speed) not in allowed:
                raise ValueError("speed must be " + ", ".join(f"{value:g}" for value in allowed))
            if self.hold is not None and self.hold.get("pose") == move:
                raise ValueError("Stop this move before changing its speed")
            self.taught_speed[move] = float(speed)
            self._save_taught()
            self.telemetry.write("taught_speed_set", move=move, speed=float(speed))
            return {"move": move, "speed": float(speed)}

    def _save_taught(self):
        data = {name: [list(step) for step in steps] for name, steps in self.taught.items()}
        data["speeds"] = dict(self.taught_speed)
        temporary = self.taught_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.taught_path)

    def _complete_pose(self, snapshot):
        previous = self.hold
        self.hold = None
        timing = self.timing.summary()
        timing["settle_ms"] = round(
            (self.clock.perf_ns() - previous["started_perf_ns"]) / 1e6, 3
        )
        self.telemetry.write("motion_completed", mode="pose", pose=previous["pose"],
                             token=previous["token"], timing=timing,
                             snapshot=snapshot)

    def _advance_upright_trajectory(self, now):
        """Track one coordinated Home/Extend segment using encoder feedback."""
        hold = self.hold
        snapshot = self._snapshot()
        envelope = hold["bounds"]
        self._check_snapshot(snapshot, bounds=envelope, overshoot=POSE_MEASURED_OVERSHOOT)
        positions = {name: snapshot[name]["position"] for name in JOINT_NAMES}
        hold["measured"] = positions
        frames = hold["trajectory"]
        frame_index = min(hold["frame_index"], len(frames) - 1)
        frame = frames[frame_index]
        goals = {}
        behind = False
        # Staged Home uses the same loaded-joint tracking as taught moves.
        taught = hold["pose"] in TAUGHT_MOVES or hold.get("staged", False)
        if hold.get("staged"):
            leads = (POSE_MAX_LEAD,) * 6  # Home's normal limit, one joint at a time.
        else:
            leads = TAUGHT_MAX_LEAD if taught else (EXTEND_MAX_LEAD,) * 6
        easing = False
        if taught:
            low_voltage = min(snapshot[name]["voltage"] for name in JOINT_NAMES)
            if "rest_voltage" not in hold:
                hold["rest_voltage"] = low_voltage
            ceiling = HOME_EASE_RAW if hold["pose"] == "home" else VOLTAGE_EASE_RAW
            threshold = max(MIN_VOLTAGE_RAW + 1,
                            min(ceiling, hold["rest_voltage"] - VOLTAGE_EASE_DROP))
            easing = low_voltage < threshold
            ease = hold.get("ease", 1.0)
            ease = (max(TAUGHT_EASE_MIN, ease * 0.5) if easing
                    else min(1.0, ease + TAUGHT_EASE_RECOVERY))
            if easing and hold.get("ease", 1.0) == 1.0:
                self.telemetry.write("voltage_ease", pose=hold["pose"], voltage=low_voltage,
                                     snapshot=snapshot)
            hold["ease"] = ease
        full_leads = leads
        if taught:
            leads = tuple(max(10, round(lead * hold["ease"])) for lead in leads)
        last_frame = frame_index == len(frames) - 1
        settled = hold.setdefault("settled", set())
        for index, name in enumerate(JOINT_NAMES):
            position = positions[name]
            load = abs(snapshot[name]["load"])
            hold["high_load"][name] = hold["high_load"][name] + 1 if load >= MAX_LOAD else 0
            if hold["high_load"][name] >= 3:
                raise RuntimeError(f"Motor {index + 1} sustained high load {load}")
            if taught and name == "gripper" and frame[index] < position - POSE_TOLERANCE:
                # Closing jaws may stop on the object. Keep a small squeeze
                # instead of treating the contact as a stall.
                goals[name] = position - GRIP_HOLD_LEAD
                continue
            if (abs(position - hold["progress_position"][name]) >= POSE_STALL_TICKS
                    or (taught and hold["ease"] < 1.0)):
                # Progress, or a deliberately eased push waiting for voltage to
                # recover; neither is a stall.
                hold["progress_position"][name] = position
                hold["progress_time"][name] = now
            elif (abs(frame[index] - position) > POSE_TOLERANCE
                  and now - hold["progress_time"][name] >= POSE_STALL_SECONDS):
                gap = abs(frame[index] - position)
                # A loaded taught move only stalls when the full push cannot
                # move it, or when it stops well short of the last step.
                if not taught or gap > (TAUGHT_SETTLE_TOLERANCE if last_frame else full_leads[index]):
                    raise RuntimeError(f"Motor {index + 1} stalled at {position}")
                if last_frame:
                    settled.add(name)
            distance = frame[index] - position
            lead = leads[index]
            behind |= abs(distance) > lead
            goals[name] = position + max(-lead, min(lead, distance))
        validate_pose(tuple(goals[name] for name in JOINT_NAMES), envelope)
        for name in JOINT_NAMES:
            if not self.firmware_min[name] <= goals[name] <= self.firmware_max[name]:
                raise RuntimeError(f"{name} trajectory goal exceeds firmware limits")
        if frame_index == len(frames) - 1 and all(
            abs(hold["target"][index] - positions[name]) <= POSE_TOLERANCE
            or name in settled
            or (taught and name == "gripper" and hold["target"][index] < positions[name])
            for index, name in enumerate(JOINT_NAMES)
        ):
            # Keep the exact validated pose as the holding goal. Never drop torque.
            final = dict(zip(JOINT_NAMES, hold["target"], strict=True))
            if taught:
                final["gripper"] = max(final["gripper"], positions["gripper"] - GRIP_HOLD_LEAD)
            self._write_pose_goals(final)
            if hold["pose"] == "dance" and hold["stage"] == "home":
                hold["trajectory"] = None
                hold["stage"] = "entry"
                hold["target"] = DANCE_CENTER
                hold["start_positions"] = positions
                hold["progress_position"] = positions.copy()
                hold["progress_time"] = {name: now for name in JOINT_NAMES}
                hold["trim_pending"] = set()
                hold["progress"] = .05
                hold["next"] = now + POSE_STEP_SECONDS
                hold["expires"] = now + 60
            elif hold.get("staged"):
                # The joints are near Home. Finish with Home's usual small
                # trim so it lands as precisely as before.
                hold["trajectory"] = None
                hold["staged"] = False
                hold["start_positions"] = positions
                hold["progress_position"] = positions.copy()
                hold["progress_time"] = {name: now for name in JOINT_NAMES}
                hold["trim_pending"] = set()
                hold["next"] = now + POSE_STEP_SECONDS
                hold["expires"] = now + POSE_TIMEOUT_SECONDS
            else:
                hold["progress"] = 1.0
                self._complete_pose(snapshot)
            return
        self._write_pose_goals(goals)
        hold["writes"] += 1
        hold["last_update"] = now
        if not behind and hold["frame_index"] < len(frames) - 1:
            hold["frame_index"] += 1
        fraction = hold["frame_index"] / max(1, len(frames) - 1)
        hold["progress"] = .05 * fraction if hold["pose"] == "dance" else fraction
        hold["next"] = now + 1 / EXTEND_FPS
