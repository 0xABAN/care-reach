"""Explicit, Pi-only wrist-flex shake from the saved six-joint Home pose.

Default mode is an offline preview. Stop the dashboard service and give this
command sole ownership of the URT-2 before using --execute. The 150-tick,
0.5 Hz waveform is an aggressive bounded trial, not a physical speed maximum.
Real-bus runs verify sole port ownership with ``fuser`` and then hold the same
exclusive ``BusLock`` as the dashboard for the full connection lifetime.
"""

from __future__ import annotations

import math
import subprocess
import time
from typing import Any, Callable

from motion_core import JOINT_NAMES, SafetyBounds, validate_pose
from park import HOME

from .bus import BusOwnership
from .timing import MonotonicClock, MotionTiming, TimedBus

WRIST = "wrist_flex"
WRIST_INDEX = 3
DURATION_S = 15.0
PERIOD_S = 0.05
AMPLITUDE_TICKS = 150
FREQUENCY_HZ = 0.5
RAMP_S = 0.5
HOME_TOLERANCE = 30
LEAD_TICKS = 65
MAX_TRACKING_ERROR = 110
MAX_LOAD = 900
MIN_VOLTAGE_RAW = 48
TRIAL_GOAL_SPEED = 20
TRIAL_ACCELERATION = 10
TELEMETRY_PERIOD_S = 0.5
PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078563-if00"


class ShakeSafetyError(RuntimeError):
    """A prerequisite or live feedback guard stopped the wrist motion."""


def waveform(elapsed_s: float) -> int:
    """Signed wrist offset with zero position and speed at both ends."""
    if not 0 <= elapsed_s <= DURATION_S:
        raise ValueError("time outside shake duration")
    ramp = min(1.0, elapsed_s / RAMP_S, (DURATION_S - elapsed_s) / RAMP_S)
    envelope = ramp * ramp * (3 - 2 * ramp)
    return round(AMPLITUDE_TICKS * envelope * math.sin(2 * math.pi * FREQUENCY_HZ * elapsed_s))


def require_free_port(port: str) -> None:
    """Fail closed if a different process owns the serial device."""
    try:
        result = subprocess.run(["fuser", "-s", port], check=False, capture_output=True)
    except OSError as exc:
        raise ShakeSafetyError("cannot verify sole URT-2 ownership (fuser unavailable)") from exc
    if result.returncode == 0:
        raise ShakeSafetyError("URT-2 is already owned; stop its current controller first")
    if result.returncode != 1:
        raise ShakeSafetyError(f"cannot verify sole URT-2 ownership (fuser exit {result.returncode})")


class WristShake:
    def __init__(self, port: str, bounds: SafetyBounds, *, bus_factory: Callable[[str], Any] | None = None,
                 bus_lock=None, clock: Any = time, owner_check: Callable[[str], None] = require_free_port) -> None:
        self.port = port
        self.bounds = bounds
        self.bus_factory = bus_factory
        self.clock = clock
        self.owner_check = owner_check
        self._ownership = BusOwnership(port, bus_factory=bus_factory, bus_lock=bus_lock)
        self._perf = MonotonicClock(monotonic=clock.monotonic)
        self.timing = MotionTiming()
        self.bus: Any | None = None
        self.connected = False
        self.started = False
        self.last_positions: dict[str, int] | None = None
        self.firmware_min: dict[str, int] | None = None
        self.firmware_max: dict[str, int] | None = None
        self.saved_tuning: tuple[int, int] | None = None

    def _read(self, register: str) -> dict[str, int]:
        assert self.bus is not None
        values = self.bus.sync_read(register, normalize=False, num_retry=2)
        if set(values) != set(JOINT_NAMES):
            raise ShakeSafetyError(f"incomplete {register} read")
        return {name: int(values[name]) for name in JOINT_NAMES}

    def _read_one(self, register: str) -> int:
        assert self.bus is not None
        return int(self.bus.read(register, WRIST, normalize=False, num_retry=2))

    def _write_one(self, register: str, value: int) -> None:
        assert self.bus is not None
        self.bus.write(register, WRIST, value, normalize=False, num_retry=2)

    def _set_trial_tuning(self) -> None:
        # Both values are captured before the first mutation, so a partial
        # write can still be undone in finally.
        self.saved_tuning = (self._read_one("Goal_Speed"), self._read_one("Acceleration"))
        for register, value in (("Goal_Speed", TRIAL_GOAL_SPEED),
                                ("Acceleration", TRIAL_ACCELERATION)):
            self._write_one(register, value)
            if self._read_one(register) != value:
                raise ShakeSafetyError(f"{WRIST}: {register} readback mismatch")

    def _restore_tuning(self) -> None:
        if self.saved_tuning is None:
            return
        errors = []
        for register, value in zip(("Goal_Speed", "Acceleration"), self.saved_tuning, strict=True):
            try:
                self._write_one(register, value)
                if self._read_one(register) != value:
                    errors.append(f"{register} readback mismatch")
            except Exception as exc:
                errors.append(f"{register}: {exc}")
        if errors:
            raise ShakeSafetyError(f"{WRIST}: tuning restore failed: {'; '.join(errors)}")

    def _sample(self) -> dict[str, dict[str, int]]:
        fields = {field: self._read(register) for field, register in (
            ("position", "Present_Position"), ("torque", "Torque_Enable"),
            ("status", "Status"), ("voltage", "Present_Voltage"),
            ("load", "Present_Load"),
        )}
        self.last_positions = fields["position"]
        return {name: {field: values[name] for field, values in fields.items()} for name in JOINT_NAMES}

    def _check(self, state: dict[str, dict[str, int]], *, initial: bool = False) -> None:
        for i, name in enumerate(JOINT_NAMES):
            joint = state[name]
            pos = joint["position"]
            if joint["torque"] != 1:
                raise ShakeSafetyError(f"{name}: torque disabled")
            if joint["status"] != 0:
                raise ShakeSafetyError(f"{name}: servo fault {joint['status']}")
            if joint["voltage"] < MIN_VOLTAGE_RAW and self._read("Present_Voltage")[name] < MIN_VOLTAGE_RAW:
                raise ShakeSafetyError(f"{name}: sustained low voltage")
            if not self.bounds.lower[i] <= pos <= self.bounds.upper[i]:
                raise ShakeSafetyError(f"{name}: outside software limits")
            assert self.firmware_min is not None and self.firmware_max is not None
            if not self.firmware_min[name] <= pos <= self.firmware_max[name]:
                raise ShakeSafetyError(f"{name}: outside firmware limits")
            if abs(joint["load"]) >= MAX_LOAD:
                raise ShakeSafetyError(f"{name}: high load {joint['load']}")
            if initial and abs(pos - HOME[i]) > HOME_TOLERANCE:
                raise ShakeSafetyError(f"{name}: not at saved Home ({pos} versus {HOME[i]})")
            if not initial and i != WRIST_INDEX and abs(pos - HOME[i]) > HOME_TOLERANCE:
                raise ShakeSafetyError(f"{name}: drifted away from Home")

    def _hold(self) -> None:
        if not self.started or self.bus is None:
            return
        try:
            positions = self._read("Present_Position")
        except Exception:
            positions = self.last_positions
        if positions is None:
            return
        assert self.firmware_min is not None and self.firmware_max is not None
        goals = {
            name: max(max(self.bounds.lower[i], self.firmware_min[name]),
                      min(min(self.bounds.upper[i], self.firmware_max[name]), positions[name]))
            for i, name in enumerate(JOINT_NAMES)
        }
        try:
            self.bus.sync_write("Goal_Position", goals, normalize=False)
        except Exception:
            pass

    def run(self) -> dict[str, Any]:
        if self.bus is not None:
            raise RuntimeError("WristShake is single-use")
        validate_pose(HOME, self.bounds)
        lo, hi = self.bounds.lower[WRIST_INDEX], self.bounds.upper[WRIST_INDEX]
        if not lo + 100 <= HOME[WRIST_INDEX] - AMPLITUDE_TICKS or not HOME[WRIST_INDEX] + AMPLITUDE_TICKS <= hi - 100:
            raise ShakeSafetyError("wrist envelope lacks software-limit margin")
        self.owner_check(self.port)
        writes = 0
        try:
            self.bus = TimedBus(self._ownership.connect(), self._perf, self.timing)
            self.connected = True
            self.firmware_min = self._read("Min_Position_Limit")
            self.firmware_max = self._read("Max_Position_Limit")
            modes = self._read("Operating_Mode")
            if any(modes[name] != 0 for name in JOINT_NAMES):
                raise ShakeSafetyError("all motors must be in position mode 0")
            if not self.firmware_min[WRIST] + 100 <= HOME[WRIST_INDEX] - AMPLITUDE_TICKS or not HOME[WRIST_INDEX] + AMPLITUDE_TICKS <= self.firmware_max[WRIST] - 100:
                raise ShakeSafetyError("wrist envelope lacks firmware-limit margin")
            initial = self._sample()
            self._check(initial, initial=True)
            # Hold the other five at their measured Home positions throughout.
            home_hold = {name: initial[name]["position"] for name in JOINT_NAMES if name != WRIST}
            self.started = True
            self.bus.sync_write("Goal_Position", {**home_hold, WRIST: initial[WRIST]["position"]}, normalize=False)
            self._set_trial_tuning()
            start = self.clock.monotonic()
            next_sample = start
            next_log = 0.0
            while True:
                cycle_started_ns = self._perf.perf_ns()
                read_start_ns = self.timing.read.total_ns
                write_start_ns = self.timing.write.total_ns
                state = self._sample()
                self._check(state)
                now = self.clock.monotonic()
                elapsed = now - start
                if elapsed >= DURATION_S:
                    break
                desired = HOME[WRIST_INDEX] + waveform(elapsed)
                measured = state[WRIST]["position"]
                if elapsed + 1e-9 >= next_log:
                    current = self._read_one("Present_Current")
                    print(f"shake t={elapsed:.2f}s desired={desired} measured={measured} "
                          f"voltage={state[WRIST]['voltage'] / 10:.1f}V "
                          f"load={state[WRIST]['load']} current_raw={current}", flush=True)
                    next_log = (math.floor(elapsed / TELEMETRY_PERIOD_S) + 1) * TELEMETRY_PERIOD_S
                if abs(desired - measured) > MAX_TRACKING_ERROR:
                    raise ShakeSafetyError(f"{WRIST}: tracking error {abs(desired - measured)} ticks")
                goal = measured + max(-LEAD_TICKS, min(LEAD_TICKS, desired - measured))
                goal = max(lo, min(hi, goal))
                assert self.firmware_min is not None and self.firmware_max is not None
                goal = max(self.firmware_min[WRIST], min(self.firmware_max[WRIST], goal))
                self.bus.sync_write("Goal_Position", {WRIST: goal}, normalize=False)
                writes += 1
                end_ns = self._perf.perf_ns()
                cycle_ns = max(0, end_ns - cycle_started_ns)
                bus_ns = (self.timing.read.total_ns - read_start_ns) + (
                    self.timing.write.total_ns - write_start_ns
                )
                self.timing.record_cycle_ns(cycle_ns)
                self.timing.record_compute_ns(max(0, cycle_ns - bus_ns))
                next_sample += PERIOD_S
                self.clock.sleep(max(0, min(PERIOD_S, next_sample - self.clock.monotonic())))
            return {"elapsed_s": self.clock.monotonic() - start, "writes": writes,
                    "positions": {name: state[name]["position"] for name in JOINT_NAMES},
                    "timing": self.timing.summary()}
        finally:
            if self.connected:
                try:
                    try:
                        self._hold()
                    finally:
                        self._restore_tuning()
                finally:
                    assert self.bus is not None
                    try:
                        self.bus.disconnect(disable_torque=False)
                    finally:
                        self.connected = False
                        self._ownership.release()
            else:
                self._ownership.release()
