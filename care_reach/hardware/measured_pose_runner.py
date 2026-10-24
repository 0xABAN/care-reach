"""Guarded six-joint move driven by measured encoder position.

The servo's own Goal_Speed and Acceleration registers shape each small
position step. This runner never changes those registers, operating mode, or
torque. In particular, it never releases an arm that is holding its pose.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from care_reach.hardware.arm_runtime import _real_bus
from care_reach.motion.bounds import SafetyBounds, validate_pose
from care_reach.robot import JOINT_NAMES


class PoseMoveError(RuntimeError):
    """A live safety condition or lack of progress stopped a pose move."""


MEASURED_OVERSHOOT_TICKS = 5


class MeasuredPoseRunner:
    def __init__(
        self,
        port: str,
        bounds: SafetyBounds,
        bus_factory: Callable[[str], Any] | None = None,
        clock: Any = time,
    ) -> None:
        self.port = port
        self.bounds = bounds
        self.bus_factory = bus_factory or _real_bus
        self.clock = clock
        self.bus: Any | None = None
        self.connected = False
        self.position_mode = False
        self.last_positions: dict[str, int] | None = None
        self.firmware_min: dict[str, int] | None = None
        self.firmware_max: dict[str, int] | None = None

    def _read(self, register: str) -> dict[str, int]:
        assert self.bus is not None
        values = self.bus.sync_read(register, normalize=False, num_retry=2)
        if set(values) != set(JOINT_NAMES):
            raise PoseMoveError(f"Incomplete {register} read")
        return {name: int(values[name]) for name in JOINT_NAMES}

    def _sample(self) -> dict[str, dict[str, int]]:
        fields = {key: self._read(register) for key, register in (
            ("position", "Present_Position"),
            ("status", "Status"),
            ("torque", "Torque_Enable"),
            ("voltage", "Present_Voltage"),
            ("load", "Present_Load"),
        )}
        state = {name: {key: values[name] for key, values in fields.items()} for name in JOINT_NAMES}
        self.last_positions = fields["position"]
        return state

    def _check(self, state: dict[str, dict[str, int]]) -> None:
        # Encoder feedback may briefly pass a bounded target. Permit only a
        # few ticks of measured overshoot so the next goal can pull inward;
        # every commanded goal remains strictly inside the saved bounds.
        for index, name in enumerate(JOINT_NAMES):
            joint = state[name]
            position = joint["position"]
            lo, hi = self.bounds.lower[index], self.bounds.upper[index]
            if not lo - MEASURED_OVERSHOOT_TICKS <= position <= hi + MEASURED_OVERSHOOT_TICKS:
                raise PoseMoveError(f"{name}: measured position {position} exceeds [{lo}, {hi}]")
            if joint["torque"] != 1:
                raise PoseMoveError(f"{name}: torque disabled")
            if joint["status"] != 0:
                raise PoseMoveError(f"{name}: status {joint['status']}")
            if joint["voltage"] < 48 and self._read("Present_Voltage")[name] < 48:
                raise PoseMoveError(f"{name}: sustained low voltage")

    def _write(self, goals: dict[str, int]) -> None:
        assert self.bus is not None
        validate_pose(tuple(goals[name] for name in JOINT_NAMES), self.bounds)
        assert self.firmware_min is not None and self.firmware_max is not None
        for name in JOINT_NAMES:
            if not self.firmware_min[name] <= goals[name] <= self.firmware_max[name]:
                raise PoseMoveError(f"{name}: goal outside firmware limits")
        self.bus.sync_write("Goal_Position", goals, normalize=False)

    def _close(self) -> None:
        if not self.connected or self.bus is None:
            return
        try:
            if self.position_mode:
                try:
                    current = self._read("Present_Position")
                except Exception:
                    current = self.last_positions
                if current is not None:
                    try:
                        assert self.firmware_min is not None and self.firmware_max is not None
                        held = {
                            name: max(
                                max(self.bounds.lower[index], self.firmware_min[name]),
                                min(min(self.bounds.upper[index], self.firmware_max[name]), current[name]),
                            )
                            for index, name in enumerate(JOINT_NAMES)
                        }
                        self._write(held)
                    except Exception:
                        pass
        finally:
            self.bus.disconnect(disable_torque=False)
            self.connected = False

    def move(
        self,
        target: tuple[int, ...],
        *,
        max_lead: int = 50,
        period_s: float = 0.15,
        timeout_s: float = 40.0,
        tolerance: int = 35,
        stall_s: float = 2.0,
    ) -> dict[str, Any]:
        """Move all six goals toward target; check feedback before each write.

        Each goal stays at most ``max_lead`` ticks from that joint's latest
        encoder position. The loop can move from any measured pose in bounds.
        It never silently clips the requested endpoint.
        """
        pose = validate_pose(target, self.bounds)
        if self.bus is not None:
            raise RuntimeError("MeasuredPoseRunner is single-use")
        if not 1 <= max_lead <= 80 or not 0.03 <= period_s <= 1:
            raise ValueError("invalid goal lead or sample period")
        if timeout_s <= 0 or stall_s <= 0 or not 1 <= tolerance <= 60:
            raise ValueError("invalid timeout or tolerance")
        started = self.clock.monotonic()
        samples: list[dict[str, Any]] = []
        writes = 0
        high_load = {name: 0 for name in JOINT_NAMES}
        progress_position: dict[str, int] = {}
        progress_time: dict[str, float] = {}
        try:
            self.bus = self.bus_factory(self.port)
            self.bus.connect(handshake=True)
            self.connected = True
            self.firmware_min = self._read("Min_Position_Limit")
            self.firmware_max = self._read("Max_Position_Limit")
            modes = self._read("Operating_Mode")
            if any(modes[name] != 0 for name in JOINT_NAMES):
                raise PoseMoveError("all motors must be in position mode 0")
            self.position_mode = True
            if any(not self.firmware_min[name] <= pose[i] <= self.firmware_max[name]
                   for i, name in enumerate(JOINT_NAMES)):
                raise PoseMoveError("target outside firmware limits")
            initial = self._sample()
            self._check(initial)
            for name in JOINT_NAMES:
                progress_position[name] = initial[name]["position"]
                progress_time[name] = started
            while True:
                state = self._sample()
                self._check(state)
                now = self.clock.monotonic()
                positions = {name: state[name]["position"] for name in JOINT_NAMES}
                errors = {name: pose[i] - positions[name] for i, name in enumerate(JOINT_NAMES)}
                samples.append({"time_s": now - started, "state": state})
                if all(abs(error) <= tolerance for error in errors.values()):
                    return {"target": pose, "positions": positions, "writes": writes,
                            "elapsed_s": now - started, "samples": samples}
                if now - started >= timeout_s:
                    raise PoseMoveError(f"pose timeout; positions {positions}")
                goals: dict[str, int] = {}
                for index, name in enumerate(JOINT_NAMES):
                    load = abs(state[name]["load"])
                    high_load[name] = high_load[name] + 1 if load >= 900 else 0
                    if high_load[name] >= 3:
                        raise PoseMoveError(f"{name}: sustained high load {load}")
                    if abs(positions[name] - progress_position[name]) >= 3:
                        progress_position[name] = positions[name]
                        progress_time[name] = now
                    elif abs(errors[name]) > tolerance and now - progress_time[name] >= stall_s:
                        raise PoseMoveError(f"{name}: stalled at {positions[name]}")
                    direction = 1 if errors[name] > 0 else -1
                    goal = positions[name] + direction * min(abs(errors[name]), max_lead)
                    goals[name] = max(self.bounds.lower[index], min(self.bounds.upper[index], goal))
                self._write(goals)
                writes += 1
                self.clock.sleep(period_s)
        finally:
            self._close()
