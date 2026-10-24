"""Guarded, coordinated raw-tick playback for the assembled SO-101 follower.

This module has no import-time hardware effects. Constructing ``ArmRuntime``
also does not open the port; only ``run`` does. A failed run holds the measured
pose where possible and leaves torque enabled.
"""

from __future__ import annotations

import time
import math
from collections.abc import Iterable
from typing import Any

from care_reach.hardware.bus import (
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
from care_reach.motion.bounds import SafetyBounds, UnsafePoseError, validate_pose
from care_reach.robot import HOME_TICK, JOINT_NAMES


HOME_TOLERANCE = 120
FINISH_TOLERANCE = 15
FINISH_TIMEOUT_S = 8.0
HOME_TRIM_TICKS = 45
HOME_TRIM_STAGNANT_S = 1.0
HOME_TRIM_ERROR_WINDOW = 60
DEFAULT_COMMAND_SPEED = (90.0, 90.0, 90.0, 90.0, 140.0, 90.0)  # ticks/s
DEFAULT_COMMAND_ACCELERATION = (120.0, 120.0, 120.0, 120.0, 180.0, 120.0)  # ticks/s^2


class ArmRuntime:
    """Play a validated tick trajectory that starts near Home, one run per instance."""

    def __init__(self, port: str, bounds: SafetyBounds, bus_factory=None):
        self.port = port
        self.bounds = bounds
        self._bus_factory = bus_factory or make_bus
        self._bus = None
        self._connected = False
        self._closed = False
        self._last_positions: dict[str, int] | None = None
        self._firmware_limits: tuple[dict[str, int], dict[str, int]] | None = None
        self._position_mode_verified = False

    def __enter__(self) -> ArmRuntime:
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self.close()

    def _sample(self) -> dict[str, dict[str, int]]:
        readings = {
            field: read_joints(self._bus, register)
            for field, register in (
                ("position", "Present_Position"),
                ("status", "Status"),
                ("voltage", "Present_Voltage"),
                ("torque", "Torque_Enable"),
                ("load", "Present_Load"),
            )
        }
        state = {name: {field: readings[field][name] for field in readings} for name in JOINT_NAMES}
        self._last_positions = {name: state[name]["position"] for name in JOINT_NAMES}
        return state

    def _check_state(self, state: dict[str, dict[str, int]]) -> None:
        for index, name in enumerate(JOINT_NAMES):
            joint = state[name]
            if joint["torque"] != 1:
                raise ArmSafetyError(f"{name}: torque is not enabled")
            if joint["status"] != 0:
                raise ArmSafetyError(f"{name}: servo status {joint['status']}")
            position = joint["position"]
            if not self.bounds.lower[index] <= position <= self.bounds.upper[index]:
                raise ArmSafetyError(f"{name}: measured position {position} exceeds software limits")
            check_voltage(self._bus, name, joint["voltage"])

    def _write_goals(self, goals: dict[str, int]) -> None:
        pose = validate_pose(tuple(goals[name] for name in JOINT_NAMES), self.bounds)
        assert self._bus is not None
        self._bus.sync_write(
            "Goal_Position", dict(zip(JOINT_NAMES, pose, strict=True)), normalize=False
        )

    def _hold_measured(self) -> None:
        if not self._connected or self._bus is None:
            return
        try:
            positions = read_joints(self._bus, "Present_Position")
            self._last_positions = positions
        except Exception:
            positions = self._last_positions
        if positions is None or not self._position_mode_verified:
            return
        if self._firmware_limits is not None:
            minimum, maximum = self._firmware_limits
            if any(not minimum[name] <= positions[name] <= maximum[name] for name in JOINT_NAMES):
                return
        # If a joint has drifted outside software bounds, leave its last goal
        # active instead of commanding an unbounded target or a forced return.
        try:
            self._write_goals(positions)
        except (UnsafePoseError, ArmSafetyError):
            pass

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._connected:
            try:
                self._hold_measured()
            finally:
                assert self._bus is not None
                self._bus.disconnect(disable_torque=False)
                self._connected = False

    def run(
        self,
        frames: Iterable[Iterable[int]],
        fps: float = 20,
        max_lead: int = 80,
        command_speed_scale: float = 1.0,
        max_command_acceleration: tuple[float, ...] | None = DEFAULT_COMMAND_ACCELERATION,
        finish_tolerance: int = FINISH_TOLERANCE,
    ) -> dict[str, Any]:
        if self._closed or self._bus is not None:
            raise RuntimeError("ArmRuntime is single-use; construct a new instance for each run")
        if fps <= 0 or fps > 50:
            raise ValueError("fps must be in (0, 50]")
        if max_lead <= 0 or max_lead > 120:
            raise ValueError("max_lead must be in (0, 120]")
        if not math.isfinite(command_speed_scale) or not 0 < command_speed_scale <= 2:
            raise ValueError("command_speed_scale must be in (0, 2]")
        if isinstance(finish_tolerance, bool) or not isinstance(finish_tolerance, int) or not 1 <= finish_tolerance <= 60:
            raise ValueError("finish_tolerance must be an integer in [1, 60]")
        if max_command_acceleration is not None and (
            len(max_command_acceleration) != 6
            or any(not math.isfinite(value) or value <= 0 for value in max_command_acceleration)
        ):
            raise ValueError("max_command_acceleration must contain six finite positive values")

        # Validate the complete program before any serial activity. Firmware
        # compatibility is checked after connection, before the first goal.
        poses = [validate_pose(frame, self.bounds) for frame in frames]
        if not poses:
            raise ValueError("Motion program has no frames")
        run_started = time.monotonic()
        samples: list[dict[str, Any]] = []
        commanded = 0
        stalled_since: dict[str, float | None] = {name: None for name in JOINT_NAMES}
        stalled_at: dict[str, int] = {}
        load_watch = LoadWatch()

        try:
            self._bus = self._bus_factory(self.port)
            self._bus.connect(handshake=True)
            self._connected = True

            firmware_min, firmware_max = read_firmware_limits(self._bus)
            self._firmware_limits = firmware_min, firmware_max
            for index, name in enumerate(JOINT_NAMES):
                requested_min = min(pose[index] for pose in poses)
                requested_max = max(pose[index] for pose in poses)
                if requested_min < firmware_min[name] or requested_max > firmware_max[name]:
                    raise ArmSafetyError(
                        f"{name}: requested {requested_min}..{requested_max} outside firmware "
                        f"{firmware_min[name]}..{firmware_max[name]}"
                    )

            require_position_mode(self._bus)
            self._position_mode_verified = True

            initial = self._sample()
            self._check_state(initial)
            for name in JOINT_NAMES:
                if abs(initial[name]["position"] - HOME_TICK) > HOME_TOLERANCE:
                    raise ArmSafetyError(f"{name}: startup is too far from home ({initial[name]['position']})")

            last_goal = {name: initial[name]["position"] for name in JOINT_NAMES}
            command_position = {name: float(last_goal[name]) for name in JOINT_NAMES}
            command_velocity = {name: 0.0 for name in JOINT_NAMES}  # ticks per control frame

            def bounded_goal(name: str, index: int, requested: int, measured: int) -> int:
                """Bound encoder lead, then slew the commanded goal and its velocity.

                The offline planner already limits speed and acceleration. This
                independent guard also covers encoder lag and plan reversals,
                where lead clipping alone can make a sudden goal jump.
                """
                lower = max(self.bounds.lower[index], firmware_min[name], measured - max_lead)
                upper = min(self.bounds.upper[index], firmware_max[name], measured + max_lead)
                target = max(lower, min(upper, requested))
                previous = command_position[name]
                distance = target - previous
                max_step = DEFAULT_COMMAND_SPEED[index] * command_speed_scale / fps
                if max_command_acceleration is not None:
                    max_step_change = max_command_acceleration[index] / (fps * fps)
                    # Start braking before the target. Without this, a goal
                    # integrator can coast back and forth around Home despite
                    # an acceleration limit on each individual frame.
                    max_step = min(max_step, math.sqrt(2 * max_step_change * abs(distance)))
                desired_step = math.copysign(min(max_step, abs(distance)), distance) if distance else 0.0
                if max_command_acceleration is not None:
                    desired_step = max(
                        command_velocity[name] - max_step_change,
                        min(command_velocity[name] + max_step_change, desired_step),
                    )
                # A waypoint is a position target, so never let accumulated
                # command velocity carry the goal across that target.
                if (distance >= 0 and desired_step > distance) or (distance <= 0 and desired_step < distance):
                    desired_step = distance
                next_position = max(lower, min(upper, previous + desired_step))
                command_velocity[name] = next_position - previous
                command_position[name] = next_position
                return max(lower, min(upper, round(next_position)))

            def guard_joint(name: str, position: int, load: int, now: float, *, may_lag: bool = False) -> None:
                """Stop on sustained high load, or on a joint stuck far from its last goal.

                ``may_lag`` exempts a joint inside the near-Home trim window,
                where resting short of a small final goal is expected.
                """
                load_watch.check(name, load)
                if abs(last_goal[name] - position) > STALL_ERROR_TICKS and not may_lag:
                    if stalled_since[name] is None or abs(position - stalled_at[name]) >= STALL_PROGRESS_TICKS:
                        stalled_since[name] = now
                        stalled_at[name] = position
                    elif now - stalled_since[name] >= STALL_TIME_S:
                        raise ArmSafetyError(f"{name}: stalled at {position}, goal {last_goal[name]}")
                else:
                    stalled_since[name] = None

            playback_started = time.monotonic()
            for frame_index, pose in enumerate(poses):
                due = playback_started + frame_index / fps
                remaining = due - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                state = self._sample()
                self._check_state(state)
                now = time.monotonic()
                goals = {}
                for index, name in enumerate(JOINT_NAMES):
                    position = state[name]["position"]
                    guard_joint(name, position, state[name]["load"], now)
                    goals[name] = bounded_goal(name, index, pose[index], position)

                self._write_goals(goals)
                last_goal = goals
                commanded += 1
                samples.append({"frame": frame_index, "time_s": now - playback_started, "state": state, "goals": goals})

            # Do not replace the last target with a hold while it is still
            # moving. Converge to the final requested pose within a deadline.
            finish_started = time.monotonic()
            final_pose = poses[-1]
            last_progress = {name: self._last_positions[name] for name in JOINT_NAMES}
            stagnant_since = {name: finish_started for name in JOINT_NAMES}
            trim_target: dict[str, int | None] = {name: None for name in JOINT_NAMES}
            trim_attempted = {name: False for name in JOINT_NAMES}
            while True:
                time.sleep(1 / fps)
                state = self._sample()
                self._check_state(state)
                now = time.monotonic()
                if all(
                    abs(state[name]["position"] - final_pose[index]) <= finish_tolerance
                    for index, name in enumerate(JOINT_NAMES)
                ):
                    break
                if now - finish_started >= FINISH_TIMEOUT_S:
                    raise ArmSafetyError("Final pose did not settle before timeout")
                goals = {}
                for index, name in enumerate(JOINT_NAMES):
                    position = state[name]["position"]
                    error = abs(last_goal[name] - position)
                    near_home_trim_window = (
                        final_pose[index] == HOME_TICK
                        and abs(position - HOME_TICK) <= HOME_TRIM_ERROR_WINDOW
                        and (
                            not trim_attempted[name]
                            or trim_target[name] is not None and error <= HOME_TRIM_ERROR_WINDOW
                        )
                    )
                    guard_joint(name, position, state[name]["load"], now, may_lag=near_home_trim_window)
                    if abs(position - last_progress[name]) >= STALL_PROGRESS_TICKS:
                        last_progress[name] = position
                        stagnant_since[name] = now
                    if trim_target[name] is not None and abs(position - final_pose[index]) <= finish_tolerance:
                        # Another joint may still be settling. Retire this
                        # joint's trim immediately so it cannot drive past Home.
                        trim_target[name] = None
                        command_position[name] = float(position)
                        command_velocity[name] = 0.0
                    elif (
                        final_pose[index] == HOME_TICK
                        and not trim_attempted[name]
                        and abs(position - final_pose[index]) > finish_tolerance
                        and abs(last_goal[name] - position) <= HOME_TRIM_ERROR_WINDOW
                        and now - stagnant_since[name] >= HOME_TRIM_STAGNANT_S
                    ):
                        # The STS3215 can rest tens of ticks short of a small
                        # final goal. A bounded one-time target beyond Home
                        # overcomes this static-friction deadband. Stop as
                        # soon as measured position reaches Home tolerance.
                        candidate = final_pose[index] - (
                            HOME_TRIM_TICKS if position > final_pose[index] else -HOME_TRIM_TICKS
                        )
                        trim_attempted[name] = True
                        stalled_since[name] = None
                        if (
                            self.bounds.lower[index] <= candidate <= self.bounds.upper[index]
                            and firmware_min[name] <= candidate <= firmware_max[name]
                        ):
                            trim_target[name] = candidate
                    requested = trim_target[name] if trim_target[name] is not None else final_pose[index]
                    goals[name] = bounded_goal(name, index, requested, position)
                self._write_goals(goals)
                last_goal = goals
                commanded += 1
                samples.append({"frame": commanded - 1, "time_s": now - playback_started,
                                "phase": "settle", "state": state, "goals": goals})

            return {
                "frames_commanded": commanded,
                "samples": samples,
                "final_positions": self._last_positions,
                "elapsed_s": time.monotonic() - run_started,
            }
        finally:
            self.close()
