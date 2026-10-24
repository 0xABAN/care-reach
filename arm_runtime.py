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

from motion_core import JOINT_NAMES, SafetyBounds, UnsafePoseError, validate_pose
from home_pose import home_target


HOME = home_target()
DANCE_CENTER = (2000,) * 6
HOME_TOLERANCE = 120
MIN_VOLTAGE_RAW = 48  # STS3215 register is in tenths of a volt.
MAX_LOAD_MAGNITUDE = 900  # LeRobot decodes Present_Load as a signed value.
STALL_ERROR = 35
STALL_TIME_S = 0.8
STALL_PROGRESS_TICKS = 4
FINISH_TOLERANCE = 15
FINISH_TIMEOUT_S = 8.0
HOME_TRIM_TICKS = 45
HOME_TRIM_STAGNANT_S = 1.0
HOME_TRIM_ERROR_WINDOW = 60
DEFAULT_COMMAND_SPEED = (90.0, 90.0, 90.0, 90.0, 140.0, 90.0)  # ticks/s
DEFAULT_COMMAND_ACCELERATION = (120.0, 120.0, 120.0, 120.0, 180.0, 120.0)  # ticks/s^2


class ArmSafetyError(RuntimeError):
    """A preflight or live safety guard stopped playback."""


def _real_bus(port: str):
    # Keep the serial stack out of fake-bus unit tests and import-time use.
    from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus

    motors = {
        name: Motor(index, "sts3215", MotorNormMode.RANGE_0_100 if index == 6 else MotorNormMode.DEGREES)
        for index, name in enumerate(JOINT_NAMES, start=1)
    }
    return SerialMotorsBus(port=port, motors=motors)


class ArmRuntime:
    def __init__(self, port: str, bounds: SafetyBounds, bus_factory=None):
        self.port = port
        self.bounds = bounds
        self._bus_factory = bus_factory or _real_bus
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

    def _read(self, register: str) -> dict[str, int]:
        assert self._bus is not None
        values = self._bus.sync_read(register, normalize=False, num_retry=2)
        if set(values) != set(JOINT_NAMES):
            raise ArmSafetyError(f"Incomplete {register} read: {sorted(values)}")
        return {name: int(values[name]) for name in JOINT_NAMES}

    def _sample(self) -> dict[str, dict[str, int]]:
        readings = {
            field: self._read(register)
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
            if joint["voltage"] < MIN_VOLTAGE_RAW:
                # Brief STS3215 voltage dips are known on this 5 V supply.
                second = self._read("Present_Voltage")[name]
                joint["voltage_recheck"] = second
                if second < MIN_VOLTAGE_RAW:
                    raise ArmSafetyError(f"{name}: voltage stayed low ({joint['voltage']}, {second})")

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
            positions = self._read("Present_Position")
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
        high_load_count = {name: 0 for name in JOINT_NAMES}

        try:
            self._bus = self._bus_factory(self.port)
            self._bus.connect(handshake=True)
            self._connected = True

            firmware_min = self._read("Min_Position_Limit")
            firmware_max = self._read("Max_Position_Limit")
            self._firmware_limits = firmware_min, firmware_max
            for index, name in enumerate(JOINT_NAMES):
                requested_min = min(pose[index] for pose in poses)
                requested_max = max(pose[index] for pose in poses)
                if requested_min < firmware_min[name] or requested_max > firmware_max[name]:
                    raise ArmSafetyError(
                        f"{name}: requested {requested_min}..{requested_max} outside firmware "
                        f"{firmware_min[name]}..{firmware_max[name]}"
                    )

            modes = self._read("Operating_Mode")
            for name in JOINT_NAMES:
                if modes[name] != 0:
                    raise ArmSafetyError(f"{name}: operating mode {modes[name]}, expected position mode 0")
            self._position_mode_verified = True

            initial = self._sample()
            self._check_state(initial)
            if not any(
                all(abs(initial[name]["position"] - reference[index]) <= HOME_TOLERANCE
                    for index, name in enumerate(JOINT_NAMES))
                for reference in (HOME, DANCE_CENTER)
            ):
                raise ArmSafetyError("startup is too far from Home or dance center")

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
                    # integrator can coast back and forth around HOME despite
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
                    load = abs(state[name]["load"])
                    high_load_count[name] = high_load_count[name] + 1 if load >= MAX_LOAD_MAGNITUDE else 0
                    if high_load_count[name] >= 3:
                        raise ArmSafetyError(f"{name}: sustained high load {load}")

                    error = abs(last_goal[name] - position)
                    if error > STALL_ERROR:
                        if stalled_since[name] is None or abs(position - stalled_at[name]) >= STALL_PROGRESS_TICKS:
                            stalled_since[name] = now
                            stalled_at[name] = position
                        elif now - stalled_since[name] >= STALL_TIME_S:
                            raise ArmSafetyError(f"{name}: stalled at {position}, goal {last_goal[name]}")
                    else:
                        stalled_since[name] = None

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
                    load = abs(state[name]["load"])
                    high_load_count[name] = high_load_count[name] + 1 if load >= MAX_LOAD_MAGNITUDE else 0
                    if high_load_count[name] >= 3:
                        raise ArmSafetyError(f"{name}: sustained high load {load}")
                    error = abs(last_goal[name] - position)
                    near_home_trim_window = (
                        final_pose[index] in (HOME[index], DANCE_CENTER[index])
                        and abs(position - final_pose[index]) <= HOME_TRIM_ERROR_WINDOW
                        and (
                            not trim_attempted[name]
                            or trim_target[name] is not None and error <= HOME_TRIM_ERROR_WINDOW
                        )
                    )
                    if error > STALL_ERROR and not near_home_trim_window:
                        if stalled_since[name] is None or abs(position - stalled_at[name]) >= STALL_PROGRESS_TICKS:
                            stalled_since[name] = now
                            stalled_at[name] = position
                        elif now - stalled_since[name] >= STALL_TIME_S:
                            raise ArmSafetyError(f"{name}: stalled at {position}, goal {last_goal[name]}")
                    else:
                        stalled_since[name] = None
                    if abs(position - last_progress[name]) >= STALL_PROGRESS_TICKS:
                        last_progress[name] = position
                        stagnant_since[name] = now
                    if trim_target[name] is not None and abs(position - final_pose[index]) <= finish_tolerance:
                        # Another joint may still be settling. Retire this
                        # joint's trim immediately so it cannot drive past HOME.
                        trim_target[name] = None
                        command_position[name] = float(position)
                        command_velocity[name] = 0.0
                    elif (
                        final_pose[index] in (HOME[index], DANCE_CENTER[index])
                        and not trim_attempted[name]
                        and abs(position - final_pose[index]) > finish_tolerance
                        and abs(last_goal[name] - position) <= HOME_TRIM_ERROR_WINDOW
                        and now - stagnant_since[name] >= HOME_TRIM_STAGNANT_S
                    ):
                        # The STS3215 can rest tens of ticks short of a small
                        # final goal. A bounded one-time target beyond HOME
                        # overcomes this static-friction deadband. Stop as
                        # soon as measured position reaches HOME tolerance.
                        candidate = final_pose[index] - (
                            HOME_TRIM_TICKS if position > final_pose[index] else -HOME_TRIM_TICKS
                        )
                        trim_attempted[name] = True
                        stalled_since[name] = None
                        if name == "elbow_flex":
                            candidate = min(candidate, HOME[index])
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
