"""Offline, position-domain planning for the six-joint SO-101 follower.

This module deliberately has no bus, serial, or robot imports. Its output is a
sequence of candidate tick poses; the caller remains responsible for live
feedback, faults, and stopping motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from numbers import Integral, Real
from pathlib import Path
from typing import Mapping, Sequence


JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class UnsafePoseError(ValueError):
    """A requested tick pose lies outside the configured safe interval."""


def _six_ticks(values: Sequence[int], label: str) -> tuple[int, ...]:
    if len(values) != 6:
        raise ValueError(f"{label} must have six joints")
    if any(isinstance(value, bool) or not isinstance(value, Integral) for value in values):
        raise ValueError(f"{label} must contain integer ticks")
    return tuple(int(value) for value in values)


def _six_positive(values: Sequence[float], label: str) -> tuple[float, ...]:
    if len(values) != 6:
        raise ValueError(f"{label} must have six joints")
    if any(not isinstance(value, Real) or isinstance(value, bool)
           or not math.isfinite(value) or value <= 0 for value in values):
        raise ValueError(f"{label} must contain six finite positive numbers")
    return tuple(float(value) for value in values)


@dataclass(frozen=True)
class SafetyBounds:
    """Inclusive tick intervals for motors 1 through 6."""

    lower: tuple[int, ...]
    upper: tuple[int, ...]

    def __post_init__(self) -> None:
        lower = _six_ticks(self.lower, "lower")
        upper = _six_ticks(self.upper, "upper")
        if any(lo < 0 or hi > 4095 or lo > hi for lo, hi in zip(lower, upper)):
            raise ValueError("invalid or empty 0..4095 safety interval")
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    @classmethod
    def from_files(
        cls,
        settings_path: str | Path | None = None,
        calibration_path: str | Path | None = None,
    ) -> SafetyBounds:
        """Intersect panel limits, LeRobot calibration, and provisional stops.

        A null panel limit means that end uses the calibrated range. The
        provisional stops reflect the observed wrist roll jam and the tested
        gripper opening. They apply in position mode, where ticks are 0..4095.
        """
        settings_path = Path(settings_path) if settings_path is not None else Path(__file__).resolve().parent / "joint_settings.json"
        calibration_path = (Path(calibration_path) if calibration_path is not None else
                            Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/care_reach_follower.json")
        with settings_path.open(encoding="utf-8") as file:
            settings = json.load(file)
        with calibration_path.open(encoding="utf-8") as file:
            calibration = json.load(file)

        lower: list[int] = []
        upper: list[int] = []
        for index, name in enumerate(JOINT_NAMES, start=1):
            joint = calibration[name]
            panel = settings[str(index)]
            cal_lo, cal_hi = int(joint["range_min"]), int(joint["range_max"])
            panel_lo = cal_lo if panel["lower"] is None else int(panel["lower"])
            panel_hi = cal_hi if panel["upper"] is None else int(panel["upper"])
            lo = max(cal_lo, panel_lo, 1340 if index == 6 else 0)
            hi = min(cal_hi, panel_hi, 3300 if index == 5 else 4095)
            lower.append(lo)
            upper.append(hi)
        return cls(tuple(lower), tuple(upper))


def validate_pose(pose: Sequence[int], bounds: SafetyBounds) -> tuple[int, ...]:
    """Return a six-tick pose or raise before any hardware call can use it."""
    try:
        ticks = _six_ticks(pose, "pose")
    except (ValueError, TypeError) as exc:
        raise UnsafePoseError(str(exc)) from exc
    for index, (tick, lo, hi) in enumerate(zip(ticks, bounds.lower, bounds.upper), start=1):
        if not lo <= tick <= hi:
            raise UnsafePoseError(f"motor {index} tick {tick} outside [{lo}, {hi}]")
    return ticks


def action_to_ticks(
    action6: Sequence[float],
    calibration: Mapping[str, Mapping[str, int]] | str | Path,
    bounds: SafetyBounds,
) -> tuple[int, ...]:
    """Convert LeRobot degree/percent actions to ticks, rejecting unsafe poses.

    Joint actions 1..5 are degrees relative to each calibrated midpoint.
    Gripper action is percent of its calibrated range. Conversion rounds to
    the nearest tick; it never clips an invalid request to the boundary.
    """
    if isinstance(calibration, (str, Path)):
        with Path(calibration).open(encoding="utf-8") as file:
            calibration = json.load(file)
    if len(action6) != 6 or any(
        not isinstance(value, Real) or isinstance(value, bool) or not math.isfinite(value)
        for value in action6
    ):
        raise ValueError("action must contain six finite numbers")
    ticks: list[int] = []
    for index, name in enumerate(JOINT_NAMES):
        joint = calibration[name]
        lo, hi = int(joint["range_min"]), int(joint["range_max"])
        action = float(action6[index])
        if index < 5:
            tick = (lo + hi) / 2 + action * 4095 / 360
        else:
            if not 0 <= action <= 100:
                raise UnsafePoseError("gripper action must be within 0..100 percent")
            tick = lo + action * (hi - lo) / 100
        ticks.append(round(tick))
    return validate_pose(ticks, bounds)


def _segment_limits(
    limits: Sequence[float] | Sequence[Sequence[float]],
    count: int,
    label: str,
) -> list[tuple[float, ...]]:
    """Accept one six-joint limit or a separate six-joint limit per leg."""
    if len(limits) == 6 and all(isinstance(value, Real) for value in limits):
        common = _six_positive(limits, label)
        return [common] * count
    if len(limits) != count:
        raise ValueError(f"{label} must have six joints or one six-joint vector per segment")
    return [_six_positive(values, f"{label}[{i}]") for i, values in enumerate(limits)]


def _waypoint_tangents(
    points: Sequence[tuple[int, ...]], durations: Sequence[float], stop_indices: set[int]
) -> list[tuple[float, ...]]:
    """Choose monotone joint velocities; reversals and requested pauses stop."""
    tangents: list[tuple[float, ...]] = [(0.0,) * 6]
    for knot in range(1, len(points) - 1):
        if knot - 1 in stop_indices:
            tangents.append((0.0,) * 6)
            continue
        before = tuple((points[knot][j] - points[knot - 1][j]) / durations[knot - 1] for j in range(6))
        after = tuple((points[knot + 1][j] - points[knot][j]) / durations[knot] for j in range(6))
        tangents.append(tuple(
            math.copysign(min(abs(before[j]), abs(after[j])), before[j])
            if before[j] * after[j] > 0 else 0.0
            for j in range(6)
        ))
    tangents.append((0.0,) * 6)
    return tangents


def plan_blended_waypoints(
    start: Sequence[int],
    waypoints: Sequence[Sequence[int]],
    bounds: SafetyBounds,
    vmax: Sequence[float] | Sequence[Sequence[float]],
    amax: Sequence[float] | Sequence[Sequence[float]],
    fps: float = 20,
    speed_scales: Sequence[float] | None = None,
    stop_indices: Sequence[int] | None = None,
) -> list[tuple[int, ...]]:
    """Plan continuous, bounded motion through six-joint tick waypoints.

    Each leg may have its own six-joint velocity and acceleration limits.
    ``speed_scales`` changes each leg's tempo: velocity scales linearly and
    acceleration quadratically. Internal joints continue through waypoints
    when their direction stays the same; direction reversals naturally stop.
    A ``stop_indices`` value refers to an index in ``waypoints``. The planner
    stretches time until the continuous cubic's exact speed and acceleration
    extrema obey every segment limit. Integer samples can have one-tick
    quantization steps; hardware tracking remains the runtime's responsibility.
    """
    if not isinstance(fps, Real) or isinstance(fps, bool) or not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    points = [validate_pose(start, bounds), *(validate_pose(p, bounds) for p in waypoints)]
    count = len(points) - 1
    if not count:
        return points
    vel_limits = _segment_limits(vmax, count, "vmax")
    acc_limits = _segment_limits(amax, count, "amax")
    if speed_scales is None:
        scales = (1.0,) * count
    else:
        if len(speed_scales) != count:
            raise ValueError("speed_scales must have one value per segment")
        scales = tuple(float(value) for value in speed_scales)
        if any(not math.isfinite(value) or value <= 0 for value in scales):
            raise ValueError("speed_scales must be finite and positive")
    vel_limits = [tuple(limit * scales[i] for limit in row) for i, row in enumerate(vel_limits)]
    acc_limits = [tuple(limit * scales[i] ** 2 for limit in row) for i, row in enumerate(acc_limits)]
    stops = set(stop_indices or ())
    if any(isinstance(index, bool) or not isinstance(index, Integral) or index < 0 or index >= count for index in stops):
        raise ValueError("stop_indices must reference waypoints")

    durations: list[float] = []
    for i in range(count):
        distances = [abs(points[i + 1][j] - points[i][j]) for j in range(6)]
        durations.append(max(
            1 / fps,
            *(1.5 * d / vel_limits[i][j] for j, d in enumerate(distances)),
            *(math.sqrt(6 * d / acc_limits[i][j]) for j, d in enumerate(distances)),
        ))

    # Global time stretching preserves the ratio between slow and fast phases
    # while satisfying all joint-specific constraints at segment boundaries.
    for _ in range(40):
        tangents = _waypoint_tangents(points, durations, stops)
        stretch = 1.0
        for i, duration in enumerate(durations):
            for joint in range(6):
                delta = points[i + 1][joint] - points[i][joint]
                b = duration * tangents[i][joint]
                c = 3 * delta - 2 * b - duration * tangents[i + 1][joint]
                d = -2 * delta + b + duration * tangents[i + 1][joint]
                velocity_candidates = [b, b + 2 * c + 3 * d]
                if d != 0:
                    critical = -c / (3 * d)
                    if 0 < critical < 1:
                        velocity_candidates.append(b + 2 * c * critical + 3 * d * critical * critical)
                peak_velocity = max(abs(value) for value in velocity_candidates) / duration
                peak_acceleration = max(abs(2 * c), abs(2 * c + 6 * d)) / duration**2
                stretch = max(
                    stretch,
                    peak_velocity / vel_limits[i][joint],
                    math.sqrt(peak_acceleration / acc_limits[i][joint]),
                )
        if stretch <= 1 + 1e-10:
            break
        durations = [duration * max(1.001, stretch * 1.001) for duration in durations]
    else:
        raise RuntimeError("unable to satisfy blended trajectory limits")

    frames = [points[0]]
    for i, duration in enumerate(durations):
        steps = max(1, math.ceil(duration * fps))
        for step in range(1, steps + 1):
            u = step / steps
            h00 = 2 * u**3 - 3 * u**2 + 1
            h10 = u**3 - 2 * u**2 + u
            h01 = -2 * u**3 + 3 * u**2
            h11 = u**3 - u**2
            pose = tuple(round(
                h00 * points[i][j] + h10 * duration * tangents[i][j]
                + h01 * points[i + 1][j] + h11 * duration * tangents[i + 1][j]
            ) for j in range(6))
            frames.append(validate_pose(pose, bounds))
    return frames


def splice_action_chunk(
    current_goal: Sequence[int],
    old_future: Sequence[Sequence[int]],
    predicted_chunk: Sequence[Sequence[int]],
    observation_step: int,
    current_step: int,
    bounds: SafetyBounds,
    blend_steps: int = 6,
) -> list[tuple[int, ...]] | None:
    """Align and blend a late policy chunk without accepting unsafe poses.

    A prediction's first action belongs to ``observation_step``. Executed
    steps are discarded, then the overlapping head is blended with the old
    future (or latest safe goal). ``None`` means the entire chunk is stale.
    The caller must still apply live velocity, acceleration, and tracking
    guards before sending any returned target to hardware.
    """
    if any(isinstance(value, bool) or not isinstance(value, Integral) or value < 0
           for value in (observation_step, current_step, blend_steps)):
        raise ValueError("steps must be non-negative integers")
    if observation_step > current_step:
        raise ValueError("observation_step cannot be in the future")
    current = validate_pose(current_goal, bounds)
    old = [validate_pose(pose, bounds) for pose in old_future]
    predicted = [validate_pose(pose, bounds) for pose in predicted_chunk]
    if not predicted:
        raise ValueError("predicted_chunk must have at least one pose")
    elapsed = current_step - observation_step
    if elapsed >= len(predicted):
        return None
    aligned = predicted[elapsed:]
    blend_count = min(blend_steps, len(aligned))
    result: list[tuple[int, ...]] = []
    for index, pose in enumerate(aligned):
        if index >= blend_count:
            result.append(pose)
            continue
        previous = old[index] if index < len(old) else current
        new_weight = (index + 1) / (blend_count + 1)
        mixed = tuple(round((1 - new_weight) * before + new_weight * after)
                      for before, after in zip(previous, pose))
        result.append(validate_pose(mixed, bounds))
    return result
