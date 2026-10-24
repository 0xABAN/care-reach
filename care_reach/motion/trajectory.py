"""Offline trajectory planning through six-joint tick waypoints.

The output is a sequence of candidate tick poses, each already validated
against the given bounds. The caller remains responsible for live feedback,
faults, and stopping motion.
"""

from __future__ import annotations

import math
from numbers import Integral, Real
from typing import Sequence

from care_reach.motion.bounds import SafetyBounds, validate_pose


def _six_positive(values: Sequence[float], label: str) -> tuple[float, ...]:
    if len(values) != 6:
        raise ValueError(f"{label} must have six joints")
    if any(not isinstance(value, Real) or isinstance(value, bool)
           or not math.isfinite(value) or value <= 0 for value in values):
        raise ValueError(f"{label} must contain six finite positive numbers")
    return tuple(float(value) for value in values)


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
