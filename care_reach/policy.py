"""Adapters from model (policy) actions to this arm's tick poses.

Nothing here talks to hardware. A model predicts LeRobot-style joint actions;
these functions convert and splice them into validated tick poses that a
guarded controller can execute.
"""

from __future__ import annotations

import json
import math
from numbers import Integral, Real
from pathlib import Path
from typing import Mapping, Sequence

from care_reach.motion.bounds import SafetyBounds, UnsafePoseError, validate_pose
from care_reach.robot import JOINT_NAMES


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
