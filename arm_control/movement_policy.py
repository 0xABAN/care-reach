"""Pure checks for which joints a planned movement may command.

The policy checks an entire path before execution. It never reorders joint
targets: splitting a coordinated pose into separate joint moves could take the
arm through an unvalidated configuration.
"""

from __future__ import annotations

from enum import Enum
from numbers import Integral
from typing import Iterable, Sequence


class MovementMode(str, Enum):
    COORDINATED = "coordinated"
    SHOULDER_ISOLATED = "shoulder_isolated"
    SINGLE_JOINT = "single_joint"


def parse_movement_mode(value: str) -> MovementMode:
    if type(value) is not str:
        raise ValueError("movement_mode must be a string")
    try:
        return MovementMode(value)
    except ValueError as exc:
        choices = ", ".join(mode.value for mode in MovementMode)
        raise ValueError(f"movement_mode must be one of {choices}") from exc


def _ticks(values: Sequence[int]) -> tuple[int, ...]:
    if len(values) != 6 or any(isinstance(tick, bool) or not isinstance(tick, Integral)
                               for tick in values):
        raise ValueError("movement path must contain six integer ticks per pose")
    return tuple(int(tick) for tick in values)


def validate_movement_path(
    mode: MovementMode | str,
    start_ticks: Sequence[int],
    frames: Iterable[Sequence[int]],
) -> frozenset[int]:
    """Return 1-based joints changed anywhere on the path, or reject it.

    In shoulder-isolated mode, a path may move motor 2 *or* any subset of the
    other five joints. A path that changes both groups at different times is
    rejected too: this validator never invents a safe ordering for them.
    """
    policy = mode if isinstance(mode, MovementMode) else parse_movement_mode(mode)
    start = _ticks(start_ticks)
    changed: set[int] = set()
    for frame in frames:
        target = _ticks(frame)
        changed.update(index for index, (before, after) in enumerate(zip(start, target), 1)
                       if before != after)
    if policy is MovementMode.SHOULDER_ISOLATED and 2 in changed and len(changed) > 1:
        raise ValueError("This path needs motor 2 and another joint; coordinated mode is required")
    if policy is MovementMode.SINGLE_JOINT and len(changed) > 1:
        raise ValueError("This path needs more than one joint; single-joint mode cannot run it")
    return frozenset(changed)
