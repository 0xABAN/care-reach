"""Per-joint safety bounds and pose validation for the SO-101 follower.

This module deliberately has no bus or serial imports. Every motion path
validates its tick poses here before any hardware call can use them.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from numbers import Integral
from pathlib import Path
from typing import Sequence

from care_reach.robot import CALIBRATION_PATH, JOINT_NAMES, JOINT_SETTINGS_PATH
from care_reach.settings import load_joint_settings


# Provisional stops applied on top of saved limits: wrist roll resisted motion
# near 3358 ticks, and the gripper jaws measured contact at 1317 ticks.
WRIST_ROLL_MAX_TICK = 3300
GRIPPER_MIN_TICK = 1340


class UnsafePoseError(ValueError):
    """A requested tick pose lies outside the configured safe interval."""


def _six_ticks(values: Sequence[int], label: str) -> tuple[int, ...]:
    if len(values) != 6:
        raise ValueError(f"{label} must have six joints")
    if any(isinstance(value, bool) or not isinstance(value, Integral) for value in values):
        raise ValueError(f"{label} must contain integer ticks")
    return tuple(int(value) for value in values)


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
        settings_path: str | Path = JOINT_SETTINGS_PATH,
        calibration_path: str | Path = CALIBRATION_PATH,
    ) -> SafetyBounds:
        """Intersect panel limits, LeRobot calibration, and provisional stops.

        A null panel limit means that end uses the calibrated range. The
        provisional stops apply in position mode, where ticks are 0..4095.
        """
        settings = load_joint_settings(settings_path)
        with Path(calibration_path).open(encoding="utf-8") as file:
            calibration = json.load(file)

        lower: list[int] = []
        upper: list[int] = []
        for index, name in enumerate(JOINT_NAMES, start=1):
            joint = calibration[name]
            panel = settings[str(index)]
            cal_lo, cal_hi = int(joint["range_min"]), int(joint["range_max"])
            panel_lo = cal_lo if panel["lower"] is None else panel["lower"]
            panel_hi = cal_hi if panel["upper"] is None else panel["upper"]
            lo = max(cal_lo, panel_lo, GRIPPER_MIN_TICK if name == "gripper" else 0)
            hi = min(cal_hi, panel_hi, WRIST_ROLL_MAX_TICK if name == "wrist_roll" else 4095)
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
