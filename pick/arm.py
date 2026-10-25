"""The arm as the pick routine sees it: poses, level nudges, vertical moves, and the gripper.

Each motion runs the measured mover, which owns the servo bus only while the
arm moves; it stops with BusBusyError while the dashboard holds the bus. Stop
the dashboard first. Torque stays on throughout.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any, Callable

import numpy as np

from arm_control.pose_runner import MeasuredPoseRunner
from motion_core import CALIBRATION_PATH, JOINT_NAMES, SafetyBounds
from so101_ik import URDF_PATH, SO101Kinematics


# Cartesian moves are split into IK legs of at most this length.
LEG_M = 0.025

# A Cartesian move ends within this distance of its target, as the encoders
# measure it. The servos rest a few ticks short of small goals, so each move
# may add a couple of short correcting legs.
ARRIVED_M = 0.005
CORRECTING_LEGS = 2

# A held object keeps the jaws at least this far open of their squeeze goal.
HOLD_SLACK_TICKS = 15

# The measured-mover settings docs/KINEMATICS.md recommends for Cartesian moves.
MOVE_OPTIONS = {"tolerance": 15, "trim_deadband": True, "hold_target_on_success": True}

log = logging.getLogger(__name__)


class PickArm:
    def __init__(
        self,
        port: str,
        bounds: SafetyBounds,
        kinematics: SO101Kinematics,
        runner_factory: Callable[[str, SafetyBounds], Any] = MeasuredPoseRunner,
    ) -> None:
        self.port = port
        self.bounds = bounds
        self.kinematics = kinematics
        self.runner_factory = runner_factory

        # Measured joint ticks after the last motion; None until the first one.
        self.ticks: tuple[int, ...] | None = None

        # The gripper's squeeze goal while the jaws grip something.
        self.gripper_hold: int | None = None

    @classmethod
    def from_files(
        cls,
        port: str,
        settings_path: str | Path | None = None,
        calibration_path: str | Path = CALIBRATION_PATH,
    ) -> PickArm:
        """Build the arm from this robot's saved bounds, calibration, and model."""
        bounds = SafetyBounds.from_files(settings_path, calibration_path)
        return cls(port, bounds, SO101Kinematics(calibration_path, URDF_PATH, bounds))

    @property
    def holding(self) -> bool:
        """True while something keeps the jaws open of their squeeze goal."""
        return self.gripper_hold is not None and self.ticks[5] - self.gripper_hold > HOLD_SLACK_TICKS

    def move_to(self, pose: tuple[int, ...]) -> None:
        """Move all six joints to ``pose``. While gripping, the gripper keeps its squeeze instead."""
        holding_object = self.gripper_hold is not None
        if holding_object:
            pose = pose[:5] + (self.gripper_hold,)

        runner = self.runner_factory(self.port, self.bounds)
        self._record(runner.move(pose, holding_object=holding_object, **MOVE_OPTIONS))

    def nudge(self, dx_m: float, dy_m: float) -> None:
        """Move the jaws sideways by (dx, dy) meters along the gripper's own x and y axes.

        With the gripper pointing down, its x and y axes lie flat. The camera
        turns with the gripper, so a picture direction always matches the same
        gripper axis, whatever the base pan and wrist roll are.
        """
        rotation = self.kinematics.forward(self._measured())[:3, :3]
        delta = rotation[:, :2] @ np.array([dx_m, dy_m])

        # A gripper that isn't quite vertical tilts its x and y axes a little.
        # Drop that vertical part so a nudge never raises or lowers the jaws.
        delta[2] = 0.0
        self._shift(delta)

    def move_vertically(self, dz_m: float) -> None:
        """Raise (positive) or lower (negative) the jaws straight up or down."""
        self._shift(np.array([0.0, 0.0, dz_m]))

    def open_gripper(self) -> None:
        self.gripper_hold = None
        self.move_to(self._measured()[:5] + (self.bounds.upper[5],))

    def close_gripper(self) -> bool:
        """Close the jaws; True if they stopped on something, which they keep squeezing."""
        runner = self.runner_factory(self.port, self.bounds)
        result = runner.close_gripper(self.bounds.lower[5])
        self._record(result)
        self.gripper_hold = result["gripper_hold"] if result["contact"] else None
        return result["contact"]

    def _shift(self, delta: np.ndarray) -> None:
        """Move the jaws by ``delta`` meters in the arm's base frame, in short IK legs.

        Each leg is planned from the pose the encoders measured after the
        previous one, so the servos' small shortfalls don't add up.
        """
        target = self.kinematics.forward(self._measured())[:3, 3] + delta
        planned_legs = math.ceil(np.linalg.norm(delta) / LEG_M)

        for _ in range(planned_legs + CORRECTING_LEGS):
            remaining = target - self.kinematics.forward(self.ticks)[:3, 3]
            distance = np.linalg.norm(remaining)
            if distance <= ARRIVED_M:
                return

            leg = remaining * min(1.0, LEG_M / distance)
            self.move_to(self.kinematics.translate(self.ticks, tuple(leg)).target_ticks)

        log.warning("Cartesian move ended %.0f mm from its target",
                    1000 * np.linalg.norm(target - self.kinematics.forward(self.ticks)[:3, 3]))

    def _measured(self) -> tuple[int, ...]:
        if self.ticks is None:
            raise RuntimeError("Move to a pose first, so the arm's position is known")
        return self.ticks

    def _record(self, result: dict[str, Any]) -> None:
        self.ticks = tuple(result["positions"][name] for name in JOINT_NAMES)
