"""The arm as the pick routine sees it: named poses and level nudges in meters.

Each move runs the measured mover, so it owns the servo bus only while the arm
moves. Stop the dashboard service first. Torque stays on throughout.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Callable

import numpy as np

from care_reach.hardware.measured_pose_runner import MeasuredPoseRunner
from care_reach.motion.bounds import SafetyBounds
from care_reach.motion.kinematics import SO101Kinematics
from care_reach.robot import CALIBRATION_PATH, JOINT_NAMES, JOINT_SETTINGS_PATH, SO101_URDF_PATH


# Longer nudges are split into IK legs of at most this length.
NUDGE_LEG_M = 0.025

# The measured-mover settings docs/KINEMATICS.md recommends for Cartesian moves.
MOVE_OPTIONS = {"tolerance": 15, "trim_deadband": True, "hold_target_on_success": True}


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

        # Measured joint ticks after the last move; None until the first one.
        self.ticks: tuple[int, ...] | None = None

    @classmethod
    def from_files(
        cls,
        port: str,
        settings_path: str | Path = JOINT_SETTINGS_PATH,
        calibration_path: str | Path = CALIBRATION_PATH,
    ) -> PickArm:
        """Build the arm from this robot's saved bounds, calibration, and model."""
        bounds = SafetyBounds.from_files(settings_path, calibration_path)
        return cls(port, bounds, SO101Kinematics(calibration_path, SO101_URDF_PATH, bounds))

    def move_to(self, pose: tuple[int, ...]) -> None:
        result = self.runner_factory(self.port, self.bounds).move(pose, **MOVE_OPTIONS)
        self.ticks = tuple(result["positions"][name] for name in JOINT_NAMES)

    def nudge(self, dx_m: float, dy_m: float) -> None:
        """Move the jaws sideways by (dx, dy) meters along the gripper's own x and y axes.

        With the gripper pointing down, its x and y axes lie flat. The camera
        turns with the gripper, so a picture direction always matches the same
        gripper axis, whatever the base pan and wrist roll are.

        Each leg is planned from the pose the encoders measured after the
        previous one, so leg errors don't add up.
        """
        if self.ticks is None:
            raise RuntimeError("Move to a pose before nudging")

        rotation = self.kinematics.forward(self.ticks)[:3, :3]
        delta = rotation[:, :2] @ np.array([dx_m, dy_m])

        # A gripper that isn't quite vertical tilts its x and y axes a little.
        # Drop that vertical part so a nudge never raises or lowers the jaws.
        delta[2] = 0.0

        legs = max(1, math.ceil(np.linalg.norm(delta) / NUDGE_LEG_M))
        for _ in range(legs):
            leg = self.kinematics.translate(self.ticks, tuple(delta / legs))
            self.move_to(leg.target_ticks)
