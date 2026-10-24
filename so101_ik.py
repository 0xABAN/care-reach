"""Calibrated SO-101 FK and local Cartesian IK; no hardware access.

LeRobot's PlaCo model uses degrees at the five arm joints. Our bus uses raw
STS3215 encoder ticks. The gripper is preserved in ticks and is not an IK DOF.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Callable

import numpy as np

from motion_core import JOINT_NAMES, SafetyBounds, validate_pose


ARM_NAMES = JOINT_NAMES[:5]
TICKS_PER_REVOLUTION = 4095


class IKError(ValueError):
    """The requested Cartesian step cannot be commanded safely."""


@dataclass(frozen=True)
class IKResult:
    start_ticks: tuple[int, ...]
    target_ticks: tuple[int, ...]
    start_xyz_m: tuple[float, float, float]
    target_xyz_m: tuple[float, float, float]
    predicted_xyz_m: tuple[float, float, float]
    residual_mm: float


class SO101Kinematics:
    def __init__(
        self,
        calibration_path: str | Path,
        urdf_path: str | Path,
        bounds: SafetyBounds,
        *,
        kinematics_factory: Callable | None = None,
    ) -> None:
        with Path(calibration_path).open(encoding="utf-8") as file:
            calibration = json.load(file)
        if not Path(urdf_path).is_file():
            raise FileNotFoundError(urdf_path)
        try:
            if any(int(calibration[name]["drive_mode"]) != 0 for name in JOINT_NAMES):
                raise IKError("Nonzero servo drive mode needs a different sign mapping")
            self.midpoints = np.array(
                [(int(calibration[name]["range_min"]) + int(calibration[name]["range_max"])) / 2
                 for name in ARM_NAMES], dtype=float
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise IKError("Invalid SO-101 calibration") from exc
        self.bounds = bounds
        if kinematics_factory is None:
            from lerobot.model.kinematics import RobotKinematics
            kinematics_factory = RobotKinematics
        self.model = kinematics_factory(
            urdf_path=str(urdf_path), target_frame_name="gripper_frame_link", joint_names=list(ARM_NAMES)
        )

    def ticks_to_degrees(self, ticks: tuple[int, ...]) -> np.ndarray:
        pose = validate_pose(ticks, self.bounds)
        return (np.array(pose[:5], dtype=float) - self.midpoints) * 360 / TICKS_PER_REVOLUTION

    def degrees_to_ticks(self, degrees: np.ndarray, gripper_tick: int) -> tuple[int, ...]:
        angles = np.asarray(degrees, dtype=float)
        if angles.shape != (5,) or not np.isfinite(angles).all():
            raise IKError("IK returned invalid joint angles")
        if type(gripper_tick) is not int:
            raise IKError("Gripper tick must be an integer")
        raw = np.rint(self.midpoints + angles * TICKS_PER_REVOLUTION / 360)
        if not np.isfinite(raw).all() or np.max(np.abs(raw)) > 100000:
            raise IKError("IK returned impossible encoder values")
        try:
            return validate_pose(tuple(int(value) for value in raw) + (gripper_tick,), self.bounds)
        except ValueError as exc:
            raise IKError(str(exc)) from exc

    def forward(self, ticks: tuple[int, ...]) -> np.ndarray:
        pose = np.asarray(self.model.forward_kinematics(self.ticks_to_degrees(ticks)), dtype=float)
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            raise IKError("FK returned an invalid transform")
        return pose.copy()

    def translate(
        self,
        start_ticks: tuple[int, ...],
        delta_xyz_m: tuple[float, float, float],
        *,
        max_step_m: float = 0.03,
        max_joint_step_ticks: int = 250,
        max_residual_mm: float = 3.0,
    ) -> IKResult:
        start = validate_pose(start_ticks, self.bounds)
        delta = np.asarray(delta_xyz_m, dtype=float)
        if delta.shape != (3,) or not np.isfinite(delta).all():
            raise IKError("Translation must have three finite meter values")
        if not 0 < max_step_m <= 0.05 or np.linalg.norm(delta) > max_step_m + 1e-12:
            raise IKError("Cartesian step exceeds its configured limit")
        if not 1 <= max_joint_step_ticks <= 500 or max_residual_mm <= 0:
            raise IKError("Invalid IK safety limits")
        start_transform = self.forward(start)
        target_transform = start_transform.copy()
        target_transform[:3, 3] += delta
        current_deg = self.ticks_to_degrees(start)
        try:
            solved_deg = self.model.inverse_kinematics(
                current_deg, target_transform, position_weight=1.0,
                orientation_weight=0.01, max_iters=12,
            )
            target = self.degrees_to_ticks(np.asarray(solved_deg), start[5])
        except IKError:
            raise
        except Exception as exc:
            raise IKError("IK solver failed") from exc
        if any(abs(a - b) > max_joint_step_ticks for a, b in zip(target[:5], start[:5])):
            raise IKError("IK solution jumps too far in joint space")
        predicted = self.forward(target)
        residual_mm = float(np.linalg.norm(predicted[:3, 3] - target_transform[:3, 3]) * 1000)
        if not math.isfinite(residual_mm) or residual_mm > max_residual_mm:
            raise IKError(f"IK endpoint residual {residual_mm:.2f} mm exceeds limit")
        xyz = lambda vector: tuple(float(value) for value in vector[:3, 3])
        return IKResult(start, target, xyz(start_transform), xyz(target_transform), xyz(predicted), residual_mm)
