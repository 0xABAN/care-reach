"""Model-space SO-101 FK and local, position-focused Cartesian IK.

LeRobot's PlaCo model uses degrees at the five arm joints. Our bus uses raw
STS3215 encoder ticks. The gripper is preserved in ticks and is not an IK DOF.
Every result here is a candidate computed from the nominal URDF, not a measured
tool position or a collision-checked motion command. This module never opens
the servo bus.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Callable, Sequence

import numpy as np

from motion_core import JOINT_NAMES, SafetyBounds, validate_pose


ARM_NAMES = JOINT_NAMES[:5]
TICKS_PER_REVOLUTION = 4095

# The pinned official model cached on the Pi; see docs/KINEMATICS.md.
URDF_PATH = Path.home() / ".cache/care-reach/so101-model/so101_new_calib.urdf"


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


@dataclass(frozen=True)
class XYZPlan:
    """Candidate short Cartesian path in the nominal robot base frame."""

    start_ticks: tuple[int, ...]
    target_xyz_m: tuple[float, float, float]
    waypoints: tuple[IKResult, ...]
    predicted_xyz_m: tuple[float, float, float]

    @property
    def target_ticks(self) -> tuple[int, ...]:
        return self.waypoints[-1].target_ticks if self.waypoints else self.start_ticks


def _xyz(values: Sequence[float], label: str) -> np.ndarray:
    """Convert an explicit finite XYZ tuple without accepting booleans."""
    try:
        if len(values) != 3 or any(isinstance(value, bool) for value in values):
            raise ValueError
        xyz = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise IKError(f"{label} must have three finite meter values") from exc
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise IKError(f"{label} must have three finite meter values")
    return xyz


def _xyz_tuple(values: np.ndarray) -> tuple[float, float, float]:
    return tuple(float(value) for value in values)


class SO101Kinematics:
    def __init__(
        self,
        calibration_path: str | Path,
        urdf_path: str | Path,
        bounds: SafetyBounds,
        *,
        kinematics_factory: Callable | None = None,
        tool_frame_name: str = "gripper_frame_link",
    ) -> None:
        if not isinstance(tool_frame_name, str) or not tool_frame_name:
            raise IKError("tool_frame_name must name a frame in the URDF")
        with Path(calibration_path).open(encoding="utf-8") as file:
            calibration = json.load(file)
        if not Path(urdf_path).is_file():
            raise FileNotFoundError(urdf_path)
        try:
            if any(int(calibration[name]["drive_mode"]) != 0 for name in JOINT_NAMES):
                raise IKError("Nonzero servo drive mode needs a different sign mapping")
            ranges = [(int(calibration[name]["range_min"]), int(calibration[name]["range_max"]))
                      for name in JOINT_NAMES]
            if any(not 0 <= low < high <= 4095 for low, high in ranges):
                raise IKError("Invalid SO-101 calibration ranges")
            self.midpoints = np.array(
                [(low + high) / 2 for low, high in ranges[:5]], dtype=float
            )
        except IKError:
            raise
        except (KeyError, TypeError, ValueError) as exc:
            raise IKError("Invalid SO-101 calibration") from exc
        self.bounds = bounds
        if kinematics_factory is None:
            from lerobot.model.kinematics import RobotKinematics
            kinematics_factory = RobotKinematics
        self.model = kinematics_factory(
            urdf_path=str(urdf_path), target_frame_name=tool_frame_name, joint_names=list(ARM_NAMES)
        )
        self.tool_frame_name = tool_frame_name

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
        """Nominal base-to-tool transform for encoder ticks (not observed XYZ)."""
        pose = np.asarray(self.model.forward_kinematics(self.ticks_to_degrees(ticks)), dtype=float)
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            raise IKError("FK returned an invalid transform")
        return pose.copy()

    def predicted_xyz(self, ticks: tuple[int, ...]) -> tuple[float, float, float]:
        """Nominal XYZ of ``tool_frame_name`` in the URDF base frame."""
        return _xyz_tuple(self.forward(ticks)[:3, 3])

    @staticmethod
    def _validate_ik_limits(max_step_m, max_joint_step_ticks, max_residual_mm,
                            orientation_weight):
        if (isinstance(max_step_m, bool) or not isinstance(max_step_m, (int, float))
                or not math.isfinite(max_step_m) or not 0 < max_step_m <= 0.05):
            raise IKError("max_step_m must be within (0, 0.05] meters")
        if (type(max_joint_step_ticks) is not int
                or not 1 <= max_joint_step_ticks <= 500):
            raise IKError("max_joint_step_ticks must be an integer from 1 to 500")
        if (isinstance(max_residual_mm, bool) or not isinstance(max_residual_mm, (int, float))
                or not math.isfinite(max_residual_mm) or max_residual_mm <= 0):
            raise IKError("max_residual_mm must be finite and positive")
        if (isinstance(orientation_weight, bool)
                or not isinstance(orientation_weight, (int, float))
                or not math.isfinite(orientation_weight)
                or not 0 <= orientation_weight <= 1):
            raise IKError("orientation_weight must be between 0 and 1")

    def solve_xyz(
        self,
        start_ticks: tuple[int, ...],
        target_xyz_m: tuple[float, float, float],
        *,
        max_step_m: float = 0.03,
        max_joint_step_ticks: int = 250,
        max_residual_mm: float = 3.0,
        orientation_weight: float = 0.0,
    ) -> IKResult:
        """Solve one nearby absolute base-frame XYZ target, seeded from ticks.

        Position is prioritized by default. The chosen tool orientation is
        unconstrained when ``orientation_weight`` is zero. No hardware moves.
        """
        start = validate_pose(start_ticks, self.bounds)
        target_xyz = _xyz(target_xyz_m, "Target XYZ")
        self._validate_ik_limits(max_step_m, max_joint_step_ticks,
                                 max_residual_mm, orientation_weight)
        start_transform = self.forward(start)
        start_xyz = start_transform[:3, 3]
        if np.linalg.norm(target_xyz - start_xyz) > max_step_m + 1e-12:
            raise IKError("Cartesian step exceeds its configured limit")
        if np.array_equal(target_xyz, start_xyz):
            start_point = _xyz_tuple(start_xyz)
            return IKResult(start, start, start_point, start_point, start_point, 0.0)
        target_transform = start_transform.copy()
        target_transform[:3, 3] = target_xyz
        current_deg = self.ticks_to_degrees(start)
        try:
            solved_deg = self.model.inverse_kinematics(
                current_deg, target_transform, position_weight=1.0,
                orientation_weight=orientation_weight, max_iters=12,
            )
            target = self.degrees_to_ticks(np.asarray(solved_deg), start[5])
        except IKError:
            raise
        except Exception as exc:
            raise IKError("IK solver failed") from exc
        if any(abs(a - b) > max_joint_step_ticks for a, b in zip(target[:5], start[:5])):
            raise IKError("IK solution jumps too far in joint space")
        predicted_xyz = self.forward(target)[:3, 3]
        residual_mm = float(np.linalg.norm(predicted_xyz - target_xyz) * 1000)
        if not math.isfinite(residual_mm) or residual_mm > max_residual_mm:
            raise IKError(f"IK endpoint residual {residual_mm:.2f} mm exceeds limit")
        return IKResult(start, target, _xyz_tuple(start_xyz), _xyz_tuple(target_xyz),
                        _xyz_tuple(predicted_xyz), residual_mm)

    def translate(
        self,
        start_ticks: tuple[int, ...],
        delta_xyz_m: tuple[float, float, float],
        *,
        max_step_m: float = 0.03,
        max_joint_step_ticks: int = 250,
        max_residual_mm: float = 3.0,
        orientation_weight: float = 0.0,
    ) -> IKResult:
        start = validate_pose(start_ticks, self.bounds)
        delta = _xyz(delta_xyz_m, "Translation")
        return self.solve_xyz(
            start, _xyz_tuple(np.asarray(self.predicted_xyz(start)) + delta),
            max_step_m=max_step_m, max_joint_step_ticks=max_joint_step_ticks,
            max_residual_mm=max_residual_mm, orientation_weight=orientation_weight,
        )

    def plan_xyz(
        self,
        start_ticks: tuple[int, ...],
        target_xyz_m: tuple[float, float, float],
        *,
        max_step_m: float = 0.03,
        max_path_m: float = 0.15,
        max_waypoints: int = 12,
        max_joint_step_ticks: int = 250,
        max_residual_mm: float = 3.0,
        orientation_weight: float = 0.0,
        validate_waypoint: Callable[[tuple[int, ...], tuple[int, ...]], None] | None = None,
        validate_plan: Callable[[tuple[int, ...], tuple[tuple[int, ...], ...]], None] | None = None,
    ) -> XYZPlan:
        """Return bounded candidate waypoints on a straight model-space line.

        Each step starts from the previous waypoint's predicted joint pose.
        The caller may supply pure validators for individual waypoints and
        the complete path. A complete-path check is needed for policies that
        forbid different joints moving in separate segments.
        This does not validate the path between waypoints for collision or
        measure actual encoder/tool tracking.
        """
        start = validate_pose(start_ticks, self.bounds)
        target_xyz = _xyz(target_xyz_m, "Target XYZ")
        self._validate_ik_limits(max_step_m, max_joint_step_ticks,
                                 max_residual_mm, orientation_weight)
        if (isinstance(max_path_m, bool) or not isinstance(max_path_m, (int, float))
                or not math.isfinite(max_path_m) or not 0 < max_path_m <= 0.25):
            raise IKError("max_path_m must be within (0, 0.25] meters")
        if type(max_waypoints) is not int or not 1 <= max_waypoints <= 50:
            raise IKError("max_waypoints must be an integer from 1 to 50")
        if validate_waypoint is not None and not callable(validate_waypoint):
            raise IKError("validate_waypoint must be callable")
        if validate_plan is not None and not callable(validate_plan):
            raise IKError("validate_plan must be callable")
        start_xyz = np.asarray(self.predicted_xyz(start))
        difference = target_xyz - start_xyz
        distance = float(np.linalg.norm(difference))
        if distance > max_path_m + 1e-12:
            raise IKError("Cartesian path exceeds its configured limit")
        if distance == 0:
            plan = XYZPlan(start, _xyz_tuple(target_xyz), (), _xyz_tuple(start_xyz))
            if validate_plan is not None:
                try:
                    validate_plan(start, ())
                except (ValueError, RuntimeError) as exc:
                    raise IKError(f"Candidate path rejected: {exc}") from exc
            return plan
        # Leave room for each quantized waypoint's residual when the next leg
        # is measured from its predicted position rather than the ideal line.
        count = math.ceil(distance / (0.8 * max_step_m))
        if count > max_waypoints:
            raise IKError("Cartesian path needs more waypoints than allowed")
        current = start
        waypoints: list[IKResult] = []
        for index in range(1, count + 1):
            desired = start_xyz + difference * index / count
            result = self.solve_xyz(
                current, _xyz_tuple(desired), max_step_m=max_step_m,
                max_joint_step_ticks=max_joint_step_ticks,
                max_residual_mm=max_residual_mm,
                orientation_weight=orientation_weight,
            )
            if validate_waypoint is not None:
                try:
                    validate_waypoint(current, result.target_ticks)
                except (ValueError, RuntimeError) as exc:
                    raise IKError(f"Waypoint {index} rejected: {exc}") from exc
            waypoints.append(result)
            current = result.target_ticks
        final_xyz = np.asarray(waypoints[-1].predicted_xyz_m)
        final_residual_mm = float(np.linalg.norm(final_xyz - target_xyz) * 1000)
        if not math.isfinite(final_residual_mm) or final_residual_mm > max_residual_mm:
            raise IKError("Final model-space XYZ residual exceeds limit")
        plan = XYZPlan(start, _xyz_tuple(target_xyz), tuple(waypoints),
                       _xyz_tuple(final_xyz))
        if validate_plan is not None:
            try:
                validate_plan(start, tuple(result.target_ticks for result in waypoints))
            except (ValueError, RuntimeError) as exc:
                raise IKError(f"Candidate path rejected: {exc}") from exc
        return plan
