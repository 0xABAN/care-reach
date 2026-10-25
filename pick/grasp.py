"""Plan how the open jaws reach a located object: a close look, then a straight final approach.

The object is treated as an upright cylinder resting on a surface at its
bottom (see pick.locate.Target). The jaws close horizontally across it, near
its top so the gripper body clears it, with the fixed jaw just outside its
near side and the moving jaw opened past its far side.

An SO-101 points its jaws only within the upright plane through the base's
pan axis, tilted anywhere from straight down to level, so the planner tries
tilts from straight down outward and keeps the first whose whole approach a
joint solution reaches without the gripper meeting the object or its surface.

The wrist camera sits behind and beside the jaws and looks past them, so it
sees whatever is in front of the jaws at the edge of its picture. For the
close look, the arm holds the jaws near the grasp's angle, placed so the
camera looks straight at the object with all of it in view.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from motion_core import SafetyBounds
from pick.arm import MAX_LEAD_TICKS
from pick.locate import Target, outline
from pick.rig import CameraModel
from so101_ik import TICKS_PER_REVOLUTION, SO101Kinematics


# The jaws open about 9 cm at their tips; leave room on the moving jaw's side.
MAX_WIDTH_M = 0.08

# Grip this far below the object's top, or at its middle if it is shorter.
GRIP_BELOW_TOP_M = 0.02

# The jaw tips pass the object's middle by this much along the approach.
GRIP_DEPTH_M = 0.015

# The fixed jaw stops this far outside the object's near side.
FIXED_JAW_GAP_M = 0.005

# The final approach: straight along the jaws, in steps of this length.
APPROACH_M = 0.06
APPROACH_STEP_M = 0.01

# Every part of the gripper but the fixed jaw stays this far from the object, and all
# of it this far above the surface the object rests on.
CLEARANCE_M = 0.01

# Approach tilts from straight down (0) to level (90), in degrees.
TILTS_DEG = range(0, 91, 10)

# A five-joint arm can't always match the planned jaw angle exactly.
MAX_TURN_DEG = 6.0

# The close look: the wrist camera this far from the object's middle, nearest first.
LOOK_DISTANCES_M = (0.15, 0.20, 0.25)

# The whole object stays this far inside the wrist camera's picture for the close look.
LOOK_MARGIN_PX = 20

# Joint solutions keep this many ticks inside the safety bounds.
BOUNDS_MARGIN_TICKS = 20


def _box(low, high, spacing_m: float = 0.01) -> np.ndarray:
    """Points filling a box at most ``spacing_m`` apart."""
    axes = [np.linspace(lo, hi, int(np.ceil((hi - lo) / spacing_m)) + 1) for lo, hi in zip(low, high, strict=True)]
    return np.array(np.meshgrid(*axes)).reshape(3, -1).T


# Points on the gripper in its tool frame (gripper_frame_link: z out through
# the jaw tips, x from the moving jaw toward the fixed one), from the official
# SO-101 URDF meshes. Fully open, the moving jaw folds back beside the servo,
# which leaves the slot in front of the fixed jaw empty for the object.
FIXED_JAW = _box((0.0, -0.009, -0.06), (0.021, 0.009, 0.006))
GRIPPER = np.vstack([
    _box((-0.04, -0.03, -0.20), (0.03, 0.025, -0.06)),     # servo, body, and the wrist behind them
    _box((-0.11, -0.024, -0.09), (-0.018, 0.024, -0.065)),  # the moving jaw, folded open
])
CAMERA_HALF_SIZE_M = 0.02  # the wrist camera and its mount, around the calibrated lens position


class GraspError(RuntimeError):
    """No safe, reachable grasp for the object where it is."""


@dataclass(frozen=True)
class GraspPlan:
    """Joint ticks along the final approach, pre-grasp first, grasp last; the gripper open throughout.

    ``tilt_deg`` is the approach's tilt from straight down; ``side`` which way the jaws close.
    """

    approach: tuple[tuple[int, ...], ...]
    tilt_deg: int
    side: int

    @property
    def pregrasp(self) -> tuple[int, ...]:
        return self.approach[0]

    @property
    def grasp(self) -> tuple[int, ...]:
        return self.approach[-1]


class GraspPlanner:
    def __init__(self, kinematics: SO101Kinematics, bounds: SafetyBounds, wrist: CameraModel,
                 pan_axis_xy: np.ndarray) -> None:
        """``wrist``: the wrist camera's calibration, mounted in the tool frame.
        ``pan_axis_xy``: where the base's pan axis stands, in the base frame.
        """
        self.kinematics = kinematics
        self.wrist = wrist
        self.pan_axis_xy = np.asarray(pan_axis_xy)
        self.lower = np.array(bounds.lower[:5]) + BOUNDS_MARGIN_TICKS
        self.upper = np.array(bounds.upper[:5]) - BOUNDS_MARGIN_TICKS
        self.gripper_open = bounds.upper[5]
        lens = wrist.mount[:3, 3]
        self.gripper = np.vstack([GRIPPER, _box(lens - CAMERA_HALF_SIZE_M, lens + CAMERA_HALF_SIZE_M)])

    def plan(self, target: Target, start: tuple[int, ...]) -> GraspPlan:
        """The first approach, from straight down outward, that is reachable and clear from ``start``."""
        if target.width_m > MAX_WIDTH_M:
            raise GraspError(f"The object is {target.width_m * 100:.0f} cm wide; "
                             f"the jaws fit up to {MAX_WIDTH_M * 100:.0f} cm")

        middle_z = (target.top_z_m + target.bottom_z_m) / 2
        grip = np.array([*target.center[:2], max(target.top_z_m - GRIP_BELOW_TOP_M, middle_z)])

        # In the tool frame, the object's middle sits beside the fixed jaw and behind its tip.
        held_at = np.array([-(target.width_m / 2 + FIXED_JAW_GAP_M), 0.0, -GRIP_DEPTH_M])
        for tilt_deg in TILTS_DEG:
            for side in (1, -1):
                approach = self._approach(target, self._frame(grip, held_at, np.radians(tilt_deg), side), start)
                if approach is not None and self._leg_is_clear(start, approach[0], target):
                    return GraspPlan(approach, tilt_deg, side)
        raise GraspError(f"No reachable approach clears the object at {target}")

    def look_pose(self, target: Target, grasp: GraspPlan, start: tuple[int, ...]) -> tuple[int, ...]:
        """Joint ticks near ``grasp`` from which the wrist camera looks straight at the whole object.

        Tries the grasp's own tilt and side first, then tilts farther from it.
        """
        lens, view = self.wrist.mount[:3, 3], self.wrist.mount[:3, 2]
        for tilt_deg in sorted(TILTS_DEG, key=lambda tilt: abs(tilt - grasp.tilt_deg)):
            for side in (grasp.side, -grasp.side):
                for distance in LOOK_DISTANCES_M:
                    rotation, tip = self._frame(target.center, lens + view * distance, np.radians(tilt_deg), side)
                    ticks = self._solve(rotation, tip, [grasp.pregrasp, start])
                    if (ticks is not None and not self.collides(ticks, target) and self._sees_all_of(target, ticks)
                            and self._leg_is_clear(start, ticks, target)):
                        return ticks
        raise GraspError(f"No reachable pose lets the wrist camera see all of the object at {target}")

    def collides(self, ticks: tuple[int, ...], target: Target) -> bool:
        """Whether the gripper at ``ticks`` comes too close to the object or the surface it rests on.

        The fixed jaw slides past the object's side, so it may come within half its gap.
        """
        pose = self.kinematics.forward(ticks)
        for points, margin in ((self.gripper, CLEARANCE_M), (FIXED_JAW, FIXED_JAW_GAP_M / 2)):
            world = points @ pose[:3, :3].T + pose[:3, 3]
            radial = np.linalg.norm(world[:, :2] - target.center[:2], axis=1)
            inside = (radial < target.width_m / 2 + margin) & (world[:, 2] < target.top_z_m + margin)
            if inside.any() or (world[:, 2] < target.bottom_z_m + CLEARANCE_M).any():
                return True
        return False

    def _approach(self, target: Target, frame: tuple[np.ndarray, np.ndarray],
                  start: tuple[int, ...]) -> tuple[tuple[int, ...], ...] | None:
        """Joint ticks from the pre-grasp to the grasp ``frame`` (rotation, tip), or None."""
        rotation, tip = frame
        steps = round(APPROACH_M / APPROACH_STEP_M)
        approach = []
        seeds = [start]
        # Solve the grasp first, the hardest pose to reach, then back out along the jaws.
        for step in range(steps + 1):
            ticks = self._solve(rotation, tip - rotation[:, 2] * step * APPROACH_STEP_M, seeds)
            if ticks is None or self.collides(ticks, target):
                return None
            approach.append(ticks)
            seeds = [ticks]
        return tuple(reversed(approach))

    def _frame(self, point: np.ndarray, in_tool: np.ndarray, tilt: float, side: int) -> tuple[np.ndarray, np.ndarray]:
        """The tool's rotation and jaw-tip position that put tool-frame point ``in_tool`` at base-frame ``point``.

        The jaws point ``tilt`` from straight down, within the upright plane
        through the pan axis and the tip, and close horizontally toward ``side``.
        The tip and that plane depend on each other, so settle them together.
        """
        tip = point
        for _ in range(5):
            outward = np.append(tip[:2] - self.pan_axis_xy, 0.0)
            outward /= np.linalg.norm(outward)
            approach = np.cos(tilt) * np.array([0.0, 0.0, -1.0]) + np.sin(tilt) * outward
            closing = side * np.cross([0.0, 0.0, 1.0], outward)
            rotation = np.column_stack([closing, np.cross(approach, closing), approach])
            tip = point - rotation @ in_tool
        return rotation, tip

    def _solve(self, rotation: np.ndarray, position: np.ndarray,
               seeds: list[tuple[int, ...]]) -> tuple[int, ...] | None:
        """Joint ticks that put the tool at the pose, within MAX_TURN_DEG of its angle, or None."""
        goal = np.eye(4)
        goal[:3, :3], goal[:3, 3] = rotation, position
        model = self.kinematics.model
        for seed in seeds:
            degrees = self.kinematics.ticks_to_degrees(seed)
            # Position first: a five-joint arm can't match every orientation.
            for _ in range(20):
                degrees = np.asarray(model.inverse_kinematics(degrees, goal, position_weight=1.0,
                                                              orientation_weight=0.2, max_iters=50))[:5]
            reached = np.asarray(model.forward_kinematics(degrees))
            turn = np.degrees(np.arccos(np.clip((np.trace(reached[:3, :3].T @ rotation) - 1) / 2, -1, 1)))
            ticks = np.rint(self.kinematics.midpoints + degrees * TICKS_PER_REVOLUTION / 360)
            if (np.linalg.norm(reached[:3, 3] - position) < 0.003 and turn < MAX_TURN_DEG
                    and np.all(ticks >= self.lower) and np.all(ticks <= self.upper)):
                return tuple(int(tick) for tick in ticks) + (self.gripper_open,)
        return None

    def _sees_all_of(self, target: Target, ticks: tuple[int, ...]) -> bool:
        """Whether the whole object projects well inside the wrist camera's picture."""
        world_to_camera = np.linalg.inv(self.kinematics.forward(ticks) @ self.wrist.mount)
        points = outline(*target.center[:2], target.width_m, target.bottom_z_m, target.top_z_m)
        if np.any((points @ world_to_camera[:3, :3].T + world_to_camera[:3, 3])[:, 2] <= 0):
            return False
        pixels, _ = cv2.projectPoints(points, cv2.Rodrigues(world_to_camera[:3, :3])[0], world_to_camera[:3, 3],
                                      self.wrist.matrix, self.wrist.distortion)
        pixels = pixels.reshape(-1, 2)
        upper = np.array(self.wrist.image_size) - LOOK_MARGIN_PX
        return bool(np.all(pixels >= LOOK_MARGIN_PX) and np.all(pixels <= upper))

    def _leg_is_clear(self, start: tuple[int, ...], end: tuple[int, ...], target: Target) -> bool:
        """Step the joint-space move as the measured mover does and check each step."""
        pose, end = np.array(start, float), np.array(end, float)
        while True:
            if self.collides(tuple(int(tick) for tick in pose), target):
                return False
            if np.array_equal(pose, end):
                return True
            pose += np.clip(end - pose, -MAX_LEAD_TICKS, MAX_LEAD_TICKS)
