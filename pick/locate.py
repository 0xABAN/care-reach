"""Where a named object is: the detector boxes it in both pictures, and an upright cylinder is fitted to both boxes.

The two cameras' lines of sight through the boxes' middles give a first
estimate. A box's middle isn't the object's middle, though: a camera looking
down sees the top as well as the side, which moves the middle by a centimeter
or more. So the estimate is refined to the upright cylinder whose outline, as
each camera sees it, fills exactly that camera's box. The wrist camera sees
the object from above and the shoulder camera from the side, so together they
pin down its position, width, and height.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Protocol

import cv2
import numpy as np

from pick.detector import Box
from pick.rig import Rig


CAMERAS = ("wrist", "shoulder")

# Lines of sight at the same object pass this close; farther apart, the two
# cameras matched different things.
MAX_SIGHT_GAP_M = 0.03

# Nearly parallel lines of sight can't tell how far away the object is.
MIN_SIGHT_ANGLE_DEG = 5.0

# A box this close to the picture's edge may cut the object off.
EDGE_PX = 3

# The fit starts from a cylinder this wide and tall around the first estimate.
START_SIZE_M = 0.05

FIT_ITERATIONS = 30


class NotLocated(RuntimeError):
    """The object isn't fully in both pictures, or the two pictures don't agree on where it is."""


class Detector(Protocol):
    def find(self, jpeg: bytes, name: str) -> Box | None: ...


@dataclass(frozen=True)
class Target:
    """An object in the arm's base frame, in meters: an upright cylinder ``width_m`` across."""

    center: np.ndarray
    width_m: float
    bottom_z_m: float
    top_z_m: float

    def __str__(self) -> str:
        x, y, z = np.round(self.center * 1000).astype(int)
        return (f"({x}, {y}, {z}) mm, {self.width_m * 1000:.0f} mm wide, "
                f"from z {self.bottom_z_m * 1000:.0f} to {self.top_z_m * 1000:.0f} mm")


def locate(name: str, ticks: tuple[int, ...], pictures: Mapping[str, bytes], detector: Detector, rig: Rig) -> Target:
    """Find ``name`` in the wrist and shoulder pictures, both taken with the arm at ``ticks``."""
    boxes = {}
    for camera in CAMERAS:
        box = detector.find(pictures[camera], name)
        if box is None:
            raise NotLocated(f"The {camera} camera doesn't see {name!r}")
        width, height = rig.cameras[camera].image_size
        if min(box.x0, box.y0, width - box.x1, height - box.y1) < EDGE_PX:
            raise NotLocated(f"{name!r} is cut off at the edge of the {camera} camera's picture")
        boxes[camera] = box

    wrist_origin, wrist_direction = rig.ray("wrist", ticks, boxes["wrist"].center)
    side_origin, side_direction = rig.ray("shoulder", ticks, boxes["shoulder"].center)

    angle = np.degrees(np.arccos(np.clip(abs(wrist_direction @ side_direction), 0, 1)))
    if angle < MIN_SIGHT_ANGLE_DEG:
        raise NotLocated(f"The cameras see {name!r} from almost the same direction ({angle:.0f} degrees apart)")

    along, gap = closest_approach(wrist_origin, wrist_direction, side_origin, side_direction)
    if gap > MAX_SIGHT_GAP_M or along <= 0:
        raise NotLocated(f"The cameras' lines of sight to {name!r} pass {gap * 100:.0f} cm apart; "
                         f"they may have matched different objects")

    estimate = wrist_origin + along * wrist_direction
    start = np.array([*estimate[:2], START_SIZE_M, estimate[2] - START_SIZE_M / 2, estimate[2] + START_SIZE_M / 2])
    camera_poses = {camera: rig.camera_pose(camera, ticks) for camera in CAMERAS}
    x, y, width, bottom, top = fit_cylinder(start, boxes, camera_poses, rig)
    if width <= 0 or top <= bottom:
        raise NotLocated(f"The boxes around {name!r} don't fit an upright object")
    return Target(np.array([x, y, (bottom + top) / 2]), float(width), float(bottom), float(top))


def fit_cylinder(start: np.ndarray, boxes: Mapping[str, Box], camera_poses: Mapping[str, np.ndarray],
                 rig: Rig) -> np.ndarray:
    """The upright cylinder (x, y, width, bottom z, top z) whose outline fills each camera's box.

    Levenberg-Marquardt on the boxes' edges in pixels, from ``start``.
    """
    observed = np.concatenate([[box.x0, box.y0, box.x1, box.y1] for box in boxes.values()])
    world_to_camera = {camera: np.linalg.inv(pose) for camera, pose in camera_poses.items()}

    def residuals(params: np.ndarray) -> np.ndarray:
        predicted = []
        for camera in boxes:
            model, pose = rig.cameras[camera], world_to_camera[camera]
            pixels, _ = cv2.projectPoints(outline(*params), cv2.Rodrigues(pose[:3, :3])[0], pose[:3, 3],
                                          model.matrix, model.distortion)
            pixels = pixels.reshape(-1, 2)
            predicted.extend([*pixels.min(axis=0), *pixels.max(axis=0)])
        return np.array(predicted) - observed

    params, damping = np.asarray(start, float), 1e-3
    error = residuals(params)
    for _ in range(FIT_ITERATIONS):
        step_m = 1e-4
        jacobian = np.column_stack([(residuals(params + step) - error) / step_m for step in np.eye(5) * step_m])
        normal = jacobian.T @ jacobian
        delta = np.linalg.solve(normal + damping * np.diag(np.diag(normal)), -jacobian.T @ error)
        trial = residuals(params + delta)
        if trial @ trial < error @ error:
            params, error, damping = params + delta, trial, damping / 10
            if np.abs(delta).max() < 1e-5:
                break
        else:
            damping *= 10
    return params


def outline(x: float, y: float, width: float, bottom: float, top: float, points: int = 36) -> np.ndarray:
    """Points around an upright cylinder's top and bottom rims."""
    angles = np.linspace(0, 2 * np.pi, points, endpoint=False)
    rim = np.column_stack([x + width / 2 * np.cos(angles), y + width / 2 * np.sin(angles)])
    return np.vstack([np.column_stack([rim, np.full(points, z)]) for z in (bottom, top)])


def closest_approach(a0: np.ndarray, a: np.ndarray, b0: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """For two lines with unit directions, how far along the first its point nearest the second is, and the gap."""
    w = a0 - b0
    ab = a @ b
    along = (ab * (b @ w) - a @ w) / (1 - ab * ab)
    nearest_on_b = b0 + b * (b @ (a0 + along * a - b0))
    return float(along), float(np.linalg.norm(a0 + along * a - nearest_on_b))
