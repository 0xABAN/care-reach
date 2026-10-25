"""The two calibrated cameras, and where they look in the arm's base frame.

`calibrate_cameras.py` measures each camera's lens and where it is mounted,
and saves both to CAMERAS_PATH:

- wrist: on the gripper, so its mount is given in ``gripper_frame_link``.
- shoulder: low on the base's turning part, so it turns with the shoulder pan
  and its mount is given in ``shoulder_link``.

Joint ticks place each mount link through the arm's kinematic model.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from motion_core import CALIBRATION_PATH, SafetyBounds
from so101_ik import URDF_PATH, SO101Kinematics


CAMERAS_PATH = Path.home() / ".local/state/care-reach/cameras.json"

# Base-frame pose of an arm link, for the measured joint ticks.
LinkPose = Callable[[str, tuple[int, ...]], np.ndarray]


def transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """A 4x4 rigid transform."""
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.ravel(translation)
    return result


@dataclass(frozen=True)
class CameraModel:
    """One camera's lens, picture size, and mount.

    ``matrix`` and ``distortion`` are OpenCV's pinhole model. ``mount`` is the
    camera's pose in its ``link`` frame; the camera looks along its own +z,
    with +x to the right of the picture and +y down.
    """

    link: str
    image_size: tuple[int, int]
    matrix: np.ndarray
    distortion: np.ndarray
    mount: np.ndarray

    def to_json(self) -> dict:
        return {"link": self.link, "image_size": list(self.image_size), "matrix": self.matrix.tolist(),
                "distortion": self.distortion.ravel().tolist(), "mount": self.mount.tolist()}

    @classmethod
    def from_json(cls, data: dict) -> CameraModel:
        return cls(data["link"], tuple(data["image_size"]), np.array(data["matrix"], float),
                   np.array(data["distortion"], float), np.array(data["mount"], float))


def load_cameras(path: Path = CAMERAS_PATH) -> dict[str, CameraModel]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return {name: CameraModel.from_json(model) for name, model in data.items()}


def save_camera(name: str, model: CameraModel, path: Path = CAMERAS_PATH) -> None:
    """Store one camera's calibration, keeping the others already in the file."""
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    data[name] = model.to_json()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def model_link_pose(bounds: SafetyBounds) -> LinkPose:
    """Link poses from the arm's URDF model, with one kinematic model per link asked for."""
    models: dict[str, SO101Kinematics] = {}

    def link_pose(link: str, ticks: tuple[int, ...]) -> np.ndarray:
        if link not in models:
            models[link] = SO101Kinematics(CALIBRATION_PATH, URDF_PATH, bounds, tool_frame_name=link)
        return models[link].forward(ticks)

    return link_pose


class Rig:
    """Turns pixels into lines of sight in the arm's base frame."""

    def __init__(self, cameras: dict[str, CameraModel], link_pose: LinkPose) -> None:
        self.cameras = cameras
        self.link_pose = link_pose

    def camera_pose(self, name: str, ticks: tuple[int, ...]) -> np.ndarray:
        camera = self.cameras[name]
        return self.link_pose(camera.link, ticks) @ camera.mount

    def ray(self, name: str, ticks: tuple[int, ...], pixel: tuple[float, float]) -> tuple[np.ndarray, np.ndarray]:
        """The camera's position and the unit direction it sees ``pixel`` along, in the base frame."""
        camera = self.cameras[name]
        x, y = cv2.undistortPoints(np.array([[pixel]], float), camera.matrix, camera.distortion).ravel()
        pose = self.camera_pose(name, ticks)
        direction = pose[:3, :3] @ np.array([x, y, 1.0])
        return pose[:3, 3], direction / np.linalg.norm(direction)
