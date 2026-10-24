"""What centering needs to know about this arm and camera, recorded by `pick_setup`.

``config/pick.json`` looks like this:

    {
      "robot_id": "care_reach_follower",
      "survey_pose_ticks": {"shoulder_pan": 2010, ..., "gripper": 1800},
      "survey_height_m": 0.2,
      "aim_px": [604.0, 512.5],
      "image_right": "-y",
      "image_down": "-x"
    }

`pick_setup aim` writes the pose, height, and aim point. `pick_setup direction`
adds the two image directions.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path

from care_reach.robot import JOINT_NAMES, PICK_CONFIG_PATH, ROBOT_ID
from care_reach.vision.camera import FOCAL_LENGTH_PX, HEIGHT_PX, WIDTH_PX


# The gripper-frame axes an image direction can map to, as (x, y) unit vectors.
GRIPPER_DIRECTIONS = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}


@dataclass(frozen=True)
class PickConfig:
    """Where the camera looks from, and where in its picture the jaws are.

    survey_pose: joint ticks where the gripper points down at the table.
    survey_height_m: the camera lens's height above the table at the survey
        pose, measured with a ruler.
    aim_px: where an object appears in the picture, seen from the survey
        height, when the jaws are right over it.
    image_right, image_down: the gripper axis ("+x", "-y", ...) that leads
        toward an object appearing right of, or below, the aim point. Both
        stay None until `pick_setup direction` has checked them.
    """

    survey_pose: tuple[int, ...]
    survey_height_m: float
    aim_px: tuple[float, float]
    image_right: str | None = None
    image_down: str | None = None

    def __post_init__(self) -> None:
        if len(self.survey_pose) != 6 or any(type(tick) is not int or not 0 <= tick <= 4095
                                             for tick in self.survey_pose):
            raise ValueError("Survey pose must be six integer ticks")

        if not 0.05 <= self.survey_height_m <= 0.5:
            raise ValueError("Survey height must be between 5 and 50 cm")

        u, v = self.aim_px
        if not (0 <= u < WIDTH_PX and 0 <= v < HEIGHT_PX):
            raise ValueError(f"Aim point {self.aim_px} is outside the {WIDTH_PX}x{HEIGHT_PX} picture")

        directions = (self.image_right, self.image_down)
        if directions != (None, None):
            if any(direction not in GRIPPER_DIRECTIONS for direction in directions):
                raise ValueError(f"Image directions must each be one of {sorted(GRIPPER_DIRECTIONS)}")
            if self.image_right[1] == self.image_down[1]:
                raise ValueError("Image right and down must map to different gripper axes")

    @property
    def meters_per_pixel(self) -> float:
        """Estimate how far one pixel spans on the table, from the lens spec and survey height."""
        return self.survey_height_m / FOCAL_LENGTH_PX

    def offset_from_jaws(self, pixel: tuple[float, float]) -> tuple[float, float]:
        """Estimate where an object seen at ``pixel`` is, relative to the jaws.

        Returns meters along the gripper's own x and y axes.
        """
        if self.image_right is None or self.image_down is None:
            raise ValueError("Camera directions are not recorded; run `pick_setup direction` first")

        right_m = (pixel[0] - self.aim_px[0]) * self.meters_per_pixel
        down_m = (pixel[1] - self.aim_px[1]) * self.meters_per_pixel

        right_x, right_y = GRIPPER_DIRECTIONS[self.image_right]
        down_x, down_y = GRIPPER_DIRECTIONS[self.image_down]
        return (right_m * right_x + down_m * down_x, right_m * right_y + down_m * down_y)

    def with_directions(self, image_right: str, image_down: str) -> PickConfig:
        return replace(self, image_right=image_right, image_down=image_down)

    @classmethod
    def load(cls, path: str | Path = PICK_CONFIG_PATH) -> PickConfig:
        record = json.loads(Path(path).read_text(encoding="utf-8"))
        if record.get("robot_id") != ROBOT_ID:
            raise ValueError("Pick config belongs to a different robot")

        try:
            ticks = record["survey_pose_ticks"]
            return cls(
                survey_pose=tuple(ticks[name] for name in JOINT_NAMES),
                survey_height_m=float(record["survey_height_m"]),
                aim_px=tuple(float(value) for value in record["aim_px"]),
                image_right=record.get("image_right"),
                image_down=record.get("image_down"),
            )
        except (KeyError, TypeError) as exc:
            raise ValueError(f"Pick config is incomplete: {exc}") from exc

    def save(self, path: str | Path = PICK_CONFIG_PATH) -> None:
        record = asdict(self)
        record["survey_pose_ticks"] = dict(zip(JOINT_NAMES, record.pop("survey_pose"), strict=True))
        Path(path).write_text(json.dumps({"robot_id": ROBOT_ID, **record}, indent=2) + "\n", encoding="utf-8")
