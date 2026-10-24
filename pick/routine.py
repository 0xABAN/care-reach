"""The whole pick: center over the object, lower the open jaws around it, grip, and lift."""

from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING

from pick.center import Camera, CenterResult, Detector, center_over
from pick.config import PickConfig

if TYPE_CHECKING:
    from pick.arm import PickArm


# How high above the table the jaw tips stop to close: around the lower part
# of a short object, with room for error in the kinematic model's height.
DEFAULT_GRASP_HEIGHT_M = 0.02


class PickResult(Enum):
    PICKED_UP = "picked up"
    MISSED = "missed"
    DROPPED = "dropped"
    NOT_FOUND = "not found"
    COULD_NOT_CENTER = "could not center"


def pick_up(
    name: str,
    arm: PickArm,
    camera: Camera,
    detector: Detector,
    config: PickConfig,
    grasp_height_m: float = DEFAULT_GRASP_HEIGHT_M,
) -> PickResult:
    """Pick up ``name`` from the table and hold it at the survey height.

    The jaws go straight down from the survey pose until their tips are
    ``grasp_height_m`` above the table, close until they touch something,
    and come back up. A miss reopens them before rising.
    """
    descent_m = config.jaw_height_m - grasp_height_m
    if descent_m <= 0:
        raise ValueError("The jaws already sit at or below the grasp height in the survey pose")

    arm.move_to(config.survey_pose)
    centered = center_over(name, arm, camera, detector, config)
    if centered is not CenterResult.CENTERED:
        # Both results share their wording with PickResult.
        return PickResult(centered.value)

    arm.open_gripper()
    arm.move_vertically(-descent_m)
    gripped = arm.close_gripper()
    if not gripped:
        arm.open_gripper()
    arm.move_vertically(descent_m)

    if not gripped:
        return PickResult.MISSED
    return PickResult.PICKED_UP if arm.holding else PickResult.DROPPED
