"""Line the jaws up over a named object by looking and nudging, like parallel parking."""

from __future__ import annotations

from enum import Enum
import logging
import math
from typing import Protocol

from care_reach.pick.config import PickConfig
from care_reach.vision.detector import Box


# Give up after this many pictures.
MAX_LOOKS = 4

# The pixel scale is only an estimate, so each nudge covers part of the
# computed distance. Stopping short avoids overshooting; the next look
# corrects the rest.
STEP_FRACTION = 0.8

# Close enough for the jaws to close around the object.
CENTERED_M = 0.005

log = logging.getLogger(__name__)


class Camera(Protocol):
    def grab(self) -> bytes: ...


class Detector(Protocol):
    def find(self, jpeg: bytes, name: str) -> Box | None: ...


class Arm(Protocol):
    def nudge(self, dx_m: float, dy_m: float) -> None: ...


class CenterResult(Enum):
    CENTERED = "centered"
    NOT_FOUND = "not found"
    COULD_NOT_CENTER = "could not center"


def center_over(name: str, arm: Arm, camera: Camera, detector: Detector, config: PickConfig) -> CenterResult:
    """Nudge the arm until ``name`` appears at the aim point, where the jaws are right over it.

    Start from the survey pose: the aim point and pixel scale both assume the
    survey height, and nudges keep that height. Assumes the object is already
    in the picture.
    """
    for look in range(1, MAX_LOOKS + 1):
        box = detector.find(camera.grab(), name)
        if box is None:
            return CenterResult.NOT_FOUND

        dx_m, dy_m = config.offset_from_jaws(box.center)
        distance_m = math.hypot(dx_m, dy_m)
        log.info("look %d: %s at (%.0f, %.0f) px, %.1f cm from the jaws", look, name, *box.center, distance_m * 100)

        if distance_m <= CENTERED_M:
            return CenterResult.CENTERED

        # Only nudge if another look will check the result.
        if look < MAX_LOOKS:
            arm.nudge(STEP_FRACTION * dx_m, STEP_FRACTION * dy_m)

    return CenterResult.COULD_NOT_CENTER
