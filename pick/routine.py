"""The whole pick: find the object, reach it, grip, and lift.

1. **Look.** Both cameras find the object from wherever the arm is.
2. **Look again, up close.** The jaws move near the object at the angle
   they will grasp it, placed so the wrist camera looks straight at it, and
   both cameras locate it again. The arm's kinematic model is off by a couple
   of centimeters across its reach, but the error barely changes over a short
   move, so the object located from here is where the jaws will go.
3. **Pre-grasp.** The open jaws line up a few centimeters short of it.
4. **Approach.** Straight along the jaws, in short steps. If the jaws push on
   something, the measured mover stalls and stops the pick.
5. **Grip.** The jaws close until they stop on something. If they close on
   nothing, they reopen and back out.
6. **Lift.** The jaws rise, still squeezing, and the pick checks they still
   hold something.
"""

from __future__ import annotations

from enum import Enum
import logging
from typing import Callable, Protocol

from pick.grasp import GraspPlan
from pick.locate import Target


LIFT_M = 0.03

log = logging.getLogger(__name__)


class PickResult(Enum):
    PICKED_UP = "picked up"
    MISSED = "missed"
    DROPPED = "dropped"


class Planner(Protocol):
    def plan(self, target: Target, start: tuple[int, ...]) -> GraspPlan: ...
    def look_pose(self, target: Target, grasp: GraspPlan, start: tuple[int, ...]) -> tuple[int, ...]: ...


class Arm(Protocol):
    ticks: tuple[int, ...]

    @property
    def holding(self) -> bool: ...
    def move_to(self, pose: tuple[int, ...]) -> None: ...
    def open_gripper(self) -> None: ...
    def close_gripper(self) -> bool: ...
    def lift(self, dz_m: float) -> None: ...


def pick_up(arm: Arm, see: Callable[[tuple[int, ...]], Target], planner: Planner) -> PickResult:
    """Pick up what ``see`` locates from the arm's measured ticks, and hold it LIFT_M higher."""
    target = see(arm.ticks)
    log.info("first look: %s", target)
    grasp = planner.plan(target, arm.ticks)
    arm.move_to(planner.look_pose(target, grasp, arm.ticks))

    target = see(arm.ticks)
    log.info("close look: %s", target)
    grasp = planner.plan(target, arm.ticks)
    log.info("approaching at %d degrees from straight down", grasp.tilt_deg)
    for pose in grasp.approach:
        arm.move_to(pose)

    if not arm.close_gripper():
        arm.open_gripper()
        arm.move_to(grasp.pregrasp)
        return PickResult.MISSED

    arm.lift(LIFT_M)
    return PickResult.PICKED_UP if arm.holding else PickResult.DROPPED
