"""Offline checks for the whole pick, with a scripted arm and stand-in eyes and planner."""

import unittest

import numpy as np

from pick.grasp import GraspPlan
from pick.locate import Target
from pick.routine import LIFT_M, PickResult, pick_up


FAR = Target(np.array([0.30, 0.0, 0.05]), 0.066, -0.02, 0.10)
NEAR = Target(np.array([0.31, 0.01, 0.05]), 0.066, -0.02, 0.10)


class ScriptedArm:
    """Records each step; ``grips`` and ``holds`` decide how the grasp turns out."""

    def __init__(self, grips=True, holds=True):
        self.ticks = (2000,) * 6
        self.grips = grips
        self.holds = holds
        self.steps = []

    @property
    def holding(self):
        return self.grips and self.holds

    def move_to(self, pose):
        self.steps.append(("move_to", pose))
        self.ticks = pose

    def open_gripper(self):
        self.steps.append(("open",))

    def close_gripper(self):
        self.steps.append(("close",))
        return self.grips

    def lift(self, dz_m):
        self.steps.append(("lift", dz_m))


class Planner:
    """Plans a three-step approach that starts 100 ticks past the target's x in millimeters,
    and looks from 50 ticks short of that."""

    def __init__(self):
        self.plans = []

    def plan(self, target, start):
        self.plans.append((target, start))
        first = int(target.center[0] * 1000) + 100
        return GraspPlan(tuple((first + step,) * 6 for step in range(3)), tilt_deg=30, side=1)

    def look_pose(self, target, grasp, start):
        return tuple(tick - 50 for tick in grasp.pregrasp)


class PickUpTests(unittest.TestCase):
    def run_pick(self, arm):
        looks = iter([FAR, NEAR])
        seen_from = []
        planner = Planner()

        def see(ticks):
            seen_from.append(ticks)
            return next(looks)

        result = pick_up(arm, see, planner)
        return result, seen_from, planner.plans

    def test_looks_again_up_close_and_follows_the_new_plan(self):
        arm = ScriptedArm()
        result, seen_from, plans = self.run_pick(arm)

        self.assertIs(result, PickResult.PICKED_UP)
        self.assertEqual(seen_from, [(2000,) * 6, (350,) * 6])
        self.assertEqual(plans, [(FAR, (2000,) * 6), (NEAR, (350,) * 6)])
        self.assertEqual(arm.steps, [
            ("move_to", (350,) * 6),
            ("move_to", (410,) * 6), ("move_to", (411,) * 6), ("move_to", (412,) * 6),
            ("close",),
            ("lift", LIFT_M),
        ])

    def test_a_miss_reopens_and_backs_out_without_lifting(self):
        arm = ScriptedArm(grips=False)
        result, _seen_from, _plans = self.run_pick(arm)

        self.assertIs(result, PickResult.MISSED)
        self.assertEqual(arm.steps[-3:], [("close",), ("open",), ("move_to", (410,) * 6)])

    def test_an_object_that_slips_out_on_the_lift_is_dropped(self):
        result, _seen_from, _plans = self.run_pick(ScriptedArm(holds=False))
        self.assertIs(result, PickResult.DROPPED)


if __name__ == "__main__":
    unittest.main()
