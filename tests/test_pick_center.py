"""Offline checks for center_over against a simulated camera, detector, and arm."""

import math
import unittest

import numpy as np

from pick.center import CENTERED_M, MAX_LOOKS, STEP_FRACTION, CenterResult, center_over
from pick.config import GRIPPER_DIRECTIONS, PickConfig
from pick.detector import Box


CONFIG = PickConfig(
    survey_pose=(2000,) * 6,
    lens_height_m=0.2,
    jaw_height_m=0.12,
    aim_px=(640.0, 500.0),
    image_right="-y",
    image_down="-x",
)


class FakeWorld:
    """One object on the table, seen by the wrist camera and moved past by nudges.

    ``magnification`` is how much bigger the real picture is than the lens-spec
    estimate predicts; 1.0 means the estimate is exact.
    """

    def __init__(self, object_m, magnification=1.0, visible=True):
        self.object_m = np.array(object_m, dtype=float)
        self.magnification = magnification
        self.visible = visible
        self.looks = 0
        self.nudges = []

    def grab(self):
        self.looks += 1
        return b"jpeg"

    def find(self, image, name):
        if not self.visible:
            return None

        # Invert PickConfig.offset_from_jaws: gripper meters -> picture pixels.
        right = np.array(GRIPPER_DIRECTIONS[CONFIG.image_right])
        down = np.array(GRIPPER_DIRECTIONS[CONFIG.image_down])
        scale = self.magnification / CONFIG.meters_per_pixel
        u = CONFIG.aim_px[0] + scale * right @ self.object_m
        v = CONFIG.aim_px[1] + scale * down @ self.object_m
        return Box(u - 20, v - 20, u + 20, v + 20, score=0.8)

    def nudge(self, dx_m, dy_m):
        self.nudges.append((dx_m, dy_m))
        self.object_m -= (dx_m, dy_m)


def center(world):
    return center_over("coke can", world, world, world, CONFIG)


class CenterOverTests(unittest.TestCase):
    def test_already_centered_takes_one_look_and_no_nudge(self):
        world = FakeWorld((0.002, -0.001))
        self.assertIs(center(world), CenterResult.CENTERED)
        self.assertEqual(world.looks, 1)
        self.assertEqual(world.nudges, [])

    def test_each_nudge_moves_most_of_the_way_toward_the_object(self):
        world = FakeWorld((0.06, -0.03))
        self.assertIs(center(world), CenterResult.CENTERED)

        first = world.nudges[0]
        self.assertAlmostEqual(first[0], STEP_FRACTION * 0.06)
        self.assertAlmostEqual(first[1], STEP_FRACTION * -0.03)
        self.assertLessEqual(math.hypot(*world.object_m), CENTERED_M)

    def test_converges_when_the_lens_estimate_is_off(self):
        # The wide lens magnifies the middle of the picture more than the
        # spec-based estimate, so nudges overshoot slightly. The direction
        # check rejects setups whose estimate is far enough off to fail.
        for magnification in (0.85, 1.4):
            with self.subTest(magnification=magnification):
                world = FakeWorld((0.08, 0.05), magnification)
                self.assertIs(center(world), CenterResult.CENTERED)
                self.assertLessEqual(world.looks, MAX_LOOKS)

    def test_object_out_of_view_is_not_found(self):
        world = FakeWorld((0.0, 0.0), visible=False)
        self.assertIs(center(world), CenterResult.NOT_FOUND)
        self.assertEqual(world.nudges, [])

    def test_gives_up_after_four_looks_without_a_final_unchecked_nudge(self):
        # A picture 4x bigger than estimated makes every nudge far too short.
        world = FakeWorld((0.20, 0.0), magnification=0.25)
        self.assertIs(center(world), CenterResult.COULD_NOT_CENTER)
        self.assertEqual(world.looks, MAX_LOOKS)
        self.assertEqual(len(world.nudges), MAX_LOOKS - 1)
        self.assertEqual(CenterResult.COULD_NOT_CENTER.value, "could not center")


if __name__ == "__main__":
    unittest.main()
