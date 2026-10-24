"""Offline checks for the whole pick, with a scripted arm, camera, and detector."""

import unittest

from pick.config import PickConfig
from pick.detector import Box
from pick.routine import PickResult, pick_up


CONFIG = PickConfig(
    survey_pose=(2000,) * 6,
    lens_height_m=0.22,
    jaw_height_m=0.14,
    aim_px=(640.0, 500.0),
    image_right="-y",
    image_down="-x",
)
AT_AIM = Box(620.0, 480.0, 660.0, 520.0, score=0.8)


class ScriptedArm:
    """Records each step; ``grips`` and ``holds`` decide how the grasp turns out."""

    def __init__(self, grips=True, holds=True):
        self.grips = grips
        self.holds = holds
        self.steps = []

    @property
    def holding(self):
        return self.grips and self.holds

    def move_to(self, pose):
        self.steps.append(("move_to", pose))

    def nudge(self, dx_m, dy_m):
        self.steps.append(("nudge", round(dx_m, 4), round(dy_m, 4)))

    def move_vertically(self, dz_m):
        self.steps.append(("vertical", round(dz_m, 4)))

    def open_gripper(self):
        self.steps.append(("open",))

    def close_gripper(self):
        self.steps.append(("close",))
        return self.grips


class Picture:
    def __init__(self, box):
        self.box = box

    def grab(self):
        return b"jpeg"

    def find(self, jpeg, name):
        return self.box


def run(arm, box=AT_AIM, **options):
    picture = Picture(box)
    return pick_up("blue box", arm, picture, picture, CONFIG, **options)


class PickUpTests(unittest.TestCase):
    def test_picks_up_and_lifts_back_to_the_survey_height(self):
        arm = ScriptedArm()
        self.assertIs(run(arm, grasp_height_m=0.02), PickResult.PICKED_UP)
        self.assertEqual(arm.steps, [
            ("move_to", CONFIG.survey_pose),
            ("open",),
            ("vertical", -0.12),
            ("close",),
            ("vertical", 0.12),
        ])

    def test_a_miss_reopens_the_jaws_before_rising(self):
        arm = ScriptedArm(grips=False)
        self.assertIs(run(arm), PickResult.MISSED)
        self.assertEqual(arm.steps[-3:], [("close",), ("open",), ("vertical", 0.12)])

    def test_an_object_that_slips_out_while_rising_is_dropped(self):
        self.assertIs(run(ScriptedArm(holds=False)), PickResult.DROPPED)

    def test_centering_results_stop_before_the_gripper_moves(self):
        arm = ScriptedArm()
        self.assertIs(run(arm, box=None), PickResult.NOT_FOUND)
        self.assertEqual(arm.steps, [("move_to", CONFIG.survey_pose)])

        # An object that never comes closer exhausts the four looks.
        far = Box(1180.0, 480.0, 1220.0, 520.0, score=0.8)
        arm = ScriptedArm()
        self.assertIs(run(arm, box=far), PickResult.COULD_NOT_CENTER)
        self.assertNotIn(("open",), arm.steps)

    def test_grasp_height_above_the_jaws_is_rejected_before_moving(self):
        arm = ScriptedArm()
        with self.assertRaisesRegex(ValueError, "grasp height"):
            run(arm, grasp_height_m=0.2)
        self.assertEqual(arm.steps, [])


if __name__ == "__main__":
    unittest.main()
