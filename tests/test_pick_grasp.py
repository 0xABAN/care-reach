"""The grasp planner's collision model, with the gripper placed directly instead of by joints."""

import unittest

import numpy as np

from motion_core import SafetyBounds
from pick.grasp import FIXED_JAW_GAP_M, GRIP_DEPTH_M, GraspError, GraspPlanner
from pick.locate import Target
from pick.rig import CameraModel, transform


BOUNDS = SafetyBounds((0,) * 5 + (1340,), (4095,) * 5 + (2742,))
CAN = Target(np.array([0.30, 0.0, 0.04]), 0.066, -0.02, 0.10)


class PlacedGripper:
    """Kinematics whose tool pose is whatever the test last set."""

    pose = np.eye(4)

    def forward(self, ticks):
        return self.pose


# Behind and beside the jaws, looking past them, like the calibrated wrist camera.
WRIST = CameraModel("gripper_frame_link", (1280, 720), np.array([[1040.0, 0, 640], [0, 1040.0, 360], [0, 0, 1]]),
                    np.zeros(5), transform(np.eye(3), [0.041, 0.036, -0.095]))


def planner():
    return GraspPlanner(PlacedGripper(), BOUNDS, WRIST, pan_axis_xy=np.array([0.0388, 0.0]))


class CollisionTests(unittest.TestCase):
    def setUp(self):
        self.planner = planner()
        grip = np.array([*CAN.center[:2], CAN.top_z_m - 0.02])
        held_at = np.array([-(CAN.width_m / 2 + FIXED_JAW_GAP_M), 0.0, -GRIP_DEPTH_M])
        rotation, tip = self.planner._frame(grip, held_at, tilt=0.0, side=1)
        self.grasp = transform(rotation, tip)

    def collides_at(self, pose, target=CAN):
        self.planner.kinematics.pose = pose
        return self.planner.collides((), target)

    def test_jaws_straddling_the_object_at_the_grasp_are_clear(self):
        self.assertFalse(self.collides_at(self.grasp))

    def test_the_grasp_puts_the_fixed_jaw_just_outside_the_object(self):
        # The tool frame's x points from the moving jaw to the fixed one; the tip is the fixed jaw's inner face.
        tip = self.grasp[:3, 3]
        distance = np.linalg.norm(tip[:2] - CAN.center[:2])
        self.assertAlmostEqual(distance, CAN.width_m / 2 + FIXED_JAW_GAP_M, places=4)
        self.assertLess(tip[2], CAN.top_z_m)

    def test_jaws_shifted_onto_the_object_collide(self):
        shifted = self.grasp.copy()
        shifted[:3, 3] -= self.grasp[:3, 0] * 0.01  # 1 cm toward the moving jaw: the fixed jaw enters the object
        self.assertTrue(self.collides_at(shifted))

    def test_jaws_above_the_object_are_clear_and_below_its_surface_collide(self):
        above = self.grasp.copy()
        above[2, 3] = CAN.top_z_m + 0.03
        self.assertFalse(self.collides_at(above))

        away = self.grasp.copy()
        away[:2, 3] += [0.0, 0.2]
        away[2, 3] = CAN.bottom_z_m + 0.005
        self.assertTrue(self.collides_at(away))

    def test_an_object_wider_than_the_jaws_is_refused(self):
        wide = Target(CAN.center, 0.12, CAN.bottom_z_m, CAN.top_z_m)
        with self.assertRaisesRegex(GraspError, "12 cm wide"):
            self.planner.plan(wide, (2000,) * 6)


if __name__ == "__main__":
    unittest.main()
