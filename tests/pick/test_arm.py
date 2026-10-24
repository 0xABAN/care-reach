"""Offline checks for PickArm with a fake mover and stand-in kinematics."""

import math
import unittest

import numpy as np

from care_reach.motion.bounds import SafetyBounds
from care_reach.motion.kinematics import IKResult
from care_reach.pick.arm import MOVE_OPTIONS, NUDGE_LEG_M, PickArm
from care_reach.robot import JOINT_NAMES


BOUNDS = SafetyBounds((0,) * 6, (4095,) * 6)

# Gripper pointing down with the base turned 90 degrees, tipped 5 degrees:
# its x axis points along base +y with a slight downward lean, its y axis
# along base +x, and its z axis (out through the jaws) down.
TILT = math.radians(5)
TIPPED_DOWN = np.column_stack([
    (0.0, math.cos(TILT), -math.sin(TILT)),
    (1.0, 0.0, 0.0),
    (0.0, -math.sin(TILT), -math.cos(TILT)),
])


class FakeKinematics:
    def __init__(self):
        self.translations = []

    def forward(self, ticks):
        transform = np.eye(4)
        transform[:3, :3] = TIPPED_DOWN
        return transform

    def translate(self, start_ticks, delta_xyz_m):
        self.translations.append((start_ticks, delta_xyz_m))
        target = tuple(tick + 10 for tick in start_ticks[:5]) + (start_ticks[5],)
        return IKResult(start_ticks, target, (0, 0, 0), (0, 0, 0), (0, 0, 0), 0.0)


class FakeRunner:
    """Reaches each target one tick short, as the real servos often do."""

    moves = []

    def __init__(self, port, bounds):
        pass

    def move(self, target, **options):
        FakeRunner.moves.append((target, options))
        return {"positions": {name: tick - 1 for name, tick in zip(JOINT_NAMES, target, strict=True)}}


class PickArmTests(unittest.TestCase):
    def setUp(self):
        FakeRunner.moves = []
        self.kinematics = FakeKinematics()
        self.arm = PickArm("/dev/fake", BOUNDS, self.kinematics, runner_factory=FakeRunner)

    def test_nudge_needs_a_measured_pose(self):
        with self.assertRaisesRegex(RuntimeError, "Move to a pose"):
            self.arm.nudge(0.01, 0.0)

    def test_move_to_records_the_measured_pose(self):
        self.arm.move_to((2000,) * 6)
        self.assertEqual(self.arm.ticks, (1999,) * 6)
        self.assertEqual(FakeRunner.moves[0][1], MOVE_OPTIONS)

    def test_nudge_follows_the_gripper_axes_and_stays_level(self):
        self.arm.move_to((2000,) * 6)
        self.arm.nudge(0.06, 0.0)

        deltas = np.array([delta for _start, delta in self.kinematics.translations])
        self.assertEqual(len(deltas), 3)
        self.assertTrue(all(np.linalg.norm(delta) <= NUDGE_LEG_M for delta in deltas))

        # Gripper x points along base +y here; the tilt's vertical part is dropped.
        np.testing.assert_allclose(deltas.sum(axis=0), (0.0, math.cos(TILT) * 0.06, 0.0), atol=1e-12)

    def test_each_leg_starts_from_the_measured_pose(self):
        self.arm.move_to((2000,) * 6)
        self.arm.nudge(0.0, 0.04)

        # Each IK target is 10 ticks past its start; the servos stop 1 short.
        starts = [start for start, _delta in self.kinematics.translations]
        self.assertEqual(starts, [(1999,) * 6, (2008,) * 5 + (1998,)])
        self.assertEqual(self.arm.ticks, (2017,) * 5 + (1997,))

        # Gripper y points along base +x here.
        np.testing.assert_allclose(self.kinematics.translations[0][1], (0.02, 0.0, 0.0), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
