"""Offline checks for PickArm with a fake mover and stand-in kinematics."""

import math
import unittest

import numpy as np

from motion_core import JOINT_NAMES, SafetyBounds
from pick.arm import ARRIVED_M, LEG_M, MOVE_OPTIONS, PickArm
from so101_ik import IKResult


BOUNDS = SafetyBounds((0,) * 5 + (1340,), (4095,) * 5 + (2742,))

# Each of the first three joints moves the jaws 0.1 mm along x, y, or z.
METERS_PER_TICK = 1e-4

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
        self.legs = []

    def forward(self, ticks):
        transform = np.eye(4)
        transform[:3, :3] = TIPPED_DOWN
        transform[:3, 3] = (np.array(ticks[:3]) - 2000) * METERS_PER_TICK
        return transform

    def translate(self, start_ticks, delta_xyz_m):
        self.legs.append((start_ticks, np.array(delta_xyz_m)))
        moved = np.rint(np.array(delta_xyz_m) / METERS_PER_TICK).astype(int)
        target = tuple(int(tick) for tick in np.array(start_ticks[:3]) + moved) + start_ticks[3:]
        return IKResult(start_ticks, target, (0, 0, 0), (0, 0, 0), (0, 0, 0), 0.0)


class FakeServos:
    """Every move reaches ``reach`` of the way to its target, like a servo resting short of its goal."""

    def __init__(self, reach=1.0, contact_at=None):
        self.ticks = (2000, 2000, 2000, 2000, 2000, 1800)
        self.reach = reach
        self.contact_at = contact_at
        self.moves = []
        self.measured = []

    def runner(self, port, bounds):
        servos = self

        class Runner:
            def move(self, target, holding_object=False, **options):
                servos.moves.append((target, holding_object, options))
                start = np.array(servos.ticks)
                reached = np.rint(start + servos.reach * (np.array(target) - start)).astype(int)
                gripper = max(servos.contact_at, target[5]) if holding_object else target[5]
                servos.ticks = tuple(int(tick) for tick in reached[:5]) + (gripper,)
                servos.measured.append(servos.ticks)
                return {"positions": dict(zip(JOINT_NAMES, servos.ticks))}

            def close_gripper(self, closed_tick):
                contact = servos.contact_at is not None
                gripper = servos.contact_at if contact else closed_tick
                servos.ticks = servos.ticks[:5] + (gripper,)
                hold = gripper - 30 if contact else closed_tick
                return {"contact": contact, "gripper_hold": hold,
                        "positions": dict(zip(JOINT_NAMES, servos.ticks))}

        return Runner()


def make_arm(servos):
    arm = PickArm("/dev/fake", BOUNDS, FakeKinematics(), runner_factory=servos.runner)
    arm.move_to((2000, 2000, 2000, 2000, 2000, 1800))
    return arm


class PickArmTests(unittest.TestCase):
    def test_nudge_needs_a_measured_pose(self):
        arm = PickArm("/dev/fake", BOUNDS, FakeKinematics(), runner_factory=FakeServos().runner)
        with self.assertRaisesRegex(RuntimeError, "Move to a pose"):
            arm.nudge(0.01, 0.0)

    def test_moves_use_the_cartesian_settings(self):
        servos = FakeServos()
        make_arm(servos)
        self.assertEqual(servos.moves[0][2], MOVE_OPTIONS)
        self.assertFalse(servos.moves[0][1])

    def test_nudge_follows_the_gripper_axes_and_stays_level(self):
        servos = FakeServos()
        arm = make_arm(servos)
        arm.nudge(0.06, 0.0)

        legs = [delta for _start, delta in arm.kinematics.legs]
        self.assertEqual(len(legs), 3)
        self.assertTrue(all(np.linalg.norm(leg) <= LEG_M + 1e-12 for leg in legs))

        # Gripper x points along base +y here; the tilt's vertical part is dropped.
        np.testing.assert_allclose(sum(legs), (0.0, math.cos(TILT) * 0.06, 0.0), atol=1e-4)

    def test_short_servos_are_corrected_from_measured_poses(self):
        # The servos cover only 80% of each leg, so planned legs fall short and
        # correcting legs, each planned from the measured pose, close the gap.
        servos = FakeServos(reach=0.8)
        arm = make_arm(servos)
        arm.move_vertically(-0.05)

        legs = arm.kinematics.legs
        self.assertGreater(len(legs), 2)

        # servos.measured[0] is the starting move_to; leg n starts where move n ended.
        starts = [start for start, _delta in legs]
        self.assertEqual(starts, servos.measured[:len(legs)])
        z = arm.kinematics.forward(arm.ticks)[2, 3]
        self.assertLessEqual(abs(z - -0.05), ARRIVED_M + 1e-12)

    def test_gripping_an_object_keeps_its_squeeze_while_moving(self):
        servos = FakeServos(contact_at=1700)
        arm = make_arm(servos)

        arm.open_gripper()
        self.assertEqual(servos.moves[-1][0][5], BOUNDS.upper[5])
        self.assertTrue(arm.close_gripper())
        self.assertEqual(arm.gripper_hold, 1670)

        arm.move_vertically(0.03)
        target, holding_object, _options = servos.moves[-1]
        self.assertTrue(holding_object)
        self.assertEqual(target[5], 1670)
        self.assertTrue(arm.holding)

    def test_closing_on_nothing_is_a_miss_and_holds_nothing(self):
        servos = FakeServos(contact_at=None)
        arm = make_arm(servos)
        self.assertFalse(arm.close_gripper())
        self.assertIsNone(arm.gripper_hold)
        self.assertFalse(arm.holding)

    def test_a_squeeze_that_reaches_its_goal_means_the_object_slipped_out(self):
        servos = FakeServos(contact_at=1700)
        arm = make_arm(servos)
        arm.close_gripper()

        # The object falls out: the jaws close the last 30 ticks to their squeeze goal.
        servos.contact_at = 1340
        arm.move_vertically(0.01)
        self.assertFalse(arm.holding)


if __name__ == "__main__":
    unittest.main()
