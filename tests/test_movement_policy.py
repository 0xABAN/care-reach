"""Three movement modes are checked with a fake bus, never real hardware."""

import unittest

from motion_core import JOINT_NAMES
from arm_control.movement_policy import MovementMode, validate_movement_path
from test_dashboard import DashboardBase
from test_single_motor_mode import MotionBus


class PartialBus(MotionBus):
    """The real LeRobot bus accepts subsets of joint names in sync_write."""

    def sync_write(self, register, values, normalize=True):
        if register == "Goal_Position" and set(values) != set(JOINT_NAMES):
            assert self.connected and not normalize
            self.goals.update(values)
            self.writes.append((register, values.copy()))
            return
        return super().sync_write(register, values, normalize=normalize)


class PathPolicyTests(unittest.TestCase):
    def test_shoulder_mode_accepts_other_group_or_motor_two_alone(self):
        start = (2000,) * 6
        self.assertEqual(
            validate_movement_path(MovementMode.SHOULDER_ISOLATED, start,
                                   [(2020, 2000, 2030, 2000, 2000, 2000)]),
            frozenset({1, 3}),
        )
        self.assertEqual(
            validate_movement_path("shoulder_isolated", start,
                                   [(2000, 2050, 2000, 2000, 2000, 2000)]),
            frozenset({2}),
        )

    def test_separate_phases_are_not_silently_reordered(self):
        start = (2000,) * 6
        with self.assertRaisesRegex(ValueError, "motor 2 and another joint"):
            validate_movement_path("shoulder_isolated", start, [
                (2000, 2050, 2000, 2000, 2000, 2000),
                (2030, 2050, 2000, 2000, 2000, 2000),
            ])
        with self.assertRaisesRegex(ValueError, "more than one joint"):
            validate_movement_path("single_joint", start, [(2030, 2000, 2010, 2000, 2000, 2000)])


class ShoulderIsolationTests(DashboardBase):
    def test_mode_state_and_legacy_alias(self):
        bus = PartialBus((2000,) * 6)
        arm = self.arm(bus)
        self.assertEqual(arm.state()["movement_mode"], "coordinated")
        self.assertEqual(arm.set_movement_mode("shoulder_isolated"),
                         {"movement_mode": "shoulder_isolated"})
        state = arm.state()
        self.assertFalse(state["single_motor_mode"])
        self.assertFalse(state["pose_enabled"])
        self.assertFalse(state["hold_all_enabled"])
        self.assertEqual(bus.writes, [])
        self.assertEqual(arm.set_single_motor_mode(True), {"single_motor_mode": True})
        self.assertEqual(arm.state()["movement_mode"], "single_joint")

    def test_mode_change_requires_idle_and_settled(self):
        bus = PartialBus((2000,) * 6)
        arm = self.arm(bus)
        bus.moving["elbow_flex"] = 1
        with self.assertRaisesRegex(ValueError, "Motor 3 is still moving"):
            arm.set_movement_mode("shoulder_isolated")
        bus.moving["elbow_flex"] = 0
        arm.start(1, 1)
        with self.assertRaisesRegex(ValueError, "Stop the active movement"):
            arm.set_movement_mode("shoulder_isolated")
        self.assertEqual(arm.state()["movement_mode"], "coordinated")

    def test_taught_group_excluding_motor_two_never_writes_motor_two(self):
        bus = PartialBus((2000,) * 6)
        arm = self.arm(bus)
        arm.taught["press_button"] = [(2050, 2000, 2050, 2000, 2000, 2000)]
        arm.set_movement_mode("shoulder_isolated")
        self.assertTrue(arm.state()["pose_available"]["press_button"])
        self.assertFalse(arm.state()["pose_available"]["home"])
        arm.pose_start("press_button")
        arm._advance(arm.hold["next"] + 0.001)
        arm.pose_stop()
        writes = [entry[1] for entry in bus.writes if entry[0] == "Goal_Position"]
        self.assertTrue(writes)
        self.assertTrue(any({"shoulder_pan", "elbow_flex"}.issubset(goals) for goals in writes))
        self.assertTrue(all("shoulder_lift" not in goals for goals in writes))

        bus.writes.clear()
        arm.pose_start("press_button")
        bus.positions["shoulder_pan"] = 2050
        bus.positions["elbow_flex"] = 2050
        arm.hold["frame_index"] = len(arm.hold["trajectory"]) - 1
        arm._advance(arm.hold["next"] + 0.001)
        self.assertIsNone(arm.hold)
        writes = [entry[1] for entry in bus.writes if entry[0] == "Goal_Position"]
        self.assertTrue(writes)
        self.assertTrue(all("shoulder_lift" not in goals for goals in writes))

    def test_mixed_taught_path_rejected_before_any_goal_write(self):
        bus = PartialBus((2000,) * 6)
        arm = self.arm(bus)
        arm.taught["press_button"] = [(2000, 2050, 2050, 2000, 2000, 2000)]
        arm.set_movement_mode("shoulder_isolated")
        self.assertFalse(arm.state()["pose_available"]["press_button"])
        with self.assertRaisesRegex(ValueError, "motor 2 and another joint"):
            arm.pose_start("press_button")
        self.assertEqual(bus.writes, [])

    def test_taught_motor_two_alone_writes_only_motor_two(self):
        bus = PartialBus((2000,) * 6)
        arm = self.arm(bus)
        arm.taught["press_button"] = [(2000, 2050, 2000, 2000, 2000, 2000)]
        arm.set_movement_mode("shoulder_isolated")
        self.assertTrue(arm.state()["pose_available"]["press_button"])
        arm.pose_start("press_button")
        arm._advance(arm.hold["next"] + 0.001)
        arm.pose_stop()
        writes = [entry[1] for entry in bus.writes if entry[0] == "Goal_Position"]
        self.assertTrue(writes)
        self.assertTrue(all(set(goals) == {"shoulder_lift"} for goals in writes))

    def test_motor_two_jog_waits_for_other_joint_coast(self):
        bus = PartialBus((2000,) * 6)
        arm = self.arm(bus)
        arm.set_movement_mode("shoulder_isolated")
        arm.start(3, 1)
        bus.moving["elbow_flex"] = 1
        with self.assertRaisesRegex(ValueError, "Motor 3 is still moving"):
            arm.start(2, 1)
        self.assertIsNone(arm.hold)
        bus.moving["elbow_flex"] = 0
        bus.goals["elbow_flex"] = 2050
        with self.assertRaisesRegex(ValueError, "Motor 3 is still moving"):
            arm.start(2, 1)
        bus.positions["elbow_flex"] = 2050
        arm.start(2, 1)
        self.assertEqual(arm.hold["motor"], 2)

    def test_group_pose_and_hold_all_are_blocked_without_writes(self):
        bus = PartialBus((2000,) * 6)
        arm = self.arm(bus)
        arm.set_movement_mode("shoulder_isolated")
        for pose in ("home", "extend", "dance"):
            with self.assertRaisesRegex(ValueError, "coordinated path"):
                arm.pose_start(pose)
        with self.assertRaisesRegex(ValueError, "requires coordinated mode"):
            arm.hold_all()
        self.assertEqual(bus.writes, [])


if __name__ == "__main__":
    unittest.main()
