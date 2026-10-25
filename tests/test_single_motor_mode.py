"""Single-motor mode checks use a fake bus and never command hardware."""

import unittest

from motion_core import JOINT_NAMES
from test_dashboard import DashboardBase, FakeBus


class MotionBus(FakeBus):
    def __init__(self, positions, torque=1):
        super().__init__(positions, torque=torque)
        self.moving = {name: 0 for name in JOINT_NAMES}

    def sync_read(self, register, normalize=True, num_retry=0):
        if register == "Moving":
            assert self.connected and not normalize and num_retry == 2
            return self.moving.copy()
        return super().sync_read(register, normalize=normalize, num_retry=num_retry)


class SingleMotorModeTests(DashboardBase):
    def test_mode_is_explicit_and_has_no_motor_write(self):
        bus = MotionBus((2000,) * 6)
        arm = self.arm(bus)
        self.assertFalse(arm.state()["single_motor_mode"])
        self.assertEqual(arm.set_single_motor_mode(True), {"single_motor_mode": True})
        state = arm.state()
        self.assertTrue(state["single_motor_mode"])
        self.assertFalse(state["pose_enabled"])
        self.assertFalse(state["hold_all_enabled"])
        self.assertEqual(bus.writes, [])
        self.assertEqual(arm.set_single_motor_mode(False), {"single_motor_mode": False})
        self.assertTrue(arm.state()["pose_enabled"])

    def test_cannot_enable_while_joint_is_moving_or_goal_unsettled(self):
        bus = MotionBus((2000,) * 6)
        arm = self.arm(bus)
        bus.moving["shoulder_lift"] = 1
        with self.assertRaisesRegex(ValueError, "Motor 2 is still moving"):
            arm.set_single_motor_mode(True)
        bus.moving["shoulder_lift"] = 0
        bus.goals["shoulder_lift"] = 2050
        with self.assertRaisesRegex(ValueError, "Motor 2 is still moving"):
            arm.set_single_motor_mode(True)
        self.assertFalse(arm.single_motor_mode)
        self.assertEqual(bus.writes, [])

    def test_existing_gripper_preload_can_hold_stationary(self):
        bus = MotionBus((2000,) * 6)
        arm = self.arm(bus)
        bus.goals["gripper"] = 1970
        arm.set_single_motor_mode(True)
        arm.start(1, 1)
        self.assertEqual(arm.hold["motor"], 1)

    def test_encoder_motion_blocks_mode_even_if_moving_flag_is_zero(self):
        class DriftingBus(MotionBus):
            def __init__(self, positions):
                super().__init__(positions)
                self.position_reads = 0

            def sync_read(self, register, normalize=True, num_retry=0):
                if register == "Present_Position":
                    self.position_reads += 1
                    if self.position_reads == 3:
                        self.positions["elbow_flex"] += 3
                return super().sync_read(register, normalize=normalize, num_retry=num_retry)

        bus = DriftingBus((2000,) * 6)
        arm = self.arm(bus)
        with self.assertRaisesRegex(ValueError, "Motor 3 is still moving"):
            arm.set_single_motor_mode(True)
        self.assertFalse(arm.single_motor_mode)

    def test_switch_to_another_motor_waits_for_previous_settle(self):
        bus = MotionBus((2000,) * 6)
        arm = self.arm(bus)
        arm.set_single_motor_mode(True)
        arm.start(1, 1)
        bus.moving["shoulder_pan"] = 1
        with self.assertRaisesRegex(ValueError, "Motor 1 is still moving"):
            arm.start(2, 1)
        self.assertIsNone(arm.hold)
        bus.moving["shoulder_pan"] = 0
        bus.goals["shoulder_pan"] = 2050
        with self.assertRaisesRegex(ValueError, "Motor 1 is still moving"):
            arm.start(2, 1)
        bus.positions["shoulder_pan"] = 2050
        arm.start(2, 1)
        self.assertEqual(arm.hold["motor"], 2)

    def test_gripper_go_waits_for_other_joint(self):
        bus = MotionBus((2000,) * 6)
        arm = self.arm(bus)
        arm.set_single_motor_mode(True)
        bus.moving["shoulder_pan"] = 1
        with self.assertRaisesRegex(ValueError, "Motor 1 is still moving"):
            arm.go(1900)
        self.assertIsNone(arm.hold)

    def test_mode_change_requires_idle_and_rejects_group_writes(self):
        bus = MotionBus((2000,) * 6)
        arm = self.arm(bus)
        arm.start(1, 1)
        with self.assertRaisesRegex(ValueError, "Stop the active movement"):
            arm.set_single_motor_mode(True)
        arm.stop()
        arm.set_single_motor_mode(True)
        before = len(bus.writes)
        for pose in ("home", "extend", "dance", "press_button", "lift_this"):
            with self.assertRaisesRegex(ValueError, "Multi-joint poses are unavailable"):
                arm.pose_start(pose)
        with self.assertRaisesRegex(ValueError, "Hold all joints is unavailable"):
            arm.hold_all()
        self.assertEqual(len(bus.writes), before)

    def test_invalid_mode_value_is_rejected(self):
        bus = MotionBus((2000,) * 6)
        arm = self.arm(bus)
        with self.assertRaisesRegex(ValueError, "must be a boolean"):
            arm.set_single_motor_mode(1)


if __name__ == "__main__":
    unittest.main()
