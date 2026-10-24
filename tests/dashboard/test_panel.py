"""Fake-bus checks for the dashboard controller's jogs and pose moves."""

import json
import time
import unittest
from unittest.mock import patch

from care_reach.dashboard.panel import HOME_TOLERANCE, POSE_TOLERANCE, STOP_SETTLE_LEAD, PanelController
from care_reach.robot import JOINT_NAMES
from tests.dashboard.fakes import FakeBus, TrackingBus
from tests.support import FOLDED_TICKS, ArmFilesTestCase


class DashboardBase(ArmFilesTestCase):
    def arm(self, bus):
        """Build a controller on ``bus`` without starting its worker thread."""
        thread_patch = patch("care_reach.dashboard.panel.threading.Thread")
        thread_patch.start()
        self.addCleanup(thread_patch.stop)
        arm = PanelController("fake", bus_factory=lambda _: bus, settings_path=self.settings_path,
                              calibration_path=self.calibration_path, pose_path=self.pose_path)
        self.addCleanup(arm.close)
        return arm


class DashboardTests(DashboardBase):
    def test_connect_and_close_never_change_torque(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        self.assertEqual(bus.writes, [])
        self.assertTrue(all(m["motion_enabled"] for m in arm.state()["motors"]))
        arm.close()
        self.assertEqual(bus.disconnects, [False])
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_hold_jog_stop_and_lost_heartbeat_preserve_torque(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        token = arm.start(1, 1)
        arm._advance(time.monotonic() + 0.08)
        self.assertGreaterEqual(bus.goals["shoulder_pan"], 2000)
        arm.heartbeat(token)
        arm.stop(token)
        # A normal release settles forward instead of commanding a reversal,
        # and never writes past the last forward goal.
        self.assertGreaterEqual(bus.goals["shoulder_pan"], 2000)
        self.assertLessEqual(bus.goals["shoulder_pan"], 2002)
        token = arm.start(6, -1)
        arm.hold["expires"] = time.monotonic() - 1
        arm._advance(time.monotonic())
        self.assertIsNone(arm.hold)
        self.assertEqual(bus.goals["gripper"], 2000)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_folded_endpoint_only_allows_motion_toward_normal_limits(self):
        bus = FakeBus(FOLDED_TICKS)
        arm = self.arm(bus)
        state = arm.state()["motors"]
        self.assertTrue(state[1]["can_forward"])
        self.assertFalse(state[1]["can_backward"])
        self.assertTrue(state[2]["can_backward"])
        self.assertFalse(state[2]["can_forward"])
        with self.assertRaisesRegex(ValueError, "limit"):
            arm.start(2, -1)
        with self.assertRaisesRegex(ValueError, "limit"):
            arm.start(3, 1)
        arm.start(2, 1)
        arm.stop()
        self.assertFalse(any(write[0] == "Torque_Enable" for write in bus.writes))
        bus.positions["elbow_flex"] = 3070  # small torque-off encoder drift
        self.assertTrue(arm.state()["motors"][2]["can_backward"])
        self.assertFalse(arm.state()["motors"][2]["can_forward"])

    def test_explicit_hold_can_enable_a_torque_off_arm(self):
        bus = FakeBus(FOLDED_TICKS, torque=0)
        arm = self.arm(bus)
        self.assertTrue(arm.state()["hold_all_enabled"])
        with self.assertRaisesRegex(RuntimeError, "torque is off"):
            arm.start(1, 1)
        self.assertFalse(bus.writes)
        arm.hold_all()
        self.assertTrue(all(value == 1 for value in bus.torque.values()))
        arm.close()
        self.assertEqual(bus.disconnects, [False])

    def test_saved_scale_and_bounds_are_persistent(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        arm.scale(4, 8)
        settings = json.loads(self.settings_path.read_text(encoding="utf-8"))
        self.assertEqual(settings["4"]["scale"], 8)
        self.assertEqual(settings["4"]["lower"], 863)
        self.assertEqual(settings["4"]["upper"], 3110)
        motor = arm.state()["motors"][3]
        self.assertEqual(motor["lower"], 863)
        self.assertEqual(motor["upper"], 3110)
        self.assertEqual(motor["effective_lower"], 863)
        self.assertEqual(motor["effective_upper"], 3110)

    def test_holding_button_repeats_motion_and_scale_changes_distance(self):
        def travel(scale):
            bus = FakeBus((2000,) * 6)
            arm = self.arm(bus)
            arm.scale(1, scale)
            arm.start(1, 1)
            start = time.monotonic()
            arm.hold["expires"] = start + 10
            for step in range(1, 31):
                arm._advance(start + step * 0.05)
                bus.positions["shoulder_pan"] = bus.goals["shoulder_pan"]
            distance = bus.positions["shoulder_pan"] - 2000
            arm.stop()
            return distance

        slow = travel(1)
        fast = travel(8)
        self.assertGreater(slow, 10)
        self.assertGreater(fast, slow * 2)

    def jog_with_lag(self, direction, lag):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        token = arm.start(1, direction)
        start = time.monotonic()
        arm.hold["expires"] = start + 10
        for step in range(1, 41):
            arm._advance(start + step * 0.05)
            bus.positions["shoulder_pan"] = bus.goals["shoulder_pan"] + lag
        return bus, arm, token

    def test_forward_release_never_reverses_past_measured_or_last_goal(self):
        bus, arm, token = self.jog_with_lag(1, -40)
        last_goal = bus.goals["shoulder_pan"]
        measured = bus.positions["shoulder_pan"]
        self.assertGreater(last_goal - measured, 15)
        arm.stop(token)
        goal = bus.goals["shoulder_pan"]
        self.assertGreaterEqual(goal, measured)
        self.assertLessEqual(goal, last_goal)
        self.assertLessEqual(goal - measured, STOP_SETTLE_LEAD)
        self.assertGreater(goal - measured, 0)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_backward_release_never_reverses_past_measured_or_last_goal(self):
        bus, arm, token = self.jog_with_lag(-1, 40)
        last_goal = bus.goals["shoulder_pan"]
        measured = bus.positions["shoulder_pan"]
        self.assertGreater(measured - last_goal, 15)
        arm.stop(token)
        goal = bus.goals["shoulder_pan"]
        self.assertLessEqual(goal, measured)
        self.assertGreaterEqual(goal, last_goal)
        self.assertLessEqual(measured - goal, STOP_SETTLE_LEAD)
        self.assertGreater(measured - goal, 0)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_release_caps_settle_at_the_last_forward_goal(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        token = arm.start(1, 1)
        start = time.monotonic()
        arm.hold["expires"] = start + 10
        for step in range(1, 4):
            arm._advance(start + step * 0.05)
        last_goal = bus.goals["shoulder_pan"]
        self.assertGreater(last_goal, 2000)
        self.assertLess(last_goal - 2000, STOP_SETTLE_LEAD)
        arm.stop(token)
        self.assertEqual(bus.goals["shoulder_pan"], last_goal)

    def test_release_after_overshoot_holds_measured_position(self):
        bus, arm, token = self.jog_with_lag(1, -5)
        bus.positions["shoulder_pan"] = bus.goals["shoulder_pan"] + 12
        measured = bus.positions["shoulder_pan"]
        arm.stop(token)
        self.assertEqual(bus.goals["shoulder_pan"], measured)

    def test_fault_path_holds_measured_position_immediately(self):
        bus, arm, token = self.jog_with_lag(1, -40)
        bus.loads["shoulder_pan"] = 900
        arm.hold["expires"] = time.monotonic() + 10
        arm.hold["next"] = time.monotonic()
        start = time.monotonic()
        with self.assertRaises(RuntimeError):
            for attempt in range(3):
                arm._advance(start + attempt * 0.05)
        measured = bus.positions["shoulder_pan"]
        arm.error = "Motor 1 sustained high load"
        arm._stop()
        self.assertIsNone(arm.hold)
        self.assertEqual(bus.goals["shoulder_pan"], measured)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_jog_auto_stop_at_limit_never_moves_outward(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        arm.scale(1, 8)
        arm.start(1, 1)
        start = time.monotonic()
        arm.hold["expires"] = start + 60
        for step in range(1, 201):
            arm._advance(start + step * 0.05)
            bus.positions["shoulder_pan"] = min(bus.goals["shoulder_pan"], 2597)
            if arm.hold is None:
                break
        self.assertIsNone(arm.hold)
        self.assertLessEqual(bus.goals["shoulder_pan"], 2597)
        self.assertLessEqual(bus.goals["shoulder_pan"] - bus.positions["shoulder_pan"], STOP_SETTLE_LEAD)
        self.assertGreaterEqual(bus.goals["shoulder_pan"], bus.positions["shoulder_pan"])


class PoseTests(DashboardBase):
    FOLD = FOLDED_TICKS

    def run_pose(self, arm, seconds=120.0):
        start = time.monotonic()
        index = 0
        for index in range(1, int(seconds / 0.06) + 1):
            if arm.hold is None:
                return True, index
            arm._advance(start + index * 0.06)
        return arm.hold is None, index

    def test_home_from_fold_moves_all_joints_with_bounded_lead(self):
        bus = TrackingBus(self.FOLD)
        arm = self.arm(bus)
        result = arm.pose_start("home")
        self.assertEqual(result["pose"], "home")
        done, steps = self.run_pose(arm)
        self.assertTrue(done, f"pose move did not finish in {steps} steps")
        for index, name in enumerate(JOINT_NAMES):
            self.assertLessEqual(abs(bus.positions[name] - 2000), HOME_TOLERANCE, name)
            self.assertEqual(bus.goals[name], 2000)
        self.assertLessEqual(max(bus.leads), 80)
        self.assertGreater(len(bus.leads), 20)
        self.assertEqual(bus.disconnects, [])
        self.assertTrue(all(value == 1 for value in bus.torque.values()))
        self.assertFalse(arm.state()["pose"]["active"])

    def test_home_trims_a_joint_stuck_inside_the_old_completion_window(self):
        bus = TrackingBus((2000, 2000, 2026, 2000, 2000, 2000))
        arm = self.arm(bus)
        arm.pose_start("home")
        done, _ = self.run_pose(arm)
        self.assertTrue(done)
        self.assertLessEqual(abs(bus.positions["elbow_flex"] - 2000), HOME_TOLERANCE)
        self.assertTrue(any(write[0] == "Goal_Position" and len(write) == 2
                            and write[1]["elbow_flex"] == 1960 for write in bus.writes))
        self.assertEqual(bus.goals["elbow_flex"], 2000)

    def test_fold_from_home_reaches_recorded_fold(self):
        bus = TrackingBus((2000,) * 6)
        arm = self.arm(bus)
        arm.pose_start("fold")
        done, steps = self.run_pose(arm)
        self.assertTrue(done, f"pose move did not finish in {steps} steps")
        for index, name in enumerate(JOINT_NAMES):
            self.assertLessEqual(abs(bus.positions[name] - self.FOLD[index]), POSE_TOLERANCE, name)
            self.assertEqual(bus.goals[name], self.FOLD[index])
        self.assertLessEqual(max(bus.leads), 80)
        self.assertEqual(bus.disconnects, [])
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_full_dance_from_fold_uses_one_bus_and_finishes_home(self):
        bus = TrackingBus(self.FOLD)
        arm = self.arm(bus)
        arm.pose_start("dance")
        self.assertEqual(arm.state()["pose"]["name"], "dance")
        done, steps = self.run_pose(arm, seconds=160)
        self.assertTrue(done, f"dance did not finish in {steps} steps")
        self.assertGreater(len(bus.leads), 1100)
        self.assertLessEqual(max(bus.leads), 80)
        self.assertEqual(bus.disconnects, [])
        self.assertTrue(all(value == 1 for value in bus.torque.values()))
        for name in JOINT_NAMES:
            self.assertEqual(bus.goals[name], 2000)
            self.assertLessEqual(abs(bus.positions[name] - 2000), HOME_TOLERANCE)

    def test_dance_cancel_holds_measured_and_blocks_jog(self):
        bus = TrackingBus((2000,) * 6)
        arm = self.arm(bus)
        arm.pose_start("dance")
        start = time.monotonic()
        for index in range(1, 12):
            arm._advance(start + index * .06)
        self.assertTrue(arm.state()["pose"]["active"])
        self.assertGreater(arm.state()["pose"]["progress"], 0)
        with self.assertRaisesRegex(ValueError, "pose move"):
            arm.start(1, 1)
        arm.pose_stop()
        self.assertIsNone(arm.hold)
        self.assertEqual(bus.goals, bus.positions)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_fold_then_home_round_trip_from_fold_start(self):
        bus = TrackingBus(self.FOLD)
        arm = self.arm(bus)
        arm.pose_start("home")
        self.assertTrue(self.run_pose(arm)[0])
        arm.pose_start("fold")
        done, steps = self.run_pose(arm)
        self.assertTrue(done, f"pose move did not finish in {steps} steps")
        for index, name in enumerate(JOINT_NAMES):
            self.assertLessEqual(abs(bus.positions[name] - self.FOLD[index]), POSE_TOLERANCE, name)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_concurrent_requests_rejected_while_pose_active(self):
        bus = TrackingBus((2000,) * 6)
        arm = self.arm(bus)
        arm.pose_start("fold")
        with self.assertRaisesRegex(ValueError, "already active"):
            arm.pose_start("home")
        with self.assertRaisesRegex(ValueError, "pose move"):
            arm.start(1, 1)
        with self.assertRaisesRegex(ValueError, "pose move"):
            arm.go(2000)
        with self.assertRaisesRegex(ValueError, "pose move"):
            arm.hold_all()
        arm.pose_stop()
        arm.start(1, 1)
        with self.assertRaisesRegex(ValueError, "already active"):
            arm.pose_start("home")
        arm.stop()

    def test_cancel_holds_measured_pose_and_preserves_torque(self):
        bus = TrackingBus((2000,) * 6)
        arm = self.arm(bus)
        arm.pose_start("fold")
        start = time.monotonic()
        for index in range(1, 6):
            arm._advance(start + index * 0.06)
        self.assertTrue(arm.state()["pose"]["active"])
        arm.pose_stop()
        self.assertIsNone(arm.hold)
        self.assertEqual(bus.goals, bus.positions)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))
        self.assertEqual(bus.disconnects, [])
        self.assertFalse(arm.state()["pose"]["active"])

    def test_fault_holds_measured_pose_and_preserves_torque(self):
        class FaultBus(TrackingBus):
            def sync_write(self, register, values, normalize=True):
                super().sync_write(register, values, normalize)
                if register == "Goal_Position" and len(self.writes) == 4:
                    self.status["elbow_flex"] = 32

        bus = FaultBus((2000,) * 6)
        arm = self.arm(bus)
        arm.pose_start("fold")
        start = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, "fault 32"):
            for index in range(1, 20):
                arm._advance(start + index * 0.06)
        arm.error = "Motor 3 servo fault 32"
        arm._stop()
        self.assertIsNone(arm.hold)
        self.assertEqual(bus.goals, bus.positions)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))
        self.assertEqual(bus.disconnects, [])

    def test_stall_holds_measured_pose_and_preserves_torque(self):
        class FrozenBus(FakeBus):
            def sync_write(self, register, values, normalize=True):
                super().sync_write(register, values, normalize)

        bus = FrozenBus((2000,) * 6)
        arm = self.arm(bus)
        arm.pose_start("fold")
        start = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, "stalled"):
            for index in range(1, 100):
                arm._advance(start + index * 0.06)
        arm._stop()
        self.assertIsNone(arm.hold)
        self.assertEqual(bus.goals, bus.positions)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))
        self.assertEqual(bus.disconnects, [])

    def test_timeout_stops_and_holds_measured_pose(self):
        bus = TrackingBus((2000,) * 6)
        arm = self.arm(bus)
        arm.pose_start("fold")
        arm.hold["expires"] = time.monotonic() - 1
        arm._advance(time.monotonic())
        self.assertIsNone(arm.hold)
        self.assertEqual(arm.error, "Movement timed out and stopped")
        self.assertEqual(bus.goals, bus.positions)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))
        self.assertEqual(bus.disconnects, [])

    def test_close_during_pose_holds_measured_and_never_releases_torque(self):
        bus = TrackingBus((2000,) * 6)
        arm = self.arm(bus)
        arm.pose_start("fold")
        start = time.monotonic()
        for index in range(1, 6):
            arm._advance(start + index * 0.06)
        arm.close()
        self.assertEqual(bus.goals, bus.positions)
        self.assertEqual(bus.disconnects, [False])
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_pose_start_rejects_torque_off_arm(self):
        bus = FakeBus((2000,) * 6, torque=0)
        arm = self.arm(bus)
        with self.assertRaisesRegex(RuntimeError, "torque is off"):
            arm.pose_start("home")

    def test_voltage_status_is_named_and_stops_motion(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        bus.status["gripper"] = 1
        self.assertIn("Voltage alarm", arm.state()["motors"][5]["fault"])
        with self.assertRaisesRegex(RuntimeError, "Motor 6 voltage alarm"):
            arm.pose_start("dance")
        self.assertIsNone(arm.hold)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_home_target_outside_envelope_is_rejected(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        with self.assertRaises(ValueError):
            arm.pose_start("bogus")


if __name__ == "__main__":
    unittest.main()
