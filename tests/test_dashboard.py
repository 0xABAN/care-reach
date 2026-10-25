"""Fake-bus dashboard checks. These tests never open a serial device."""

import http.client
import io
import json
import re
import shutil
import subprocess
import tempfile
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

from motion_core import JOINT_NAMES
from home_pose import home_target
from server import Arm, GRIP_HOLD_LEAD, HOME_TOLERANCE, POSE_TOLERANCE, STOP_SETTLE_LEAD, handler_for


class FakeBus:
    def __init__(self, positions, torque=1):
        self.positions = dict(zip(JOINT_NAMES, positions, strict=True))
        self.goals = self.positions.copy()
        self.torque = {name: torque for name in JOINT_NAMES}
        self.loads = {name: 0 for name in JOINT_NAMES}
        self.status = {name: 0 for name in JOINT_NAMES}
        self.voltages = {name: 53 for name in JOINT_NAMES}
        self.currents = {name: 0 for name in JOINT_NAMES}
        self.connected = False
        self.writes = []
        self.disconnects = []

    def connect(self, handshake=True):
        assert handshake
        self.connected = True

    def disconnect(self, disable_torque=True):
        self.disconnects.append(disable_torque)
        self.connected = False

    def sync_read(self, register, normalize=True, num_retry=0):
        assert self.connected and not normalize and num_retry == 2
        values = {
            "Present_Position": self.positions,
            "Goal_Position": self.goals,
            "Torque_Enable": self.torque,
            "Min_Position_Limit": {name: 0 for name in JOINT_NAMES},
            "Max_Position_Limit": {name: 4095 for name in JOINT_NAMES},
            "Operating_Mode": {name: 0 for name in JOINT_NAMES},
            "Status": self.status,
            "Present_Voltage": self.voltages,
            "Present_Current": self.currents,
            "Present_Temperature": {name: 24 for name in JOINT_NAMES},
            "Present_Load": self.loads,
        }
        return values[register].copy()

    def write(self, register, name, value, normalize=True, num_retry=0):
        assert self.connected and not normalize and num_retry == 2
        assert register == "Goal_Position"
        self.goals[name] = value
        self.writes.append((register, name, value))

    def sync_write(self, register, values, normalize=True):
        assert self.connected and not normalize
        if register == "Torque_Enable":
            self.torque.update(values)
        elif register == "Goal_Position":
            assert set(values) == set(JOINT_NAMES)
            self.goals.update(values)
        else:
            raise AssertionError(f"unexpected sync_write register {register}")
        self.writes.append((register, values.copy()))


class DashboardBase(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        ranges = [(1096, 2617), (883, 3243), (861, 3007), (843, 3130), (0, 4095), (1317, 2762)]
        calibration = {
            name: {"range_min": lo, "range_max": hi}
            for name, (lo, hi) in zip(JOINT_NAMES, ranges, strict=True)
        }
        settings = {
            str(index): {"lower": lo, "upper": hi, "scale": 1}
            for index, (lo, hi) in enumerate(
                ((1116, 2597), (903, 3223), (881, 2987), (863, 3110), (None, None), (1317, 2742)), 1
            )
        }
        folded = (2040, 893, 3064, 2888, 2416, 1470)
        pose = {
            "robot_id": "care_reach_follower",
            "positions_ticks": dict(zip(JOINT_NAMES, folded, strict=True)),
        }
        self.settings_path = root / "joint_settings.json"
        self.calibration_path = root / "calibration.json"
        self.pose_path = root / "folded_pose.json"
        self.settings_path.write_text(json.dumps(settings), encoding="utf-8")
        self.calibration_path.write_text(json.dumps(calibration), encoding="utf-8")
        self.pose_path.write_text(json.dumps(pose), encoding="utf-8")

    def arm(self, bus):
        thread_patch = patch("arm_control.arm.threading.Thread")
        thread_patch.start()
        self.addCleanup(thread_patch.stop)
        arm = Arm("fake", bus_factory=lambda _: bus, settings_path=self.settings_path,
                  calibration_path=self.calibration_path, pose_path=self.pose_path,
                  telemetry_path=self.settings_path.parent / "telemetry.jsonl")
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

    def test_gripper_closing_release_keeps_a_bounded_preload(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        token = arm.start(6, -1)
        arm.hold["last_goal"] = 1900
        arm.stop(token)
        self.assertEqual(bus.goals["gripper"], 2000 - GRIP_HOLD_LEAD)
        self.assertEqual(bus.torque["gripper"], 1)

    def test_gripper_timeout_keeps_preload_but_fault_releases_it(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        arm.start(6, -1)
        arm.hold["last_goal"] = 1900
        arm.hold["expires"] = time.monotonic() - 1
        arm._advance(time.monotonic())
        self.assertEqual(bus.goals["gripper"], 2000 - GRIP_HOLD_LEAD)
        self.assertEqual(arm.error, "Movement timed out and stopped")
        arm.start(6, -1)
        arm.hold["last_goal"] = 1900
        bus.status["gripper"] = 32
        arm.stop()
        self.assertEqual(bus.goals["gripper"], 2000)

    def test_gripper_close_to_tick_keeps_target_when_reached(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        arm.go(1988)
        arm._advance(time.monotonic() + 0.08)
        self.assertIsNone(arm.hold)
        self.assertEqual(bus.goals["gripper"], 1988)

    def test_faulted_gripper_does_not_block_healthy_jog_or_receive_commands(self):
        bus = FakeBus((2000,) * 6)
        bus.status["gripper"] = 32
        arm = self.arm(bus)
        state = arm.state()
        self.assertTrue(state["motors"][0]["motion_enabled"])
        self.assertFalse(state["motors"][5]["motion_enabled"])
        self.assertFalse(state["pose_enabled"])
        token = arm.start(1, 1)
        arm._advance(time.monotonic() + 0.08)
        arm.stop(token)
        self.assertTrue(bus.writes)
        self.assertTrue(all(write[1] == "shoulder_pan" for write in bus.writes))
        with self.assertRaisesRegex(RuntimeError, "Motor 6 servo fault 32"):
            arm.start(6, 1)
        with self.assertRaisesRegex(RuntimeError, "Motor 6 servo fault 32"):
            arm.pose_start("home")

    def test_existing_inactive_voltage_alarm_allows_healthy_jog(self):
        bus = FakeBus((2000,) * 6)
        bus.status["gripper"] = 1
        arm = self.arm(bus)
        token = arm.start(1, 1)
        arm._advance(time.monotonic() + 0.08)
        arm.stop(token)
        self.assertTrue(all(write[1] == "shoulder_pan" for write in bus.writes))

    def test_new_inactive_voltage_alarm_stops_motion(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        arm.start(1, 1)
        bus.status["gripper"] = 1
        with self.assertRaisesRegex(RuntimeError, "Motor 6 voltage alarm"):
            arm._advance(time.monotonic() + 0.08)
        records = [json.loads(line) for line in (self.settings_path.parent / "telemetry.jsonl").read_text().splitlines()]
        self.assertTrue(any(record["event"] == "status_changed" for record in records))

    def test_inactive_low_voltage_still_blocks_motion(self):
        bus = FakeBus((2000,) * 6)
        bus.voltages["gripper"] = 47
        arm = self.arm(bus)
        with self.assertRaisesRegex(RuntimeError, "Motor 6 voltage stayed low"):
            arm.start(1, 1)

    def test_voltage_error_clears_after_stable_recovery(self):
        arm = self.arm(FakeBus((2000,) * 6))
        arm._set_error("Motor 6 voltage stayed low")
        arm.healthy_since = time.monotonic() - 3
        state = arm.state()
        self.assertIsNone(state["error"])
        self.assertTrue(any(event["event"] == "voltage_recovered" for event in state["recent_events"]))

    def test_folded_endpoint_only_allows_motion_toward_normal_limits(self):
        bus = FakeBus((2040, 893, 3064, 2888, 2416, 1470))
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
        bus = FakeBus((2040, 893, 3064, 2888, 2416, 1470), torque=0)
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
    FOLD = (2040, 893, 3064, 2888, 2416, 1470)
    HOME = home_target()
    EXTEND = (2000, 1903, 1126, 1988, 387, 1993)

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
            self.assertLessEqual(abs(bus.positions[name] - self.HOME[index]), HOME_TOLERANCE, name)
            self.assertEqual(bus.goals[name], self.HOME[index])
        self.assertLessEqual(max(bus.leads), 80)
        self.assertGreater(len(bus.leads), 20)
        self.assertEqual(bus.disconnects, [])
        self.assertTrue(all(value == 1 for value in bus.torque.values()))
        self.assertFalse(arm.state()["pose"]["active"])

    def test_home_trims_a_joint_stuck_inside_the_old_completion_window(self):
        bus = TrackingBus((2000, 1056, 3039, 2027, 387, 1993))
        arm = self.arm(bus)
        arm.pose_start("home")
        done, _ = self.run_pose(arm)
        self.assertTrue(done)
        self.assertLessEqual(abs(bus.positions["elbow_flex"] - self.HOME[2]), HOME_TOLERANCE)
        self.assertTrue(any(write[0] == "Goal_Position" and len(write) == 2
                            and write[1]["elbow_flex"] == 3065 for write in bus.writes))
        self.assertFalse(any(write[0] == "Goal_Position" and len(write) == 2
                             and write[1]["elbow_flex"] > 3065 for write in bus.writes))
        self.assertEqual(bus.goals["elbow_flex"], 3065)

    def test_extend_from_home_reaches_confirmed_upright_target(self):
        bus = TrackingBus(self.HOME)
        arm = self.arm(bus)
        arm.pose_start("extend")
        done, steps = self.run_pose(arm)
        self.assertTrue(done, f"pose move did not finish in {steps} steps")
        for index, name in enumerate(JOINT_NAMES):
            self.assertLessEqual(abs(bus.positions[name] - self.EXTEND[index]), POSE_TOLERANCE, name)
            self.assertEqual(bus.goals[name], self.EXTEND[index])
        self.assertLessEqual(max(bus.leads), 55)
        for register, goals in bus.writes:
            if register != "Goal_Position":
                continue
            shoulder_fraction = ((goals["shoulder_lift"] - self.HOME[1])
                                 / (self.EXTEND[1] - self.HOME[1]))
            elbow_fraction = ((goals["elbow_flex"] - self.HOME[2])
                              / (self.EXTEND[2] - self.HOME[2]))
            self.assertLess(abs(shoulder_fraction - elbow_fraction), .05)
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
        for index, name in enumerate(JOINT_NAMES):
            self.assertEqual(bus.goals[name], self.HOME[index])
            self.assertLessEqual(abs(bus.positions[name] - self.HOME[index]), HOME_TOLERANCE)

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

    def test_extend_then_home_round_trip(self):
        bus = TrackingBus(self.HOME)
        arm = self.arm(bus)
        arm.pose_start("extend")
        self.assertTrue(self.run_pose(arm)[0])
        arm.pose_start("home")
        done, steps = self.run_pose(arm)
        self.assertTrue(done, f"pose move did not finish in {steps} steps")
        for index, name in enumerate(JOINT_NAMES):
            self.assertLessEqual(abs(bus.positions[name] - self.HOME[index]), POSE_TOLERANCE, name)
        self.assertLessEqual(max(bus.leads), 55)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_concurrent_requests_rejected_while_pose_active(self):
        bus = TrackingBus((2000,) * 6)
        arm = self.arm(bus)
        arm.pose_start("extend")
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
        arm.pose_start("extend")
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
        arm.pose_start("extend")
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
        arm.pose_start("extend")
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
        arm.pose_start("extend")
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
        arm.pose_start("extend")
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

    def test_fold_pose_is_rejected_with_wrist_camera(self):
        bus = TrackingBus(self.HOME)
        arm = self.arm(bus)
        with self.assertRaisesRegex(ValueError, "folded pose is unavailable"):
            arm.pose_start("fold")
        self.assertIsNone(arm.hold)
        self.assertEqual(bus.writes, [])

    def test_dance_from_extend_uses_coordinated_home_leg(self):
        bus = TrackingBus(self.EXTEND)
        arm = self.arm(bus)
        arm.pose_start("dance")
        self.assertIsNotNone(arm.hold["trajectory"])
        start = time.monotonic()
        for index in range(1, 60):
            arm._advance(start + index * .06)
        self.assertLessEqual(max(bus.leads), 55)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))
        arm.pose_stop()

    def test_full_dance_from_extend_returns_home(self):
        bus = TrackingBus(self.EXTEND)
        arm = self.arm(bus)
        arm.pose_start("dance")
        done, steps = self.run_pose(arm, seconds=200)
        self.assertTrue(done, f"dance from upright did not finish in {steps} steps")
        self.assertEqual(bus.goals, dict(zip(JOINT_NAMES, self.HOME, strict=True)))
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_home_target_outside_envelope_is_rejected(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        with self.assertRaises(ValueError):
            arm.pose_start("bogus")


class TrackingBus(FakeBus):
    """Encoder follows commanded goals by up to 8 ticks per joint write."""

    def __init__(self, positions, torque=1):
        super().__init__(positions, torque)
        self.leads = []

    def sync_write(self, register, values, normalize=True):
        before = self.positions.copy()
        super().sync_write(register, values, normalize)
        if register == "Goal_Position":
            self.leads.append(max(abs(values[name] - before[name]) for name in JOINT_NAMES))
            for name in JOINT_NAMES:
                distance = self.goals[name] - before[name]
                self.positions[name] += max(-8, min(8, distance))


class BlockedGripBus(TrackingBus):
    """The gripper stops on an object at ``floor`` ticks."""

    def __init__(self, positions, floor):
        super().__init__(positions)
        self.floor = floor

    def sync_write(self, register, values, normalize=True):
        super().sync_write(register, values, normalize)
        self.positions["gripper"] = max(self.positions["gripper"], self.floor)


class WeakShoulderBus(TrackingBus):
    """Under load the shoulder needs a 50-tick lead to lift and sags 45 ticks."""

    def __init__(self, positions, lifted):
        super().__init__(positions)
        self.lifted = lifted

    def sync_write(self, register, values, normalize=True):
        before = self.positions["shoulder_lift"]
        super().sync_write(register, values, normalize)
        if register != "Goal_Position":
            return
        lifting = values["shoulder_lift"] < before
        if lifting and before - values["shoulder_lift"] < 50:
            self.positions["shoulder_lift"] = before
        self.positions["shoulder_lift"] = max(self.positions["shoulder_lift"], self.lifted + 45)


class TaughtMoveTests(DashboardBase):
    HOME = home_target()

    def taught_arm(self, bus):
        thread_patch = patch("server.threading.Thread")
        thread_patch.start()
        self.addCleanup(thread_patch.stop)
        self.taught_path = self.settings_path.parent / "taught_moves.json"
        arm = Arm("fake", bus_factory=lambda _: bus, settings_path=self.settings_path,
                  calibration_path=self.calibration_path, pose_path=self.pose_path,
                  telemetry_path=self.settings_path.parent / "telemetry.jsonl",
                  taught_path=self.taught_path)
        self.addCleanup(arm.close)
        return arm

    def offset(self, *deltas):
        return tuple(tick + delta for tick, delta in zip(self.HOME, deltas, strict=True))

    def teach(self, arm, bus, move, steps):
        for step in steps:
            bus.positions.update(zip(JOINT_NAMES, step, strict=True))
            arm.teach_add(move)
        bus.positions.update(zip(JOINT_NAMES, self.HOME, strict=True))
        bus.goals = bus.positions.copy()

    def run_pose(self, arm, seconds=60.0):
        start = time.monotonic()
        for index in range(1, int(seconds / 0.05) + 1):
            if arm.hold is None:
                return True
            arm._advance(start + index * 0.05)
        return arm.hold is None

    def test_teach_saves_measured_steps_and_reloads(self):
        bus = FakeBus(self.HOME)
        arm = self.taught_arm(bus)
        step = self.offset(40, 60, -60, 30, 0, 0)
        self.teach(arm, bus, "press_button", [step])
        self.assertEqual(arm.state()["taught_moves"], {"press_button": 1, "lift_this": 0})
        saved = json.loads(self.taught_path.read_text())
        self.assertEqual(saved["press_button"], [list(step)])
        self.assertEqual(bus.writes, [])  # Teaching never moves the arm.
        arm.teach_clear("press_button")
        self.assertEqual(arm.state()["taught_moves"]["press_button"], 0)

    def test_untaught_or_unknown_move_is_refused(self):
        arm = self.taught_arm(FakeBus(self.HOME))
        with self.assertRaisesRegex(ValueError, "teach it first"):
            arm.pose_start("lift_this")
        with self.assertRaisesRegex(ValueError, "move must be"):
            arm.teach_add("wave")
        self.assertIsNone(arm.hold)

    def test_press_button_plays_steps_and_returns(self):
        bus = TrackingBus(self.HOME)
        arm = self.taught_arm(bus)
        above = self.offset(120, 150, -150, 60, 0, 0)
        press = self.offset(120, 190, -150, 100, 0, 0)
        self.teach(arm, bus, "press_button", [above, press, above, self.HOME])
        arm.pose_start("press_button")
        self.assertEqual(arm.state()["pose"]["name"], "press_button")
        deepest = 0
        for _ in range(4000):
            if arm.hold is None:
                break
            arm._advance(time.monotonic() + _ * 0.05)
            deepest = max(deepest, bus.positions["shoulder_lift"])
        self.assertIsNone(arm.hold)
        self.assertGreaterEqual(deepest, press[1] - POSE_TOLERANCE)
        self.assertLessEqual(max(bus.leads), 55)
        for index, name in enumerate(JOINT_NAMES):
            self.assertEqual(bus.goals[name], self.HOME[index])
        self.assertTrue(all(value == 1 for value in bus.torque.values()))

    def test_lift_this_grip_stops_on_object_and_keeps_small_squeeze(self):
        floor = self.HOME[5] - 60
        bus = BlockedGripBus(self.HOME, floor)
        arm = self.taught_arm(bus)
        open_ = self.offset(0, 120, -120, 40, 0, 200)
        closed = self.offset(0, 120, -120, 40, 0, -300)
        lifted = self.offset(0, 40, -40, 0, 0, -300)
        # Teach with a free gripper, then replay with the object in the jaws.
        free = TrackingBus(self.HOME)
        teacher = self.taught_arm(free)
        self.teach(teacher, free, "lift_this", [open_, closed, lifted])
        arm.taught = teacher.taught
        arm.pose_start("lift_this")
        self.assertTrue(self.run_pose(arm), "lift move did not finish")
        self.assertIsNone(arm.error)
        self.assertEqual(bus.positions["gripper"], floor)
        self.assertEqual(bus.goals["gripper"], floor - GRIP_HOLD_LEAD)
        self.assertLessEqual(abs(bus.positions["shoulder_lift"] - lifted[1]), POSE_TOLERANCE)

    def test_loaded_lift_uses_larger_lead_and_settles_short_of_target(self):
        lowered = self.offset(0, 300, -200, 0, 0, 0)
        lifted = self.offset(0, 0, -200, 0, 0, 0)
        free = TrackingBus(self.HOME)
        teacher = self.taught_arm(free)
        self.teach(teacher, free, "lift_this", [lowered, lifted])
        bus = WeakShoulderBus(lowered, lifted[1])
        arm = self.taught_arm(bus)
        arm.taught = teacher.taught
        arm.pose_start("lift_this")
        self.assertTrue(self.run_pose(arm), "loaded lift did not finish")
        self.assertIsNone(arm.error)
        self.assertGreaterEqual(bus.positions["shoulder_lift"], lifted[1] + 45)
        self.assertLessEqual(bus.positions["shoulder_lift"], lifted[1] + 60)

    def test_taught_leads_are_120_for_shoulder_and_elbow_and_80_elsewhere(self):
        bus = TrackingBus(self.HOME)
        arm = self.taught_arm(bus)
        far = self.offset(400, 500, -500, 400, 400, 400)
        self.teach(arm, bus, "press_button", [far])
        before = bus.positions.copy()
        arm.pose_start("press_button")
        worst = [0] * 6
        start = time.monotonic()
        for step in range(1, 4000):
            if arm.hold is None:
                break
            snapshot = bus.positions.copy()
            arm._advance(start + step * 0.05)
            for index, name in enumerate(JOINT_NAMES):
                worst[index] = max(worst[index], abs(bus.goals[name] - snapshot[name]))
        self.assertIsNone(arm.hold)
        self.assertLessEqual(worst[1], 120)
        self.assertLessEqual(worst[2], 120)
        for index in (0, 3, 4, 5):
            self.assertLessEqual(worst[index], 80, JOINT_NAMES[index])

    def test_blocked_joint_far_from_target_still_stalls(self):
        lowered = self.offset(0, 300, -200, 0, 0, 0)
        lifted = self.offset(0, 0, -200, 0, 0, 0)
        free = TrackingBus(self.HOME)
        teacher = self.taught_arm(free)
        self.teach(teacher, free, "lift_this", [lowered, lifted])
        bus = WeakShoulderBus(lowered, lifted[1] + 100)  # Stops 145 ticks short.
        arm = self.taught_arm(bus)
        arm.taught = teacher.taught
        arm.pose_start("lift_this")
        with self.assertRaisesRegex(RuntimeError, "Motor 2 stalled"):
            self.run_pose(arm)

    def test_loaded_lift_moves_elbow_then_shoulder_one_at_a_time(self):
        open_ = self.offset(0, 150, -150, 0, 0, 200)
        lowered = self.offset(0, 300, -300, 0, 0, 200)
        closed = self.offset(0, 300, -300, 0, 0, -300)
        lifted = self.offset(0, 60, -500, 200, 0, -300)
        bus = TrackingBus(self.HOME)
        arm = self.taught_arm(bus)
        self.teach(arm, bus, "lift_this", [open_, lowered, closed, lifted])
        arm.pose_start("lift_this")
        both_moving_while_carrying = 0
        both_moving_free = 0
        start = time.monotonic()
        for step in range(1, 6000):
            if arm.hold is None:
                break
            before = bus.positions.copy()
            arm._advance(start + step * 0.05)
            moved = {name for name in ("shoulder_lift", "elbow_flex")
                     if abs(bus.positions[name] - before[name]) > 2}
            carrying = bus.positions["gripper"] < self.HOME[5]
            if moved == {"shoulder_lift", "elbow_flex"}:
                if carrying:
                    both_moving_while_carrying += 1
                else:
                    both_moving_free += 1
        self.assertIsNone(arm.hold)
        self.assertIsNone(arm.error)
        self.assertEqual(both_moving_while_carrying, 0)
        self.assertGreater(both_moving_free, 0)  # Reaching stays smooth.
        for index, name in enumerate(JOINT_NAMES[:5]):
            self.assertLessEqual(abs(bus.positions[name] - lifted[index]), POSE_TOLERANCE, name)

    def test_taught_segments_split_only_loaded_steps(self):
        from arm_control.arm import taught_segments
        home = self.HOME
        open_ = self.offset(0, 300, -300, 0, 0, 200)
        closed = self.offset(0, 300, -300, 0, 0, -300)
        lifted = self.offset(0, 60, -500, 200, 0, -300)
        segments = taught_segments(home, [open_, closed, lifted])
        self.assertEqual(segments[:2], [(home, open_), (open_, closed)])
        loaded = segments[2:]
        self.assertEqual(len(loaded), 3)
        self.assertEqual(loaded[0][1][2], lifted[2])   # Elbow first...
        self.assertEqual(loaded[0][1][1], closed[1])   # ...shoulder still.
        self.assertEqual(loaded[1][1][1], lifted[1])   # Then shoulder.
        self.assertEqual(loaded[-1][1], lifted)
        press = [self.offset(100, 150, -150, 60, 0, 0), self.HOME]
        self.assertEqual(taught_segments(home, press), [(home, press[0]), (press[0], home)])

    def run_steps(self, arm):
        start = time.monotonic()
        for step in range(1, 20000):
            if arm.hold is None:
                return step
            arm._advance(start + step * 0.05)
        self.fail("taught move did not finish")

    def test_speed_setting_persists_and_old_files_default_to_1x(self):
        bus = FakeBus(self.HOME)
        arm = self.taught_arm(bus)
        self.assertEqual(arm.state()["taught_speeds"], {"press_button": 1.0, "lift_this": 1.0})
        self.teach(arm, bus, "lift_this", [self.offset(0, 200, -200, 0, 0, 0)])
        arm.teach_speed("lift_this", 2)
        self.assertEqual(json.loads(self.taught_path.read_text())["speeds"]["lift_this"], 2.0)
        from arm_control.arm import load_taught, load_taught_speeds
        self.assertEqual(load_taught_speeds(self.taught_path)["lift_this"], 2.0)
        self.assertEqual(len(load_taught(self.taught_path)["lift_this"]), 1)
        for bad in (3, 0.5, "2", True, None):
            with self.assertRaisesRegex(ValueError, "speed must be"):
                arm.teach_speed("lift_this", bad)
        old = self.taught_path.parent / "old.json"
        old.write_text(json.dumps({"lift_this": [], "press_button": []}))
        self.assertEqual(load_taught_speeds(old), {"press_button": 1.0, "lift_this": 1.0})

    def test_2x_speed_finishes_faster_within_command_limits(self):
        from arm_control.arm import MAX_COMMAND_SPEED, taught_limits
        far = self.offset(300, 600, -600, 300, 0, 0)
        durations = {}
        for speed in (1, 2):
            bus = TrackingBus(self.HOME)
            arm = self.taught_arm(bus)
            self.teach(arm, bus, "press_button", [far, self.HOME])
            arm.teach_speed("press_button", speed)
            arm.pose_start("press_button")
            durations[speed] = self.run_steps(arm)
            self.assertIsNone(arm.error)
            self.assertLessEqual(max(bus.leads), 120)
        self.assertLess(durations[2], durations[1] * 0.8)
        vmax, _ = taught_limits(2.0)
        self.assertTrue(all(v <= cap for v, cap in zip(vmax, MAX_COMMAND_SPEED)))

    def test_stop_cancels_taught_move_and_holds_measured(self):
        bus = TrackingBus(self.HOME)
        arm = self.taught_arm(bus)
        self.teach(arm, bus, "press_button", [self.offset(150, 150, -150, 60, 0, 0)])
        arm.pose_start("press_button")
        start = time.monotonic()
        for index in range(1, 15):
            arm._advance(start + index * 0.05)
        self.assertTrue(arm.state()["pose"]["active"])
        arm.pose_stop()
        self.assertIsNone(arm.hold)
        self.assertEqual(bus.goals, bus.positions)
        self.assertTrue(all(value == 1 for value in bus.torque.values()))


class HttpHostTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        ranges = [(1096, 2617), (883, 3243), (861, 3007), (843, 3130), (0, 4095), (1317, 2762)]
        calibration = {
            name: {"range_min": lo, "range_max": hi}
            for name, (lo, hi) in zip(JOINT_NAMES, ranges, strict=True)
        }
        settings = {
            str(index): {"lower": lo, "upper": hi, "scale": 1}
            for index, (lo, hi) in enumerate(
                ((1116, 2597), (903, 3223), (881, 2987), (863, 3110), (None, None), (1317, 2742)), 1
            )
        }
        pose = {"robot_id": "care_reach_follower",
                "positions_ticks": dict(zip(JOINT_NAMES, (2040, 893, 3064, 2888, 2416, 1470), strict=True))}
        self.settings_path = root / "joint_settings.json"
        self.calibration_path = root / "calibration.json"
        self.pose_path = root / "folded_pose.json"
        self.settings_path.write_text(json.dumps(settings), encoding="utf-8")
        self.calibration_path.write_text(json.dumps(calibration), encoding="utf-8")
        self.pose_path.write_text(json.dumps(pose), encoding="utf-8")
        thread_patch = patch("arm_control.arm.threading.Thread")
        thread_patch.start()
        self.addCleanup(thread_patch.stop)
        arm = Arm("fake", bus_factory=lambda _: FakeBus((2000,) * 6), settings_path=self.settings_path,
                  calibration_path=self.calibration_path, pose_path=self.pose_path,
                  telemetry_path=self.settings_path.parent / "telemetry.jsonl")
        self.addCleanup(arm.close)
        self.handler = handler_for(arm, 8765)

    @staticmethod
    def call(handler_class, headers, client_ip="192.168.1.23", local_ip="192.168.1.42",
             path="/api/state", parse_json=True):
        handler = handler_class.__new__(handler_class)
        handler.path = path
        handler.headers = headers
        handler.client_address = (client_ip, 43210)
        handler.connection = type("FakeConnection", (), {"getsockname": lambda self: (local_ip, 8765)})()
        handler.wfile = io.BytesIO()
        handler._headers_buffer = []
        handler.requestline = f"GET {path} HTTP/1.1"
        handler.request_version = "HTTP/1.1"
        handler.server_version = "Fake/1.0"
        handler.sys_version = "Python/3"
        handler.protocol_version = "HTTP/1.1"
        handler.do_GET()
        raw = handler.wfile.getvalue()
        header_blob, _, payload = raw.partition(b"\r\n\r\n")
        status = int(header_blob.split(b" ", 2)[1])
        return status, json.loads(payload) if parse_json else (header_blob, payload)

    @staticmethod
    def call_post(handler_class, path, data, headers=None):
        body = json.dumps(data).encode()
        handler = handler_class.__new__(handler_class)
        handler.path = path
        handler.headers = {"Host": "pi.local:8765", "Content-Type": "application/json",
                           "Content-Length": str(len(body)), **(headers or {})}
        handler.client_address = ("192.168.1.23", 43210)
        handler.connection = type("FakeConnection", (), {"getsockname": lambda self: ("192.168.1.42", 8765)})()
        handler.rfile = io.BytesIO(body)
        handler.wfile = io.BytesIO()
        handler._headers_buffer = []
        handler.requestline = f"POST {path} HTTP/1.1"
        handler.request_version = "HTTP/1.1"
        handler.server_version = "Fake/1.0"
        handler.sys_version = "Python/3"
        handler.protocol_version = "HTTP/1.1"
        handler.do_POST()
        header_blob, _, payload = handler.wfile.getvalue().partition(b"\r\n\r\n")
        return int(header_blob.split(b" ", 2)[1]), json.loads(payload)

    def test_transcription_switch_proxies_only_voice_state(self):
        page = (Path(__file__).resolve().parents[1] / "static" / "index.html").read_text()
        self.assertIn('id="transcription-toggle"', page)
        self.assertIn("'/api/transcription'", page)

        with patch("server.urlopen", return_value=io.BytesIO(
                b'{"listening":true,"stt_mode":"stream","raw":"private speech"}')) as request:
            status, state = self.call(self.handler, {"Host": "pi.local:8765"},
                                      path="/api/transcription")
        self.assertEqual((status, state), (200, {"enabled": True, "stt_mode": "stream"}))
        request.assert_called_once_with("http://127.0.0.1:8770/state", timeout=3)

        for enabled in (False, True):
            with patch("server.urlopen", return_value=io.BytesIO(b"OK")) as request:
                status, state = self.call_post(self.handler, "/api/transcription", {"enabled": enabled})
            self.assertEqual((status, state), (200, {"enabled": enabled}))
            outbound = request.call_args.args[0]
            self.assertEqual(outbound.full_url, "http://127.0.0.1:8770/listen")
            self.assertEqual(json.loads(outbound.data), {"enabled": enabled})

        with patch("server.urlopen") as request:
            status, state = self.call_post(self.handler, "/api/transcription", {"enabled": 1})
        self.assertEqual((status, state), (400, {"error": "enabled must be boolean"}))
        request.assert_not_called()

    def test_transcription_service_unavailable_is_separate_from_arm_state(self):
        with patch("server.urlopen", side_effect=OSError("voice offline")):
            status, state = self.call(self.handler, {"Host": "pi.local:8765"},
                                      path="/api/transcription")
        self.assertEqual((status, state), (503, {"error": "Transcription service unavailable"}))
        status, state = self.call(self.handler, {"Host": "pi.local:8765"}, path="/api/state")
        self.assertEqual(status, 200)
        self.assertIn("motors", state)

    def test_camera_tab_uses_latest_frame_snapshots_and_stream_stays_available(self):
        page = (Path(__file__).resolve().parents[1] / "static" / "index.html").read_text()
        self.assertIn('id="tab-camera"', page)
        self.assertIn('id="camera-panel"', page)
        self.assertIn("'/camera/snapshot'", page)
        self.assertIn('id="camera2-feed"', page)
        self.assertIn("'/camera2/snapshot'", page)
        self.assertIn('id="range-card"', page)
        self.assertIn("CAMERA_MIN_INTERVAL_MS", page)
        self.assertIn("pollSnapshot", page)
        self.assertIn("cache: 'no-store'", page)
        self.assertNotIn("'/camera/stream'", page)
        self.assertNotIn("'/camera2/stream'", page)

        class FakeCamera(io.BytesIO):
            headers = {"Content-Type": "multipart/x-mixed-replace;boundary=frame"}

            def read1(self, size=-1):
                return self.read(size)

        with patch("server.urlopen", return_value=FakeCamera(b"--frame\r\nJPEG\r\n")) as open_camera:
            status, (headers, body) = self.call(
                self.handler, {"Host": "pi.local:8765"},
                path="/camera/stream", parse_json=False,
            )
        self.assertEqual(status, 200)
        self.assertIn(b"multipart/x-mixed-replace;boundary=frame", headers)
        self.assertIn(b"Connection: close", headers)
        self.assertEqual(body, b"--frame\r\nJPEG\r\n")
        open_camera.assert_called_once_with("http://127.0.0.1:8766/stream", timeout=5)

    def test_second_camera_stream_and_snapshot_use_separate_service(self):
        class FakeCamera(io.BytesIO):
            headers = {"Content-Type": "image/jpeg", "Content-Length": "4"}

            def read1(self, size=-1):
                return self.read(size)

        for suffix in ("stream", "snapshot"):
            with self.subTest(suffix=suffix), patch(
                "server.urlopen", return_value=FakeCamera(b"JPEG")
            ) as open_camera:
                status, (headers, body) = self.call(
                    self.handler, {"Host": "pi.local:8765"},
                    path=f"/camera2/{suffix}", parse_json=False,
                )
                self.assertEqual(status, 200)
                self.assertIn(b"image/jpeg", headers)
                self.assertEqual(body, b"JPEG")
                open_camera.assert_called_once_with(
                    f"http://127.0.0.1:8768/{suffix}", timeout=5
                )

        with patch("server.urlopen", side_effect=OSError("second camera offline")):
            status, payload = self.call(
                self.handler, {"Host": "pi.local:8765"}, path="/camera2/stream"
            )
        self.assertEqual((status, payload), (503, {"error": "Camera unavailable"}))

    def test_snapshot_content_length_is_computed_when_upstream_omits_it(self):
        class HeaderlessCamera(io.BytesIO):
            headers = {"Content-Type": "image/jpeg"}

        with patch("server.urlopen", return_value=HeaderlessCamera(b"jpeg-body")) as open_camera:
            status, (headers, body) = self.call(
                self.handler, {"Host": "pi.local:8765"},
                path="/camera/snapshot", parse_json=False,
            )
        self.assertEqual(status, 200)
        self.assertIn(b"Content-Length: 9", headers)
        self.assertNotIn(b"Connection: close", headers)
        self.assertEqual(body, b"jpeg-body")
        open_camera.assert_called_once_with("http://127.0.0.1:8766/snapshot", timeout=5)

    def test_camera_unavailable_returns_503(self):
        with patch("server.urlopen", side_effect=OSError("camera offline")):
            status, payload = self.call(self.handler, {"Host": "pi.local:8765"}, path="/camera/stream")
        self.assertEqual(status, 503)
        self.assertEqual(payload, {"error": "Camera unavailable"})

    def test_ultrasonic_readout_is_independent_and_sanitized(self):
        page = (Path(__file__).resolve().parents[1] / "static" / "index.html").read_text()
        self.assertIn('id="range-card"', page)
        self.assertIn("'/api/ultrasonic'", page)
        self.assertIn("setInterval(refreshRange, 200)", page)

        sample = {"status": "ok", "distance_cm": 42.25, "sample_age_ms": 80,
                  "updated_at": "2026-10-24T22:00:00Z", "private": "do not proxy"}
        with patch("server.urlopen", return_value=io.BytesIO(json.dumps(sample).encode())) as request:
            status, state = self.call(self.handler, {"Host": "pi.local:8765"},
                                      path="/api/ultrasonic")
        self.assertEqual((status, state), (200, {"status": "ok", "distance_cm": 42.25,
                                                 "sample_age_ms": 80,
                                                 "updated_at": "2026-10-24T22:00:00Z"}))
        request.assert_called_once_with("http://127.0.0.1:8767/state", timeout=1)

        with patch("server.urlopen", side_effect=OSError("sensor offline")):
            status, state = self.call(self.handler, {"Host": "pi.local:8765"},
                                      path="/api/ultrasonic")
        self.assertEqual((status, state), (503, {"error": "Ultrasonic service unavailable"}))

        with patch("server.urlopen", return_value=io.BytesIO(
                b'{"status":"ok","distance_cm":NaN,"sample_age_ms":80}')):
            status, state = self.call(self.handler, {"Host": "pi.local:8765"},
                                      path="/api/ultrasonic")
        self.assertEqual(status, 503)

    def test_accepts_pi_local_pi_and_loopback_hosts(self):
        for host in ("pi.local:8765", "pi:8765", "localhost:8765", "127.0.0.1:8765"):
            status, payload = self.call(self.handler, {"Host": host}, client_ip="127.0.0.1")
            self.assertEqual(status, 200, host)
            self.assertIn("motors", payload)

    def test_accepts_the_address_that_accepted_the_connection(self):
        status, payload = self.call(self.handler, {"Host": "192.168.1.42:8765"}, client_ip="192.168.1.23")
        self.assertEqual(status, 200)
        self.assertIn("motors", payload)

    def test_rejects_foreign_hostnames_and_wrong_ports(self):
        cases = (
            ({"Host": "evil.example:8765"}, "192.168.1.23"),
            ({"Host": "192.168.1.23:8765"}, "127.0.0.1"),
            ({"Host": "pi.local:9999"}, "192.168.1.23"),
            ({}, "192.168.1.23"),
        )
        for headers, client_ip in cases:
            status, payload = self.call(self.handler, headers, client_ip=client_ip)
            self.assertEqual(status, 403, headers)
            self.assertEqual(payload, {"error": "Invalid host"})

    def test_origin_must_match_the_accepted_host_exactly(self):
        accepted = (
            ("pi.local:8765", "http://pi.local:8765"),
            ("192.168.1.42:8765", "http://192.168.1.42:8765"),
        )
        for host, origin in accepted:
            status, payload = self.call(self.handler, {"Host": host, "Origin": origin}, client_ip="192.168.1.23")
            self.assertEqual(status, 200, (host, origin))
        rejected = (
            ("pi.local:8765", "http://evil.example:8765"),
            ("pi.local:8765", "https://pi.local:8765"),
            ("pi.local:8765", "http://127.0.0.1:8765"),
            ("pi.local:8765", "http://pi.local:9999"),
        )
        for host, origin in rejected:
            status, payload = self.call(self.handler, {"Host": host, "Origin": origin}, client_ip="192.168.1.23")
            self.assertEqual(status, 403, (host, origin))
            self.assertEqual(payload, {"error": "Invalid origin"})

    @staticmethod
    def post(handler_class, path, body):
        handler = handler_class.__new__(handler_class)
        handler.path = path
        payload = json.dumps(body).encode()
        handler.headers = {
            "Host": "127.0.0.1:8765",
            "Content-Type": "application/json",
            "Content-Length": str(len(payload)),
        }
        handler.rfile = io.BytesIO(payload)
        handler.client_address = ("127.0.0.1", 43210)
        handler.connection = type("FakeConnection", (), {"getsockname": lambda self: ("127.0.0.1", 8765)})()
        handler.wfile = io.BytesIO()
        handler._headers_buffer = []
        handler.requestline = f"POST {path} HTTP/1.1"
        handler.request_version = "HTTP/1.1"
        handler.server_version = "Fake/1.0"
        handler.sys_version = "Python/3"
        handler.protocol_version = "HTTP/1.1"
        handler.do_POST()
        raw = handler.wfile.getvalue()
        header_blob, _, payload = raw.partition(b"\r\n\r\n")
        status = int(header_blob.split(b" ", 2)[1])
        return status, json.loads(payload)

    def test_pose_endpoints_run_and_state_shows_active_pose(self):
        status, payload = self.post(self.handler, "/api/pose/start", {"pose": "home"})
        self.assertEqual(status, 200)
        self.assertEqual(payload["pose"], "home")
        self.assertIn("token", payload)
        status, state = self.call(self.handler, {"Host": "127.0.0.1:8765"}, client_ip="127.0.0.1")
        self.assertEqual(status, 200)
        self.assertTrue(state["pose"]["active"])
        self.assertEqual(state["pose"]["name"], "home")
        self.assertEqual(state["pose"]["target"], dict(zip(JOINT_NAMES, home_target(), strict=True)))
        status, payload = self.post(self.handler, "/api/pose/stop", {})
        self.assertEqual(status, 200)
        _, state = self.call(self.handler, {"Host": "127.0.0.1:8765"}, client_ip="127.0.0.1")
        self.assertFalse(state["pose"]["active"])

    def test_pose_start_validates_name(self):
        status, payload = self.post(self.handler, "/api/pose/start", {"pose": "coiled"})
        self.assertEqual(status, 400)
        self.assertIn("pose must be", payload["error"])
        status, payload = self.post(self.handler, "/api/pose/start", {"pose": "fold"})
        self.assertEqual(status, 400)
        self.assertIn("folded pose is unavailable", payload["error"])

    def test_extend_endpoint_uses_saved_upright_target(self):
        status, payload = self.post(self.handler, "/api/pose/start", {"pose": "extend"})
        self.assertEqual(status, 200)
        self.assertEqual(payload["pose"], "extend")
        _, state = self.call(self.handler, {"Host": "127.0.0.1:8765"}, client_ip="127.0.0.1")
        self.assertEqual(state["pose"]["target"],
                         dict(zip(JOINT_NAMES, (2000, 1903, 1126, 1988, 387, 1993), strict=True)))

    def test_dance_endpoint_starts_and_cancels(self):
        status, payload = self.post(self.handler, "/api/pose/start", {"pose": "dance"})
        self.assertEqual(status, 200)
        self.assertEqual(payload["pose"], "dance")
        _, state = self.call(self.handler, {"Host": "127.0.0.1:8765"}, client_ip="127.0.0.1")
        self.assertTrue(state["pose"]["active"])
        self.assertEqual(state["pose"]["name"], "dance")
        status, _ = self.post(self.handler, "/api/pose/stop", {})
        self.assertEqual(status, 200)

    def test_limit_endpoint_removed(self):
        status, payload = self.post(
            self.handler, "/api/limit", {"motor": 1, "bound": "lower", "clear": False}
        )
        self.assertEqual(status, 404)
        self.assertEqual(payload, {"error": "Not found"})


class HttpKeepAliveTests(unittest.TestCase):
    """Real-socket checks: HTTP/1.1 keep-alive reuses one connection for
    finite responses, and stream framing stays accurate and close-delimited."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        ranges = [(1096, 2617), (883, 3243), (861, 3007), (843, 3130), (0, 4095), (1317, 2762)]
        calibration = {
            name: {"range_min": lo, "range_max": hi}
            for name, (lo, hi) in zip(JOINT_NAMES, ranges, strict=True)
        }
        settings = {
            str(index): {"lower": lo, "upper": hi, "scale": 1}
            for index, (lo, hi) in enumerate(
                ((1116, 2597), (903, 3223), (881, 2987), (863, 3110), (None, None), (1317, 2742)), 1
            )
        }
        pose = {"robot_id": "care_reach_follower",
                "positions_ticks": dict(zip(JOINT_NAMES, (2040, 893, 3064, 2888, 2416, 1470), strict=True))}
        settings_path = root / "joint_settings.json"
        calibration_path = root / "calibration.json"
        pose_path = root / "folded_pose.json"
        settings_path.write_text(json.dumps(settings), encoding="utf-8")
        calibration_path.write_text(json.dumps(calibration), encoding="utf-8")
        pose_path.write_text(json.dumps(pose), encoding="utf-8")
        self.arm = Arm("fake", bus_factory=lambda _: FakeBus((2000,) * 6),
                       settings_path=settings_path, calibration_path=calibration_path,
                       pose_path=pose_path, telemetry_path=root / "telemetry.jsonl")
        self.addCleanup(self.arm.close)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(self.arm, 0))
        self.server.RequestHandlerClass = handler_for(self.arm, self.server.server_address[1])
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.shutdown)
        self.addCleanup(self.server.server_close)

    def connect(self):
        connection = http.client.HTTPConnection(
            "127.0.0.1", self.server.server_address[1], timeout=5
        )
        self.addCleanup(connection.close)
        return connection

    def test_keepalive_serves_consecutive_api_and_snapshot_requests(self):
        connection = self.connect()
        connection.request("GET", "/api/state")
        response = connection.getresponse()
        state = json.loads(response.read())
        self.assertEqual(response.status, 200)
        self.assertIn("motors", state)
        self.assertIsNone(response.getheader("Connection"))
        shared_socket = connection.sock

        payload = b"fresh-jpeg-bytes"

        class SnapshotCamera(io.BytesIO):
            headers = {"Content-Type": "image/jpeg"}  # deliberately no Content-Length

        with patch("server.urlopen", return_value=SnapshotCamera(payload)):
            connection.request("GET", "/camera/snapshot")
            response = connection.getresponse()
            body = response.read()
        self.assertEqual(response.status, 200)
        self.assertEqual(body, payload)
        self.assertEqual(response.getheader("Content-Length"), str(len(payload)))
        self.assertIs(connection.sock, shared_socket)

        connection.request("GET", "/api/state")
        response = connection.getresponse()
        response.read()
        self.assertEqual(response.status, 200)
        self.assertIs(connection.sock, shared_socket)

    def test_stream_is_close_delimited_and_relays_prompt_chunks(self):
        class FramedCamera:
            """Only offers read1: read(n) head-of-line buffering must not be used."""

            headers = {"Content-Type": "multipart/x-mixed-replace;boundary=frame"}

            def __init__(self, frames):
                self.frames = iter(frames)
                self.sizes = []

            def read1(self, size=-1):
                self.sizes.append(size)
                return next(self.frames, b"")

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return None

        frames = [b"--frame\r\n", b"AA", b"\r\n--frame\r\n", b"BBB", b"\r\n"]
        upstream = FramedCamera(frames)
        connection = self.connect()
        with patch("server.urlopen", return_value=upstream):
            connection.request("GET", "/camera/stream")
            response = connection.getresponse()
            body = response.read()
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Connection"), "close")
        self.assertTrue(response.will_close)
        self.assertEqual(body, b"".join(frames))
        self.assertEqual(upstream.sizes, [16 * 1024] * (len(frames) + 1))


class StaticUiTests(unittest.TestCase):
    PAGE = Path(__file__).resolve().parent.parent / "static" / "index.html"

    def setUp(self):
        self.page = self.PAGE.read_text(encoding="utf-8")

    def test_home_extend_and_dance_buttons_replace_fold(self):
        self.assertIn('id="pose-home"', self.page)
        self.assertIn('id="pose-extend"', self.page)
        self.assertIn('id="pose-dance"', self.page)
        self.assertIn(">Home / Retract</button>", self.page)
        self.assertIn(">Extend</button>", self.page)
        self.assertIn(">Dance</button>", self.page)
        self.assertNotIn('id="pose-fold"', self.page)
        self.assertIn("/api/pose/start", self.page)
        self.assertIn("/api/pose/stop", self.page)
        self.assertIn("limit-range", self.page)
        self.assertNotIn("/api/limit", self.page)
        self.assertNotIn("data-bound", self.page)
        self.assertNotIn("Save ${bound}", self.page)

    @unittest.skipUnless(shutil.which("node"), "node is not available")
    def test_inline_javascript_parses(self):
        match = re.search(r"<script>(.*)</script>", self.page, re.S)
        self.assertIsNotNone(match)
        result = subprocess.run(
            ["node", "--check", "-"], input=match.group(1), capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
