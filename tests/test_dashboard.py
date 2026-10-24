"""Fake-bus dashboard checks. These tests never open a serial device."""

import io
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from motion_core import JOINT_NAMES
from server import Arm, STOP_SETTLE_LEAD, handler_for


class FakeBus:
    def __init__(self, positions, torque=1):
        self.positions = dict(zip(JOINT_NAMES, positions, strict=True))
        self.goals = self.positions.copy()
        self.torque = {name: torque for name in JOINT_NAMES}
        self.loads = {name: 0 for name in JOINT_NAMES}
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
            "Status": {name: 0 for name in JOINT_NAMES},
            "Present_Voltage": {name: 53 for name in JOINT_NAMES},
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
        assert register == "Torque_Enable"
        self.torque.update(values)
        self.writes.append((register, values.copy()))


class DashboardTests(unittest.TestCase):
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
        thread_patch = patch("server.threading.Thread")
        thread_patch.start()
        self.addCleanup(thread_patch.stop)
        arm = Arm("fake", bus_factory=lambda _: bus, settings_path=self.settings_path,
                  calibration_path=self.calibration_path, pose_path=self.pose_path)
        self.addCleanup(arm.close)
        return arm

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

    def test_saved_scale_and_limits_are_persistent(self):
        bus = FakeBus((2000,) * 6)
        arm = self.arm(bus)
        arm.scale(4, 8)
        arm.limit(4, "lower", False)
        settings = json.loads(self.settings_path.read_text(encoding="utf-8"))
        self.assertEqual(settings["4"]["scale"], 8)
        self.assertEqual(settings["4"]["lower"], 2000)
        self.assertEqual(arm.state()["motors"][3]["effective_lower"], 2000)

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
        thread_patch = patch("server.threading.Thread")
        thread_patch.start()
        self.addCleanup(thread_patch.stop)
        arm = Arm("fake", bus_factory=lambda _: FakeBus((2000,) * 6), settings_path=self.settings_path,
                  calibration_path=self.calibration_path, pose_path=self.pose_path)
        self.addCleanup(arm.close)
        self.handler = handler_for(arm, 8765)

    @staticmethod
    def call(handler_class, headers, client_ip="192.168.1.23", local_ip="192.168.1.42"):
        handler = handler_class.__new__(handler_class)
        handler.path = "/api/state"
        handler.headers = headers
        handler.client_address = (client_ip, 43210)
        handler.connection = type("FakeConnection", (), {"getsockname": lambda self: (local_ip, 8765)})()
        handler.wfile = io.BytesIO()
        handler._headers_buffer = []
        handler.requestline = "GET /api/state HTTP/1.1"
        handler.request_version = "HTTP/1.1"
        handler.server_version = "Fake/1.0"
        handler.sys_version = "Python/3"
        handler.protocol_version = "HTTP/1.1"
        handler.do_GET()
        raw = handler.wfile.getvalue()
        header_blob, _, payload = raw.partition(b"\r\n\r\n")
        status = int(header_blob.split(b" ", 2)[1])
        return status, json.loads(payload)

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


if __name__ == "__main__":
    unittest.main()
