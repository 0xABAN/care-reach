"""Fake-bus checks only; these tests never open the serial device."""

import unittest

from measured_pose_runner import MeasuredPoseRunner, PoseMoveError
from motion_core import JOINT_NAMES, SafetyBounds, UnsafePoseError


BOUNDS = SafetyBounds((1000, 800, 800, 800, 0, 1340), (2600, 3200, 3100, 3110, 3300, 2742))


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeBus:
    def __init__(self, _port):
        self.connected = False
        self.disconnects = []
        self.writes = []
        self.positions = {name: 2000 for name in JOINT_NAMES}
        self.torque = {name: 1 for name in JOINT_NAMES}
        self.status = {name: 0 for name in JOINT_NAMES}
        self.voltage = {name: 53 for name in JOINT_NAMES}
        self.load = {name: 50 for name in JOINT_NAMES}
        self.mode = {name: 0 for name in JOINT_NAMES}
        self.reads = 0

    def connect(self, handshake=True):
        self.connected = True
        assert handshake

    def disconnect(self, disable_torque=True):
        self.disconnects.append(disable_torque)
        self.connected = False

    def sync_read(self, register, normalize=True, num_retry=0):
        assert self.connected and not normalize and num_retry == 2
        self.reads += 1
        values = {
            "Present_Position": self.positions,
            "Torque_Enable": self.torque,
            "Status": self.status,
            "Present_Voltage": self.voltage,
            "Present_Load": self.load,
            "Operating_Mode": self.mode,
            "Min_Position_Limit": {name: 0 for name in JOINT_NAMES},
            "Max_Position_Limit": {name: 4095 for name in JOINT_NAMES},
        }
        return values[register].copy()

    def sync_write(self, register, goals, normalize=True):
        assert self.connected and register == "Goal_Position" and not normalize
        assert set(goals) == set(JOINT_NAMES)
        self.writes.append(goals.copy())
        for name in JOINT_NAMES:
            distance = goals[name] - self.positions[name]
            self.positions[name] += max(-8, min(8, distance))


class RunnerTests(unittest.TestCase):
    def test_slow_encoder_reaches_fold_with_measured_lead(self):
        bus = FakeBus("unused")
        bus.positions["shoulder_lift"] = 1132
        bus.positions["elbow_flex"] = 2864
        target = (2040, 893, 3064, 2888, 2416, 1470)
        result = MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, clock=FakeClock()).move(target)
        self.assertTrue(all(abs(result["positions"][name] - target[i]) <= 35
                            for i, name in enumerate(JOINT_NAMES)))
        self.assertGreater(result["writes"], 20)
        for goals in bus.writes[:-1]:  # the final measured hold is an intentional exception
            self.assertTrue(all(BOUNDS.lower[i] <= goals[name] <= BOUNDS.upper[i]
                                for i, name in enumerate(JOINT_NAMES)))
        self.assertEqual(bus.disconnects, [False])
        self.assertTrue(all(bus.torque.values()))

    def test_fault_stops_and_holds_without_disabling_torque(self):
        class FaultBus(FakeBus):
            def sync_write(self, register, goals, normalize=True):
                super().sync_write(register, goals, normalize)
                if len(self.writes) == 4:
                    self.status["elbow_flex"] = 32

        bus = FaultBus("unused")
        with self.assertRaisesRegex(PoseMoveError, "elbow_flex: status 32"):
            MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, clock=FakeClock()).move(
                (2200, 1800, 2200, 2100, 2100, 1900)
            )
        self.assertEqual(bus.disconnects, [False])
        self.assertEqual(bus.writes[-1], bus.positions)
        self.assertTrue(all(bus.torque.values()))

    def test_outside_bounds_rejected_before_connect(self):
        bus = FakeBus("unused")
        with self.assertRaisesRegex(UnsafePoseError, "motor 3"):
            MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, clock=FakeClock()).move(
                (2000, 2000, 3200, 2000, 2000, 2000)
            )
        self.assertFalse(bus.connected)
        self.assertEqual(bus.writes, [])

    def test_one_tick_encoder_overshoot_recovers_with_bounded_hold(self):
        class OvershootBus(FakeBus):
            def sync_write(self, register, goals, normalize=True):
                super().sync_write(register, goals, normalize)
                if len(self.writes) == 1:
                    self.positions["elbow_flex"] = BOUNDS.upper[2] + 1

        bus = OvershootBus("unused")
        bus.positions["elbow_flex"] = BOUNDS.upper[2] - 10
        target = (2000, 2000, BOUNDS.upper[2], 2000, 2000, 2000)
        result = MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, clock=FakeClock()).move(
            target, tolerance=1
        )
        self.assertEqual(result["positions"]["elbow_flex"], BOUNDS.upper[2] + 1)
        self.assertTrue(all(write["elbow_flex"] <= BOUNDS.upper[2] for write in bus.writes))
        self.assertEqual(bus.writes[-1]["elbow_flex"], BOUNDS.upper[2])
        self.assertEqual(bus.disconnects, [False])

    def test_larger_encoder_excursion_stops_and_holds_inward(self):
        bus = FakeBus("unused")
        bus.positions["elbow_flex"] = BOUNDS.upper[2] + 6
        target = (2000, 2000, BOUNDS.upper[2], 2000, 2000, 2000)
        with self.assertRaisesRegex(PoseMoveError, "elbow_flex: measured position"):
            MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, clock=FakeClock()).move(target)
        self.assertEqual(bus.writes[-1]["elbow_flex"], BOUNDS.upper[2])
        self.assertEqual(bus.disconnects, [False])


if __name__ == "__main__":
    unittest.main()
