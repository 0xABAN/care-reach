"""Fake-bus checks only; these tests never open the serial device."""

import tempfile
import unittest
from pathlib import Path

from arm_control.bus_lock import BusLock
from arm_control.pose_runner import GRIP_HOLD_LEAD, MeasuredPoseRunner, PoseMoveError
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


class ObjectBus(FakeBus):
    """An object between the jaws stops the gripper at ``object_tick``; None means no object."""

    def __init__(self, port, object_tick):
        super().__init__(port)
        self.object_tick = object_tick
        self.history = []

    def sync_write(self, register, goals, normalize=True):
        self.history.append(self.positions["gripper"])
        super().sync_write(register, goals, normalize)
        if self.object_tick is not None:
            self.positions["gripper"] = max(self.object_tick, self.positions["gripper"])

    def positions_before(self, write):
        return self.history[self.writes.index(write)]


def runner(bus):
    return MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, clock=FakeClock())


class RunnerTests(unittest.TestCase):
    def test_deadband_trim_reaches_precise_target_and_holds_it(self):
        class DeadbandBus(FakeBus):
            def sync_write(self, register, goals, normalize=True):
                assert self.connected and register == "Goal_Position" and not normalize
                self.writes.append(goals.copy())
                for name in JOINT_NAMES:
                    distance = goals[name] - self.positions[name]
                    if abs(distance) >= 35:
                        self.positions[name] += max(-8, min(8, distance))

        bus = DeadbandBus("unused")
        target = (2000, 2120, 1880, 2070, 2000, 2000)
        result = MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus,
                                    clock=FakeClock()).move(
            target, tolerance=15, stall_s=6, trim_deadband=True,
            hold_target_on_success=True)
        self.assertTrue(all(abs(result["positions"][name] - target[i]) <= 15
                            for i, name in enumerate(JOINT_NAMES)))
        self.assertEqual(bus.writes[-1], dict(zip(JOINT_NAMES, target)))
        self.assertEqual(bus.disconnects, [False])

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

    def test_closing_gripper_stops_on_an_object_and_keeps_squeezing(self):
        bus = ObjectBus("unused", object_tick=1700)
        bus.positions["gripper"] = 1950
        result = runner(bus).close_gripper(BOUNDS.lower[5])

        self.assertTrue(result["contact"])
        self.assertEqual(result["gripper_hold"], 1700 - GRIP_HOLD_LEAD)
        self.assertEqual(bus.writes[-1]["gripper"], 1700 - GRIP_HOLD_LEAD)
        self.assertTrue(all(write["shoulder_lift"] == 2000 for write in bus.writes))
        self.assertTrue(all(abs(write["gripper"] - bus.positions_before(write)) <= 30
                            for write in bus.writes[:-1]))
        self.assertEqual(bus.disconnects, [False])
        self.assertTrue(all(bus.torque.values()))

    def test_closing_gripper_on_nothing_reports_a_miss(self):
        bus = ObjectBus("unused", object_tick=None)
        bus.positions["gripper"] = 1600
        result = runner(bus).close_gripper(BOUNDS.lower[5])

        self.assertFalse(result["contact"])
        self.assertEqual(result["gripper_hold"], BOUNDS.lower[5])
        self.assertEqual(bus.disconnects, [False])

    def test_closed_tick_outside_bounds_is_rejected_before_connect(self):
        bus = ObjectBus("unused", object_tick=None)
        with self.assertRaises(UnsafePoseError):
            runner(bus).close_gripper(BOUNDS.lower[5] - 1)
        self.assertFalse(bus.connected)

    def test_holding_an_object_moves_the_arm_and_keeps_the_squeeze(self):
        # The jaws stop on the object 30 ticks short of their squeeze goal,
        # under high load: neither may count as a stall or an overload.
        bus = ObjectBus("unused", object_tick=1700)
        bus.positions["gripper"] = 1700
        bus.load["gripper"] = 950
        squeeze = 1700 - GRIP_HOLD_LEAD
        target = (2000, 2100, 1900, 2000, 2000, squeeze)
        result = runner(bus).move(target, tolerance=15, holding_object=True, hold_target_on_success=True)

        self.assertLessEqual(abs(result["positions"]["shoulder_lift"] - 2100), 15)
        self.assertEqual(result["positions"]["gripper"], 1700)
        self.assertTrue(all(write["gripper"] == squeeze for write in bus.writes))
        self.assertEqual(bus.disconnects, [False])

    def test_larger_encoder_excursion_stops_and_holds_inward(self):
        bus = FakeBus("unused")
        bus.positions["elbow_flex"] = BOUNDS.upper[2] + 6
        target = (2000, 2000, BOUNDS.upper[2], 2000, 2000, 2000)
        with self.assertRaisesRegex(PoseMoveError, "elbow_flex: measured position"):
            MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, clock=FakeClock()).move(target)
        self.assertEqual(bus.writes[-1]["elbow_flex"], BOUNDS.upper[2])
        self.assertEqual(bus.disconnects, [False])

    def test_result_carries_measured_bus_and_compute_windows(self):
        bus = FakeBus("unused")
        result = MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, clock=FakeClock()).move(
            (2000, 2120, 1880, 2070, 2000, 2000)
        )
        timing = result["timing"]
        for field in ("bus_read_ms", "bus_write_ms", "compute_ms", "cycle_ms"):
            window = timing[field]
            self.assertIsInstance(window, dict, field)
            self.assertGreater(window["count"], 0, field)
            self.assertLessEqual(window["mean_ms"], window["max_ms"] + 1e-9, field)
        self.assertLessEqual(timing["compute_ms"]["mean_ms"], timing["cycle_ms"]["mean_ms"] + 1e-9)
        self.assertEqual(bus.disconnects, [False])

    def test_explicit_lock_is_released_after_a_completed_move(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "arm-bus.lock"
            lock = BusLock(path)
            bus = FakeBus("unused")
            MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, bus_lock=lock,
                               clock=FakeClock()).move((2000,) * 6)
            self.assertFalse(lock.locked)
            next_owner = BusLock(path)
            next_owner.acquire("next-owner")
            self.assertTrue(next_owner.locked)
            next_owner.release()

    def test_disconnect_failure_still_releases_lock(self):
        class DisconnectingBus(FakeBus):
            def disconnect(self, disable_torque=True):
                super().disconnect(disable_torque=disable_torque)
                raise OSError("disconnect failed")

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "arm-bus.lock"
            lock = BusLock(path)
            bus = DisconnectingBus("unused")
            with self.assertRaisesRegex(OSError, "disconnect failed"):
                MeasuredPoseRunner("unused", BOUNDS, bus_factory=lambda _: bus, bus_lock=lock,
                                   clock=FakeClock()).move((2000,) * 6)
            self.assertEqual(bus.disconnects, [False])
            self.assertFalse(lock.locked)


if __name__ == "__main__":
    unittest.main()
