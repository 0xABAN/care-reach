"""Fake-bus checks only; this test suite never opens a serial device."""

import unittest

from arm_runtime import ArmRuntime, ArmSafetyError
from motion_core import JOINT_NAMES, SafetyBounds, UnsafePoseError


BOUNDS = SafetyBounds((1500, 1500, 1500, 1500, 1500, 1340), (2500, 2500, 2500, 2500, 2500, 2742))


class FakeBus:
    def __init__(self, _port):
        self.connected = False
        self.disconnects = []
        self.writes = []
        self.positions = {name: 2000 for name in JOINT_NAMES}
        self.status = {name: 0 for name in JOINT_NAMES}
        self.torque = {name: 1 for name in JOINT_NAMES}
        self.voltage = {name: 53 for name in JOINT_NAMES}
        self.modes = {name: 0 for name in JOINT_NAMES}
        self.loads = {name: 50 for name in JOINT_NAMES}
        self.low_samples = 0

    def connect(self, handshake=True):
        assert handshake
        self.connected = True

    def disconnect(self, disable_torque=True):
        self.disconnects.append(disable_torque)
        self.connected = False

    def sync_read(self, register, normalize=True, num_retry=0):
        assert self.connected and not normalize and num_retry == 2
        if register == "Present_Voltage" and self.low_samples:
            self.low_samples -= 1
            return {name: 41 for name in JOINT_NAMES}
        values = {
            "Present_Position": self.positions,
            "Status": self.status,
            "Present_Voltage": self.voltage,
            "Torque_Enable": self.torque,
            "Present_Load": self.loads,
            "Operating_Mode": self.modes,
            "Min_Position_Limit": {name: 0 for name in JOINT_NAMES},
            "Max_Position_Limit": {name: 4095 for name in JOINT_NAMES},
        }
        return values[register].copy()

    def sync_write(self, register, goals, normalize=True):
        assert register == "Goal_Position" and not normalize
        assert set(goals) == set(JOINT_NAMES)
        for index, name in enumerate(JOINT_NAMES):
            assert BOUNDS.lower[index] <= goals[name] <= BOUNDS.upper[index]
        self.writes.append(goals.copy())
        # An ideal encoder for deterministic control-flow tests.
        self.positions = goals.copy()


class RuntimeTests(unittest.TestCase):
    def test_explicit_two_x_command_speed_is_bounded(self):
        bus = FakeBus("unused")
        target = (2050,) * 6
        result = ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run(
            [target] * 2, fps=20, command_speed_scale=2.0,
            max_command_acceleration=None,
        )
        first = bus.writes[0]
        self.assertEqual(first["shoulder_pan"], 2009)
        self.assertEqual(first["wrist_roll"], 2014)
        self.assertLessEqual(abs(result["final_positions"]["shoulder_pan"] - 2050), 15)
        self.assertEqual(bus.disconnects, [False])

    def test_non_home_finish_can_use_explicit_tolerance(self):
        class DeadbandBus(FakeBus):
            def sync_write(self, register, goals, normalize=True):
                before = self.positions.copy()
                super().sync_write(register, goals, normalize)
                self.positions = {
                    name: before[name] + min(5, max(0, goals[name] - before[name] - 25))
                    for name in JOINT_NAMES
                }

        bus = DeadbandBus("unused")
        target = (2100,) * 6
        result = ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run(
            [target] * 50, fps=20, finish_tolerance=35,
            max_command_acceleration=None,
        )
        self.assertTrue(all(15 < target[i] - result["final_positions"][name] <= 35
                            for i, name in enumerate(JOINT_NAMES)))
        self.assertEqual(bus.disconnects, [False])

    def test_coordinated_writes_and_hold_never_disable_torque(self):
        bus = FakeBus("unused")
        with ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus) as runtime:
            result = runtime.run([(2010, 2020, 2030, 2040, 2050, 2060)] * 2, fps=50)
        self.assertGreaterEqual(result["frames_commanded"], 2)
        self.assertEqual(result["samples"][0]["state"]["gripper"]["position"], 2000)
        self.assertEqual(len(bus.writes), result["frames_commanded"] + 1)  # final measured hold
        self.assertEqual(bus.disconnects, [False])

    def test_lagging_encoder_and_reversal_keep_command_steps_bounded(self):
        class LaggingBus(FakeBus):
            def sync_write(self, register, goals, normalize=True):
                before = self.positions.copy()
                super().sync_write(register, goals, normalize)
                self.positions = {
                    name: before[name] + max(-2, min(2, goals[name] - before[name]))
                    for name in JOINT_NAMES
                }

        bus = LaggingBus("unused")
        outward = (2300, 2300, 2300, 2300, 2300, 2300)
        inward = (1700, 1700, 1700, 1700, 1700, 1700)
        frames = [outward] * 24 + [inward] * 30 + [(2000,) * 6] * 4
        result = ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run(frames, fps=20)
        commands = bus.writes[:-1]  # final hold is intentionally immediate
        for index, name in enumerate(JOINT_NAMES):
            deltas = [b[name] - a[name] for a, b in zip(commands, commands[1:])]
            max_step = 7 if index == 4 else 5
            self.assertTrue(all(abs(step) <= max_step for step in deltas), name)
            # A measured-position lead clamp may brake faster than the nominal
            # acceleration limit, but a reversal must still avoid an instant
            # full-speed sign flip.
            max_step_change = 4 if index == 4 else 2
            self.assertTrue(
                all(abs(after - before) <= max_step_change for before, after in zip(deltas, deltas[1:])),
                f"{name}: {deltas}",
            )
        self.assertEqual(bus.disconnects, [False])
        self.assertGreaterEqual(result["frames_commanded"], len(frames))

    def test_home_settle_overcomes_small_goal_deadband(self):
        class DeadbandBus(FakeBus):
            def sync_write(self, register, goals, normalize=True):
                before = self.positions.copy()
                super().sync_write(register, goals, normalize)
                self.positions = {
                    name: before[name] + max(-2, min(2, goals[name] - before[name]))
                    if abs(goals[name] - before[name]) > 30 else before[name]
                    for name in JOINT_NAMES
                }

        bus = DeadbandBus("unused")
        bus.positions["shoulder_lift"] = 2041
        result = ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run([(2000,) * 6], fps=20)
        self.assertLessEqual(abs(result["final_positions"]["shoulder_lift"] - 2000), 15)
        self.assertTrue(any(goal["shoulder_lift"] < 2000 for goal in bus.writes))
        self.assertEqual(bus.disconnects, [False])

    def test_completed_home_trim_stops_while_another_joint_settles(self):
        class TwoJointBus(FakeBus):
            def __init__(self, port):
                super().__init__(port)
                self.positions["shoulder_lift"] = 2048
                self.shoulder_history = []
                self.after_shoulder_home = 0

            def sync_write(self, register, goals, normalize=True):
                before = self.positions.copy()
                super().sync_write(register, goals, normalize)
                count = len(self.writes)
                self.positions = before.copy()
                for name in JOINT_NAMES:
                    if name == "elbow_flex":
                        continue
                    deadband = 30 if name == "shoulder_lift" else 0
                    if abs(goals[name] - before[name]) > deadband:
                        self.positions[name] += max(-2, min(2, goals[name] - before[name]))
                if abs(self.positions["shoulder_lift"] - 2000) <= 15:
                    self.after_shoulder_home += 1
                    self.positions["elbow_flex"] = 2018 if self.after_shoulder_home <= 8 else 2014
                self.shoulder_history.append(self.positions["shoulder_lift"])

        bus = TwoJointBus("unused")
        result = ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run([(2000,) * 6], fps=20)
        first_home = next(
            index for index, position in enumerate(bus.shoulder_history)
            if abs(position - 2000) <= 15
        )
        self.assertGreater(len(bus.shoulder_history) - first_home, 5)
        self.assertTrue(all(1985 <= position <= 2015 for position in bus.shoulder_history[first_home:]))
        self.assertLessEqual(abs(result["final_positions"]["elbow_flex"] - 2000), 15)

    def test_true_stall_beyond_home_trim_window_still_stops(self):
        class FrozenBus(FakeBus):
            def sync_write(self, register, goals, normalize=True):
                before = self.positions.copy()
                super().sync_write(register, goals, normalize)
                self.positions = before

        bus = FrozenBus("unused")
        bus.positions["shoulder_lift"] = 2100
        with self.assertRaisesRegex(ArmSafetyError, "stalled"):
            ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run([(2000,) * 6], fps=20)
        self.assertEqual(bus.disconnects, [False])

    def test_reject_program_before_connect(self):
        bus = FakeBus("unused")
        runtime = ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus)
        with self.assertRaises(UnsafePoseError):
            runtime.run([(2000, 2000, 2000, 2000, 3301, 2000)])
        self.assertFalse(bus.connected)
        self.assertEqual(bus.writes, [])

    def test_fault_holds_position_and_disconnects_with_torque_on(self):
        bus = FakeBus("unused")
        bus.status["elbow_flex"] = 2
        with self.assertRaisesRegex(ArmSafetyError, "servo status"):
            ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run([(2000,) * 6])
        self.assertEqual(bus.writes, [bus.positions])
        self.assertEqual(bus.disconnects, [False])

    def test_isolated_voltage_dip_is_rechecked(self):
        bus = FakeBus("unused")
        bus.low_samples = 1
        result = ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run([(2000,) * 6])
        self.assertEqual(result["frames_commanded"], 1)
        self.assertEqual(bus.disconnects, [False])

    def test_negative_load_is_not_mistaken_for_high_load(self):
        bus = FakeBus("unused")
        bus.loads["wrist_flex"] = -20
        result = ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run([(2000,) * 6] * 4, fps=50)
        self.assertGreaterEqual(result["frames_commanded"], 4)
        self.assertEqual(bus.disconnects, [False])

    def test_sustained_low_voltage_stops_before_playback(self):
        bus = FakeBus("unused")
        bus.low_samples = 2
        with self.assertRaisesRegex(ArmSafetyError, "voltage stayed low"):
            ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run([(2000,) * 6])
        self.assertEqual(len(bus.writes), 1)  # hold only
        self.assertEqual(bus.disconnects, [False])

    def test_wrong_mode_stops_without_changing_mode(self):
        bus = FakeBus("unused")
        bus.modes["gripper"] = 1
        with self.assertRaisesRegex(ArmSafetyError, "operating mode"):
            ArmRuntime("unused", BOUNDS, bus_factory=lambda _: bus).run([(2000,) * 6])
        self.assertEqual(bus.disconnects, [False])
        self.assertEqual(bus.writes, [])  # Goal_Position is unsafe in PWM mode.


if __name__ == "__main__":
    unittest.main()
