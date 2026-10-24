"""Offline fake-bus checks for the explicit wrist-only shake command."""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from motion_core import JOINT_NAMES, SafetyBounds
from park import HOME
from shake_it_crazy_style import (
    AMPLITUDE_TICKS, DURATION_S, HOME_TOLERANCE, WRIST, WRIST_INDEX,
    ShakeSafetyError, WristShake, TRIAL_ACCELERATION, TRIAL_GOAL_SPEED, main, waveform,
)

BOUNDS = SafetyBounds((1116, 903, 881, 863, 0, 1317),
                      (2597, 3223, 3200, 3110, 3300, 2742))


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
        self.positions = dict(zip(JOINT_NAMES, HOME, strict=True))
        self.torque = dict.fromkeys(JOINT_NAMES, 1)
        self.status = dict.fromkeys(JOINT_NAMES, 0)
        self.voltage = dict.fromkeys(JOINT_NAMES, 53)
        self.load = dict.fromkeys(JOINT_NAMES, 50)
        self.mode = dict.fromkeys(JOINT_NAMES, 0)
        self.current = dict.fromkeys(JOINT_NAMES, 3)
        self.speed = dict.fromkeys(JOINT_NAMES, 8)
        self.acceleration = dict.fromkeys(JOINT_NAMES, 3)
        self.events = []

    def connect(self, handshake=True):
        assert handshake
        self.connected = True

    def disconnect(self, disable_torque=True):
        self.events.append(("disconnect", disable_torque))
        self.disconnects.append(disable_torque)
        self.connected = False

    def sync_read(self, register, normalize=True, num_retry=0):
        assert self.connected and not normalize and num_retry == 2
        return {
            "Present_Position": self.positions,
            "Torque_Enable": self.torque,
            "Status": self.status,
            "Present_Voltage": self.voltage,
            "Present_Load": self.load,
            "Operating_Mode": self.mode,
            "Present_Current": self.current,
            "Min_Position_Limit": dict.fromkeys(JOINT_NAMES, 0),
            "Max_Position_Limit": dict.fromkeys(JOINT_NAMES, 4095),
        }[register].copy()

    def read(self, register, name, normalize=True, num_retry=0):
        assert self.connected and name == WRIST and not normalize and num_retry == 2
        return {"Goal_Speed": self.speed, "Acceleration": self.acceleration,
                "Present_Current": self.current}[register][name]

    def write(self, register, name, value, normalize=True, num_retry=0):
        assert self.connected and name == WRIST and not normalize and num_retry == 2
        self.events.append((register, value))
        {"Goal_Speed": self.speed, "Acceleration": self.acceleration}[register][name] = value

    def sync_write(self, register, goals, normalize=True):
        assert self.connected and register == "Goal_Position" and not normalize
        self.events.append((register, goals.copy()))
        self.writes.append(goals.copy())
        for name, goal in goals.items():
            delta = goal - self.positions[name]
            self.positions[name] += max(-30, min(30, delta))


class WristShakeTests(unittest.TestCase):
    def _run(self, bus):
        return WristShake("unused", BOUNDS, bus_factory=lambda _: bus,
                          clock=FakeClock(), owner_check=lambda _: None).run()

    def test_waveform_is_bounded_and_starts_and_ends_smoothly(self):
        values = [waveform(i / 100) for i in range(1501)]
        self.assertEqual(values[0], 0)
        self.assertEqual(values[-1], 0)
        self.assertTrue(all(abs(value) <= AMPLITUDE_TICKS for value in values))
        self.assertGreater(max(values) - min(values), 2 * AMPLITUDE_TICKS - 2)
        self.assertLessEqual(abs(values[1] - values[0]), 1)
        self.assertLessEqual(abs(values[-1] - values[-2]), 1)
        with self.assertRaises(ValueError):
            waveform(DURATION_S + 0.01)

    def test_exact_duration_bounds_only_wrist_moves_and_torque_remains_on(self):
        bus = FakeBus("unused")
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = self._run(bus)
        lines = [line for line in output.getvalue().splitlines() if line.startswith("shake t=")]
        self.assertGreaterEqual(len(lines), 29)
        self.assertIn("desired=", lines[0])
        self.assertIn("measured=", lines[0])
        self.assertIn("voltage=", lines[0])
        self.assertIn("load=", lines[0])
        self.assertIn("current_raw=", lines[0])
        self.assertEqual(bus.speed[WRIST], 8)
        self.assertEqual(bus.acceleration[WRIST], 3)
        self.assertIn(("Goal_Speed", TRIAL_GOAL_SPEED), bus.events)
        self.assertIn(("Acceleration", TRIAL_ACCELERATION), bus.events)
        self.assertEqual(bus.events[-4][0], "Goal_Position")
        self.assertEqual(bus.events[-3:-1], [("Goal_Speed", 8), ("Acceleration", 3)])
        self.assertGreaterEqual(result["elapsed_s"], DURATION_S)
        self.assertLess(result["elapsed_s"], DURATION_S + 0.051)
        self.assertGreater(result["writes"], 200)
        self.assertTrue(all(set(write) == {WRIST} for write in bus.writes[1:-1]))
        self.assertEqual(bus.writes[-1], bus.positions)
        self.assertEqual(bus.disconnects, [False])
        self.assertTrue(all(bus.torque.values()))
        for write in bus.writes:
            self.assertTrue(BOUNDS.lower[WRIST_INDEX] <= write[WRIST] <= BOUNDS.upper[WRIST_INDEX])
            self.assertTrue(abs(write[WRIST] - HOME[WRIST_INDEX]) <= AMPLITUDE_TICKS + 65)
        for i, name in enumerate(JOINT_NAMES):
            if i != WRIST_INDEX:
                self.assertEqual(bus.positions[name], HOME[i])

    def test_rejects_non_home_without_motion(self):
        bus = FakeBus("unused")
        bus.positions["shoulder_lift"] += HOME_TOLERANCE + 1
        with self.assertRaisesRegex(ShakeSafetyError, "not at saved Home"):
            self._run(bus)
        self.assertEqual(bus.writes, [])
        self.assertEqual(bus.disconnects, [False])

    def test_rejects_bad_mode_and_lost_torque(self):
        for register, name in (("mode", "mode"), ("torque", "torque")):
            bus = FakeBus("unused")
            getattr(bus, register)[WRIST] = 1 if register == "mode" else 0
            with self.subTest(register=register), self.assertRaisesRegex(ShakeSafetyError, name):
                self._run(bus)
            self.assertEqual(bus.writes, [])
            self.assertEqual(bus.disconnects, [False])

    def test_live_voltage_fault_stops_and_holds_measured(self):
        class VoltageBus(FakeBus):
            def sync_write(self, register, goals, normalize=True):
                super().sync_write(register, goals, normalize)
                if len(self.writes) == 10:
                    self.voltage[WRIST] = 47

        bus = VoltageBus("unused")
        with self.assertRaisesRegex(ShakeSafetyError, "sustained low voltage"):
            self._run(bus)
        self.assertEqual(bus.writes[-1], bus.positions)
        self.assertEqual(bus.disconnects, [False])
        self.assertTrue(all(bus.torque.values()))
        self.assertEqual(bus.speed[WRIST], 8)
        self.assertEqual(bus.acceleration[WRIST], 3)

    def test_fault_and_high_load_stop(self):
        for field, value, message in (("status", 32, "servo fault"), ("load", 901, "high load")):
            class FaultBus(FakeBus):
                def sync_write(self, register, goals, normalize=True):
                    super().sync_write(register, goals, normalize)
                    if len(self.writes) == 8:
                        getattr(self, field)[WRIST] = value

            bus = FaultBus("unused")
            with self.subTest(field=field), self.assertRaisesRegex(ShakeSafetyError, message):
                self._run(bus)
            self.assertEqual(bus.writes[-1], bus.positions)
            self.assertEqual(bus.disconnects, [False])
            self.assertEqual(bus.speed[WRIST], 8)
            self.assertEqual(bus.acceleration[WRIST], 3)

    def test_partial_tuning_failure_restores_saved_registers(self):
        class PartialBus(FakeBus):
            def write(self, register, name, value, normalize=True, num_retry=0):
                if register == "Acceleration" and value == TRIAL_ACCELERATION:
                    raise OSError("trial tuning write failed")
                super().write(register, name, value, normalize, num_retry)

        bus = PartialBus("unused")
        with self.assertRaisesRegex(OSError, "trial tuning write failed"):
            self._run(bus)
        self.assertEqual(bus.speed[WRIST], 8)
        self.assertEqual(bus.acceleration[WRIST], 3)
        self.assertEqual(bus.writes[-1], bus.positions)
        self.assertEqual(bus.disconnects, [False])

    def test_tuning_readback_mismatch_stops_before_oscillation(self):
        class MismatchBus(FakeBus):
            def write(self, register, name, value, normalize=True, num_retry=0):
                if register == "Goal_Speed" and value == TRIAL_GOAL_SPEED:
                    return
                super().write(register, name, value, normalize, num_retry)

        bus = MismatchBus("unused")
        with self.assertRaisesRegex(ShakeSafetyError, "Goal_Speed readback mismatch"):
            self._run(bus)
        self.assertEqual(bus.speed[WRIST], 8)
        self.assertEqual(bus.acceleration[WRIST], 3)
        self.assertEqual(len(bus.writes), 2)  # initial hold and final measured hold
        self.assertEqual(bus.disconnects, [False])

    def test_owner_check_blocks_before_connect(self):
        bus = FakeBus("unused")
        def occupied(_):
            raise ShakeSafetyError("busy")
        with self.assertRaisesRegex(ShakeSafetyError, "busy"):
            WristShake("unused", BOUNDS, bus_factory=lambda _: bus, owner_check=occupied).run()
        self.assertFalse(bus.connected)
        self.assertEqual(bus.writes, [])

    def test_default_cli_is_offline(self):
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            settings = Path(directory) / "settings.json"
            calibration = Path(directory) / "calibration.json"
            settings.write_text(json.dumps({str(i): {"lower": BOUNDS.lower[i - 1],
                                                       "upper": BOUNDS.upper[i - 1]}
                                            for i in range(1, 7)}))
            calibration.write_text(json.dumps({name: {"range_min": BOUNDS.lower[i],
                                                            "range_max": BOUNDS.upper[i]}
                                               for i, name in enumerate(JOINT_NAMES)}))
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(["--settings", str(settings), "--calibration", str(calibration)]), 0)
        self.assertIn("no servo bus opened", output.getvalue())


if __name__ == "__main__":
    unittest.main()
