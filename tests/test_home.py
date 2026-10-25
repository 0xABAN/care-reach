"""Fake-bus check for the faster, torque-preserving Home path."""

import unittest
from unittest.mock import patch

import arm_control.home as home_lib
import home


class FakeBus:
    def __init__(self, _port, motors):
        self.positions = dict(zip(motors, (2004, 1170, 3005, 2124, 401, 1997)))
        self.registers = {
            "Min_Position_Limit": {name: 0 for name in motors},
            "Max_Position_Limit": {name: 4095 for name in motors},
            "Torque_Enable": {name: 1 for name in motors},
            "Status": {name: 0 for name in motors},
            "Present_Voltage": {name: 53 for name in motors},
            "P_Coefficient": {name: 32 for name in motors},
            "Goal_Speed": {name: 2 for name in motors},
            "Acceleration": {name: 3 for name in motors},
        }
        self.disconnected_with = None

    def connect(self, handshake=True):
        assert handshake

    def disconnect(self, disable_torque=True):
        self.disconnected_with = disable_torque

    def read(self, register, name, normalize=False, num_retry=2):
        assert not normalize and num_retry == 2
        if register == "Present_Position":
            return self.positions[name]
        return self.registers[register][name]

    def write(self, register, name, value, normalize=False, num_retry=2):
        assert not normalize and num_retry == 2
        if register == "Goal_Position":
            self.positions[name] = value
        else:
            self.registers[register][name] = value


class FakeLock:
    def __init__(self, path=None):
        self.events = []

    def acquire(self, port):
        self.events.append(("acquire", port))

    def release(self):
        self.events.append("release")


class HomeTests(unittest.TestCase):
    def test_speed_four_moves_all_joints_and_preserves_torque(self):
        created = []
        lock = FakeLock()

        def factory(port):
            bus = FakeBus(port, list(home_lib.NAMES))
            created.append(bus)
            return bus

        with patch.object(home_lib, "real_bus", factory), \
             patch.object(home_lib, "BusLock", lambda: lock), \
             patch.object(home.time, "sleep") as sleep:
            result = home.main(["--speed", "4", "--port", "unused"])
        bus = created[0]
        self.assertTrue(all(abs(bus.positions[name] - home.HOME[index]) <= home.TOLERANCE
                            for index, name in enumerate(home.NAMES)))
        self.assertLessEqual(bus.positions["elbow_flex"], home.HOME[2])
        self.assertTrue(all(value == 4 for value in bus.registers["Goal_Speed"].values()))
        self.assertTrue(all(value == 1 for value in bus.registers["Torque_Enable"].values()))
        self.assertEqual(bus.disconnected_with, False)
        sleep.assert_called_with(home.FAST_PERIOD_S)
        self.assertEqual(lock.events, [("acquire", "unused"), "release"])
        timing = result["timing"]
        for field in ("bus_read_ms", "bus_write_ms", "compute_ms", "cycle_ms"):
            self.assertGreater(timing[field]["count"], 0, field)
        self.assertLessEqual(timing["compute_ms"]["mean_ms"], timing["cycle_ms"]["mean_ms"] + 1e-9)


if __name__ == "__main__":
    unittest.main()
