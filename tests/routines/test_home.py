"""Fake-bus check for the faster, torque-preserving Home path."""

import unittest
from unittest.mock import patch

from care_reach.robot import JOINT_NAMES
from care_reach.routines import home


class FakeBus:
    def __init__(self, _port):
        self.positions = dict(zip(JOINT_NAMES, (2004, 1770, 2215, 2024, 2001, 1997), strict=True))
        self.registers = {
            "Min_Position_Limit": {name: 0 for name in JOINT_NAMES},
            "Max_Position_Limit": {name: 4095 for name in JOINT_NAMES},
            "Torque_Enable": {name: 1 for name in JOINT_NAMES},
            "Status": {name: 0 for name in JOINT_NAMES},
            "Present_Voltage": {name: 53 for name in JOINT_NAMES},
            "P_Coefficient": {name: 32 for name in JOINT_NAMES},
            "Goal_Speed": {name: 2 for name in JOINT_NAMES},
            "Acceleration": {name: 3 for name in JOINT_NAMES},
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


class HomeTests(unittest.TestCase):
    def test_speed_four_moves_all_joints_and_preserves_torque(self):
        created = []

        def factory(port):
            bus = FakeBus(port)
            created.append(bus)
            return bus

        with patch.object(home, "make_bus", factory), patch.object(home.time, "sleep") as sleep:
            home.main(["--speed", "4", "--port", "unused"])
        bus = created[0]
        self.assertTrue(all(abs(value - 2000) <= home.TOLERANCE for value in bus.positions.values()))
        self.assertTrue(all(value == 4 for value in bus.registers["Goal_Speed"].values()))
        self.assertTrue(all(value == 1 for value in bus.registers["Torque_Enable"].values()))
        self.assertEqual(bus.disconnected_with, False)
        sleep.assert_called_with(home.FAST_PERIOD_S)


if __name__ == "__main__":
    unittest.main()
