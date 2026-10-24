"""Fake-bus check for the faster, torque-preserving Home path."""

import unittest
from unittest.mock import patch

from care_reach.routines import home


class FakeBus:
    def __init__(self, _port, motors):
        self.positions = dict(zip(motors, (2004, 1770, 2215, 2024, 2001, 1997)))
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


class HomeTests(unittest.TestCase):
    def test_speed_four_moves_all_joints_and_preserves_torque(self):
        created = []

        def factory(port, motors):
            bus = FakeBus(port, motors)
            created.append(bus)
            return bus

        with patch.object(home, "SerialMotorsBus", factory), patch.object(home.time, "sleep") as sleep:
            home.main(["--speed", "4", "--port", "unused"])
        bus = created[0]
        self.assertTrue(all(abs(value - 2000) <= home.TOLERANCE for value in bus.positions.values()))
        self.assertTrue(all(value == 4 for value in bus.registers["Goal_Speed"].values()))
        self.assertTrue(all(value == 1 for value in bus.registers["Torque_Enable"].values()))
        self.assertEqual(bus.disconnected_with, False)
        sleep.assert_called_with(home.FAST_PERIOD_S)


if __name__ == "__main__":
    unittest.main()
