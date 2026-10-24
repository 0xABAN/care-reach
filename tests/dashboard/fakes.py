"""Fake servo buses for dashboard tests; they never open a serial device."""

from care_reach.robot import JOINT_NAMES


class FakeBus:
    def __init__(self, positions, torque=1):
        self.positions = dict(zip(JOINT_NAMES, positions, strict=True))
        self.goals = self.positions.copy()
        self.torque = {name: torque for name in JOINT_NAMES}
        self.loads = {name: 0 for name in JOINT_NAMES}
        self.status = {name: 0 for name in JOINT_NAMES}
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
        if register == "Torque_Enable":
            self.torque.update(values)
        elif register == "Goal_Position":
            assert set(values) == set(JOINT_NAMES)
            self.goals.update(values)
        else:
            raise AssertionError(f"unexpected sync_write register {register}")
        self.writes.append((register, values.copy()))


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
