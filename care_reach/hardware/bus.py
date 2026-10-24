"""Shared access to the URT-2 servo bus and the checks every controller applies.

LeRobot and pyserial are imported only when a real port is used, so offline
planning and fake-bus tests need neither. Nothing here changes torque,
operating mode, or firmware limits.
"""

from __future__ import annotations

from typing import Any

from care_reach.robot import JOINT_NAMES


URT2_USB_ID = (0x1A86, 0x55D3)
POSITION_MODE = 0

# Live guard thresholds shared by the controllers.
MIN_VOLTAGE_RAW = 48  # STS3215 reports tenths of a volt.
MAX_LOAD_MAGNITUDE = 900  # LeRobot decodes Present_Load as a signed value.
SUSTAINED_LOAD_SAMPLES = 3
STALL_ERROR_TICKS = 35  # Distance from the goal that counts as lagging.
STALL_PROGRESS_TICKS = 4  # Movement that counts as progress.
STALL_TIME_S = 0.8  # Lagging without progress for this long is a stall.


class ArmSafetyError(RuntimeError):
    """A preflight or live safety guard stopped motion."""


def find_port() -> str:
    """Return the serial path of the single connected URT-2 board."""
    from serial.tools import list_ports

    vendor, product = URT2_USB_ID
    ports = [port.device for port in list_ports.comports() if port.vid == vendor and port.pid == product]
    if len(ports) != 1:
        raise RuntimeError(f"Expected one URT-2 (USB 1A86:55D3), found {len(ports)}; use --port")
    return ports[0]


def make_bus(port: str) -> Any:
    """Build, but do not connect, a LeRobot bus for the six STS3215 servos."""
    from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus

    motors = {
        name: Motor(index, "sts3215", MotorNormMode.RANGE_0_100 if index == 6 else MotorNormMode.DEGREES)
        for index, name in enumerate(JOINT_NAMES, start=1)
    }
    return SerialMotorsBus(port=port, motors=motors)


def read_joints(bus: Any, register: str) -> dict[str, int]:
    """Read one raw register from all six joints; a partial read is a safety stop."""
    values = bus.sync_read(register, normalize=False, num_retry=2)
    if set(values) != set(JOINT_NAMES):
        raise ArmSafetyError(f"Incomplete {register} read: {sorted(values)}")
    return {name: int(values[name]) for name in JOINT_NAMES}


def read_firmware_limits(bus: Any) -> tuple[dict[str, int], dict[str, int]]:
    """Return each servo's (minimum, maximum) firmware position limits."""
    return read_joints(bus, "Min_Position_Limit"), read_joints(bus, "Max_Position_Limit")


def require_position_mode(bus: Any) -> None:
    """Refuse to continue unless every servo is in position mode.

    Goal_Position writes are unsafe in any other mode. Changing a mode is a
    deliberate setup step, so this check never does it.
    """
    modes = read_joints(bus, "Operating_Mode")
    for name in JOINT_NAMES:
        if modes[name] != POSITION_MODE:
            raise ArmSafetyError(f"{name}: operating mode {modes[name]}, expected position mode {POSITION_MODE}")


def check_voltage(bus: Any, name: str, reading: int) -> None:
    """Stop when a joint's supply voltage is low on two consecutive reads.

    Brief STS3215 voltage dips are known on this arm's 5 V supply, so a single
    low sample is re-read before it counts as a fault.
    """
    if reading >= MIN_VOLTAGE_RAW:
        return

    second = read_joints(bus, "Present_Voltage")[name]
    if second < MIN_VOLTAGE_RAW:
        raise ArmSafetyError(f"{name}: voltage stayed low ({reading}, {second})")


class LoadWatch:
    """Stop a joint whose load magnitude stays high for several samples in a row."""

    def __init__(self) -> None:
        self._high_samples = dict.fromkeys(JOINT_NAMES, 0)

    def check(self, name: str, load: int) -> None:
        magnitude = abs(load)
        if magnitude >= MAX_LOAD_MAGNITUDE:
            self._high_samples[name] += 1
        else:
            self._high_samples[name] = 0

        if self._high_samples[name] >= SUSTAINED_LOAD_SAMPLES:
            raise ArmSafetyError(f"{name}: sustained high load {magnitude}")
