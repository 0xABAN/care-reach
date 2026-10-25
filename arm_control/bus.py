"""Shared URT-2 serial-bus construction and exclusive ownership for direct runners.

Every real-hardware caller (the dashboard ``Arm`` and each direct runner)
builds the same six STS3215 motors and holds the same exclusive ``BusLock``
for its full connection lifetime, so a second caller fails fast with a clear
busy error instead of fighting over the serial device. Injected fake bus
factories in tests bypass the default lock; an explicit ``BusLock`` is always
honored. The serial stack is imported lazily so importing this module has no
hardware effects.
"""

from __future__ import annotations

from typing import Any, Callable

from motion_core import JOINT_NAMES

from .bus_lock import BusLock

_TIMING_FIELDS = ("bus_read_ms", "bus_write_ms", "compute_ms", "cycle_ms")


def build_motors() -> dict[str, Any]:
    """The six SO-101 motors in bus order; the gripper is normalized 0..100."""
    from lerobot.motors import Motor, MotorNormMode

    return {
        name: Motor(index, "sts3215", MotorNormMode.RANGE_0_100 if index == 6 else MotorNormMode.DEGREES)
        for index, name in enumerate(JOINT_NAMES, start=1)
    }


def real_bus(port: str):
    """Construct the real URT-2 bus; kept out of fake-bus tests and import time."""
    from lerobot.motors import SerialMotorsBus

    return SerialMotorsBus(port=port, motors=build_motors())


class BusOwnership:
    """Exclusive lock plus connected bus, released safely on any failure.

    The lock rule mirrors the dashboard ``Arm``: an explicit ``bus_lock`` is
    always used, the default state-file lock is taken only when the caller
    will construct the real bus, and a connect failure releases the lock so
    the caller can retry.
    """

    def __init__(self, port: str, *, bus_factory: Callable[[str], Any] | None = None,
                 bus_lock: BusLock | None = None):
        self.port = port
        self.bus_factory = bus_factory or real_bus
        self.lock = bus_lock if bus_lock is not None else (BusLock() if bus_factory is None else None)

    def connect(self):
        if self.lock is not None:
            self.lock.acquire(self.port)
        bus = None
        try:
            bus = self.bus_factory(self.port)
            bus.connect(handshake=True)
        except BaseException:
            try:
                if bus is not None:
                    bus.disconnect(disable_torque=False)
            except Exception:
                pass
            finally:
                if self.lock is not None:
                    self.lock.release()
            raise
        return bus

    def release(self) -> None:
        if self.lock is not None:
            self.lock.release()


def format_timing_summary(timing: dict | None) -> str:
    """One-line CLI summary of measured bus and compute windows.

    ``bus_write_ms`` is the time to enqueue a serial write, not physical
    actuator latency; the formatter never implies otherwise. Returns an empty
    string when there is nothing to report so callers can print it
    unconditionally.
    """
    if not timing:
        return ""
    parts = []
    for name in _TIMING_FIELDS:
        window = timing.get(name)
        if not window or not window.get("count"):
            parts.append(f"{name}: none")
            continue
        parts.append(f"{name}: n={window['count']} mean={window['mean_ms']}ms max={window['max_ms']}ms")
    return "bus/compute " + " ".join(parts)
