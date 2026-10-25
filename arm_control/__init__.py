"""Importable movement core for the Care Reach SO-101 follower.

``arm_control`` owns the serial bus and every piece of movement logic.
``server.py`` is only the HTTP/camera adapter and process entry point;
``motion_core`` stays pure offline planning.
"""

from arm_control.arm import (  # noqa: F401
    Arm,
    GRIP_HOLD_LEAD,
    HOME_TOLERANCE,
    POSE_TOLERANCE,
    STOP_SETTLE_LEAD,
    find_port,
    load_settings,
)
from arm_control.bus_lock import BusBusyError, BusLock, DEFAULT_LOCK_PATH  # noqa: F401
from arm_control.timing import MonotonicClock, MotionTiming, TimedBus  # noqa: F401

__all__ = [
    "Arm",
    "BusBusyError",
    "BusLock",
    "DEFAULT_LOCK_PATH",
    "GRIP_HOLD_LEAD",
    "HOME_TOLERANCE",
    "MonotonicClock",
    "MotionTiming",
    "POSE_TOLERANCE",
    "STOP_SETTLE_LEAD",
    "TimedBus",
    "find_port",
    "load_settings",
]
