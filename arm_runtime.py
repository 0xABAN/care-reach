"""Backward-compatible re-export of the guarded playback runtime.

The implementation lives in ``arm_control/runtime.py``; this module keeps the
historical import path working for existing callers and tests. It has no
import-time hardware effects.
"""

from __future__ import annotations

from arm_control.bus import real_bus as _real_bus  # noqa: F401
from arm_control.runtime import (  # noqa: F401
    ArmRuntime,
    ArmSafetyError,
    DANCE_CENTER,
    DEFAULT_COMMAND_ACCELERATION,
    DEFAULT_COMMAND_SPEED,
    FINISH_TOLERANCE,
    FINISH_TIMEOUT_S,
    HOME,
    HOME_TOLERANCE,
    HOME_TRIM_ERROR_WINDOW,
    HOME_TRIM_STAGNANT_S,
    HOME_TRIM_TICKS,
    MAX_LOAD_MAGNITUDE,
    MIN_VOLTAGE_RAW,
    STALL_ERROR,
    STALL_PROGRESS_TICKS,
    STALL_TIME_S,
)
