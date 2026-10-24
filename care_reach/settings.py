"""The joint settings file: saved panel limits and jog speed for each motor.

The dashboard edits this tracked file, and ``SafetyBounds.from_files``
intersects its limits with the LeRobot calibration. Each motor key "1".."6"
maps to ``{"lower": int | None, "upper": int | None, "scale": int}``; a null
limit means that end uses the calibrated range.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from care_reach.robot import JOINT_SETTINGS_PATH


SPEED_SCALES = (1, 2, 4, 8)
MOTOR_KEYS = tuple(str(motor) for motor in range(1, 7))
ENTRY_KEYS = {"lower", "upper", "scale"}


def load_joint_settings(path: str | Path = JOINT_SETTINGS_PATH) -> dict[str, dict]:
    """Read and validate the settings for motors 1 through 6.

    A missing file, motor, or field is an error rather than an implied
    default, because these values bound every commanded pose.
    """
    saved = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(saved, dict) or set(saved) != set(MOTOR_KEYS):
        raise ValueError("Joint settings must have one entry for each motor 1 through 6")

    settings = {}
    for key in MOTOR_KEYS:
        entry = saved[key]
        if not isinstance(entry, dict) or set(entry) != ENTRY_KEYS:
            raise ValueError(f"Motor {key} settings must have exactly lower, upper, and scale")

        lower, upper, scale = entry["lower"], entry["upper"], entry["scale"]
        if any(value is not None and (type(value) is not int or not 0 <= value <= 4095)
               for value in (lower, upper)):
            raise ValueError(f"Invalid limit for motor {key}")
        if lower is not None and upper is not None and lower >= upper:
            raise ValueError(f"Lower limit must be less than upper limit for motor {key}")
        if type(scale) is not int or scale not in SPEED_SCALES:
            raise ValueError(f"Invalid scale for motor {key}")

        settings[key] = {"lower": lower, "upper": upper, "scale": scale}
    return settings


def save_joint_settings(path: str | Path, settings: dict[str, dict]) -> None:
    """Replace the settings file atomically so a crash never leaves partial limits."""
    path = Path(path)
    temporary = path.with_suffix(".json.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(settings, stream, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
