"""Camera-safe upright pose confirmed on this SO-101 follower arm."""

import json
from pathlib import Path

from motion_core import JOINT_NAMES


POSE_PATH = Path(__file__).with_name("extended_pose.json")


def extended_target(path: str | Path = POSE_PATH) -> tuple[int, ...]:
    record = json.loads(Path(path).read_text(encoding="utf-8"))
    if record.get("robot_id") != "care_reach_follower":
        raise ValueError("extended pose belongs to a different robot")
    positions = record.get("positions_ticks")
    if not isinstance(positions, dict) or set(positions) != set(JOINT_NAMES):
        raise ValueError("extended pose must name all six follower joints")
    ticks = tuple(positions[name] for name in JOINT_NAMES)
    if any(type(tick) is not int or not 0 <= tick <= 4095 for tick in ticks):
        raise ValueError("extended pose contains invalid ticks")
    return ticks
