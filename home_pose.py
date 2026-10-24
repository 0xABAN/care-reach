"""Validated, recorded Home position and its narrow motor 3 exception."""

import json
from pathlib import Path

from motion_core import JOINT_NAMES, SafetyBounds, UnsafePoseError

POSE_PATH = Path(__file__).with_name("home_pose.json")
MAX_HOME_EXTENSION = (0, 0, 80, 0, 0, 0)
MAX_RECORDED_MOTOR3 = 3065


def home_target(path=POSE_PATH):
    record = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(record, dict) or record.get("robot_id") != "care_reach_follower":
        raise ValueError("home pose belongs to a different robot")
    positions = record.get("positions_ticks")
    if not isinstance(positions, dict) or set(positions) != set(JOINT_NAMES):
        raise ValueError("home pose must name exactly the six follower joints")
    ticks = tuple(positions[name] for name in JOINT_NAMES)
    if any(type(tick) is not int or not 0 <= tick <= 4095 for tick in ticks):
        raise ValueError("home pose must contain six valid integer encoder ticks")
    if ticks[2] > MAX_RECORDED_MOTOR3:
        raise ValueError("home motor 3 exceeds recorded maximum 3065")
    return ticks


def home_bounds(bounds: SafetyBounds, target=None):
    target = home_target() if target is None else tuple(target)
    if len(target) != len(JOINT_NAMES):
        raise ValueError("home target must contain six joints")
    if target[2] > MAX_RECORDED_MOTOR3:
        raise UnsafePoseError("motor 3 home tick exceeds recorded maximum 3065")
    lower, upper = [], []
    for index, (tick, lo, hi, allowance) in enumerate(
        zip(target, bounds.lower, bounds.upper, MAX_HOME_EXTENSION, strict=True), start=1
    ):
        if max(lo - tick, tick - hi, 0) > allowance:
            raise UnsafePoseError(f"motor {index} home tick {tick} exceeds recorded-pose extension")
        lower.append(min(lo, tick))
        upper.append(max(hi, tick))
    return SafetyBounds(tuple(lower), tuple(upper))
