"""Facts about this one assembled SO-101 follower and where its records live."""

from __future__ import annotations

import json
from pathlib import Path


ROBOT_ID = "care_reach_follower"

# Bus order: each joint's servo ID is its position in this tuple plus one.
JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
JOINT_LABELS = ("Shoulder pan", "Shoulder lift", "Elbow flex", "Wrist flex", "Wrist roll", "Gripper")

# Home is a convention for powered tests and the dance, not a guarantee that
# every joint is at its geometric center.
HOME_TICK = 2000
HOME_POSE = (HOME_TICK,) * len(JOINT_NAMES)

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = REPO_ROOT / "config"

# Panel limits and jog speeds; the dashboard rewrites this tracked file.
JOINT_SETTINGS_PATH = CONFIG_DIR / "joint_settings.json"

# Lance's hand-folded pose, recorded as raw encoder ticks.
FOLDED_POSE_PATH = CONFIG_DIR / "folded_pose.json"

# The survey pose, aim point, and camera directions that `pick_setup` records.
PICK_CONFIG_PATH = CONFIG_DIR / "pick.json"

# The official SO-101 model cached on the Pi; see docs/KINEMATICS.md.
SO101_URDF_PATH = Path.home() / ".cache/care-reach/so101-model/so101_new_calib.urdf"

# LeRobot reads and writes the active calibration here. A versioned recovery
# copy lives at config/care_reach_follower.json; never overwrite one blindly.
CALIBRATION_PATH = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower" / f"{ROBOT_ID}.json"
)


def load_folded_pose(path: str | Path = FOLDED_POSE_PATH) -> tuple[int, ...]:
    """Read the six exact encoder ticks of Lance's physical folded pose."""
    with Path(path).open(encoding="utf-8") as stream:
        record = json.load(stream)
    if record.get("robot_id") != ROBOT_ID:
        raise ValueError("folded pose belongs to a different robot")

    positions = record.get("positions_ticks")
    if not isinstance(positions, dict) or set(positions) != set(JOINT_NAMES):
        raise ValueError("folded pose must name exactly the six follower joints")

    ticks = tuple(positions[name] for name in JOINT_NAMES)
    if any(isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 4095 for value in ticks):
        raise ValueError("folded pose must contain six valid integer encoder ticks")
    return ticks
