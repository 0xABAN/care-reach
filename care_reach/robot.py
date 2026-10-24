"""Facts about this one assembled SO-101 follower and where its records live."""

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

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = REPO_ROOT / "config"

# Panel limits and jog speeds; the dashboard rewrites this tracked file.
JOINT_SETTINGS_PATH = CONFIG_DIR / "joint_settings.json"

# Lance's hand-folded pose, recorded as raw encoder ticks.
FOLDED_POSE_PATH = CONFIG_DIR / "folded_pose.json"

# LeRobot reads and writes the active calibration here. A versioned recovery
# copy lives at config/care_reach_follower.json; never overwrite one blindly.
CALIBRATION_PATH = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower" / f"{ROBOT_ID}.json"
)
