"""Shared offline fixtures built from this arm's recorded ranges."""

import json
import tempfile
import unittest
from pathlib import Path

from care_reach.motion.bounds import SafetyBounds
from care_reach.robot import JOINT_NAMES, ROBOT_ID


# Calibrated tick ranges and saved panel limits measured on the real arm.
CALIBRATED_RANGES = ((1096, 2617), (883, 3243), (861, 3007), (843, 3130), (0, 4095), (1317, 2762))
PANEL_LIMITS = ((1116, 2597), (903, 3223), (881, 2987), (863, 3110), (None, None), (1317, 2742))
FOLDED_TICKS = (2040, 893, 3064, 2888, 2416, 1470)


class ArmFilesTestCase(unittest.TestCase):
    """Write calibration, joint settings, and folded pose files for each test.

    Subclasses get ``calibration`` (the parsed dictionary), the three file
    paths, and ``bounds`` loaded from those files exactly as the routines do.
    """

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        directory = Path(temporary.name)

        self.calibration = {
            name: {"id": index, "range_min": lo, "range_max": hi}
            for index, (name, (lo, hi)) in enumerate(zip(JOINT_NAMES, CALIBRATED_RANGES, strict=True), start=1)
        }
        settings = {
            str(index): {"lower": lo, "upper": hi, "scale": 1}
            for index, (lo, hi) in enumerate(PANEL_LIMITS, start=1)
        }
        pose = {
            "robot_id": ROBOT_ID,
            "positions_ticks": dict(zip(JOINT_NAMES, FOLDED_TICKS, strict=True)),
        }

        self.calibration_path = directory / "calibration.json"
        self.settings_path = directory / "joint_settings.json"
        self.pose_path = directory / "folded_pose.json"
        self.calibration_path.write_text(json.dumps(self.calibration), encoding="utf-8")
        self.settings_path.write_text(json.dumps(settings), encoding="utf-8")
        self.pose_path.write_text(json.dumps(pose), encoding="utf-8")

        self.bounds = SafetyBounds.from_files(self.settings_path, self.calibration_path)
