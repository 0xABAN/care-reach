"""Offline checks for the compact fold; never opens a serial device."""

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from motion_core import JOINT_NAMES, SafetyBounds, UnsafePoseError, validate_pose
from park import FOLD_VMAX, HOME, build_plan, folded_bounds, folded_target, main


class ParkTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        directory = Path(temporary.name)
        ranges = [(1096, 2617), (883, 3243), (861, 3007), (843, 3130), (0, 4095), (1317, 2762)]
        calibration = {
            name: {"id": index, "range_min": lo, "range_max": hi}
            for index, (name, (lo, hi)) in enumerate(zip(JOINT_NAMES, ranges), 1)
        }
        settings = {
            str(index): {"lower": lo, "upper": hi, "scale": 1}
            for index, (lo, hi) in enumerate(
                [(1116, 2597), (903, 3223), (881, 2987), (863, 3110), (None, None), (1317, 2742)], 1
            )
        }
        self.calibration = calibration
        self.calibration_path = directory / "calibration.json"
        self.settings_path = directory / "settings.json"
        self.pose_path = directory / "folded_pose.json"
        self.calibration_path.write_text(json.dumps(calibration), encoding="utf-8")
        self.settings_path.write_text(json.dumps(settings), encoding="utf-8")
        self.pose_path.write_text(json.dumps({
            "robot_id": "care_reach_follower",
            "positions_ticks": dict(zip(JOINT_NAMES, (2040, 893, 3064, 2888, 2416, 1470), strict=True)),
        }), encoding="utf-8")
        self.bounds = SafetyBounds.from_files(self.settings_path, self.calibration_path)

    def test_manually_measured_fold_is_saved_exactly(self):
        target = folded_target(self.pose_path)
        self.assertEqual(target, (2040, 893, 3064, 2888, 2416, 1470))
        with self.assertRaisesRegex(UnsafePoseError, "motor 2"):
            validate_pose(target, self.bounds)
        with self.assertRaisesRegex(UnsafePoseError, "motor 3"):
            validate_pose((2000, 2000, 3064, 2000, 2000, 2000), self.bounds)
        task_bounds = folded_bounds(self.bounds, target)
        self.assertEqual(task_bounds.lower[1], 893)
        self.assertEqual(task_bounds.upper[2], 3064)
        self.assertEqual(task_bounds.lower[0], self.bounds.lower[0])
        self.assertEqual(task_bounds.upper[5], self.bounds.upper[5])
        self.assertEqual(validate_pose(target, task_bounds), target)

    def test_fold_moves_loaded_joints_together_inside_bounds(self):
        target, task_bounds, frames = build_plan(self.bounds, self.pose_path)
        self.assertEqual(frames[0], HOME)
        self.assertEqual(frames[-1], target)
        self.assertTrue(all(validate_pose(frame, task_bounds) == frame for frame in frames))
        halfway = frames[len(frames) // 2]
        self.assertLess(halfway[1], 2000)
        self.assertGreater(halfway[2], 2000)
        self.assertGreater(halfway[3], 2000)
        self.assertLess(halfway[5], 2000)
        for joint, vmax in enumerate(FOLD_VMAX):
            largest_step = max(abs(after[joint] - before[joint]) for before, after in zip(frames, frames[1:]))
            self.assertLessEqual(largest_step, math.ceil(vmax / 20) + 1)

    def test_fold_rejects_a_restricted_arm_without_opening_bus(self):
        restricted = SafetyBounds((1500, 1500, 1500, 1500, 1500, 1340), (2500, 2500, 2500, 2500, 2500, 2742))
        with self.assertRaisesRegex(UnsafePoseError, "motor 2"):
            build_plan(restricted, self.pose_path)
        with patch.dict(sys.modules, {"arm_runtime": None}):
            self.assertEqual(main(["--dry-run", "--settings", str(self.settings_path),
                                   "--calibration", str(self.calibration_path),
                                   "--pose", str(self.pose_path)]), 0)

    def test_pose_file_cannot_extend_another_joint_or_robot(self):
        pose = json.loads(self.pose_path.read_text(encoding="utf-8"))
        pose["positions_ticks"]["shoulder_pan"] = 1000
        self.pose_path.write_text(json.dumps(pose), encoding="utf-8")
        with self.assertRaisesRegex(UnsafePoseError, "motor 1"):
            build_plan(self.bounds, self.pose_path)
        pose["robot_id"] = "someone_else"
        self.pose_path.write_text(json.dumps(pose), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "different robot"):
            folded_target(self.pose_path)

    def test_execute_uses_measured_feedback_runner_and_park_bounds(self):
        with patch("measured_pose_runner.MeasuredPoseRunner") as runner_type:
            runner_type.return_value.move.return_value = {
                "elapsed_s": 1.2,
                "positions": dict(zip(JOINT_NAMES, folded_target(self.pose_path), strict=True)),
            }
            self.assertEqual(main([
                "--execute", "--port", "fake", "--settings", str(self.settings_path),
                "--calibration", str(self.calibration_path), "--pose", str(self.pose_path),
            ]), 0)
        self.assertEqual(runner_type.call_args.args[0], "fake")
        task_bounds = runner_type.call_args.args[1]
        self.assertEqual(task_bounds.lower[1], 893)
        self.assertEqual(task_bounds.upper[2], 3064)
        self.assertEqual(runner_type.return_value.move.call_args.args[0], folded_target(self.pose_path))
        self.assertEqual(runner_type.return_value.move.call_args.kwargs["max_lead"], 80)


if __name__ == "__main__":
    unittest.main()
