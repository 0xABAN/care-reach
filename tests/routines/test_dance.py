"""Offline checks for the coordinated demonstration."""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from care_reach.motion.bounds import SafetyBounds, validate_pose
from care_reach.robot import HOME_POSE, JOINT_NAMES
from care_reach.routines import dance


class DanceDemoTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.settings = Path(self.temp.name) / "limits.json"
        self.settings.write_text(
            json.dumps({str(i): {"lower": 1500, "upper": 2500, "scale": 1} for i in range(1, 7)})
        )
        self.calibration = Path(self.temp.name) / "calibration.json"
        self.calibration.write_text(
            json.dumps({name: {"range_min": 1200, "range_max": 2800} for name in JOINT_NAMES})
        )

    def test_data_inspired_phases_include_counterrotation_and_gripper_beats(self) -> None:
        waypoints = dance.build_waypoints(1.0)
        self.assertEqual(len(waypoints), len(dance.PHASE_VMAX))
        self.assertEqual(waypoints[-1][1], HOME_POSE)
        # The first broad arm arc counter-rotates loaded lift and elbow.
        self.assertGreater(waypoints[0][1][1] - 2000, 0)
        self.assertLess(waypoints[0][1][2] - 2000, 0)
        # Two short accents change only the gripper.
        for index in (3, 5):
            before, after = waypoints[index - 1][1], waypoints[index][1]
            self.assertEqual(before[:5], after[:5])
            self.assertNotEqual(before[5], after[5])
        self.assertGreater(max(row[4] for row in dance.PHASE_VMAX),
                           min(row[4] for row in dance.PHASE_VMAX))

    def test_full_scale_path_is_bounded_and_home_to_home(self) -> None:
        bounds = SafetyBounds.from_files(self.settings, self.calibration)
        waypoints, frames = dance.build_plan(bounds, 1.0)
        self.assertEqual(frames[0], HOME_POSE)
        self.assertEqual(frames[-1], HOME_POSE)
        self.assertGreater(len(frames), len(waypoints))
        self.assertGreater(max(frame[4] for frame in frames) - min(frame[4] for frame in frames), 750)
        for pose in frames:
            validate_pose(pose, bounds)
            self.assertLessEqual(pose[4], 3300)
            self.assertGreaterEqual(pose[5], 1340)

    def test_planned_speed_stays_within_phase_envelope(self) -> None:
        bounds = SafetyBounds.from_files(self.settings, self.calibration)
        _, frames = dance.build_plan(bounds, 1.0, fps=20)
        ceiling = tuple(max(row[j] for row in dance.PHASE_VMAX) for j in range(6))
        for before, after in zip(frames, frames[1:]):
            for joint in range(6):
                # One tick accounts for frame quantization.
                self.assertLessEqual(abs(after[joint] - before[joint]),
                                     ceiling[joint] / 20 + 1.0)

    def test_default_dry_run_never_imports_runtime(self) -> None:
        output = io.StringIO()
        # If a future change imports the runtime on the offline path, this
        # missing-module sentinel exposes it without attaching any hardware.
        with patch.dict(sys.modules, {"care_reach.hardware.bus": None, "care_reach.hardware.arm_runtime": None}):
            with contextlib.redirect_stdout(output):
                result = dance.main(
                    ["--settings", str(self.settings), "--calibration", str(self.calibration)]
                )
        self.assertEqual(result, 0)
        self.assertIn("no servo bus opened", output.getvalue())

    def test_unsafe_scale_and_bounds_are_rejected(self) -> None:
        bounds = SafetyBounds.from_files(self.settings, self.calibration)
        for invalid in (0, -0.1, 1.01, float("nan")):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                dance.build_plan(bounds, invalid)
        tight = Path(self.temp.name) / "tight.json"
        tight.write_text(
            json.dumps({str(i): {"lower": 1990, "upper": 2010, "scale": 1} for i in range(1, 7)})
        )
        with self.assertRaises(ValueError):
            dance.build_plan(SafetyBounds.from_files(tight, self.calibration), 0.5)


if __name__ == "__main__":
    unittest.main()
