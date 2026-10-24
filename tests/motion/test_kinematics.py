"""Offline checks for the calibrated FK/IK adapter, using a stand-in model."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from care_reach.motion.bounds import SafetyBounds
from care_reach.motion.kinematics import IKError, SO101Kinematics
from care_reach.robot import JOINT_NAMES


class LinearModel:
    def __init__(self, **kwargs):
        self.kwargs = kwargs

    def forward_kinematics(self, degrees):
        matrix = np.eye(4)
        matrix[:3, 3] = np.asarray(degrees[:3]) / 1000
        return matrix

    def inverse_kinematics(self, current, target, **kwargs):
        self.ik_options = kwargs
        result = np.array(current, dtype=float)
        result[:3] = target[:3, 3] * 1000
        return result


class BrokenModel(LinearModel):
    def inverse_kinematics(self, current, target, **kwargs):
        return np.full(5, np.nan)


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.urdf = root / "model.urdf"
        self.urdf.write_text("<robot/>")
        self.calibration = root / "calibration.json"
        self.calibration.write_text(json.dumps({name: {"range_min": 1000 + i * 100,
                                                             "range_max": 3000 - i * 100,
                                                             "drive_mode": 0}
                                                for i, name in enumerate(JOINT_NAMES)}))
        self.bounds = SafetyBounds((1000,) * 6, (3000,) * 6)

    def make(self, model=LinearModel):
        return SO101Kinematics(self.calibration, self.urdf, self.bounds,
                               kinematics_factory=model)

    def test_calibrated_midpoint_and_gripper_preserved(self):
        solver = self.make()
        self.assertEqual(solver.model.kwargs["joint_names"][-1], "wrist_roll")
        self.assertEqual(solver.model.kwargs["target_frame_name"], "gripper_frame_link")
        start = (2100, 2000, 2000, 2000, 2000, 1760)
        self.assertAlmostEqual(solver.ticks_to_degrees(start)[0], 100 * 360 / 4095)
        result = solver.translate(start, (0.01, 0, 0))
        self.assertEqual(result.target_ticks[5], 1760)
        self.assertEqual(result.target_ticks[0], 2214)
        self.assertLess(result.residual_mm, 0.1)
        solver.translate(start, (0, 0.01, 0), orientation_weight=0)
        self.assertEqual(solver.model.ik_options["orientation_weight"], 0)

    def test_outside_bounds_and_oversized_step_rejected(self):
        solver = self.make()
        with self.assertRaises(ValueError):
            solver.forward((999, 2000, 2000, 2000, 2000, 1800))
        with self.assertRaises(IKError):
            solver.translate((2000,) * 6, (0.031, 0, 0))
        with self.assertRaises(IKError):
            solver.translate((2000,) * 6, (0.02, 0, 0), max_joint_step_ticks=100)
        with self.assertRaises(IKError):
            solver.translate((2000,) * 6, (0.01, 0, 0), orientation_weight=float("nan"))
        with self.assertRaises(IKError):
            solver.translate((2990, 2000, 2000, 2000, 2000, 1800), (0.01, 0, 0))

    def test_nonfinite_solution_and_bad_calibration_rejected(self):
        with self.assertRaises(IKError):
            self.make(BrokenModel).translate((2000,) * 6, (0.01, 0, 0))
        data = json.loads(self.calibration.read_text())
        data["wrist_roll"]["drive_mode"] = 1
        self.calibration.write_text(json.dumps(data))
        with self.assertRaises(IKError):
            self.make()


if __name__ == "__main__":
    unittest.main()
