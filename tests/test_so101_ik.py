import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from motion_core import SafetyBounds
from so101_ik import IKError, SO101Kinematics


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


class StuckModel(LinearModel):
    def inverse_kinematics(self, current, target, **kwargs):
        return np.asarray(current, dtype=float)


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.urdf = root / "model.urdf"
        self.urdf.write_text("<robot/>")
        self.calibration = root / "calibration.json"
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
        self.calibration.write_text(json.dumps({name: {"range_min": 1000 + i * 100,
                                                             "range_max": 3000 - i * 100,
                                                             "drive_mode": 0}
                                                for i, name in enumerate(names)}))
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

    def test_absolute_xyz_is_position_focused_and_preserves_gripper(self):
        solver = self.make()
        start = (2000, 2000, 2000, 2000, 2000, 1760)
        start_xyz = np.asarray(solver.predicted_xyz(start))
        requested = tuple(start_xyz + np.array((0.01, -0.005, 0.015)))
        result = solver.solve_xyz(start, requested)
        self.assertEqual(result.start_ticks, start)
        self.assertEqual(result.target_ticks[5], start[5])
        self.assertEqual(result.target_xyz_m, requested)
        self.assertLess(result.residual_mm, 0.5)
        self.assertEqual(solver.model.ik_options["orientation_weight"], 0.0)
        self.assertEqual(solver.solve_xyz(start, tuple(start_xyz)).target_ticks, start)

    def test_nominal_frame_is_explicit(self):
        solver = SO101Kinematics(self.calibration, self.urdf, self.bounds,
                                 kinematics_factory=LinearModel,
                                 tool_frame_name="test_tool_point")
        self.assertEqual(solver.tool_frame_name, "test_tool_point")
        self.assertEqual(solver.model.kwargs["target_frame_name"], "test_tool_point")
        with self.assertRaises(IKError):
            SO101Kinematics(self.calibration, self.urdf, self.bounds,
                            kinematics_factory=LinearModel, tool_frame_name="")

    def test_short_xyz_plan_has_ordered_bounded_waypoints(self):
        solver = self.make()
        start = (2000, 2000, 2000, 2000, 2000, 1700)
        initial = np.asarray(solver.predicted_xyz(start))
        target = tuple(initial + np.array((0.05233, 0.01447, 0.0)))
        pairs = []
        complete = []
        plan = solver.plan_xyz(
            start, target, validate_waypoint=lambda a, b: pairs.append((a, b)),
            validate_plan=lambda first, frames: complete.append((first, frames)),
        )
        self.assertGreater(len(plan.waypoints), 1)
        self.assertEqual(len(pairs), len(plan.waypoints))
        self.assertEqual(complete, [(start, tuple(step.target_ticks for step in plan.waypoints))])
        self.assertEqual(plan.start_ticks, start)
        self.assertEqual(plan.target_ticks, plan.waypoints[-1].target_ticks)
        self.assertEqual(plan.target_ticks[5], 1700)
        self.assertEqual(pairs[0][0], start)
        for previous, following in pairs:
            self.assertTrue(all(abs(a - b) <= 250 for a, b in zip(previous[:5], following[:5])))
        for first, second in zip(plan.waypoints, plan.waypoints[1:]):
            self.assertEqual(first.target_ticks, second.start_ticks)
            self.assertLessEqual(np.linalg.norm(
                np.asarray(second.target_xyz_m) - np.asarray(second.start_xyz_m)), 0.03 + 1e-12)
        final_error = np.linalg.norm(np.asarray(plan.predicted_xyz_m) - target)
        self.assertGreater(final_error, 0.0)  # Tick rounding leaves a small model residual.
        self.assertLessEqual(final_error, 0.003)

    def test_xyz_planning_rejects_bad_or_unreachable_requests(self):
        solver = self.make()
        start = (2000,) * 6
        for bad in ((float("nan"), 0, 0), (True, 0, 0), (0, 0)):
            with self.subTest(bad=bad), self.assertRaises(IKError):
                solver.solve_xyz(start, bad)
        current = np.asarray(solver.predicted_xyz(start))
        with self.assertRaisesRegex(IKError, "step exceeds"):
            solver.solve_xyz(start, tuple(current + (0.04, 0, 0)))
        with self.assertRaisesRegex(IKError, "path exceeds"):
            solver.plan_xyz(start, tuple(current + (0.20, 0, 0)))
        with self.assertRaisesRegex(IKError, "more waypoints"):
            solver.plan_xyz(start, tuple(current + (0.10, 0, 0)), max_waypoints=2)
        with self.assertRaisesRegex(IKError, "Waypoint 1 rejected"):
            solver.plan_xyz(start, tuple(current + (0.01, 0, 0)),
                            validate_waypoint=lambda _a, _b: (_ for _ in ()).throw(ValueError("policy")))
        with self.assertRaisesRegex(IKError, "Candidate path rejected"):
            solver.plan_xyz(start, tuple(current + (0.01, 0, 0)),
                            validate_plan=lambda _a, _frames: (_ for _ in ()).throw(ValueError("whole path")))
        seen_noop = []
        zero = solver.plan_xyz(start, tuple(current),
                               validate_plan=lambda first, frames: seen_noop.append((first, frames)))
        self.assertEqual(zero.waypoints, ())
        self.assertEqual(seen_noop, [(start, ())])
        with self.assertRaisesRegex(IKError, "residual"):
            self.make(StuckModel).solve_xyz(start, tuple(current + (0.01, 0, 0)))
        with self.assertRaisesRegex(IKError, "outside"):
            solver.plan_xyz((2990, 2000, 2000, 2000, 2000, 2000),
                            tuple(np.asarray(solver.predicted_xyz((2990, 2000, 2000, 2000, 2000, 2000)))
                                  + (0.01, 0, 0)))

    def test_xyz_limit_parameters_are_finite_and_explicit(self):
        solver = self.make()
        start = (2000,) * 6
        point = solver.predicted_xyz(start)
        for options in ({"max_step_m": float("nan")}, {"max_residual_mm": float("nan")},
                        {"max_joint_step_ticks": True}, {"orientation_weight": True}):
            with self.subTest(options=options), self.assertRaises(IKError):
                solver.solve_xyz(start, point, **options)
        data = json.loads(self.calibration.read_text())
        data["shoulder_pan"]["range_min"] = 3500
        self.calibration.write_text(json.dumps(data))
        with self.assertRaisesRegex(IKError, "calibration ranges"):
            self.make()


if __name__ == "__main__":
    unittest.main()
