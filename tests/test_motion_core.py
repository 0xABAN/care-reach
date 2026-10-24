"""Offline safety checks for motion planning; these tests never open the bus."""

import json
import tempfile
import unittest
from pathlib import Path

from motion_core import (
    JOINT_NAMES, SafetyBounds, UnsafePoseError, action_to_ticks,
    plan_blended_waypoints, plan_waypoints, quintic_segment, splice_action_chunk,
    validate_pose,
)


class MotionCoreTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        directory = Path(temporary.name)
        ranges = [(1096, 2617), (883, 3243), (861, 3007), (843, 3130), (0, 4095), (1317, 2762)]
        self.calibration = {
            name: {"id": index, "range_min": lo, "range_max": hi}
            for index, (name, (lo, hi)) in enumerate(zip(JOINT_NAMES, ranges), start=1)
        }
        settings = {
            str(index): {"lower": lo, "upper": hi, "scale": 1}
            for index, (lo, hi) in enumerate(
                [(1116, 2597), (903, 3223), (881, 2987), (863, 3110), (None, None), (1317, 2742)],
                start=1,
            )
        }
        calibration_path = directory / "calibration.json"
        settings_path = directory / "settings.json"
        calibration_path.write_text(json.dumps(self.calibration))
        settings_path.write_text(json.dumps(settings))
        self.bounds = SafetyBounds.from_files(settings_path, calibration_path)

    def test_panel_and_provisional_stops_are_enforced(self):
        self.assertEqual(self.bounds.lower, (1116, 903, 881, 863, 0, 1340))
        self.assertEqual(self.bounds.upper, (2597, 3223, 2987, 3110, 3300, 2742))
        home = (2000,) * 6
        self.assertEqual(validate_pose(home, self.bounds), home)
        with self.assertRaisesRegex(UnsafePoseError, "motor 5"):
            validate_pose((2000, 2000, 2000, 2000, 3358, 2000), self.bounds)
        with self.assertRaisesRegex(UnsafePoseError, "motor 6"):
            validate_pose((2000, 2000, 2000, 2000, 2000, 1317), self.bounds)

    def test_dataset_actions_convert_and_reject_outside_real_arm(self):
        center = action_to_ticks((0, 0, 0, 0, 0, 50), self.calibration, self.bounds)
        self.assertEqual(center, (1856, 2063, 1934, 1986, 2048, 2040))
        with self.assertRaisesRegex(UnsafePoseError, "motor 5"):
            action_to_ticks((0, 0, 0, 0, 120, 50), self.calibration, self.bounds)
        with self.assertRaisesRegex(UnsafePoseError, "motor 6"):
            action_to_ticks((0, 0, 0, 0, 0, 0), self.calibration, self.bounds)
        with self.assertRaisesRegex(UnsafePoseError, "gripper action"):
            action_to_ticks((0, 0, 0, 0, 0, 101), self.calibration, self.bounds)

    def test_synchronized_quintic_respects_limits_and_has_gentle_ends(self):
        start = (2000,) * 6
        end = (2100, 1900, 2000, 2000, 2000, 2000)
        frames = quintic_segment(start, end, (200,) * 6, (500,) * 6, fps=20)
        self.assertEqual(frames[0], start)
        self.assertEqual(frames[-1], end)
        self.assertGreaterEqual((len(frames) - 1) / 20, (5.774 * 100 / 500) ** 0.5)
        self.assertTrue(all(a[0] <= b[0] and a[1] >= b[1] for a, b in zip(frames, frames[1:])))
        self.assertTrue(all(frame[2:] == (2000, 2000, 2000, 2000) for frame in frames))
        gaps = [b[0] - a[0] for a, b in zip(frames, frames[1:])]
        self.assertLess(gaps[0], max(gaps))
        self.assertLess(gaps[-1], max(gaps))

    def test_waypoint_planner_rejects_unsafe_destination(self):
        home = (2000,) * 6
        safe = (2050, 2000, 2000, 2000, 2050, 2000)
        frames = plan_waypoints(home, [safe, home], self.bounds, [200] * 6, [500] * 6)
        self.assertEqual(frames[0], home)
        self.assertEqual(frames[-1], home)
        self.assertIn(safe, frames)
        self.assertTrue(all(validate_pose(frame, self.bounds) == frame for frame in frames))
        with self.assertRaisesRegex(UnsafePoseError, "motor 5"):
            plan_waypoints(home, [safe, (2000, 2000, 2000, 2000, 3358, 2000)],
                           self.bounds, [200] * 6, [500] * 6)

    def test_blended_path_passes_through_waypoints_without_a_full_stop(self):
        home = (2000,) * 6
        first = (2100, 2050, 2000, 2000, 2000, 2000)
        second = (2200, 2100, 2000, 2000, 2000, 2000)
        frames = plan_blended_waypoints(
            home, [first, second, home], self.bounds,
            [(100,) * 6, (200,) * 6, (100,) * 6],
            [(200,) * 6] * 3, fps=20,
        )
        self.assertEqual((frames[0], frames[-1]), (home, home))
        self.assertTrue(all(validate_pose(frame, self.bounds) == frame for frame in frames))
        index = frames.index(first)
        self.assertGreater(frames[index][0] - frames[index - 1][0], 0)
        self.assertGreater(frames[index + 1][0] - frames[index][0], 0)
        self.assertLessEqual(max(abs(b[0] - a[0]) for a, b in zip(frames, frames[1:])), 6)

    def test_blended_path_varies_leg_speeds_and_stops_on_request(self):
        home = (2000,) * 6
        first = (2200, 2100, 2000, 2000, 2000, 2000)
        second = (2400, 2200, 2000, 2000, 2000, 2000)
        frames = plan_blended_waypoints(
            home, [first, second, home], self.bounds,
            [(80,) * 6, (240,) * 6, (80,) * 6],
            [(200,) * 6, (500,) * 6, (200,) * 6], fps=20,
            stop_indices=(0,),
        )
        i, j = frames.index(first), frames.index(second)
        self.assertGreater(i, j - i)
        self.assertLessEqual(abs(frames[i][0] - frames[i - 1][0]), 1)
        self.assertLessEqual(abs(frames[i + 1][0] - frames[i][0]), 1)
        with self.assertRaisesRegex(UnsafePoseError, "motor 5"):
            plan_blended_waypoints(home, [(*home[:4], 3358, 2000)], self.bounds,
                                   (100,) * 6, (200,) * 6)

    def test_policy_chunk_splice_discards_past_and_blends_future(self):
        home = (2000,) * 6
        old_future = [(2010,) * 6] * 3
        prediction = [(2200,) * 6, (2210,) * 6, (2220,) * 6, (2230,) * 6]
        blended = splice_action_chunk(home, old_future, prediction, 10, 12,
                                      self.bounds, blend_steps=2)
        self.assertEqual(blended, [(2080,) * 6, (2157,) * 6])
        self.assertIsNone(splice_action_chunk(home, old_future, prediction, 10, 14,
                                              self.bounds))

    def test_policy_chunk_splice_rejects_any_unsafe_prediction(self):
        home = (2000,) * 6
        unsafe = (2000, 2000, 2000, 2000, 3358, 2000)
        with self.assertRaisesRegex(UnsafePoseError, "motor 5"):
            splice_action_chunk(home, [], [home, unsafe], 0, 0, self.bounds)
        with self.assertRaisesRegex(ValueError, "future"):
            splice_action_chunk(home, [], [home], 2, 1, self.bounds)


if __name__ == "__main__":
    unittest.main()
