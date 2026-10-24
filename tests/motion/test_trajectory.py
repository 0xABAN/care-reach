"""Offline trajectory-planning checks; these tests never open the bus."""

import unittest

from care_reach.motion.bounds import UnsafePoseError, validate_pose
from care_reach.motion.trajectory import plan_blended_waypoints
from tests.support import ArmFilesTestCase


class BlendedTrajectoryTests(ArmFilesTestCase):
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


if __name__ == "__main__":
    unittest.main()
