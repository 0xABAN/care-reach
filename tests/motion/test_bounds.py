"""Offline safety-bound checks; these tests never open the bus."""

import unittest

from care_reach.motion.bounds import UnsafePoseError, validate_pose
from tests.support import ArmFilesTestCase


class BoundsTests(ArmFilesTestCase):
    def test_panel_and_provisional_stops_are_enforced(self):
        self.assertEqual(self.bounds.lower, (1116, 903, 881, 863, 0, 1340))
        self.assertEqual(self.bounds.upper, (2597, 3223, 2987, 3110, 3300, 2742))
        home = (2000,) * 6
        self.assertEqual(validate_pose(home, self.bounds), home)
        with self.assertRaisesRegex(UnsafePoseError, "motor 5"):
            validate_pose((2000, 2000, 2000, 2000, 3358, 2000), self.bounds)
        with self.assertRaisesRegex(UnsafePoseError, "motor 6"):
            validate_pose((2000, 2000, 2000, 2000, 2000, 1317), self.bounds)


if __name__ == "__main__":
    unittest.main()
