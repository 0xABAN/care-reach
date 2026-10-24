"""Offline checks for converting and splicing model actions."""

import unittest

from care_reach.motion.bounds import UnsafePoseError
from care_reach.policy import action_to_ticks, splice_action_chunk
from tests.support import ArmFilesTestCase


class PolicyActionTests(ArmFilesTestCase):
    def test_dataset_actions_convert_and_reject_outside_real_arm(self):
        center = action_to_ticks((0, 0, 0, 0, 0, 50), self.calibration, self.bounds)
        self.assertEqual(center, (1856, 2063, 1934, 1986, 2048, 2040))
        with self.assertRaisesRegex(UnsafePoseError, "motor 5"):
            action_to_ticks((0, 0, 0, 0, 120, 50), self.calibration, self.bounds)
        with self.assertRaisesRegex(UnsafePoseError, "motor 6"):
            action_to_ticks((0, 0, 0, 0, 0, 0), self.calibration, self.bounds)
        with self.assertRaisesRegex(UnsafePoseError, "gripper action"):
            action_to_ticks((0, 0, 0, 0, 0, 101), self.calibration, self.bounds)

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
