"""Offline checks for choosing the detector's match; no server or camera."""

import unittest

from care_reach.vision.detector import AmbiguousDetection, Box, best_match


def box(score, x=100.0):
    return Box(x, 50.0, x + 40.0, 90.0, score)


class BestMatchTests(unittest.TestCase):
    def test_clear_winner_is_chosen(self):
        chosen = best_match([box(0.41, x=10), box(0.78, x=300)], "coke can")
        self.assertEqual(chosen.score, 0.78)
        self.assertEqual(chosen.center, (320.0, 70.0))

    def test_weak_or_missing_matches_are_not_found(self):
        self.assertIsNone(best_match([], "coke can"))

        # The gripper's jaw scored 0.37 for an absent coke can in a live test.
        self.assertIsNone(best_match([box(0.37)], "coke can"))

    def test_two_close_matches_are_too_close_to_call(self):
        with self.assertRaisesRegex(AmbiguousDetection, "too close to call"):
            best_match([box(0.62), box(0.60, x=400)], "coke can")

        # A weak second match below the threshold doesn't count.
        self.assertEqual(best_match([box(0.47), box(0.44, x=400)], "coke can").score, 0.47)


if __name__ == "__main__":
    unittest.main()
