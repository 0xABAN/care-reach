"""Offline checks for the direction check's decision; no camera or arm."""

import unittest

from pick_setup import image_directions


EXPECTED_PX = 52.0


class ImageDirectionTests(unittest.TestCase):
    def test_axes_and_signs_follow_how_the_object_slid(self):
        # +x slid the object down the picture, so objects lower down lie toward
        # -x. +y slid it left, so objects to the right lie toward +y.
        self.assertEqual(image_directions((2.0, 60.0), (-58.0, 3.0), EXPECTED_PX), ("+y", "-x"))
        self.assertEqual(image_directions((-55.0, 0.0), (0.0, -50.0), EXPECTED_PX), ("+x", "+y"))

    def test_scale_far_from_the_lens_estimate_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "lens height"):
            image_directions((20.0, 0.0), (0.0, 50.0), EXPECTED_PX)
        with self.assertRaisesRegex(ValueError, "lens height"):
            image_directions((0.0, 0.0), (0.0, 50.0), EXPECTED_PX)

    def test_crooked_camera_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "diagonally"):
            image_directions((45.0, 40.0), (0.0, 55.0), EXPECTED_PX)

    def test_both_nudges_along_one_picture_axis_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "same picture axis"):
            image_directions((55.0, 0.0), (-55.0, 0.0), EXPECTED_PX)


if __name__ == "__main__":
    unittest.main()
