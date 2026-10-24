"""Offline checks for loading, saving, and using the pick config."""

import json
import math
from pathlib import Path
import tempfile
import unittest

from motion_core import JOINT_NAMES
from pick.config import PickConfig


CONFIG = PickConfig(
    survey_pose=(2010, 1500, 2600, 2900, 2000, 1800),
    lens_height_m=0.2,
    jaw_height_m=0.12,
    aim_px=(612.0, 540.5),
)


class PickConfigTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "pick_config.json"

    def test_round_trip_keeps_directions_added_later(self):
        CONFIG.save(self.path)
        self.assertEqual(PickConfig.load(self.path), CONFIG)

        checked = PickConfig.load(self.path).with_directions("-y", "+x")
        checked.save(self.path)
        record = json.loads(self.path.read_text())
        self.assertEqual(record["survey_pose_ticks"]["shoulder_pan"], 2010)
        self.assertEqual(set(record["survey_pose_ticks"]), set(JOINT_NAMES))
        self.assertEqual(record["jaw_height_m"], 0.12)
        self.assertEqual(PickConfig.load(self.path), checked)

    def test_scale_comes_from_the_lens_spec_and_lens_height(self):
        # 102 degrees across 1280 pixels: 20 cm up, the picture spans
        # 2 * 0.2 * tan(51 degrees) meters.
        expected = 2 * 0.2 * math.tan(math.radians(51)) / 1280
        self.assertAlmostEqual(CONFIG.meters_per_pixel, expected)

    def test_picture_offset_maps_onto_the_checked_gripper_axes(self):
        config = CONFIG.with_directions("-y", "+x")
        step = 100 * config.meters_per_pixel

        dx, dy = config.offset_from_jaws((612.0 + 100, 540.5))
        self.assertAlmostEqual(dx, 0.0)
        self.assertAlmostEqual(dy, -step)

        dx, dy = config.offset_from_jaws((612.0, 540.5 + 100))
        self.assertAlmostEqual(dx, step)
        self.assertAlmostEqual(dy, 0.0)

    def test_offset_needs_the_direction_check_first(self):
        with self.assertRaisesRegex(ValueError, "pick_setup.py direction"):
            CONFIG.offset_from_jaws((0.0, 0.0))

    def test_bad_values_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "Lens height"):
            PickConfig((2000,) * 6, 2.0, 0.1, (600.0, 400.0))
        with self.assertRaisesRegex(ValueError, "Jaw height"):
            PickConfig((2000,) * 6, 0.2, 0.0, (600.0, 400.0))
        with self.assertRaisesRegex(ValueError, "outside"):
            PickConfig((2000,) * 6, 0.2, 0.1, (1300.0, 400.0))
        with self.assertRaisesRegex(ValueError, "different gripper axes"):
            CONFIG.with_directions("+x", "-x")
        with self.assertRaisesRegex(ValueError, "one of"):
            CONFIG.with_directions("+z", "+x")

        self.path.write_text(json.dumps({"robot_id": "someone_else"}))
        with self.assertRaisesRegex(ValueError, "different robot"):
            PickConfig.load(self.path)


if __name__ == "__main__":
    unittest.main()
