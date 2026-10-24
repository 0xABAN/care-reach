"""Offline checks for the joint settings file format."""

import json
import tempfile
import unittest
from pathlib import Path

from care_reach.settings import load_joint_settings, save_joint_settings


class JointSettingsTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "joint_settings.json"
        self.settings = {
            str(motor): {"lower": 1000 + motor, "upper": 3000, "scale": 1} for motor in range(1, 7)
        }
        self.settings["5"] = {"lower": None, "upper": None, "scale": 8}

    def test_save_then_load_round_trips_and_leaves_no_temporary_file(self):
        save_joint_settings(self.path, self.settings)
        self.assertEqual(load_joint_settings(self.path), self.settings)
        self.assertFalse(self.path.with_suffix(".json.tmp").exists())

    def test_missing_file_motor_or_field_is_an_error(self):
        with self.assertRaises(FileNotFoundError):
            load_joint_settings(self.path)

        del self.settings["6"]
        self.path.write_text(json.dumps(self.settings), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "each motor"):
            load_joint_settings(self.path)

        self.settings["6"] = {"lower": 1000, "upper": 3000}
        self.path.write_text(json.dumps(self.settings), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Motor 6"):
            load_joint_settings(self.path)

    def test_invalid_limits_and_scales_are_rejected(self):
        for motor, entry, message in (
            ("1", {"lower": 3000, "upper": 1000, "scale": 1}, "Lower limit"),
            ("2", {"lower": -1, "upper": 1000, "scale": 1}, "Invalid limit"),
            ("3", {"lower": 1000, "upper": 3000, "scale": 3}, "Invalid scale"),
        ):
            with self.subTest(motor=motor):
                settings = {**self.settings, motor: entry}
                self.path.write_text(json.dumps(settings), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, message):
                    load_joint_settings(self.path)


if __name__ == "__main__":
    unittest.main()
