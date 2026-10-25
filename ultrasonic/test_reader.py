"""Offline checks for Arduino serial selection and range parsing."""

import os
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from reader import Ranger, find_arduino_port, parse_measurement


class RangerTest(unittest.TestCase):
    def test_reader_accepts_only_bounded_centimeters(self):
        self.assertEqual(parse_measurement(b"123.4\r\n"), 123.4)
        self.assertIsNone(parse_measurement(b"NA\n"))
        for frame in (b"0", b"401", b"nan", b"inf", b"1;rm -rf /", b""):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                parse_measurement(frame)

    def test_discovers_arduino_without_urt_controller(self):
        ports = [
            SimpleNamespace(vid=0x1A86, pid=0x55D3, device="/dev/ttyACM0",
                            description="USB Single Serial"),
            SimpleNamespace(vid=0x2341, pid=0x0043, device="/dev/ttyACM1",
                            description="Arduino Uno"),
        ]
        with patch.dict(os.environ, {}, clear=True), patch("reader.list_ports.comports", return_value=ports):
            self.assertEqual(find_arduino_port(), "/dev/ttyACM1")
        with patch.dict(os.environ, {"CARE_REACH_ULTRASONIC_PORT": "/dev/ttyACM0"}), \
             patch("reader.list_ports.comports", return_value=ports):
            with self.assertRaisesRegex(ValueError, "URT-2"):
                find_arduino_port()

    def test_reports_stale_and_never_presents_old_reading_as_live(self):
        ranger = Ranger()
        self.assertEqual(ranger.snapshot()["status"], "waiting")
        ranger.connected = True
        ranger.accept(b"20.0\n")
        ranger.accept(b"200.0\n")
        ranger.accept(b"21.0\n")
        self.assertEqual(ranger.snapshot()["distance_cm"], 21.0)
        ranger.last_valid_monotonic = time.monotonic() - 2
        state = ranger.snapshot()
        self.assertEqual(state["status"], "stale")
        self.assertIsNone(state["distance_cm"])


if __name__ == "__main__":
    unittest.main()
