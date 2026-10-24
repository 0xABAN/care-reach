"""Command-line options and plan output shared by the motion routines."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import Sequence

from care_reach.robot import CALIBRATION_PATH, JOINT_NAMES, JOINT_SETTINGS_PATH
from care_reach.vision.camera import CAMERA_SERVICE_URL
from care_reach.vision.detector import Detector


PORT_HELP = "URT-2 serial port; detected by its USB ID when omitted"

DETECTOR_URL_ENV = "CARE_REACH_DETECTOR_URL"
DETECTOR_TOKEN_ENV = "CARE_REACH_DETECTOR_TOKEN"


def planning_parser(description: str) -> argparse.ArgumentParser:
    """Build a parser with the options every planned routine shares.

    Planning is offline by default; only ``--execute`` opens the servo bus.
    """
    parser = argparse.ArgumentParser(description=description)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="plan only (default)")
    mode.add_argument("--execute", action="store_true", help="move the connected arm")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--port", help=PORT_HELP)
    parser.add_argument("--settings", type=Path, default=JOINT_SETTINGS_PATH, help="joint settings JSON")
    parser.add_argument("--calibration", type=Path, default=CALIBRATION_PATH, help="LeRobot calibration JSON")
    parser.add_argument("--csv", type=Path, help="save planned target ticks")
    return parser


def add_vision_options(parser: argparse.ArgumentParser) -> None:
    """Add the object name plus the wrist camera and detector options."""
    parser.add_argument("name", help='object to look for, such as "coke can"')
    parser.add_argument("--camera", default=CAMERA_SERVICE_URL, help="wrist camera service (ustreamer) URL")
    parser.add_argument("--detector", default=os.environ.get(DETECTOR_URL_ENV),
                        help=f"Grounding DINO server URL; defaults to ${DETECTOR_URL_ENV}")


def open_detector(parser: argparse.ArgumentParser, url: str | None) -> Detector:
    """Build the detector client, or stop with a usage error if it isn't configured."""
    token = os.environ.get(DETECTOR_TOKEN_ENV)
    if not url:
        parser.error(f"set --detector or ${DETECTOR_URL_ENV}")
    if not token:
        parser.error(f"set ${DETECTOR_TOKEN_ENV} to the detector's token")
    return Detector(url, token)


def write_plan_csv(path: Path, frames: Sequence[Sequence[int]], fps: int) -> None:
    """Save commanded target ticks; they are never measured positions."""
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", *(f"target_{name}_ticks" for name in JOINT_NAMES)))
        for index, frame in enumerate(frames):
            writer.writerow((f"{index / fps:.3f}", *frame))
