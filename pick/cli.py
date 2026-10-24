"""Command-line options shared by `pick_up.py` and `pick_setup.py`."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from arm_runtime import URT2_PORT, ArmSafetyError
from measured_pose_runner import PoseMoveError
from pick.camera import CAMERA_SERVICE_URL
from pick.config import CONFIG_PATH
from pick.detector import AmbiguousDetection, Detector


DETECTOR_URL_ENV = "CARE_REACH_DETECTOR_URL"
DETECTOR_TOKEN_ENV = "CARE_REACH_DETECTOR_TOKEN"

# Expected ways a pick stops early: a safety guard, an unsafe or unsolvable
# move (ValueError, including IKError), an unclear picture, or an unreachable
# camera or detector (OSError). The movers hold the measured pose with torque
# on before raising.
STOP_ERRORS = (ArmSafetyError, PoseMoveError, AmbiguousDetection, ValueError, OSError)


def add_pick_options(parser: argparse.ArgumentParser) -> None:
    """Add the object name, the camera and detector, and the arm's port and config."""
    parser.add_argument("name", help='object to look for, such as "blue box"')
    parser.add_argument("--camera", default=CAMERA_SERVICE_URL, help="wrist camera service (ustreamer) URL")
    parser.add_argument("--detector", default=os.environ.get(DETECTOR_URL_ENV),
                        help=f"Grounding DINO server URL; defaults to ${DETECTOR_URL_ENV}")
    parser.add_argument("--port", default=URT2_PORT, help="URT-2 serial port")
    parser.add_argument("--config", type=Path, default=CONFIG_PATH, help="pick config JSON")


def open_detector(parser: argparse.ArgumentParser, url: str | None) -> Detector:
    """Build the detector client, or stop with a usage error if it isn't configured."""
    token = os.environ.get(DETECTOR_TOKEN_ENV)
    if not url:
        parser.error(f"set --detector or ${DETECTOR_URL_ENV}")
    if not token:
        parser.error(f"set ${DETECTOR_TOKEN_ENV} to the detector's token")
    return Detector(url, token)
