"""The wrist camera: an InnoMaker U20CAM-720P on the SO-101 wrist mount.

Only one program can open the camera device. On the Pi that is ustreamer, run
by ``care-reach-camera.service`` for the live view. It also serves single JPEG
pictures, so this module asks it for those instead of opening the camera.
"""

from __future__ import annotations

import math
import time
from typing import Any
import urllib.request


# Lens spec for the U20CAM-720P at its full MJPEG resolution.
WIDTH_PX = 1280
HEIGHT_PX = 720
HORIZONTAL_FOV_DEG = 102.0

# The focal length a distortion-free lens with that field of view would have.
# This wide lens bends the picture, so the true value differs, most near the
# middle. Good enough to estimate distances that each new look corrects.
FOCAL_LENGTH_PX = (WIDTH_PX / 2) / math.tan(math.radians(HORIZONTAL_FOV_DEG / 2))

CAMERA_SERVICE_URL = "http://127.0.0.1:8766"

# With no live viewer, ustreamer slows to about one picture per second.
FRESH_PICTURE_TIMEOUT_S = 2.0
RETRY_S = 0.05


class WristCamera:
    """Take single pictures from the camera service running on this same machine."""

    def __init__(self, url: str = CAMERA_SERVICE_URL, clock: Any = time) -> None:
        self.url = url.rstrip("/")
        self.clock = clock

    def grab(self) -> bytes:
        """Return a JPEG taken after this call started. Call it once the arm has settled.

        ustreamer stamps each picture with this machine's monotonic clock, so
        an older picture, from before the last move, is never returned.
        """
        asked = self.clock.monotonic()
        while True:
            with urllib.request.urlopen(f"{self.url}/snapshot", timeout=FRESH_PICTURE_TIMEOUT_S) as response:
                taken = float(response.headers["X-UStreamer-Grab-Timestamp"])
                width = int(response.headers["X-UStreamer-Width"])
                height = int(response.headers["X-UStreamer-Height"])
                jpeg = response.read()

            if taken >= asked:
                break
            if self.clock.monotonic() - asked > FRESH_PICTURE_TIMEOUT_S:
                raise RuntimeError("The camera service has no fresh picture; is the camera online?")
            self.clock.sleep(RETRY_S)

        # The aim point and pixel scale assume the full-resolution picture.
        if (width, height) != (WIDTH_PX, HEIGHT_PX):
            raise RuntimeError(f"Wrist camera picture is {width}x{height}, expected {WIDTH_PX}x{HEIGHT_PX}")
        return jpeg
