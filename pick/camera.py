"""Fresh pictures from the Pi's camera services.

Only one program can open a camera device. On the Pi, ustreamer serves each
camera for the dashboard's live views: the wrist camera (InnoMaker U20CAM-720P)
on port 8766, and the shoulder camera (w200 webcam) on port 8768. This module
reads their MJPEG streams instead of opening the cameras.

Why the stream and not ustreamer's ``/snapshot``: with nobody watching,
``--slowdown`` reads the camera about once a second, and each read takes the
oldest of the frames queued in the camera driver. A snapshot was then about
2.5 s old, although its ``X-UStreamer-Grab-Timestamp`` (when ustreamer read
the frame, not when the camera took it) looked fresh. With the stream open,
ustreamer reads at the full 30 fps; a brightness change made with
``v4l2-ctl`` showed up in the 4th frame after it.
"""

from __future__ import annotations

import threading
from typing import Any, Callable
import urllib.request


SERVICE_URLS = {"wrist": "http://127.0.0.1:8766", "shoulder": "http://127.0.0.1:8768"}

# A frame this many frames after the request was exposed after it: twice
# the measured 3-frame lag, about 0.2 s at 30 fps.
FRESH_FRAMES = 6

# The stream normally delivers a frame every 33 ms. A read with no data for
# this long ends the stream.
STALL_TIMEOUT_S = 3.0

# A dim scene can slow the camera's frame rate; grab() still gives up here.
GRAB_TIMEOUT_S = 10.0


class CameraError(RuntimeError):
    """The camera service is offline, set to the wrong size, or stopped sending pictures."""


class Camera:
    """Fresh pictures from one camera service running on this same machine.

    Open it with ``with`` for the duration of a pick: that keeps the live
    stream running, so the camera service reads every frame as it arrives.

        with Camera(SERVICE_URLS["wrist"], (1280, 720)) as camera:
            jpeg = camera.grab()

    Opening refuses a camera that is offline or, when ``size`` is given, set
    to any other picture size than the one it was calibrated at.
    """

    def __init__(self, url: str, size: tuple[int, int] | None = None,
                 opener: Callable[..., Any] = urllib.request.urlopen) -> None:
        self.url = url.rstrip("/")
        self.size = None if size is None else tuple(size)
        self.opener = opener

        self._stream: Any | None = None
        self._reader: threading.Thread | None = None

        # Guarded by _changed: the newest frame, how many frames have arrived,
        # and why the stream ended, once it has.
        self._changed = threading.Condition()
        self._latest: bytes | None = None
        self._count = 0
        self._ended: str | None = None

    def __enter__(self) -> Camera:
        self._check_service()
        self._stream = self.opener(f"{self.url}/stream", timeout=STALL_TIMEOUT_S)
        self._reader = threading.Thread(target=self._read_frames, name=f"camera {self.url}", daemon=True)
        self._reader.start()
        return self

    def __exit__(self, *exc_info: object) -> None:
        with self._changed:
            self._ended = self._ended or f"The camera stream at {self.url} is closed"
        self._stream.close()
        self._reader.join(timeout=STALL_TIMEOUT_S)

    def grab(self) -> bytes:
        """Return a JPEG the camera took after this call started.

        Call it once the arm has stopped. It drops frames that may predate
        the call and returns the FRESH_FRAMES-th new one.
        """
        if self._reader is None:
            raise CameraError("Open the camera with `with Camera(...) as camera:` first")

        with self._changed:
            wanted = self._count + FRESH_FRAMES
            arrived = self._changed.wait_for(lambda: self._count >= wanted or self._ended, timeout=GRAB_TIMEOUT_S)
            if self._ended:
                raise CameraError(self._ended)
            if not arrived:
                raise CameraError(f"The camera stream at {self.url} stopped sending pictures")
            return self._latest

    def _check_service(self) -> None:
        with self.opener(f"{self.url}/snapshot", timeout=STALL_TIMEOUT_S) as response:
            online = response.headers["X-UStreamer-Online"] == "true"
            size = (int(response.headers["X-UStreamer-Width"]), int(response.headers["X-UStreamer-Height"]))

        if not online:
            raise CameraError(f"The camera service at {self.url} is running, but its camera is offline")
        if self.size is not None and size != self.size:
            raise CameraError(f"The camera at {self.url} sends {size[0]}x{size[1]} pictures; "
                               f"it was calibrated at {self.size[0]}x{self.size[1]}")

    def _read_frames(self) -> None:
        """Keep the newest frame from the multipart stream until it ends or is closed.

        Each part is a boundary line, headers including Content-Length, a
        blank line, and the JPEG.
        """
        try:
            while True:
                line = self._stream.readline()
                if not line:
                    raise EOFError("the camera service closed the stream")
                if not line.lower().startswith(b"content-length:"):
                    continue

                length = int(line.split(b":", 1)[1])
                while self._stream.readline().strip():
                    pass

                jpeg = self._stream.read(length)
                if len(jpeg) != length:
                    raise EOFError("the stream ended mid-picture")

                with self._changed:
                    self._latest = jpeg
                    self._count += 1
                    self._changed.notify_all()
        except Exception as exc:
            with self._changed:
                # Closing the stream on exit also lands here; keep that reason.
                self._ended = self._ended or f"The camera stream at {self.url} failed: {exc}"
                self._changed.notify_all()
