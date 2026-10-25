"""Camera against a fake ustreamer that numbers its stream frames."""

import threading
import time
import unittest

from pick.camera import FRESH_FRAMES, Camera


SIZE = (1280, 720)


class FakeSnapshot:
    def __init__(self, width, height, online):
        self.headers = {"X-UStreamer-Online": "true" if online else "false",
                        "X-UStreamer-Width": str(width), "X-UStreamer-Height": str(height)}

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        return False


class FakeStream:
    """A multipart MJPEG stream whose frames are b"0", b"1", ..., one every 2 ms.

    ``frames`` limits how many it sends before it ends; ``fail`` makes the
    read after the last frame raise, as a socket timeout does.
    """

    def __init__(self, frames=None, fail=None):
        self.frames = frames
        self.fail = fail
        self.sent = 0
        self.closed = threading.Event()
        self.pending = []

    def readline(self):
        if not self.pending:
            if self.closed.is_set():
                raise ValueError("I/O operation on closed file")
            if self.frames is not None and self.sent >= self.frames:
                if self.fail:
                    raise self.fail
                return b""

            time.sleep(0.002)
            body = str(self.sent).encode()
            self.pending = [b"--boundarydonotcross\r\n", b"Content-Type: image/jpeg\r\n",
                            f"Content-Length: {len(body)}\r\n".encode(), b"X-Timestamp: 1.0\r\n", b"\r\n", body]
        return self.pending.pop(0)

    def read(self, length):
        body = self.pending.pop(0)
        self.sent += 1
        return body

    def close(self):
        self.closed.set()


class FakeUstreamer:
    def __init__(self, stream, width=1280, height=720, online=True):
        self.stream = stream
        self.snapshot = FakeSnapshot(width, height, online)

    def open(self, url, timeout):
        return self.stream if url.endswith("/stream") else self.snapshot


class CameraTests(unittest.TestCase):
    def test_returns_a_frame_sent_well_after_it_asked(self):
        stream = FakeStream()
        with Camera("http://pi", SIZE, FakeUstreamer(stream).open) as camera:
            for _ in range(3):
                sent_before = stream.sent
                frame = int(camera.grab())

                # Frame sent_before is the first sent after the call. One older
                # frame may be read but not yet counted, and so count as new.
                self.assertGreaterEqual(frame, sent_before + FRESH_FRAMES - 2)
        self.assertTrue(stream.closed.is_set())

    def test_reports_a_stream_that_ends(self):
        with Camera("http://pi", SIZE, FakeUstreamer(FakeStream(frames=2)).open) as camera:
            with self.assertRaisesRegex(RuntimeError, "closed the stream"):
                camera.grab()

    def test_reports_a_stalled_stream(self):
        stream = FakeStream(frames=2, fail=TimeoutError("timed out"))
        with Camera("http://pi", SIZE, FakeUstreamer(stream).open) as camera:
            with self.assertRaisesRegex(RuntimeError, "failed: timed out"):
                camera.grab()

    def test_refuses_an_offline_camera(self):
        with self.assertRaisesRegex(RuntimeError, "offline"):
            Camera("http://pi", SIZE, FakeUstreamer(FakeStream(), online=False).open).__enter__()

    def test_refuses_a_lower_resolution(self):
        with self.assertRaisesRegex(RuntimeError, "640x480 pictures; it was calibrated at 1280x720"):
            Camera("http://pi", SIZE, FakeUstreamer(FakeStream(), width=640, height=480).open).__enter__()

    def test_needs_the_stream_open(self):
        with self.assertRaisesRegex(RuntimeError, "with Camera"):
            Camera("http://pi", SIZE, FakeUstreamer(FakeStream()).open).grab()


if __name__ == "__main__":
    unittest.main()
