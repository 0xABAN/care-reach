"""WristCamera against a fake ustreamer snapshot endpoint and a fake clock."""

from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import unittest

from care_reach.vision.camera import WristCamera


class FakeClock:
    def __init__(self, now):
        self.now = now

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeUstreamer:
    """Serve queued (grab timestamp, width, height, body) pictures at /snapshot."""

    def __init__(self, pictures):
        self.pictures = list(pictures)
        self.requests = 0
        fake = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                fake.requests += 1
                taken, width, height, body = fake.pictures.pop(0) if len(fake.pictures) > 1 else fake.pictures[0]
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("X-UStreamer-Grab-Timestamp", f"{taken:.6f}")
                self.send_header("X-UStreamer-Width", str(width))
                self.send_header("X-UStreamer-Height", str(height))
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        self.server = HTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def close(self):
        self.server.shutdown()
        self.server.server_close()


class WristCameraTests(unittest.TestCase):
    def serve(self, pictures):
        fake = FakeUstreamer(pictures)
        self.addCleanup(fake.close)
        return fake

    def test_skips_pictures_taken_before_it_asked(self):
        fake = self.serve([(99.0, 1280, 720, b"old"), (99.5, 1280, 720, b"old"), (100.2, 1280, 720, b"new")])
        self.assertEqual(WristCamera(fake.url, FakeClock(100.0)).grab(), b"new")
        self.assertEqual(fake.requests, 3)

    def test_gives_up_when_no_fresh_picture_arrives(self):
        fake = self.serve([(50.0, 1280, 720, b"frozen")])
        with self.assertRaisesRegex(RuntimeError, "no fresh picture"):
            WristCamera(fake.url, FakeClock(100.0)).grab()

    def test_rejects_a_lower_resolution(self):
        fake = self.serve([(101.0, 640, 480, b"small")])
        with self.assertRaisesRegex(RuntimeError, "640x480"):
            WristCamera(fake.url, FakeClock(100.0)).grab()


if __name__ == "__main__":
    unittest.main()
