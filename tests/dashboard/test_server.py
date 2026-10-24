"""HTTP and page checks for the dashboard server, against a fake-bus controller."""

import io
import json
import re
import shutil
import subprocess
import unittest
from unittest.mock import patch

from care_reach.dashboard.panel import PanelController
from care_reach.dashboard.server import STATIC_DIR, handler_for
from care_reach.robot import JOINT_NAMES
from tests.dashboard.fakes import FakeBus
from tests.support import ArmFilesTestCase


class HttpHostTests(ArmFilesTestCase):
    def setUp(self):
        super().setUp()
        thread_patch = patch("care_reach.dashboard.panel.threading.Thread")
        thread_patch.start()
        self.addCleanup(thread_patch.stop)
        arm = PanelController("fake", bus_factory=lambda _: FakeBus((2000,) * 6), settings_path=self.settings_path,
                              calibration_path=self.calibration_path, pose_path=self.pose_path)
        self.addCleanup(arm.close)
        self.handler = handler_for(arm, 8765)

    @staticmethod
    def call(handler_class, headers, client_ip="192.168.1.23", local_ip="192.168.1.42"):
        handler = handler_class.__new__(handler_class)
        handler.path = "/api/state"
        handler.headers = headers
        handler.client_address = (client_ip, 43210)
        handler.connection = type("FakeConnection", (), {"getsockname": lambda self: (local_ip, 8765)})()
        handler.wfile = io.BytesIO()
        handler._headers_buffer = []
        handler.requestline = "GET /api/state HTTP/1.1"
        handler.request_version = "HTTP/1.1"
        handler.server_version = "Fake/1.0"
        handler.sys_version = "Python/3"
        handler.protocol_version = "HTTP/1.1"
        handler.do_GET()
        raw = handler.wfile.getvalue()
        header_blob, _, payload = raw.partition(b"\r\n\r\n")
        status = int(header_blob.split(b" ", 2)[1])
        return status, json.loads(payload)

    def test_accepts_pi_local_pi_and_loopback_hosts(self):
        for host in ("pi.local:8765", "pi:8765", "localhost:8765", "127.0.0.1:8765"):
            status, payload = self.call(self.handler, {"Host": host}, client_ip="127.0.0.1")
            self.assertEqual(status, 200, host)
            self.assertIn("motors", payload)

    def test_accepts_the_address_that_accepted_the_connection(self):
        status, payload = self.call(self.handler, {"Host": "192.168.1.42:8765"}, client_ip="192.168.1.23")
        self.assertEqual(status, 200)
        self.assertIn("motors", payload)

    def test_rejects_foreign_hostnames_and_wrong_ports(self):
        cases = (
            ({"Host": "evil.example:8765"}, "192.168.1.23"),
            ({"Host": "192.168.1.23:8765"}, "127.0.0.1"),
            ({"Host": "pi.local:9999"}, "192.168.1.23"),
            ({}, "192.168.1.23"),
        )
        for headers, client_ip in cases:
            status, payload = self.call(self.handler, headers, client_ip=client_ip)
            self.assertEqual(status, 403, headers)
            self.assertEqual(payload, {"error": "Invalid host"})

    def test_origin_must_match_the_accepted_host_exactly(self):
        accepted = (
            ("pi.local:8765", "http://pi.local:8765"),
            ("192.168.1.42:8765", "http://192.168.1.42:8765"),
        )
        for host, origin in accepted:
            status, payload = self.call(self.handler, {"Host": host, "Origin": origin}, client_ip="192.168.1.23")
            self.assertEqual(status, 200, (host, origin))
        rejected = (
            ("pi.local:8765", "http://evil.example:8765"),
            ("pi.local:8765", "https://pi.local:8765"),
            ("pi.local:8765", "http://127.0.0.1:8765"),
            ("pi.local:8765", "http://pi.local:9999"),
        )
        for host, origin in rejected:
            status, payload = self.call(self.handler, {"Host": host, "Origin": origin}, client_ip="192.168.1.23")
            self.assertEqual(status, 403, (host, origin))
            self.assertEqual(payload, {"error": "Invalid origin"})

    @staticmethod
    def post(handler_class, path, body):
        handler = handler_class.__new__(handler_class)
        handler.path = path
        payload = json.dumps(body).encode()
        handler.headers = {
            "Host": "127.0.0.1:8765",
            "Content-Type": "application/json",
            "Content-Length": str(len(payload)),
        }
        handler.rfile = io.BytesIO(payload)
        handler.client_address = ("127.0.0.1", 43210)
        handler.connection = type("FakeConnection", (), {"getsockname": lambda self: ("127.0.0.1", 8765)})()
        handler.wfile = io.BytesIO()
        handler._headers_buffer = []
        handler.requestline = f"POST {path} HTTP/1.1"
        handler.request_version = "HTTP/1.1"
        handler.server_version = "Fake/1.0"
        handler.sys_version = "Python/3"
        handler.protocol_version = "HTTP/1.1"
        handler.do_POST()
        raw = handler.wfile.getvalue()
        header_blob, _, payload = raw.partition(b"\r\n\r\n")
        status = int(header_blob.split(b" ", 2)[1])
        return status, json.loads(payload)

    def test_pose_endpoints_run_and_state_shows_active_pose(self):
        status, payload = self.post(self.handler, "/api/pose/start", {"pose": "home"})
        self.assertEqual(status, 200)
        self.assertEqual(payload["pose"], "home")
        self.assertIn("token", payload)
        status, state = self.call(self.handler, {"Host": "127.0.0.1:8765"}, client_ip="127.0.0.1")
        self.assertEqual(status, 200)
        self.assertTrue(state["pose"]["active"])
        self.assertEqual(state["pose"]["name"], "home")
        self.assertEqual(state["pose"]["target"], dict(zip(JOINT_NAMES, (2000,) * 6, strict=True)))
        status, payload = self.post(self.handler, "/api/pose/stop", {})
        self.assertEqual(status, 200)
        _, state = self.call(self.handler, {"Host": "127.0.0.1:8765"}, client_ip="127.0.0.1")
        self.assertFalse(state["pose"]["active"])

    def test_pose_start_validates_name(self):
        status, payload = self.post(self.handler, "/api/pose/start", {"pose": "coiled"})
        self.assertEqual(status, 400)
        self.assertIn("pose must be", payload["error"])

    def test_dance_endpoint_starts_and_cancels(self):
        status, payload = self.post(self.handler, "/api/pose/start", {"pose": "dance"})
        self.assertEqual(status, 200)
        self.assertEqual(payload["pose"], "dance")
        _, state = self.call(self.handler, {"Host": "127.0.0.1:8765"}, client_ip="127.0.0.1")
        self.assertTrue(state["pose"]["active"])
        self.assertEqual(state["pose"]["name"], "dance")
        status, _ = self.post(self.handler, "/api/pose/stop", {})
        self.assertEqual(status, 200)

    def test_limit_endpoint_removed(self):
        status, payload = self.post(
            self.handler, "/api/limit", {"motor": 1, "bound": "lower", "clear": False}
        )
        self.assertEqual(status, 404)
        self.assertEqual(payload, {"error": "Not found"})


class StaticUiTests(unittest.TestCase):
    PAGE = STATIC_DIR / "index.html"

    def setUp(self):
        self.page = self.PAGE.read_text(encoding="utf-8")

    def test_home_and_fold_buttons_replace_limit_editing(self):
        self.assertIn('id="pose-home"', self.page)
        self.assertIn('id="pose-fold"', self.page)
        self.assertIn('id="pose-dance"', self.page)
        self.assertIn(">Home</button>", self.page)
        self.assertIn(">Fold</button>", self.page)
        self.assertIn(">Dance</button>", self.page)
        self.assertIn("/api/pose/start", self.page)
        self.assertIn("/api/pose/stop", self.page)
        self.assertIn("limit-range", self.page)
        self.assertNotIn("/api/limit", self.page)
        self.assertNotIn("data-bound", self.page)
        self.assertNotIn("Save ${bound}", self.page)

    @unittest.skipUnless(shutil.which("node"), "node is not available")
    def test_inline_javascript_parses(self):
        match = re.search(r"<script>(.*)</script>", self.page, re.S)
        self.assertIsNotNone(match)
        result = subprocess.run(
            ["node", "--check", "-"], input=match.group(1), capture_output=True, text=True
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
