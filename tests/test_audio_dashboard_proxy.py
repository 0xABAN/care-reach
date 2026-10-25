"""The dashboard exposes only fixed local voice endpoints, with no robot bus."""

import io
import json
import unittest
from unittest.mock import patch
from urllib.error import URLError

from server import handler_for


class Response(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


class AudioDashboardProxyTests(unittest.TestCase):
    def setUp(self):
        self.handler_class = handler_for(object(), 8765)

    def call(self, path, payload=None, *, origin=None):
        body = json.dumps(payload).encode() if payload is not None else b""
        handler = self.handler_class.__new__(self.handler_class)
        handler.path = path
        handler.headers = {"Host": "pi.local:8765", "Content-Type": "application/json",
                           "Content-Length": str(len(body))}
        if origin is not None:
            handler.headers["Origin"] = origin
        handler.client_address = ("192.168.1.23", 43210)
        handler.connection = type("Connection", (), {
            "getsockname": lambda self: ("192.168.1.42", 8765)})()
        handler.rfile = io.BytesIO(body)
        handler.wfile = io.BytesIO()
        handler._headers_buffer = []
        handler.requestline = f"{'POST' if payload is not None else 'GET'} {path} HTTP/1.1"
        handler.request_version = "HTTP/1.1"
        handler.server_version = "Fake/1.0"
        handler.sys_version = "Python/3"
        handler.protocol_version = "HTTP/1.1"
        if payload is None:
            handler.do_GET()
        else:
            handler.do_POST()
        headers, _, response = handler.wfile.getvalue().partition(b"\r\n\r\n")
        return int(headers.split(b" ", 2)[1]), headers, response

    def test_state_returns_full_voice_panel_data(self):
        voice = {"listening": False, "raw": "Hey Arm dance", "messages": ["heard"],
                 "report": "No trials yet.", "audio_sequence": 3}
        with patch("server.urlopen", return_value=Response(json.dumps(voice).encode())) as upstream:
            status, _, body = self.call("/api/audio/state")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body), voice)
        upstream.assert_called_once_with("http://127.0.0.1:8770/state", timeout=3)

    def test_typed_request_has_fixed_upstream_and_rejects_bad_inputs(self):
        result = {"decision": "accepted", "command": "home", "action": "Moving to home pose"}
        with patch("server.urlopen", return_value=Response(json.dumps(result).encode())) as upstream:
            status, _, body = self.call("/api/audio/text", {"text": "home"})
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body), result)
        request = upstream.call_args.args[0]
        self.assertEqual(request.full_url, "http://127.0.0.1:8770/text")
        self.assertEqual(json.loads(request.data), {"text": "home"})
        with patch("server.urlopen") as upstream:
            for payload in ({}, {"text": "  "}, {"text": "x" * 501},
                            {"text": 1}, {"text": "home", "path": "/other"}):
                with self.subTest(payload=payload):
                    self.assertEqual(self.call("/api/audio/text", payload)[0], 400)
            upstream.assert_not_called()

    def test_diagnostic_and_trial_validate_exact_payloads(self):
        with patch("server.urlopen", side_effect=lambda *_args, **_kwargs: Response(b"OK")) as upstream:
            self.assertEqual(self.call("/api/audio/diagnostic", {})[0], 200)
            self.assertEqual(upstream.call_args.args[0].full_url,
                             "http://127.0.0.1:8770/diagnostic")
            self.assertEqual(self.call("/api/audio/trial", {"expected": "close_gripper"})[0], 200)
            self.assertEqual(upstream.call_args.args[0].full_url,
                             "http://127.0.0.1:8770/trial")
            count = upstream.call_count
            for payload in ({}, {"expected": "dance"}, {"expected": []},
                            {"expected": "none", "extra": 1}):
                with self.subTest(payload=payload):
                    self.assertEqual(self.call("/api/audio/trial", payload)[0], 400)
            self.assertEqual(self.call("/api/audio/diagnostic", {"extra": 1})[0], 400)
            self.assertEqual(upstream.call_count, count)

    def test_saved_audio_is_bounded_and_same_origin(self):
        with patch("server.urlopen", return_value=Response(b"RIFFtest")):
            status, headers, body = self.call("/api/audio/last-utterance.wav?v=2")
        self.assertEqual(status, 200)
        self.assertIn(b"Content-Type: audio/wav", headers)
        self.assertEqual(body, b"RIFFtest")
        with patch("server.urlopen") as upstream:
            self.assertEqual(self.call("/api/audio/state", origin="http://evil.example")[0], 403)
            upstream.assert_not_called()

    def test_voice_failure_is_sanitized(self):
        with patch("server.urlopen", side_effect=URLError("private backend detail")):
            status, _, body = self.call("/api/audio/text", {"text": "home"})
        self.assertEqual(status, 503)
        self.assertNotIn("private backend detail", body.decode())


if __name__ == "__main__":
    unittest.main()
