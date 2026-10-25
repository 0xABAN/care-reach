"""The movement mode route accepts only an explicit boolean switch."""

import io
import json
import unittest

from server import handler_for


class FakeArm:
    def __init__(self):
        self.enabled = False
        self.calls = []

    def set_single_motor_mode(self, enabled):
        self.calls.append(enabled)
        self.enabled = enabled
        return {"single_motor_mode": enabled}


class SingleMotorRouteTests(unittest.TestCase):
    def setUp(self):
        self.arm = FakeArm()
        self.handler_class = handler_for(self.arm, 8765)

    def post(self, body):
        payload = json.dumps(body).encode()
        handler = self.handler_class.__new__(self.handler_class)
        handler.path = "/api/movement/mode"
        handler.headers = {"Host": "pi.local:8765", "Content-Type": "application/json",
                           "Content-Length": str(len(payload))}
        handler.client_address = ("192.168.1.23", 43210)
        handler.connection = type("Connection", (), {"getsockname": lambda self: ("192.168.1.42", 8765)})()
        handler.rfile = io.BytesIO(payload)
        handler.wfile = io.BytesIO()
        handler._headers_buffer = []
        handler.requestline = "POST /api/movement/mode HTTP/1.1"
        handler.request_version = "HTTP/1.1"
        handler.server_version = "Fake/1.0"
        handler.sys_version = "Python/3"
        handler.protocol_version = "HTTP/1.1"
        handler.do_POST()
        headers, _, response = handler.wfile.getvalue().partition(b"\r\n\r\n")
        return int(headers.split(b" ", 2)[1]), json.loads(response)

    def test_changes_mode(self):
        self.assertEqual(self.post({"single_motor_mode": True}),
                         (200, {"single_motor_mode": True}))
        self.assertEqual(self.post({"single_motor_mode": False}),
                         (200, {"single_motor_mode": False}))
        self.assertEqual(self.arm.calls, [True, False])

    def test_rejects_implicit_or_extra_values(self):
        for body in ({}, {"single_motor_mode": 1}, {"single_motor_mode": "true"},
                     {"single_motor_mode": True, "motor": 1}):
            with self.subTest(body=body):
                status, _ = self.post(body)
                self.assertEqual(status, 400)
        self.assertEqual(self.arm.calls, [])


if __name__ == "__main__":
    unittest.main()
