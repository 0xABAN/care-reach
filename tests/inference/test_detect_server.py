"""Request handling for the detect server, with a fake model in place of Grounding DINO."""

import base64
import json
from http.server import HTTPServer
import threading
import unittest
import urllib.error
import urllib.request

from care_reach.inference.detect_server import handler_for


class DetectServerTests(unittest.TestCase):
    def setUp(self):
        self.calls = []

        def fake_detect(jpeg, prompt):
            self.calls.append((jpeg, prompt))
            return [{"box": [1.0, 2.0, 3.0, 4.0], "score": 0.9}]

        self.server = HTTPServer(("127.0.0.1", 0), handler_for(fake_detect, "secret"))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def post(self, body, token="secret"):
        request = urllib.request.Request(
            f"{self.url}/detect",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {token}"},
        )
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            return error.code, json.load(error)

    def test_detect_returns_the_model_boxes(self):
        image = base64.b64encode(b"jpeg bytes").decode()
        status, body = self.post({"image": image, "prompt": "coke can."})
        self.assertEqual(status, 200)
        self.assertEqual(body, {"detections": [{"box": [1.0, 2.0, 3.0, 4.0], "score": 0.9}]})
        self.assertEqual(self.calls, [(b"jpeg bytes", "coke can.")])

    def test_wrong_token_and_bad_requests_never_reach_the_model(self):
        image = base64.b64encode(b"jpeg bytes").decode()
        self.assertEqual(self.post({"image": image, "prompt": "cup."}, token="guess")[0], 401)
        self.assertEqual(self.post({"image": "not base64!", "prompt": "cup."})[0], 400)
        self.assertEqual(self.post({"image": image, "prompt": ""})[0], 400)
        self.assertEqual(self.post({"prompt": "cup."})[0], 400)
        self.assertEqual(self.calls, [])

    def test_health(self):
        with urllib.request.urlopen(f"{self.url}/health", timeout=5) as response:
            self.assertEqual(json.load(response), {"ok": True})


if __name__ == "__main__":
    unittest.main()
