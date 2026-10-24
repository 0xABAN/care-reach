"""Grounding DINO behind a small HTTP API, for the GPU pod.

    CARE_REACH_DETECTOR_TOKEN=<secret> python -m care_reach.inference.detect_server --port 8050

GET /health answers {"ok": true}. POST /detect implements the contract in
care_reach/vision/detector.py. It binds 127.0.0.1 by default; reach it through
an SSH tunnel. Needs torch, transformers, and Pillow, which the pod's LeRobot
environment has.
"""

from __future__ import annotations

import argparse
import base64
import binascii
import hmac
import io
import json
import logging
import os
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Callable


MODEL_ID = "IDEA-Research/grounding-dino-base"

# Return weaker matches too: the client needs them to judge ambiguity.
BOX_THRESHOLD = 0.25
TEXT_THRESHOLD = 0.25

MAX_BODY_BYTES = 8 * 1024 * 1024
MAX_PROMPT_CHARS = 200

Detect = Callable[[bytes, str], list[dict]]


class GroundingDino:
    """Load the model once; each call detects one prompt in one JPEG."""

    def __init__(self) -> None:
        import torch
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        self.torch = torch
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.processor = AutoProcessor.from_pretrained(MODEL_ID)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(MODEL_ID).to(self.device).eval()

    def __call__(self, jpeg: bytes, prompt: str) -> list[dict]:
        from PIL import Image

        image = Image.open(io.BytesIO(jpeg)).convert("RGB")
        inputs = self.processor(images=image, text=prompt, return_tensors="pt").to(self.device)
        with self.torch.inference_mode():
            outputs = self.model(**inputs)

        result = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            threshold=BOX_THRESHOLD,
            text_threshold=TEXT_THRESHOLD,
            target_sizes=[(image.height, image.width)],
        )[0]
        return [
            {"box": [round(value, 1) for value in box.tolist()], "score": round(float(score), 3)}
            for box, score in zip(result["boxes"], result["scores"], strict=True)
        ]


def handler_for(detect: Detect, token: str) -> type[BaseHTTPRequestHandler]:
    expected_auth = f"Bearer {token}".encode()

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code: int, data: dict) -> None:
            body = json.dumps(data).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            if self.path == "/health":
                self.reply(200, {"ok": True})
            else:
                self.reply(404, {"error": "Not found"})

        def do_POST(self) -> None:
            if self.path != "/detect":
                self.reply(404, {"error": "Not found"})
                return
            if not hmac.compare_digest(self.headers.get("Authorization", "").encode(), expected_auth):
                self.reply(401, {"error": "Missing or wrong token"})
                return

            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= MAX_BODY_BYTES:
                    raise ValueError("Invalid request size")

                request = json.loads(self.rfile.read(length))
                prompt = request["prompt"]
                if not isinstance(prompt, str) or not 0 < len(prompt) <= MAX_PROMPT_CHARS:
                    raise ValueError("Prompt must be a short string")
                jpeg = base64.b64decode(request["image"], validate=True)
            except (ValueError, KeyError, TypeError, binascii.Error) as exc:
                self.reply(400, {"error": str(exc)})
                return

            started = time.monotonic()
            detections = detect(jpeg, prompt)
            logging.info("%r: %d detections in %.0f ms", prompt, len(detections), (time.monotonic() - started) * 1000)
            self.reply(200, {"detections": detections})

        def log_message(self, fmt, *args) -> None:
            logging.info("%s %s", self.address_string(), fmt % args)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8050)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

    token = os.environ.get("CARE_REACH_DETECTOR_TOKEN")
    if not token:
        parser.error("set CARE_REACH_DETECTOR_TOKEN")

    # One request at a time: the model is not shared across threads.
    server = HTTPServer((args.host, args.port), handler_for(GroundingDino(), token))
    logging.info("Grounding DINO detector on http://%s:%d", args.host, args.port)
    server.serve_forever()


if __name__ == "__main__":
    main()
