"""Find a named object in a picture with Grounding DINO on the GPU pod.

The pod's server is not part of this module. Its contract:

    POST {url}/detect
    Authorization: Bearer {token}
    {"image": "<base64 JPEG>", "prompt": "coke can."}

    -> {"detections": [{"box": [x0, y0, x1, y1], "score": 0.82}, ...]}

Boxes are in the picture's pixels.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
import json
from typing import Iterable
import urllib.request


# The gripper's own jaws show at the bottom of the wrist camera's picture.
# In live tests they scored up to 0.37 for objects that weren't there, such
# as "coke can"; objects actually in view scored 0.52 to 0.75.
MIN_SCORE = 0.45

# Two matches closer than this in score are too close to call.
AMBIGUOUS_MARGIN = 0.05

# The model answers in about 0.2 s, but the Pi's Wi-Fi sometimes stalls for
# seconds. The arm holds still while it waits.
TIMEOUT_S = 10.0


class AmbiguousDetection(RuntimeError):
    """Two objects match the name about equally well; picking either is a guess."""


@dataclass(frozen=True)
class Box:
    x0: float
    y0: float
    x1: float
    y1: float
    score: float

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)


def best_match(boxes: Iterable[Box], name: str) -> Box | None:
    """Return the clear best match, None when nothing scores high enough.

    Raises AmbiguousDetection when the top two are too close to call.
    """
    ranked = sorted((box for box in boxes if box.score >= MIN_SCORE), key=lambda box: box.score, reverse=True)
    if not ranked:
        return None

    if len(ranked) > 1 and ranked[0].score - ranked[1].score < AMBIGUOUS_MARGIN:
        raise AmbiguousDetection(
            f"Two matches for {name!r} are too close to call ({ranked[0].score:.2f} and {ranked[1].score:.2f})"
        )
    return ranked[0]


class Detector:
    def __init__(self, url: str, token: str) -> None:
        self.url = url.rstrip("/")
        self.token = token

    def find(self, jpeg: bytes, name: str) -> Box | None:
        """Return where ``name`` is in the JPEG picture, or None if it isn't there."""
        # Grounding DINO expects a lowercase phrase ending with a period.
        prompt = name.strip().lower().rstrip(".") + "."
        body = json.dumps({"image": base64.b64encode(jpeg).decode("ascii"), "prompt": prompt})
        request = urllib.request.Request(
            f"{self.url}/detect",
            data=body.encode(),
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {self.token}"},
        )
        with urllib.request.urlopen(request, timeout=TIMEOUT_S) as response:
            detections = json.load(response)["detections"]

        return best_match((Box(*item["box"], score=item["score"]) for item in detections), name)
