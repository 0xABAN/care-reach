"""Bounded, persistent JSONL telemetry for the Pi arm controller."""

from __future__ import annotations

import json
import logging
import uuid
from collections import deque
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
from pathlib import Path


DEFAULT_PATH = Path.home() / ".local" / "state" / "care-reach" / "telemetry.jsonl"


class TelemetryLog:
    def __init__(self, path: Path = DEFAULT_PATH):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.recent = deque(maxlen=50)
        if self.path.exists():
            with self.path.open(encoding="utf-8") as stream:
                for line in stream:
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if record.get("event") != "sample":
                        self.recent.append(record)
        self.logger = logging.getLogger(f"care_reach.telemetry.{uuid.uuid4().hex}")
        self.logger.setLevel(logging.INFO)
        self.logger.propagate = False
        self.handler = RotatingFileHandler(self.path, maxBytes=20_000_000, backupCount=5)
        self.handler.setFormatter(logging.Formatter("%(message)s"))
        self.logger.addHandler(self.handler)

    def write(self, event: str, **data):
        record = {"time": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                  "event": event, **data}
        if event != "sample":
            self.recent.append(record)
        self.logger.info(json.dumps(record, separators=(",", ":")))

    def close(self):
        self.logger.removeHandler(self.handler)
        self.handler.close()
