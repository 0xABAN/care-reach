"""Exclusive, nonblocking ownership lock for the SO-101 serial bus.

Exactly one process may own the URT-2. The dashboard's ``Arm`` holds this lock
for its whole connection lifetime so a second direct caller fails fast with a
clear message instead of fighting over the serial device. The lock is a
``flock`` on a state file; it is never used by fake-bus callers in tests.
"""

from __future__ import annotations

import errno
import fcntl
import os
from pathlib import Path

DEFAULT_LOCK_PATH = Path.home() / ".local" / "state" / "care-reach" / "arm-bus.lock"


class BusBusyError(RuntimeError):
    """Another process owns the serial bus; retry after it disconnects."""


class BusLock:
    """Nonblocking exclusive lock, one per ``Arm`` connection lifetime."""

    def __init__(self, path: str | Path = DEFAULT_LOCK_PATH):
        self.path = Path(path)
        self._fd: int | None = None

    @property
    def locked(self) -> bool:
        return self._fd is not None

    def acquire(self, port: str = "") -> None:
        if self._fd is not None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            os.close(fd)
            if exc.errno in (errno.EACCES, errno.EAGAIN):
                raise BusBusyError(
                    f"Serial bus {port or self.path} is owned by another process; "
                    f"stop that owner first (lock file {self.path})"
                ) from exc
            raise
        try:
            os.ftruncate(fd, 0)
            os.write(fd, f"{os.getpid()} {port}\n".encode())
            os.fsync(fd)
        except Exception:
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)
            raise
        self._fd = fd

    def release(self) -> None:
        if self._fd is None:
            return
        fd, self._fd = self._fd, None
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)

    def __enter__(self) -> "BusLock":
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.release()
