"""Monotonic timing instrumentation for the Care Reach movement library.

Durations are measured with ``time.perf_counter_ns`` (via an injectable clock)
so they are monotonic and immune to wall-clock adjustments. UTC wall timestamps
remain the responsibility of ``telemetry.TelemetryLog``; nothing here produces
wall time.
"""

from __future__ import annotations

import time


class MonotonicClock:
    """Injectable monotonic/perf clock pair for tests and instrumentation."""

    def __init__(self, monotonic=time.monotonic, perf_ns=time.perf_counter_ns):
        self._monotonic = monotonic
        self._perf_ns = perf_ns

    def monotonic(self) -> float:
        return self._monotonic()

    def perf_ns(self) -> int:
        return self._perf_ns()


class _SampleWindow:
    """Count, mean, and max of a non-negative nanosecond duration stream."""

    __slots__ = ("count", "total_ns", "max_ns")

    def __init__(self):
        self.count = 0
        self.total_ns = 0
        self.max_ns = 0

    def add(self, nanoseconds: int) -> None:
        self.count += 1
        self.total_ns += nanoseconds
        if nanoseconds > self.max_ns:
            self.max_ns = nanoseconds

    def summary(self) -> dict:
        if not self.count:
            return {"count": 0, "mean_ms": 0.0, "max_ms": 0.0}
        return {
            "count": self.count,
            "mean_ms": round(self.total_ns / self.count / 1e6, 3),
            "max_ms": round(self.max_ns / 1e6, 3),
        }


class MotionTiming:
    """Per-motion aggregates for one control loop's bus and cycle durations."""

    def __init__(self):
        self.first_command_ms: float | None = None
        self.read = _SampleWindow()
        self.write = _SampleWindow()
        self.compute = _SampleWindow()
        self.cycle = _SampleWindow()
        self.lag = _SampleWindow()

    def reset(self) -> None:
        self.first_command_ms = None
        self.read = _SampleWindow()
        self.write = _SampleWindow()
        self.compute = _SampleWindow()
        self.cycle = _SampleWindow()
        self.lag = _SampleWindow()

    def record_read_ns(self, nanoseconds: int) -> None:
        self.read.add(nanoseconds)

    def record_write_ns(self, nanoseconds: int) -> None:
        self.write.add(nanoseconds)

    def record_compute_ns(self, nanoseconds: int) -> None:
        self.compute.add(nanoseconds)

    def record_cycle_ns(self, nanoseconds: int) -> None:
        self.cycle.add(nanoseconds)

    def record_lag_s(self, seconds: float) -> None:
        self.lag.add(max(0, int(seconds * 1e9)))

    def record_first_command_ns(self, nanoseconds: int) -> None:
        if self.first_command_ms is None:
            self.first_command_ms = round(max(0, nanoseconds) / 1e6, 3)

    def summary(self) -> dict:
        """Return unambiguous ``*_ms`` windows.

        ``bus_write_ms`` times a bus write call. An addressed write may wait
        for an acknowledgement; a sync write does not. Neither measures
        physical actuator latency.
        """
        return {
            "first_command_ms": self.first_command_ms,
            "bus_read_ms": self.read.summary(),
            "bus_write_ms": self.write.summary(),
            "compute_ms": self.compute.summary(),
            "cycle_ms": self.cycle.summary(),
            "schedule_lag_ms": self.lag.summary(),
        }


class TimedBus:
    """Delegate that timestamps every serial read and write on a bus.

    ``__getattr__`` forwards everything else (connect, disconnect, attributes)
    to the wrapped bus, so fake test buses behave exactly as before.
    """

    def __init__(self, bus, clock: MonotonicClock, timing: MotionTiming):
        self._bus = bus
        self._clock = clock
        self._timing = timing

    def sync_read(self, register, *args, **kwargs):
        start = self._clock.perf_ns()
        try:
            return self._bus.sync_read(register, *args, **kwargs)
        finally:
            self._timing.record_read_ns(self._clock.perf_ns() - start)

    def read(self, register, *args, **kwargs):
        start = self._clock.perf_ns()
        try:
            return self._bus.read(register, *args, **kwargs)
        finally:
            self._timing.record_read_ns(self._clock.perf_ns() - start)

    def write(self, register, *args, **kwargs):
        start = self._clock.perf_ns()
        try:
            return self._bus.write(register, *args, **kwargs)
        finally:
            self._timing.record_write_ns(self._clock.perf_ns() - start)

    def sync_write(self, register, *args, **kwargs):
        start = self._clock.perf_ns()
        try:
            return self._bus.sync_write(register, *args, **kwargs)
        finally:
            self._timing.record_write_ns(self._clock.perf_ns() - start)

    def __getattr__(self, name):
        return getattr(self._bus, name)
