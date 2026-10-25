"""Timing instrumentation and bus-ownership lock checks for arm_control.

These tests use fake buses and an injectable monotonic clock. No serial device
is opened. Lock files live inside the temporary directory, except one test that
proves fake-bus callers never contend on the real default lock.
"""

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from arm_control import Arm
from arm_control.bus_lock import BusBusyError, BusLock
from test_dashboard import DashboardBase, FakeBus, TrackingBus


class FakeClock:
    """Controllable monotonic clock; the perf counter ticks 1 µs per call."""

    def __init__(self, start=1000.0):
        self._now = float(start)
        self._ns = int(start * 1e9)

    def monotonic(self):
        return self._now

    def perf_ns(self):
        value = self._ns
        self._ns += 1000
        return value

    def advance(self, seconds):
        self._now += seconds
        self._ns = max(self._ns, int(self._now * 1e9))


class TimingBase(DashboardBase):
    def arm_with_clock(self, bus, clock, **kwargs):
        thread_patch = patch("arm_control.arm.threading.Thread")
        thread_patch.start()
        self.addCleanup(thread_patch.stop)
        arm = Arm("fake", bus_factory=lambda _: bus, settings_path=self.settings_path,
                  calibration_path=self.calibration_path, pose_path=self.pose_path,
                  telemetry_path=self.settings_path.parent / "telemetry.jsonl",
                  clock=clock, **kwargs)
        self.addCleanup(arm.close)
        return arm

    def records(self):
        path = self.settings_path.parent / "telemetry.jsonl"
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


class MovementTimingTests(TimingBase):
    TIMING_FIELDS = ("bus_read_ms", "bus_write_ms", "compute_ms", "cycle_ms", "schedule_lag_ms")

    @staticmethod
    def jog_steps(arm, clock, steps=5, step_s=0.05):
        for _ in range(steps):
            clock.advance(step_s)
            arm._advance(clock.monotonic())

    def test_stop_carries_positive_timing_windows(self):
        clock = FakeClock()
        arm = self.arm_with_clock(FakeBus((2000,) * 6), clock)
        token = arm.start(1, 1)
        self.jog_steps(arm, clock)
        arm.stop(token)
        stopped = [record for record in self.records() if record["event"] == "motion_stopped"]
        self.assertEqual(len(stopped), 1)
        timing = stopped[0]["timing"]
        for field in self.TIMING_FIELDS:
            window = timing[field]
            self.assertIsInstance(window, dict, field)
            self.assertGreater(window["count"], 0, field)
            self.assertGreater(window["max_ms"], 0, field)
            self.assertGreaterEqual(window["mean_ms"], 0, field)
            self.assertLessEqual(window["mean_ms"], window["max_ms"] + 1e-9, field)
        self.assertGreater(timing["stop_command_ms"], 0)
        self.assertGreater(timing["motion_elapsed_ms"], 0)
        self.assertGreater(timing["first_command_ms"], 0)
        self.assertLess(timing["first_command_ms"], timing["motion_elapsed_ms"])
        self.assertLess(timing["compute_ms"]["mean_ms"], timing["cycle_ms"]["mean_ms"])

    def test_start_and_stop_records_share_one_token(self):
        clock = FakeClock()
        arm = self.arm_with_clock(FakeBus((2000,) * 6), clock)
        first = arm.start(1, 1)
        self.jog_steps(arm, clock, steps=2)
        arm.stop(first)
        second = arm.start(1, -1)
        self.jog_steps(arm, clock, steps=2)
        arm.stop(second)
        records = self.records()
        started = [r for r in records if r["event"] == "motion_started" and r["mode"] == "jog"]
        stopped = [r for r in records if r["event"] == "motion_stopped"]
        self.assertEqual([r["token"] for r in started], [first, second])
        self.assertEqual([r["token"] for r in stopped], [first, second])
        self.assertNotEqual(first, second)

    def test_compute_time_excludes_a_slow_serial_read(self):
        clock = FakeClock()

        class SlowBus(FakeBus):
            def sync_read(self, *args, **kwargs):
                clock._ns += 10_000_000  # A measured 10 ms bus call.
                return super().sync_read(*args, **kwargs)

        arm = self.arm_with_clock(SlowBus((2000,) * 6), clock)
        arm.start(1, 1)
        clock.advance(0.05)
        arm._advance(clock.monotonic())
        timing = arm.timing.summary()
        self.assertGreater(timing["bus_read_ms"]["count"], 0)
        self.assertGreater(
            timing["cycle_ms"]["mean_ms"] - timing["compute_ms"]["mean_ms"], 50
        )

    def test_pose_completion_emits_token_and_timing(self):
        clock = FakeClock()
        arm = self.arm_with_clock(TrackingBus((2000,) * 6), clock)
        result = arm.pose_start("fold")
        for _ in range(400):
            if arm.hold is None:
                break
            clock.advance(0.06)
            arm._advance(clock.monotonic())
        self.assertIsNone(arm.hold)
        completed = [r for r in self.records() if r["event"] == "motion_completed"]
        self.assertEqual(len(completed), 1)
        self.assertEqual(completed[0]["token"], result["token"])
        self.assertEqual(completed[0]["pose"], "fold")
        self.assertGreater(completed[0]["timing"]["settle_ms"], 0)
        self.assertGreater(completed[0]["timing"]["first_command_ms"], 0)
        self.assertGreater(completed[0]["timing"]["bus_write_ms"]["count"], 0)
        self.assertGreater(completed[0]["timing"]["compute_ms"]["count"], 0)

    def test_timeout_error_record_keeps_token_and_timing(self):
        clock = FakeClock()
        arm = self.arm_with_clock(FakeBus((2000,) * 6), clock)
        token = arm.start(1, 1)
        self.jog_steps(arm, clock, steps=1)
        arm.hold["expires"] = clock.monotonic() - 1
        clock.advance(0.05)
        arm._advance(clock.monotonic())
        self.assertIsNone(arm.hold)
        errors = [r for r in self.records() if r["event"] == "motion_error"]
        self.assertTrue(errors)
        self.assertEqual(errors[-1]["token"], token)
        self.assertGreater(errors[-1]["timing"]["compute_ms"]["count"], 0)

    def test_long_motion_emits_throttled_timing_samples(self):
        clock = FakeClock()
        bus = FakeBus((2000,) * 6)
        arm = self.arm_with_clock(bus, clock)
        token = arm.start(1, 1)
        arm.hold["expires"] = clock.monotonic() + 30
        for _ in range(140):
            clock.advance(0.05)
            arm._advance(clock.monotonic())
            bus.positions["shoulder_pan"] = bus.goals["shoulder_pan"]
        arm.stop(token)
        samples = [r for r in self.records() if r["event"] == "timing_sample"]
        self.assertTrue(samples)
        self.assertTrue(all(r["token"] == token for r in samples))
        self.assertLess(len(samples), 10)
        for sample in samples:
            self.assertGreater(sample["timing"]["bus_read_ms"]["count"], 0)


class BusLockUnitTests(unittest.TestCase):
    def test_exclusive_nonblocking_acquire_release_cycle(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "arm-bus.lock"
            first = BusLock(path)
            first.acquire("port-a")
            self.assertTrue(first.locked)
            self.assertTrue(path.exists())
            second = BusLock(path)
            with self.assertRaises(BusBusyError) as raised:
                second.acquire("port-b")
            self.assertIn("another process", str(raised.exception))
            self.assertFalse(second.locked)
            first.release()
            self.assertFalse(first.locked)
            second.acquire("port-b")
            self.assertTrue(second.locked)
            second.release()
            self.assertFalse(second.locked)

    def test_acquiring_an_owned_lock_is_idempotent(self):
        with tempfile.TemporaryDirectory() as tmp:
            lock = BusLock(Path(tmp) / "arm-bus.lock")
            lock.acquire("port-a")
            lock.acquire("port-a")  # no error, still one holder
            self.assertTrue(lock.locked)
            lock.release()
            self.assertFalse(lock.locked)


class ArmLockTests(TimingBase):
    def lock_path(self):
        return self.settings_path.parent / "arm-bus.lock"

    def test_second_arm_with_the_same_lock_gets_a_busy_error(self):
        path = self.lock_path()
        first = self.arm_with_clock(FakeBus((2000,) * 6), FakeClock(), bus_lock=BusLock(path))
        with self.assertRaises(BusBusyError):
            Arm("fake", bus_factory=lambda _: FakeBus((2000,) * 6),
                settings_path=self.settings_path, calibration_path=self.calibration_path,
                pose_path=self.pose_path,
                telemetry_path=self.settings_path.parent / "telemetry2.jsonl",
                bus_lock=BusLock(path))
        first.close()
        second = self.arm_with_clock(FakeBus((2000,) * 6), FakeClock(), bus_lock=BusLock(path))
        self.assertIsNotNone(second.state())

    def test_constructor_failure_releases_the_lock(self):
        path = self.lock_path()

        class ExplodingBus(FakeBus):
            def connect(self, handshake=True):
                raise OSError("port disappeared")

        failed_bus = ExplodingBus((2000,) * 6)
        with self.assertRaises(OSError):
            Arm("fake", bus_factory=lambda _: failed_bus,
                settings_path=self.settings_path, calibration_path=self.calibration_path,
                pose_path=self.pose_path,
                telemetry_path=self.settings_path.parent / "telemetry2.jsonl",
                bus_lock=BusLock(path))
        self.assertEqual(failed_bus.disconnects, [False])
        retry = self.arm_with_clock(FakeBus((2000,) * 6), FakeClock(), bus_lock=BusLock(path))
        self.assertIsNotNone(retry.state())

    def test_worker_start_failure_releases_bus_and_lock(self):
        path = self.lock_path()
        failed_bus = FakeBus((2000,) * 6)
        with patch("arm_control.arm.threading.Thread") as thread:
            thread.return_value.start.side_effect = RuntimeError("worker unavailable")
            with self.assertRaisesRegex(RuntimeError, "worker unavailable"):
                Arm("fake", bus_factory=lambda _: failed_bus,
                    settings_path=self.settings_path, calibration_path=self.calibration_path,
                    pose_path=self.pose_path,
                    telemetry_path=self.settings_path.parent / "telemetry2.jsonl",
                    bus_lock=BusLock(path))
        self.assertEqual(failed_bus.disconnects, [False])
        retry = self.arm_with_clock(FakeBus((2000,) * 6), FakeClock(), bus_lock=BusLock(path))
        self.assertIsNotNone(retry.state())

    def test_fake_bus_skips_the_real_default_lock(self):
        holder = BusLock()  # the real state-file lock
        holder.acquire("timing-test-holder")
        self.addCleanup(holder.release)
        arm = self.arm_with_clock(FakeBus((2000,) * 6), FakeClock())
        self.assertIsNotNone(arm.state())


if __name__ == "__main__":
    unittest.main()
