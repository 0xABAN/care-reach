"""Electrical telemetry checks use a fake bus; no hardware is opened."""

from motion_core import JOINT_NAMES
from test_dashboard import FakeBus
from test_movement_timing import FakeClock, TimingBase


class PowerTelemetryTests(TimingBase):
    def test_current_is_sampled_early_and_at_bounded_rate_per_motion(self):
        clock = FakeClock()
        bus = FakeBus((2000,) * 6)
        bus.currents["shoulder_lift"] = 7
        arm = self.arm_with_clock(bus, clock)
        self.assertFalse(any("current_raw" in row for row in self.records()))

        first = arm.start(2, -1)
        clock.advance(0.05)
        arm._advance(clock.monotonic())
        first_samples = [row for row in self.records()
                         if row["event"] == "sample" and row.get("token") == first]
        self.assertEqual(len(first_samples), 1)
        self.assertEqual(first_samples[0]["current_raw"]["shoulder_lift"], 7)
        self.assertEqual(set(first_samples[0]["current_raw"]), set(JOINT_NAMES))

        bus.currents["shoulder_lift"] = 12
        clock.advance(0.1)
        arm._advance(clock.monotonic())
        early_samples = [row for row in self.records()
                         if row["event"] == "sample" and row.get("token") == first]
        self.assertEqual(len(early_samples), 2)
        self.assertNotIn("current_raw", early_samples[-1])

        clock.advance(0.5)
        arm._advance(clock.monotonic())
        later_samples = [row for row in self.records()
                         if row["event"] == "sample" and row.get("token") == first]
        self.assertEqual(later_samples[-1]["current_raw"]["shoulder_lift"], 12)
        arm.stop(first)

        # A new movement gets its own early current sample, even if it starts
        # less than 0.5 seconds after the previous one ended.
        second = arm.start(2, 1)
        clock.advance(0.05)
        arm._advance(clock.monotonic())
        new_samples = [row for row in self.records()
                       if row["event"] == "sample" and row.get("token") == second]
        self.assertEqual(len(new_samples), 1)
        self.assertEqual(new_samples[0]["current_raw"]["shoulder_lift"], 12)

    def test_failed_current_read_is_logged_without_stopping_motion(self):
        class CurrentUnavailable(FakeBus):
            def sync_read(self, register, normalize=True, num_retry=0):
                if register == "Present_Current":
                    raise OSError("current register unavailable")
                return super().sync_read(register, normalize=normalize,
                                         num_retry=num_retry)

        clock = FakeClock()
        bus = CurrentUnavailable((2000,) * 6)
        arm = self.arm_with_clock(bus, clock)
        token = arm.start(1, 1)
        clock.advance(0.05)
        arm._advance(clock.monotonic())
        samples = [row for row in self.records()
                   if row["event"] == "sample" and row.get("token") == token]
        self.assertEqual(len(samples), 1)
        self.assertIn("current register unavailable", samples[0]["current_read_error"])
        self.assertNotIn("current_raw", samples[0])
        self.assertIsNotNone(arm.hold)
        self.assertIsNone(arm.error)
        arm.stop(token)
        self.assertTrue(all(bus.torque.values()))
