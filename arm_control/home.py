"""Move all six SO-101 joints toward the recorded Home pose together.

Torque remains enabled on exit, including errors and Ctrl-C. Keep the arm clear
of obstacles and leave servo power connected while it holds its pose. Real-bus
runs hold the same exclusive ``BusLock`` as the dashboard for the full
connection lifetime. The single-motor register reads and writes here are kept
verbatim from the original CLI so behavior does not change; timing windows are
measured on the side and returned without altering the control decisions.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from home_pose import home_target
from motion_core import JOINT_NAMES

from .bus import BusOwnership, real_bus
from .bus_lock import BusLock
from .timing import MonotonicClock, MotionTiming, TimedBus

NAMES = JOINT_NAMES
HOME = home_target()
TOLERANCE = 15
MAX_STEP = 100
GOAL_SPEED = 2  # About 100 encoder ticks/s with the current servo phase.
ACCELERATION = 3  # Gentle hardware acceleration and deceleration ramp.
SLOW_PERIOD_S = 0.35
FAST_PERIOD_S = 0.15


def run_home(port: str, speed: int = GOAL_SPEED, *, bus_factory: Callable[[str], Any] | None = None,
             bus_lock: BusLock | None = None, clock: Any = time) -> dict[str, Any]:
    """Run the torque-preserving Home routine; return a small timing summary."""
    period_s = FAST_PERIOD_S if speed >= 4 else SLOW_PERIOD_S
    lock = bus_lock if bus_lock is not None else (BusLock() if bus_factory is None else None)
    ownership = BusOwnership(port, bus_factory=bus_factory or real_bus, bus_lock=lock)
    perf = MonotonicClock(monotonic=clock.monotonic)
    timing = MotionTiming()
    started = clock.monotonic()
    bus = TimedBus(ownership.connect(), perf, timing)

    def read(register, name):
        return int(bus.read(register, name, normalize=False, num_retry=2))

    def write(register, name, value):
        bus.write(register, name, value, normalize=False, num_retry=2)

    try:
        # Certain calibrated STS3215 firmware bounds make a valid goal drive in
        # the wrong direction. Keep motor 3's maximum at least as high as its
        # recorded Home, without raising it beyond that position here.
        for i in (2, 3, 4, 6):
            name = NAMES[i - 1]
            if read("Min_Position_Limit", name) != 0:
                write("Min_Position_Limit", name, 0)
            if i == 3 and read("Max_Position_Limit", name) < HOME[2]:
                raise RuntimeError("Motor 3 firmware maximum is below recorded Home")
            if i != 3 and read("Max_Position_Limit", name) != 4095:
                write("Max_Position_Limit", name, 4095)

        for i, name in enumerate(NAMES, 1):
            position = read("Present_Position", name)
            write("Goal_Position", name, position)
            if read("Torque_Enable", name) != 1:
                write("Torque_Enable", name, 1)
            write("Goal_Speed", name, speed)
            write("Acceleration", name, ACCELERATION)
            if i in (2, 3) and read("P_Coefficient", name) != 32:
                write("P_Coefficient", name, 32)

        trim_pending = set()
        # Keep a wall-clock-sized budget when the faster setting uses more
        # frequent, smaller measured-position updates.
        for cycle in range(max(60, int(25 / period_s))):
            cycle_started_ns = perf.perf_ns()
            read_start_ns = timing.read.total_ns
            write_start_ns = timing.write.total_ns
            state = {
                name: {
                    "position": read("Present_Position", name),
                    "status": read("Status", name),
                    "voltage": read("Present_Voltage", name),
                    "torque": read("Torque_Enable", name),
                }
                for name in NAMES
            }
            positions = {i: state[name]["position"] for i, name in enumerate(NAMES, 1)}
            print(f"{cycle:02d} {positions}", flush=True)
            if any(v["status"] or v["voltage"] < 48 or v["torque"] != 1 for v in state.values()):
                raise RuntimeError(f"Servo fault, low voltage, or lost torque: {state}")
            if all(abs(positions[i] - HOME[i - 1]) <= TOLERANCE for i in range(1, 7)):
                for i, name in enumerate(NAMES):
                    write("Goal_Position", name, HOME[i])
                end_ns = perf.perf_ns()
                cycle_ns = max(0, end_ns - cycle_started_ns)
                bus_ns = (timing.read.total_ns - read_start_ns) + (timing.write.total_ns - write_start_ns)
                timing.record_cycle_ns(cycle_ns)
                timing.record_compute_ns(max(0, cycle_ns - bus_ns))
                print("HOME REACHED; torque remains enabled", flush=True)
                return {
                    "cycles": cycle + 1,
                    "elapsed_s": clock.monotonic() - started,
                    "timing": timing.summary(),
                }

            # Command every joint on every cycle, including joints already home.
            for i, name in enumerate(NAMES):
                position = state[name]["position"]
                target = HOME[i]
                error = position - target
                if name in trim_pending:
                    goal = target
                    trim_pending.remove(name)
                elif TOLERANCE < abs(error) <= 40:
                    # A small error may sit in the servo's static-friction deadband.
                    # Briefly approach from beyond home, then settle.
                    goal = target - (40 if error > 0 else -40)
                    if name == "elbow_flex":
                        goal = min(goal, target)
                    trim_pending.add(name)
                else:
                    goal = max(position - MAX_STEP, target) if error > 0 else min(position + MAX_STEP, target)
                write("Goal_Position", name, goal)
            end_ns = perf.perf_ns()
            cycle_ns = max(0, end_ns - cycle_started_ns)
            bus_ns = (timing.read.total_ns - read_start_ns) + (timing.write.total_ns - write_start_ns)
            timing.record_cycle_ns(cycle_ns)
            timing.record_compute_ns(max(0, cycle_ns - bus_ns))
            clock.sleep(period_s)
        raise RuntimeError("Home timed out; torque remains enabled")
    except BaseException:
        # Freeze the position targets. Never make a fault or Ctrl-C release torque.
        for name in NAMES:
            try:
                write("Goal_Position", name, read("Present_Position", name))
            except Exception:
                pass
        raise
    finally:
        try:
            bus.disconnect(disable_torque=False)
        finally:
            ownership.release()
