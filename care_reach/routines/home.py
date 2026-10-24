#!/usr/bin/env python3
"""Move all six SO-101 joints toward the 2000-tick home pose together.

Torque remains enabled on exit, including errors and Ctrl-C. Keep the arm clear
of obstacles and leave servo power connected while it holds its pose.
"""

import argparse
import time

from care_reach.hardware.bus import MIN_VOLTAGE_RAW, find_port, make_bus
from care_reach.robot import HOME_TICK, JOINT_NAMES
from care_reach.routines.cli import PORT_HELP


TOLERANCE = 15
MAX_STEP = 100
GOAL_SPEED = 2  # About 100 encoder ticks/s with the current servo phase.
ACCELERATION = 3  # Gentle hardware acceleration and deceleration ramp.
SLOW_PERIOD_S = 0.35
FAST_PERIOD_S = 0.15


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help=PORT_HELP)
    parser.add_argument("--speed", type=int, choices=range(1, 9), default=GOAL_SPEED,
                        help="STS3215 running-speed register (4 is the faster tested setting)")
    args = parser.parse_args(argv)
    period_s = FAST_PERIOD_S if args.speed >= 4 else SLOW_PERIOD_S
    bus = make_bus(args.port or find_port())
    bus.connect(handshake=True)

    def read(register, name):
        return int(bus.read(register, name, normalize=False, num_retry=2))

    def write(register, name, value):
        bus.write(register, name, value, normalize=False, num_retry=2)

    try:
        # Certain calibrated STS3215 firmware bounds make a valid goal drive in
        # the wrong direction. Widen only these four known affected motors; the
        # trajectory below enforces the 2000-tick target and bounded steps.
        for i in (2, 3, 4, 6):
            name = JOINT_NAMES[i - 1]
            if read("Min_Position_Limit", name) != 0:
                write("Min_Position_Limit", name, 0)
            if read("Max_Position_Limit", name) != 4095:
                write("Max_Position_Limit", name, 4095)

        for i, name in enumerate(JOINT_NAMES, 1):
            position = read("Present_Position", name)
            write("Goal_Position", name, position)
            if read("Torque_Enable", name) != 1:
                write("Torque_Enable", name, 1)
            write("Goal_Speed", name, args.speed)
            write("Acceleration", name, ACCELERATION)
            if i in (2, 3) and read("P_Coefficient", name) != 32:
                write("P_Coefficient", name, 32)

        trim_pending = set()
        # Keep a wall-clock-sized budget when the faster setting uses more
        # frequent, smaller measured-position updates.
        for cycle in range(max(60, int(25 / period_s))):
            state = {
                name: {
                    "position": read("Present_Position", name),
                    "status": read("Status", name),
                    "voltage": read("Present_Voltage", name),
                    "torque": read("Torque_Enable", name),
                }
                for name in JOINT_NAMES
            }
            positions = {i: state[name]["position"] for i, name in enumerate(JOINT_NAMES, 1)}
            print(f"{cycle:02d} {positions}", flush=True)
            if any(v["status"] or v["voltage"] < MIN_VOLTAGE_RAW or v["torque"] != 1 for v in state.values()):
                raise RuntimeError(f"Servo fault, low voltage, or lost torque: {state}")
            if all(abs(p - HOME_TICK) <= TOLERANCE for p in positions.values()):
                for name in JOINT_NAMES:
                    write("Goal_Position", name, HOME_TICK)
                print("HOME REACHED; torque remains enabled", flush=True)
                return

            # Command every joint on every cycle, including joints already home.
            for name in JOINT_NAMES:
                position = state[name]["position"]
                error = position - HOME_TICK
                if name in trim_pending:
                    goal = HOME_TICK
                    trim_pending.remove(name)
                elif TOLERANCE < abs(error) <= 40:
                    # A small error may sit in the servo's static-friction deadband.
                    # Briefly approach from beyond home, then settle at 2000.
                    goal = HOME_TICK - (40 if error > 0 else -40)
                    trim_pending.add(name)
                else:
                    goal = max(position - MAX_STEP, HOME_TICK) if error > 0 else min(position + MAX_STEP, HOME_TICK)
                write("Goal_Position", name, goal)
            time.sleep(period_s)
        raise RuntimeError("Home timed out; torque remains enabled")
    except BaseException:
        # Freeze the position targets. Never make a fault or Ctrl-C release torque.
        for name in JOINT_NAMES:
            try:
                write("Goal_Position", name, read("Present_Position", name))
            except Exception:
                pass
        raise
    finally:
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
