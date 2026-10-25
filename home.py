#!/usr/bin/env python3
"""Move all six SO-101 joints toward the recorded Home pose together.

Torque remains enabled on exit, including errors and Ctrl-C. Keep the arm clear
of obstacles and leave servo power connected while it holds its pose.

The routine lives in ``arm_control/home.py``; this module is the thin CLI that
parses arguments and reports the routine's timing summary.
"""

import argparse
import time

from arm_control.bus import format_timing_summary
from arm_control.home import (  # noqa: F401  (re-exported for existing callers and tests)
    ACCELERATION,
    FAST_PERIOD_S,
    GOAL_SPEED,
    HOME,
    MAX_STEP,
    NAMES,
    SLOW_PERIOD_S,
    TOLERANCE,
    run_home,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem5B790785631")
    parser.add_argument("--speed", type=int, choices=range(1, 9), default=GOAL_SPEED,
                        help="STS3215 running-speed register (4 is the faster tested setting)")
    args = parser.parse_args(argv)
    result = run_home(args.port, args.speed, clock=time)
    print(format_timing_summary(result.get("timing")), flush=True)
    return result


if __name__ == "__main__":
    main()
