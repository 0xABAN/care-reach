"""Explicit, Pi-only wrist-flex shake from the saved six-joint Home pose.

Default mode is an offline preview. Stop the dashboard service and give this
command sole ownership of the URT-2 before using --execute. The 150-tick,
0.5 Hz waveform is an aggressive bounded trial, not a physical speed maximum.

The movement logic lives in ``arm_control/shake.py``; this module keeps the
CLI and the historical import path working for existing callers and tests.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from arm_control.bus import format_timing_summary
from arm_control.shake import (  # noqa: F401  (re-exported for existing callers and tests)
    AMPLITUDE_TICKS,
    DURATION_S,
    FREQUENCY_HZ,
    HOME_TOLERANCE,
    LEAD_TICKS,
    MAX_LOAD,
    MAX_TRACKING_ERROR,
    MIN_VOLTAGE_RAW,
    PERIOD_S,
    PORT,
    RAMP_S,
    TELEMETRY_PERIOD_S,
    TRIAL_ACCELERATION,
    TRIAL_GOAL_SPEED,
    WRIST,
    WRIST_INDEX,
    ShakeSafetyError,
    WristShake,
    require_free_port,
    waveform,
)
from motion_core import SafetyBounds, validate_pose
from park import HOME


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="offline preview (default)")
    mode.add_argument("--execute", action="store_true", help="run on the Pi, with sole URT-2 ownership")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--settings", type=Path, default=Path(__file__).with_name("joint_settings.json"))
    parser.add_argument("--calibration", type=Path, default=Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/care_reach_follower.json")
    args = parser.parse_args(argv)
    try:
        bounds = SafetyBounds.from_files(args.settings, args.calibration)
        validate_pose(HOME, bounds)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(f"shake it crazy style: motor 4, {DURATION_S:g}s, ±{AMPLITUDE_TICKS} ticks around Home {HOME[WRIST_INDEX]}, {FREQUENCY_HZ:g} Hz")
    if not args.execute:
        print("dry run complete; no servo bus opened")
        return 0
    result = WristShake(args.port, bounds).run()
    print(f"shake completed in {result['elapsed_s']:.1f}s; {result['writes']} wrist commands; torque remains enabled")
    print(format_timing_summary(result.get("timing")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
