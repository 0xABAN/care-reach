"""Plan a coordinated move to this arm's manually recorded folded pose.

The endpoint is stored as raw encoder ticks in folded_pose.json. Lance
established it by hand with torque disabled; encoder readings were confirmed
twice. An automated home-to-folded move reached within eight ticks of this
pose. This command never releases torque.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Sequence

from motion_core import JOINT_NAMES, SafetyBounds, UnsafePoseError, plan_blended_waypoints, validate_pose


HOME = (2000,) * 6
POSE_PATH = Path(__file__).with_name("folded_pose.json")
FOLD_VMAX = (150.0, 170.0, 170.0, 170.0, 200.0, 150.0)
FOLD_AMAX = (240.0, 250.0, 250.0, 250.0, 300.0, 240.0)
# The measured fold is 10 ticks below motor 2's old panel bound and 77 above
# motor 3's old calibration/panel bound. No other limit may be widened here.
MAX_RECORDED_EXTENSION = (0, 20, 100, 0, 0, 0)


def folded_target(pose_path: str | Path = POSE_PATH) -> tuple[int, ...]:
    """Read the six exact encoder ticks of Lance's physical folded pose."""
    with Path(pose_path).open(encoding="utf-8") as stream:
        record = json.load(stream)
    if record.get("robot_id") != "care_reach_follower":
        raise ValueError("folded pose belongs to a different robot")
    positions = record.get("positions_ticks")
    if not isinstance(positions, dict) or set(positions) != set(JOINT_NAMES):
        raise ValueError("folded pose must name exactly the six follower joints")
    ticks = tuple(positions[name] for name in JOINT_NAMES)
    if any(isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 4095 for value in ticks):
        raise ValueError("folded pose must contain six valid integer encoder ticks")
    return ticks


def folded_bounds(bounds: SafetyBounds, target: Sequence[int]) -> SafetyBounds:
    """Extend only the small measured departures from prior software bounds.

    The saved LeRobot calibration stays untouched. This envelope is local to
    park.py; ordinary actions and model output retain their existing limits.
    """
    if len(target) != 6:
        raise ValueError("folded target must contain six joints")
    lower = []
    upper = []
    for index, (tick, lo, hi, allowance) in enumerate(
        zip(target, bounds.lower, bounds.upper, MAX_RECORDED_EXTENSION, strict=True), start=1
    ):
        below = max(0, lo - tick)
        above = max(0, tick - hi)
        if below > allowance or above > allowance:
            raise UnsafePoseError(
                f"motor {index} folded tick {tick} exceeds recorded-pose extension from [{lo}, {hi}]"
            )
        lower.append(min(lo, tick))
        upper.append(max(hi, tick))
    return SafetyBounds(tuple(lower), tuple(upper))


def build_plan(
    bounds: SafetyBounds,
    pose_path: str | Path = POSE_PATH,
    fps: int = 20,
) -> tuple[tuple[int, ...], list[tuple[int, ...]]]:
    if not isinstance(fps, int) or isinstance(fps, bool) or not 5 <= fps <= 50:
        raise ValueError("fps must be an integer between 5 and 50")
    target = folded_target(pose_path)
    task_bounds = folded_bounds(bounds, target)
    validate_pose(HOME, task_bounds)
    frames = plan_blended_waypoints(HOME, (target,), task_bounds, FOLD_VMAX, FOLD_AMAX, fps=fps)
    if frames[0] != HOME or frames[-1] != target:
        raise ValueError("fold planner did not reach the recorded endpoint")
    return target, frames


def write_plan_csv(path: Path, frames: Sequence[Sequence[int]], fps: int) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", *(f"target_{name}_ticks" for name in JOINT_NAMES)))
        for index, frame in enumerate(frames):
            writer.writerow((f"{index / fps:.3f}", *frame))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="plan only (default)")
    mode.add_argument("--execute", action="store_true", help="move the connected arm")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--port", default="/dev/cu.usbmodem5B790785631")
    parser.add_argument("--settings", type=Path, default=Path(__file__).with_name("joint_settings.json"))
    parser.add_argument("--calibration", type=Path, default=Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/care_reach_follower.json")
    parser.add_argument("--pose", type=Path, default=POSE_PATH, help="recorded folded pose JSON")
    parser.add_argument("--csv", type=Path, help="save planned target ticks")
    args = parser.parse_args(argv)

    try:
        bounds = SafetyBounds.from_files(args.settings, args.calibration)
        target, frames = build_plan(bounds, args.pose, fps=args.fps)
        task_bounds = folded_bounds(bounds, target)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))

    print(f"fold target {target}; offline plan {len(frames)} frames over {(len(frames)-1)/args.fps:.1f}s at {args.fps} Hz")
    if args.csv:
        write_plan_csv(args.csv, frames, args.fps)
        print(f"planned targets: {args.csv}")
    if not args.execute:
        print("dry run complete; no servo bus opened")
        return 0

    from measured_pose_runner import MeasuredPoseRunner

    result = MeasuredPoseRunner(args.port, task_bounds).move(
        target, max_lead=80, period_s=0.06, timeout_s=35, tolerance=35
    )
    print(f"fold completed in {result['elapsed_s']:.1f}s; measured {result['positions']}; torque remains enabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
