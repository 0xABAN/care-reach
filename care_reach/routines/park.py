"""Plan a coordinated move from Home to this arm's manually recorded folded pose.

The endpoint is stored as raw encoder ticks in config/folded_pose.json. Lance
established it by hand with torque disabled; encoder readings were confirmed
twice. An automated home-to-folded move reached within eight ticks of this
pose. This command never releases torque.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from care_reach.motion.bounds import SafetyBounds, UnsafePoseError
from care_reach.motion.trajectory import plan_blended_waypoints
from care_reach.robot import FOLDED_POSE_PATH, HOME_POSE, load_folded_pose
from care_reach.routines.cli import planning_parser, write_plan_csv


FOLD_VMAX = (150.0, 170.0, 170.0, 170.0, 200.0, 150.0)
FOLD_AMAX = (240.0, 250.0, 250.0, 250.0, 300.0, 240.0)
# The measured fold is 10 ticks below motor 2's old panel bound and 77 above
# motor 3's old calibration/panel bound. No other limit may be widened here.
MAX_RECORDED_EXTENSION = (0, 20, 100, 0, 0, 0)


def folded_bounds(bounds: SafetyBounds, target: Sequence[int]) -> SafetyBounds:
    """Extend only the small measured departures from prior software bounds.

    The saved LeRobot calibration stays untouched. This envelope is local to
    the park routine; ordinary actions and model output keep their limits.
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
    pose_path: str | Path = FOLDED_POSE_PATH,
    fps: int = 20,
) -> tuple[tuple[int, ...], SafetyBounds, list[tuple[int, ...]]]:
    """Plan Home to the recorded fold; return the target, fold bounds, and frames.

    The planner validates Home, the target, and every frame against the fold
    bounds, so a plan that returns is safe to hand to the measured runner.
    """
    if not isinstance(fps, int) or isinstance(fps, bool) or not 5 <= fps <= 50:
        raise ValueError("fps must be an integer between 5 and 50")
    target = load_folded_pose(pose_path)
    task_bounds = folded_bounds(bounds, target)
    frames = plan_blended_waypoints(HOME_POSE, (target,), task_bounds, FOLD_VMAX, FOLD_AMAX, fps=fps)
    return target, task_bounds, frames


def main(argv: Sequence[str] | None = None) -> int:
    parser = planning_parser(__doc__)
    parser.add_argument("--pose", type=Path, default=FOLDED_POSE_PATH, help="recorded folded pose JSON")
    args = parser.parse_args(argv)

    try:
        bounds = SafetyBounds.from_files(args.settings, args.calibration)
        target, task_bounds, frames = build_plan(bounds, args.pose, fps=args.fps)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))

    print(f"fold target {target}; offline plan {len(frames)} frames over {(len(frames)-1)/args.fps:.1f}s at {args.fps} Hz")
    if args.csv:
        write_plan_csv(args.csv, frames, args.fps)
        print(f"planned targets: {args.csv}")
    if not args.execute:
        print("dry run complete; no servo bus opened")
        return 0

    # Hardware imports stay below the complete offline preflight.
    from care_reach.hardware.bus import find_port
    from care_reach.hardware.measured_pose_runner import MeasuredPoseRunner

    result = MeasuredPoseRunner(args.port or find_port(), task_bounds).move(
        target, max_lead=80, period_s=0.06, timeout_s=35, tolerance=35
    )
    print(f"fold completed in {result['elapsed_s']:.1f}s; measured {result['positions']}; torque remains enabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
