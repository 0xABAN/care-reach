"""A varied, bounded SO-101 free-space dance.

Planning and the default CLI mode are offline. Only ``--execute`` opens the
servo bus, after every requested waypoint and interpolated frame is checked.
The routine is a dance in free space; it does not attempt to grasp an object.
"""

from __future__ import annotations

import math
from typing import Sequence

from care_reach.motion.bounds import SafetyBounds
from care_reach.motion.trajectory import plan_blended_waypoints
from care_reach.robot import HOME_POSE
from care_reach.routines.cli import planning_parser, write_plan_csv


# Each row controls one segment ending at the corresponding named pose. These
# measured-arm rates stay near the pace of the first live dance; changing the
# traveled distance and phase tempo creates the larger motion. Wrist roll may
# move somewhat faster, while loaded shoulder lift and elbow remain slower.
PHASE_VMAX = (
    (80, 75, 75, 85, 120, 80),
    (90, 85, 85, 90, 140, 80),
    (80, 70, 70, 80, 135, 80),
    (70, 70, 70, 70, 100, 90),
    (85, 80, 80, 90, 135, 80),
    (70, 70, 70, 70, 100, 90),
    (90, 85, 85, 90, 140, 85),
    (85, 80, 80, 90, 125, 80),
    (85, 75, 75, 85, 120, 85),
)
PHASE_AMAX = (
    (115, 105, 105, 120, 165, 115),
    (155, 125, 125, 155, 190, 115),
    (125, 105, 105, 125, 180, 115),
    (115, 115, 115, 115, 140, 150),
    (135, 120, 120, 145, 185, 120),
    (115, 115, 115, 115, 140, 160),
    (160, 130, 130, 160, 190, 130),
    (125, 110, 110, 130, 170, 115),
    (125, 110, 110, 130, 165, 125),
)

# Named poses are offsets from Home at full scale. Shoulder lift and elbow
# counter-rotate in the broad arcs, as seen in the source pick-and-place data.
# The short gripper accents simulate a gesture in free space, not a grasp.
DANCE_OFFSETS = (
    ("wind-up", (-180, 120, -120, 150, -260, 0)),
    ("wide sweep", (280, -190, 190, -210, 350, 0)),
    ("wrist flourish", (240, -170, 170, -150, -430, 0)),
    ("gripper beat", (240, -170, 170, -150, -430, -180)),
    ("lift and turn", (-260, 220, -220, 270, 420, -180)),
    ("open beat", (-260, 220, -220, 270, 420, 160)),
    ("reverse sweep", (290, -230, 230, -280, -390, 160)),
    ("return arc", (-120, 100, -100, 150, 230, 160)),
    ("home", (0, 0, 0, 0, 0, 0)),
)


def build_waypoints(scale: float = 0.75) -> list[tuple[str, tuple[int, ...]]]:
    """Return named tick poses; scale is limited to the conservative design."""
    if not math.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError("scale must be greater than zero and at most 1")
    return [
        (name, tuple(home + round(scale * offset) for home, offset in zip(HOME_POSE, offsets, strict=True)))
        for name, offsets in DANCE_OFFSETS
    ]


def build_plan(
    bounds: SafetyBounds, scale: float = 0.75, fps: int = 20
) -> tuple[list[tuple[str, tuple[int, ...]]], list[tuple[int, ...]]]:
    """Validate the complete path before any hardware can be constructed."""
    if fps < 5 or fps > 50:
        raise ValueError("fps must be between 5 and 50")
    waypoints = build_waypoints(scale)
    # The planner validates Home, every waypoint, and every interpolated frame.
    frames = plan_blended_waypoints(
        HOME_POSE, [pose for _, pose in waypoints], bounds, PHASE_VMAX, PHASE_AMAX,
        fps=fps, stop_indices=(3, 5),
    )
    return waypoints, frames


def main(argv: Sequence[str] | None = None) -> int:
    parser = planning_parser(__doc__)
    parser.add_argument("--scale", type=float, default=0.75)
    args = parser.parse_args(argv)

    try:
        bounds = SafetyBounds.from_files(args.settings, args.calibration)
        waypoints, frames = build_plan(bounds, args.scale, args.fps)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))

    print(f"dance: {len(frames)} frames, {(len(frames) - 1) / args.fps:.1f}s at {args.fps} Hz, scale {args.scale:g}")
    for phase, pose in waypoints:
        print(f"  {phase:14} {pose}")
    if args.csv:
        write_plan_csv(args.csv, frames, args.fps)
        print(f"planned targets: {args.csv}")
    if not args.execute:
        print("dry run complete; no servo bus opened")
        return 0

    # Hardware imports and bus construction stay below the complete preflight.
    from care_reach.hardware.arm_runtime import ArmRuntime
    from care_reach.hardware.bus import find_port

    with ArmRuntime(args.port or find_port(), bounds) as runtime:
        result = runtime.run(frames, fps=args.fps)
    print(
        f"executed {result['frames_commanded']} frames in "
        f"{result['elapsed_s']:.1f}s; final {result['final_positions']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
