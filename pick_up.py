"""Pick up a named object with the wrist camera.

By default this takes one picture from wherever the arm is and prints where
the object is relative to the jaws; nothing moves. With --execute, stop the
dashboard first: the arm moves to the survey pose, centers the jaws over the
object, lowers them around it, grips, and lifts it back to the survey height.
Torque stays on, so the jaws keep holding it; open them from the dashboard.

    python pick_up.py "blue box" --execute

Run `pick_setup.py` once before this. --center-only stops after centering.
"""

from __future__ import annotations

import argparse
import logging
from typing import Sequence

from pick.camera import WristCamera
from pick.cli import STOP_ERRORS, add_pick_options, open_detector
from pick.config import PickConfig
from pick.detector import Detector
from pick.routine import DEFAULT_GRASP_HEIGHT_M


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_pick_options(parser)
    parser.add_argument("--execute", action="store_true", help="move the arm")
    parser.add_argument("--center-only", action="store_true", help="stop once the jaws are over the object")
    parser.add_argument("--grasp-height-cm", type=float, default=DEFAULT_GRASP_HEIGHT_M * 100,
                        help="jaw tip height above the table when they close")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    detector = open_detector(parser, args.detector)
    try:
        config = PickConfig.load(args.config)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    camera = WristCamera(args.camera)

    try:
        if args.execute:
            return execute(args, config, camera, detector)
        return look(args, config, camera, detector)
    except STOP_ERRORS as exc:
        print(f"stopped: {exc}")
        return 1


def look(args: argparse.Namespace, config: PickConfig, camera: WristCamera, detector: Detector) -> int:
    """Report where the object is relative to the jaws, without moving."""
    box = detector.find(camera.grab(), args.name)
    if box is None:
        print("not found")
        return 1

    dx_m, dy_m = config.offset_from_jaws(box.center)
    print(f"{args.name} at ({box.center[0]:.0f}, {box.center[1]:.0f}) px, score {box.score:.2f}: "
          f"{dx_m * 100:+.1f} cm along gripper x, {dy_m * 100:+.1f} cm along y from the jaws")
    print("dry run; the arm did not move")
    return 0


def execute(args: argparse.Namespace, config: PickConfig, camera: WristCamera, detector: Detector) -> int:
    # Hardware imports stay below the offline checks.
    from arm_runtime import require_free_port
    from pick.arm import PickArm
    from pick.center import CenterResult, center_over
    from pick.routine import PickResult, pick_up

    require_free_port(args.port)
    arm = PickArm.from_files(args.port)
    if args.center_only:
        arm.move_to(config.survey_pose)
        result = center_over(args.name, arm, camera, detector, config)
        succeeded = result is CenterResult.CENTERED
    else:
        result = pick_up(args.name, arm, camera, detector, config, args.grasp_height_cm / 100)
        succeeded = result is PickResult.PICKED_UP

    print(result.value)
    return 0 if succeeded else 1


if __name__ == "__main__":
    raise SystemExit(main())
