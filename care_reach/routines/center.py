"""Center the jaws over a named object seen by the wrist camera.

By default this takes one picture from wherever the arm is and prints where
the object is relative to the jaws; nothing moves. With --execute, stop the
dashboard first: the arm moves to the survey pose, then looks and nudges until
the object sits at the aim point, giving up after four looks.

    python -m care_reach.routines.center "coke can" --execute

Run `pick_setup` once before this.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Sequence

from care_reach.pick.center import CenterResult, center_over
from care_reach.pick.config import PickConfig
from care_reach.robot import PICK_CONFIG_PATH
from care_reach.routines.cli import PORT_HELP, add_vision_options, open_detector
from care_reach.vision.camera import WristCamera


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_vision_options(parser)
    parser.add_argument("--execute", action="store_true", help="move the arm")
    parser.add_argument("--port", help=PORT_HELP)
    parser.add_argument("--config", type=Path, default=PICK_CONFIG_PATH, help="pick config JSON")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    detector = open_detector(parser, args.detector)
    try:
        config = PickConfig.load(args.config)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))

    camera = WristCamera(args.camera)
    if not args.execute:
        box = detector.find(camera.grab(), args.name)
        if box is None:
            print(CenterResult.NOT_FOUND.value)
            return 1

        dx_m, dy_m = config.offset_from_jaws(box.center)
        print(f"{args.name} at ({box.center[0]:.0f}, {box.center[1]:.0f}) px, score {box.score:.2f}: "
              f"{dx_m * 100:+.1f} cm along gripper x, {dy_m * 100:+.1f} cm along y from the jaws")
        print("dry run; the arm did not move")
        return 0

    # Hardware imports stay below the offline checks.
    from care_reach.hardware.bus import find_port
    from care_reach.pick.arm import PickArm

    arm = PickArm.from_files(args.port or find_port())
    arm.move_to(config.survey_pose)
    result = center_over(args.name, arm, camera, detector, config)

    print(result.value)
    return 0 if result is CenterResult.CENTERED else 1


if __name__ == "__main__":
    raise SystemExit(main())
