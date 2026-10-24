"""One-time setup for centering the jaws over objects with the wrist camera.

1. aim: with the dashboard running, jog the arm so the gripper points down at
   the height it will survey from, with a test object right under the jaws.
   Measure the camera lens's height above the table, then run:

       python -m care_reach.routines.pick_setup aim "coke can" --survey-height-cm 20

   This saves the arm's pose, that height, and where the object appears in
   the picture: the aim point.

2. direction: leave the object where it is, stop the dashboard, and run:

       python -m care_reach.routines.pick_setup direction "coke can"

   The arm nudges 2 cm along each gripper axis, checks the object moves the
   way the lens estimate expects, and saves which picture direction matches
   which axis. Rerun it after every aim.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence
import urllib.request

import numpy as np

from care_reach.motion.bounds import SafetyBounds, validate_pose
from care_reach.pick.config import PickConfig
from care_reach.robot import PICK_CONFIG_PATH
from care_reach.routines.cli import PORT_HELP, add_vision_options, open_detector
from care_reach.vision.camera import WristCamera


DASHBOARD_URL = "http://127.0.0.1:8765"

# Long enough to move the object well clear of detection noise in the picture.
NUDGE_M = 0.02

# How square the camera must sit on the gripper: the object's sideways drift
# may be at most this fraction of its move along the expected picture axis.
MAX_SKEW = 0.35

# How much bigger the real picture may be than the lens estimate predicts.
# Inside this range, four looks center an object starting about 10 cm away.
SCALE_RANGE = (0.8, 1.7)


def read_dashboard_pose(url: str) -> tuple[int, ...]:
    """Read the arm's measured joint ticks from the running dashboard."""
    with urllib.request.urlopen(f"{url}/api/state", timeout=5) as response:
        state = json.load(response)

    if state["pose"]["active"]:
        raise RuntimeError("The dashboard is still moving the arm")
    return tuple(motor["position"] for motor in sorted(state["motors"], key=lambda motor: motor["id"]))


def image_directions(shift_x_px: Sequence[float], shift_y_px: Sequence[float], expected_px: float) -> tuple[str, str]:
    """Work out which gripper axis each picture direction leads toward.

    ``shift_x_px`` and ``shift_y_px`` are how far the object moved in the
    picture, as (right, down) pixels, when the jaws moved +NUDGE_M along the
    gripper's x and then its y axis. ``expected_px`` is how far the lens
    estimate says it should have moved.

    Returns (image_right, image_down) for PickConfig. Raises ValueError if the
    camera sits crooked, or if the estimate is too far off to center with.
    """
    directions = {}
    for axis, (right, down) in (("x", shift_x_px), ("y", shift_y_px)):
        picture = "right" if abs(right) >= abs(down) else "down"
        along, across = (right, down) if picture == "right" else (down, right)

        scale = abs(along) / expected_px
        if not SCALE_RANGE[0] <= scale <= SCALE_RANGE[1]:
            raise ValueError(
                f"A {NUDGE_M * 100:.0f} cm nudge along gripper {axis} moved the object {abs(along):.0f} px; "
                f"the lens estimate expects {expected_px:.0f} px. Check the survey height."
            )
        if abs(across) > MAX_SKEW * abs(along):
            raise ValueError(f"A nudge along gripper {axis} moved the object diagonally; mount the camera square")
        if picture in directions:
            raise ValueError("Both nudges moved the object along the same picture axis")

        # Moving the jaws toward something makes it slide the other way in
        # the picture, so the axis points away from the slide.
        directions[picture] = ("-" if along > 0 else "+") + axis

    return directions["right"], directions["down"]


def aim(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    detector = open_detector(parser, args.detector)
    pose = validate_pose(read_dashboard_pose(args.dashboard), SafetyBounds.from_files())

    box = detector.find(WristCamera(args.camera).grab(), args.name)
    if box is None:
        raise SystemExit(f"{args.name!r} is not in the picture")

    # A new aim clears the directions, so the next direction check also
    # verifies the scale at this survey height.
    config = PickConfig(pose, args.survey_height_cm / 100, box.center)
    config.save(args.config)
    print(f"aim point ({box.center[0]:.0f}, {box.center[1]:.0f}) px from survey pose {pose}; saved {args.config}")
    print("next: stop the dashboard and run `pick_setup direction`")


def direction(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    detector = open_detector(parser, args.detector)
    config = PickConfig.load(args.config)

    # Hardware imports stay below the offline checks.
    from care_reach.hardware.bus import find_port
    from care_reach.pick.arm import PickArm

    arm = PickArm.from_files(args.port or find_port())
    camera = WristCamera(args.camera)

    def locate() -> np.ndarray:
        box = detector.find(camera.grab(), args.name)
        if box is None:
            raise SystemExit(f"Lost sight of {args.name!r}")
        return np.array(box.center)

    arm.move_to(config.survey_pose)
    start = locate()
    arm.nudge(NUDGE_M, 0.0)
    after_x = locate()
    arm.nudge(0.0, NUDGE_M)
    after_y = locate()
    arm.move_to(config.survey_pose)

    shift_x, shift_y = after_x - start, after_y - after_x
    print(f"gripper +x nudge moved the object {shift_x.round(1)} px; +y moved it {shift_y.round(1)} px")

    expected_px = NUDGE_M / config.meters_per_pixel
    image_right, image_down = image_directions(shift_x, shift_y, expected_px)
    config.with_directions(image_right, image_down).save(args.config)
    print(f"picture right is gripper {image_right}, down is {image_down}; saved {args.config}")


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=PICK_CONFIG_PATH, help="pick config JSON")
    steps = parser.add_subparsers(dest="step", required=True)

    aim_parser = steps.add_parser("aim", help="record the survey pose and aim point")
    add_vision_options(aim_parser)
    aim_parser.add_argument("--survey-height-cm", type=float, required=True,
                            help="camera lens height above the table")
    aim_parser.add_argument("--dashboard", default=DASHBOARD_URL, help="dashboard URL to read the arm pose from")

    direction_parser = steps.add_parser("direction", help="check which way nudges move the object")
    add_vision_options(direction_parser)
    direction_parser.add_argument("--port", help=PORT_HELP)

    args = parser.parse_args(argv)
    try:
        {"aim": aim, "direction": direction}[args.step](args, parser)
    except (ValueError, OSError) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
