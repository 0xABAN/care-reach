"""Pick up a named object that both cameras can see.

    python pick_up.py "coke can"            # look and plan; the arm holds still
    python pick_up.py "coke can" --execute  # reach, grip, and lift it 3 cm

Stop the dashboard first: this reads the arm's position from the servo bus.
Calibrate the cameras once with `calibrate_cameras.py`. The detector runs on
the GPU pod, reached through CARE_REACH_DETECTOR_URL with
CARE_REACH_DETECTOR_TOKEN. Torque stays on, so the jaws keep holding the
object afterward; open them from the dashboard.
"""

from __future__ import annotations

import argparse
import logging
import os
from typing import Sequence

from arm_control.bus import URT2_PORT
from arm_control.bus_lock import BusBusyError
from arm_control.pose_runner import PoseMoveError
from pick.arm import PickArm
from pick.camera import SERVICE_URLS, Camera, CameraError
from pick.detector import AmbiguousDetection, Detector
from pick.grasp import GraspError, GraspPlanner
from pick.locate import NotLocated, locate
from pick.rig import CAMERAS_PATH, Rig, load_cameras, model_link_pose
from pick.routine import PickResult, pick_up


DETECTOR_URL_ENV = "CARE_REACH_DETECTOR_URL"
DETECTOR_TOKEN_ENV = "CARE_REACH_DETECTOR_TOKEN"

# Expected ways a pick stops early. The arm holds its measured pose with
# torque on. ValueError covers an unsafe or unsolvable move; OSError an
# unreachable detector.
STOP_ERRORS = (PoseMoveError, BusBusyError, CameraError, AmbiguousDetection, NotLocated, GraspError,
               ValueError, OSError)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", help='the object to pick up, such as "coke can"')
    parser.add_argument("--execute", action="store_true", help="move the arm")
    parser.add_argument("--detector", default=os.environ.get(DETECTOR_URL_ENV),
                        help=f"detector server URL; defaults to ${DETECTOR_URL_ENV}")
    parser.add_argument("--port", default=URT2_PORT, help="URT-2 serial port")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    token = os.environ.get(DETECTOR_TOKEN_ENV)
    if not args.detector or not token:
        parser.error(f"set --detector or ${DETECTOR_URL_ENV}, and ${DETECTOR_TOKEN_ENV}")
    if not CAMERAS_PATH.exists():
        parser.error(f"no camera calibration at {CAMERAS_PATH}; run calibrate_cameras.py")

    try:
        return run(args, Detector(args.detector, token))
    except STOP_ERRORS as exc:
        print(f"stopped: {exc}")
        return 1


def run(args: argparse.Namespace, detector: Detector) -> int:
    cameras = load_cameras()
    arm = PickArm.from_files(args.port)
    rig = Rig(cameras, model_link_pose(arm.bounds))
    pan_axis = rig.link_pose("shoulder_link", arm.ticks)[:2, 3]  # the pan joint's link sits on its axis
    planner = GraspPlanner(arm.kinematics, arm.bounds, cameras["wrist"], pan_axis)

    with Camera(SERVICE_URLS["wrist"], cameras["wrist"].image_size) as wrist, \
            Camera(SERVICE_URLS["shoulder"], cameras["shoulder"].image_size) as shoulder:

        def see(ticks: tuple[int, ...]):
            pictures = {"wrist": wrist.grab(), "shoulder": shoulder.grab()}
            return locate(args.name, ticks, pictures, detector, rig)

        if not args.execute:
            target = see(arm.ticks)
            print(f"{args.name}: {target}")
            plan = planner.plan(target, arm.ticks)
            print(f"close look from {planner.look_pose(target, plan, arm.ticks)}; grasp at {plan.tilt_deg} degrees "
                  f"from straight down, pre-grasp {plan.pregrasp}, grasp {plan.grasp}")
            print("dry run; the arm did not move")
            return 0

        result = pick_up(arm, see, planner)

    print(result.value)
    return 0 if result is PickResult.PICKED_UP else 1


if __name__ == "__main__":
    raise SystemExit(main())
