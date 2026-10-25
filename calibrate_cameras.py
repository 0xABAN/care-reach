"""Calibrate the wrist and shoulder cameras against a ChArUco board.

    python calibrate_cameras.py pattern board.png   # the board, to show on a screen or print
    python calibrate_cameras.py wrist               # sweep the wrist camera over a still board
    python calibrate_cameras.py shoulder            # swing the base past a board both cameras see

Calibrate the wrist camera first; the shoulder camera's solve uses it.

- wrist: set the board down in front of the arm, then pose the arm from the
  dashboard so the wrist camera sees the board. The sweep moves around that
  pose but never brings the jaws lower or farther out.
- shoulder: prop the board up facing the shoulder camera, and pose the arm so
  the wrist camera sees the board too. The sweep only turns the base,
  SHOULDER_SWING_TICKS either way: keep that arc clear.

Stop the dashboard first. Each run saves its pictures in a new folder under
~/.local/state/care-reach/calibration/ and the result in
~/.local/state/care-reach/cameras.json. ``--from DIR`` solves a saved run
again without moving the arm. Calibrate again after a camera is knocked on its
mount or set to another picture size.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import logging
from pathlib import Path
import time
from typing import Sequence

import cv2
import numpy as np

from arm_control.bus import URT2_PORT
from arm_control.bus_lock import BusBusyError
from arm_control.pose_runner import PoseMoveError
from motion_core import SafetyBounds
from pick.arm import PickArm, read_ticks
from pick.calibration import SQUARE_M, View, board_in_base, find_corners, pattern_image, solve_shoulder, solve_wrist
from pick.camera import SERVICE_URLS, Camera, CameraError
from pick.rig import CAMERAS_PATH, LinkPose, load_cameras, model_link_pose, save_camera


SESSIONS_DIR = Path.home() / ".local/state/care-reach/calibration"
CAMERAS = ("wrist", "shoulder")
LINKS = {"wrist": "gripper_frame_link", "shoulder": "shoulder_link"}

# Wrist sweep: offsets from the starting pose, in ticks. The lift and elbow
# pairs raise the arm; the rest turn the base and tilt and roll the wrist.
WRIST_ARM_SHAPES = ((0, 0), (-40, -30), (-80, -60))
WRIST_PANS = (-60, -30, 0, 30, 60)
WRIST_FLEXES = (-100, 0, 100)
WRIST_ROLLS = (-150, 0, 150)

# The wrist sweep keeps the jaw tip this close to its starting height and reach, or better.
SWEEP_SLACK_M = 0.005

# Shoulder sweep: the base turns this far either way (about 26 degrees), in steps.
SHOULDER_SWING_TICKS = 300
SHOULDER_STEP_TICKS = 50

# The measured mover returns within 15 ticks and then commands the exact
# target; give the joints this long to arrive before reading them.
SETTLE_S = 0.8

log = logging.getLogger(__name__)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    pattern = commands.add_parser("pattern", help="write the board as an image")
    pattern.add_argument("out", type=Path)
    pattern.add_argument("--square-px", type=int, default=224,
                         help="pixels per square; 224 shows one-inch squares on a 224 ppi screen")
    for camera in CAMERAS:
        command = commands.add_parser(camera, help=f"calibrate the {camera} camera")
        command.add_argument("--from", dest="session", type=Path, help="solve a saved run again; the arm stays still")
        command.add_argument("--square-mm", type=float, default=SQUARE_M * 1000, help="the board's square size")
        command.add_argument("--port", default=URT2_PORT, help="URT-2 serial port")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    if args.command == "pattern":
        cv2.imwrite(str(args.out), pattern_image(args.square_px))
        print(f"wrote {args.out}; show it at one image pixel per screen pixel, then measure a square")
        return 0

    link_pose = model_link_pose(SafetyBounds.from_files())
    try:
        session = args.session or capture(args.command, PickArm.from_files(args.port), link_pose)
    except (BusBusyError, PoseMoveError, CameraError) as exc:
        print(f"stopped: {exc}")
        return 1
    model, fit = solve(args.command, session, link_pose, args.square_mm / 1000)
    save_camera(args.command, model)
    print(f"{args.command} camera: {fit}; saved to {CAMERAS_PATH}")
    return 0


def capture(camera: str, arm: PickArm, link_pose: LinkPose) -> Path:
    """Sweep the arm, saving both cameras' pictures and the joint ticks at each pose."""
    session = SESSIONS_DIR / f"{datetime.now():%Y%m%d-%H%M%S}-{camera}"
    session.mkdir(parents=True)
    poses = wrist_sweep(arm, link_pose) if camera == "wrist" else shoulder_sweep(arm.ticks)
    log.info("%d poses; saving to %s", len(poses), session)

    with Camera(SERVICE_URLS["wrist"]) as wrist, Camera(SERVICE_URLS["shoulder"]) as shoulder:
        for index, pose in enumerate(poses):
            arm.move_to(pose)
            time.sleep(SETTLE_S)
            ticks = read_ticks(arm.port)
            (session / f"{index:02d}_wrist.jpg").write_bytes(wrist.grab())
            (session / f"{index:02d}_shoulder.jpg").write_bytes(shoulder.grab())
            with (session / "poses.jsonl").open("a") as file:
                file.write(json.dumps({"index": index, "ticks": list(ticks)}) + "\n")
            log.info("%02d %s", index, ticks)
    return session


def wrist_sweep(arm: PickArm, link_pose: LinkPose) -> list[tuple[int, ...]]:
    """Poses around the start that keep the jaw tip at least as high, and no farther out."""
    start = arm.ticks
    pan_axis = link_pose(LINKS["shoulder"], start)[:2, 3]  # the pan joint's link sits on its axis

    def tip(ticks):
        return arm.kinematics.forward(ticks)[:3, 3]

    def reach(point):
        return np.linalg.norm(point[:2] - pan_axis)

    lowest, farthest = tip(start)[2] - SWEEP_SLACK_M, reach(tip(start)) + SWEEP_SLACK_M
    poses = []
    for lift, elbow in WRIST_ARM_SHAPES:
        for pan in WRIST_PANS:
            for flex in WRIST_FLEXES:
                roll = WRIST_ROLLS[len(poses) % len(WRIST_ROLLS)]
                pose = (start[0] + pan, start[1] + lift, start[2] + elbow, start[3] + flex, start[4] + roll, start[5])
                if np.any(np.array(pose) < arm.bounds.lower) or np.any(np.array(pose) > arm.bounds.upper):
                    continue
                point = tip(pose)
                if point[2] >= lowest and reach(point) <= farthest:
                    poses.append(pose)
    return poses


def shoulder_sweep(start: tuple[int, ...]) -> list[tuple[int, ...]]:
    """The start, then the base turned each way and back."""
    offsets = range(SHOULDER_STEP_TICKS, SHOULDER_SWING_TICKS + 1, SHOULDER_STEP_TICKS)
    pans = [start[0] - offset for offset in offsets] + [start[0] + offset for offset in offsets] + [start[0]]
    return [start] + [(pan, *start[1:]) for pan in pans]


def solve(camera: str, session: Path, link_pose: LinkPose, square_m: float):
    records = [json.loads(line) for line in (session / "poses.jsonl").open()]

    def views(name):
        found = []
        for record in records:
            corners = find_corners((session / f"{record['index']:02d}_{name}.jpg").read_bytes())
            if corners is not None:
                found.append(View(link_pose(LINKS[name], tuple(record["ticks"])), *corners))
        if not found:
            raise SystemExit(f"The {name} camera saw the board in none of the pictures in {session}")
        return found

    size = _picture_size(session / f"00_{camera}.jpg")
    if camera == "wrist":
        return solve_wrist(views("wrist"), size, LINKS["wrist"], square_m)

    board_pose = board_in_base(views("wrist"), load_cameras()["wrist"], square_m)
    return solve_shoulder(views("shoulder"), board_pose, size, LINKS["shoulder"], square_m)


def _picture_size(path: Path) -> tuple[int, int]:
    height, width = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE).shape
    return width, height


if __name__ == "__main__":
    raise SystemExit(main())
