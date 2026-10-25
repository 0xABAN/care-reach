"""Calibrate the cameras against a ChArUco board: each lens, and where each camera is mounted.

The board is the 9 x 6-square ChArUco pattern from `calibrate_cameras.py
pattern`, shown on a screen or printed. Its markers name every corner, so a
picture of part of the board still counts.

Wrist camera: the arm looks at a still board from many poses. The lens comes
from the pictures alone. The mount on the gripper comes from how the board
moves in the pictures as the gripper moves (OpenCV's hand-eye calibration).

Shoulder camera: the arm swings its base past a still board that both cameras
see. The shoulder camera turns with the pan, so its own pictures locate the
pan axis relative to the board. The wrist camera's view places the board
along and around that axis, which turning about it can't show.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from pick.rig import CameraModel, transform


SQUARES = (9, 6)
MARKER_FRACTION = 0.7
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

# The pattern shown with 224-pixel squares on a 224 ppi MacBook Air screen is
# one inch per square. The wrist calibration solved it as 25.35 mm from arm
# motion alone, which confirms that.
SQUARE_M = 0.0254

# A picture needs this many board corners to count.
MIN_CORNERS = 8

# Pan turns smaller than this say little about the pan axis's direction.
MIN_TURN_RAD = np.radians(3)


@dataclass(frozen=True)
class View:
    """One picture of the board: its mount link's base-frame pose, and the corners seen (ids, pixels)."""

    link_pose: np.ndarray
    ids: np.ndarray
    corners: np.ndarray


@dataclass(frozen=True)
class Fit:
    """How well a calibration explains its pictures."""

    views: int
    reprojection_px: float
    spread_mm: float

    def __str__(self) -> str:
        return f"{self.views} pictures, {self.reprojection_px:.2f} px reprojection, {self.spread_mm:.1f} mm spread"


def board(square_m: float = SQUARE_M) -> cv2.aruco.CharucoBoard:
    return cv2.aruco.CharucoBoard(SQUARES, square_m, square_m * MARKER_FRACTION, DICTIONARY)


def pattern_image(square_px: int) -> np.ndarray:
    """The board as a picture with ``square_px``-pixel squares and a one-square white margin."""
    size = ((SQUARES[0] + 2) * square_px, (SQUARES[1] + 2) * square_px)
    return board().generateImage(size, marginSize=square_px)


def _detector() -> cv2.aruco.CharucoDetector:
    # A screen seen at an angle shows small, low-contrast markers: search more
    # threshold windows and tolerate a few wrong bits, then let the board's
    # layout recover markers missed on the first pass.
    markers = cv2.aruco.DetectorParameters()
    markers.adaptiveThreshWinSizeMin = 3
    markers.adaptiveThreshWinSizeMax = 53
    markers.adaptiveThreshWinSizeStep = 4
    markers.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    markers.perspectiveRemovePixelPerCell = 8
    markers.perspectiveRemoveIgnoredMarginPerCell = 0.2
    markers.maxErroneousBitsInBorderRate = 0.5
    markers.errorCorrectionRate = 0.8
    charuco = cv2.aruco.CharucoParameters()
    charuco.tryRefineMarkers = True
    return cv2.aruco.CharucoDetector(board(), charuco, markers)


def find_corners(jpeg: bytes) -> tuple[np.ndarray, np.ndarray] | None:
    """The board corners in a JPEG, as (ids, pixels), or None if fewer than MIN_CORNERS show."""
    gray = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    corners, ids, _, _ = _detector().detectBoard(gray)
    if ids is None or len(ids) < MIN_CORNERS:
        return None
    return ids.ravel(), corners.reshape(-1, 2)


def solve_lens(views: list[View], image_size: tuple[int, int], square_m: float):
    """Fit the lens to the views; returns (matrix, distortion, reprojection px, board pose in each camera).

    A sweep shows the board from similar angles, which doesn't pin down every
    lens term: with all of them free, the wrist camera's fit settled on pixels
    12% taller than wide. Square pixels and two radial distortion terms fit
    both cameras as well (0.83 and 0.19 px) and come out the same every time.
    """
    corners_3d = board(square_m).getChessboardCorners()
    width, height = image_size
    guess = np.array([[width, 0, width / 2], [0, width, height / 2], [0, 0, 1]], float)
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_K3
             | cv2.CALIB_ZERO_TANGENT_DIST)
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        [corners_3d[view.ids].astype(np.float32) for view in views],
        [view.corners.astype(np.float32).reshape(-1, 1, 2) for view in views],
        tuple(image_size), guess, None, flags=flags)
    poses = [transform(cv2.Rodrigues(r)[0], t) for r, t in zip(rvecs, tvecs, strict=True)]
    return matrix, distortion.ravel(), float(rms), poses


def solve_wrist(views: list[View], image_size: tuple[int, int], link: str,
                square_m: float = SQUARE_M) -> tuple[CameraModel, Fit]:
    """The wrist camera's lens and its mount on ``link``, from views of one still board.

    The mount's rotation comes from OpenCV's hand-eye calibration. Its position
    is then the one that makes every view put the still board in the same
    place: the views barely constrain it otherwise, and OpenCV's methods gave
    positions 4 cm apart that fit the board equally well.
    """
    matrix, distortion, rms, board_in_camera = solve_lens(views, image_size, square_m)
    rotation, _ = cv2.calibrateHandEye(
        [view.link_pose[:3, :3] for view in views], [view.link_pose[:3, 3] for view in views],
        [pose[:3, :3] for pose in board_in_camera], [pose[:3, 3] for pose in board_in_camera],
        method=cv2.CALIB_HAND_EYE_PARK)

    # Each view: link rotation @ (rotation @ board-in-camera + mount position) + link position = board position.
    rows = [np.hstack([view.link_pose[:3, :3], -np.eye(3)]) for view in views]
    rhs = [-(view.link_pose[:3, :3] @ rotation @ pose[:3, 3] + view.link_pose[:3, 3])
           for view, pose in zip(views, board_in_camera, strict=True)]
    translation = np.linalg.lstsq(np.vstack(rows), np.hstack(rhs), rcond=None)[0][:3]
    model = CameraModel(link, tuple(image_size), matrix, distortion, transform(rotation, translation))

    # The board stood still, so every view should put its corners in the same place.
    corners_3d = board(square_m).getChessboardCorners()
    placed = np.array([_apply(view.link_pose @ model.mount @ pose, corners_3d)
                       for view, pose in zip(views, board_in_camera, strict=True)])
    spread = np.linalg.norm(placed - placed.mean(axis=0), axis=2).mean()
    return model, Fit(len(views), rms, spread * 1000)


def board_in_base(views: list[View], camera: CameraModel, square_m: float = SQUARE_M) -> np.ndarray:
    """The still board's base-frame pose, averaged over a calibrated camera's views of it."""
    corners_3d = board(square_m).getChessboardCorners()
    poses = []
    for view in views:
        _, rvec, tvec = cv2.solvePnP(corners_3d[view.ids], view.corners, camera.matrix, camera.distortion)
        poses.append(view.link_pose @ camera.mount @ transform(cv2.Rodrigues(rvec)[0], tvec))
    return transform(_mean_rotation([pose[:3, :3] for pose in poses]), np.mean([pose[:3, 3] for pose in poses], axis=0))


def solve_shoulder(views: list[View], board_pose: np.ndarray, image_size: tuple[int, int], link: str,
                   square_m: float = SQUARE_M) -> tuple[CameraModel, Fit]:
    """The shoulder camera's lens and its mount on ``link``, the link the base pan turns.

    ``views`` are the shoulder camera's pictures as the pan swings past a still
    board; ``board_pose`` is that board's base-frame pose from the wrist camera.
    """
    matrix, distortion, rms, board_in_camera = solve_lens(views, image_size, square_m)
    camera_in_board = [np.linalg.inv(pose) for pose in board_in_camera]

    # The pan axis in the base frame, from the link's own turns away from the first view.
    reference = views[0].link_pose
    turns = [_rotation_vector(view.link_pose[:3, :3] @ reference[:3, :3].T) for view in views]
    big = [turn for turn in turns if np.linalg.norm(turn) > MIN_TURN_RAD]
    if not big:
        raise ValueError("The pan barely turned between pictures; swing it farther")
    axis = _unit(sum(turn * np.sign(turn @ big[0]) for turn in big))
    angles = [turn @ axis for turn in turns]
    axis_point = reference[:3, 3]  # the link's origin sits on its joint's axis

    # The same axis in the board frame: the camera turned about it by the same angles.
    in_board = [_rotation_vector(pose[:3, :3] @ camera_in_board[0][:3, :3].T) / angle
                for pose, angle in zip(camera_in_board, angles, strict=True) if abs(angle) > MIN_TURN_RAD]
    axis_in_board = _unit(np.mean(in_board, axis=0))
    first_center = camera_in_board[0][:3, 3]
    rows, rhs = [], []
    for pose, angle in zip(camera_in_board, angles, strict=True):
        turn = cv2.Rodrigues(axis_in_board * angle)[0]
        rows.append(np.eye(3) - turn)
        rhs.append(pose[:3, 3] - turn @ first_center)
    point_in_board = np.linalg.lstsq(np.vstack(rows), np.hstack(rhs), rcond=None)[0]

    # Tilt the wrist camera's board pose so the two axes line up, about the board's middle,
    # then shift it sideways onto the axis. Its position along and around the axis stays.
    middle = board(square_m).getChessboardCorners().mean(axis=0)
    tilt = _rotation_between(board_pose[:3, :3] @ axis_in_board, axis)
    rotation = tilt @ board_pose[:3, :3]
    translation = _apply(board_pose, middle) - rotation @ middle
    offset = axis_point - (rotation @ point_in_board + translation)
    board_pose = transform(rotation, translation + offset - (offset @ axis) * axis)

    mounts = [np.linalg.inv(view.link_pose) @ board_pose @ pose
              for view, pose in zip(views, camera_in_board, strict=True)]
    mount = transform(_mean_rotation([m[:3, :3] for m in mounts]), np.mean([m[:3, 3] for m in mounts], axis=0))
    spread = np.mean([np.linalg.norm(m[:3, 3] - mount[:3, 3]) for m in mounts])

    model = CameraModel(link, tuple(image_size), matrix, distortion, mount)
    return model, Fit(len(views), _reprojection(model, views, board_pose, square_m), spread * 1000)


def _reprojection(camera: CameraModel, views: list[View], board_pose: np.ndarray, square_m: float) -> float:
    """RMS pixel error of the whole model: board pose, link poses, mount, and lens together."""
    corners_3d = board(square_m).getChessboardCorners()
    errors = []
    for view in views:
        board_in_camera = np.linalg.inv(view.link_pose @ camera.mount) @ board_pose
        projected, _ = cv2.projectPoints(corners_3d[view.ids], cv2.Rodrigues(board_in_camera[:3, :3])[0],
                                         board_in_camera[:3, 3], camera.matrix, camera.distortion)
        errors.append(projected.reshape(-1, 2) - view.corners)
    return float(np.sqrt(np.mean(np.vstack(errors) ** 2) * 2))


def _apply(pose: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ pose[:3, :3].T + pose[:3, 3]


def _unit(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    return cv2.Rodrigues(rotation)[0].ravel()


def _rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """The smallest rotation taking direction ``a`` to direction ``b``."""
    a, b = _unit(a), _unit(b)
    cross = np.cross(a, b)
    if np.linalg.norm(cross) < 1e-12:
        return np.eye(3)
    return cv2.Rodrigues(_unit(cross) * np.arctan2(np.linalg.norm(cross), a @ b))[0]


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    u, _, vt = np.linalg.svd(sum(rotations))
    return u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
