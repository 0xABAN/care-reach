"""The calibration solvers recover known lenses and mounts from synthetic pictures of the board."""

import unittest

import cv2
import numpy as np

from pick.calibration import SQUARE_M, View, board, solve_shoulder, solve_wrist
from pick.rig import CameraModel, transform
from pick_scene import SHOULDER_SIZE, WRIST_SIZE, lens, look_at, project, rotation


CORNERS = board(SQUARE_M).getChessboardCorners().astype(float)
MIDDLE = CORNERS.mean(axis=0)


def views_of(board_pose, camera_poses, link_poses, camera):
    """The corners each camera pose sees inside its picture, as calibration views."""
    views = []
    points = CORNERS @ board_pose[:3, :3].T + board_pose[:3, 3]
    for camera_pose, link_pose in zip(camera_poses, link_poses, strict=True):
        pixels = project(camera_pose, camera, points)
        inside = np.all((pixels >= 0) & (pixels < camera.image_size), axis=1)
        if inside.sum() >= 8:
            views.append(View(link_pose, np.flatnonzero(inside), pixels[inside]))
    return views


def angle_between(a, b):
    return np.degrees(np.linalg.norm(cv2.Rodrigues(a[:3, :3] @ b[:3, :3].T)[0]))


class WristTests(unittest.TestCase):
    def test_recovers_the_mount_on_the_gripper(self):
        # A flat board on the table, and a camera behind and beside the jaws, tipped toward them.
        board_pose = transform(np.eye(3), [0.20, -0.10, 0.0])
        mount = transform(rotation((1, 1, 0), 30), [0.041, 0.036, -0.095])
        camera = CameraModel("gripper_frame_link", WRIST_SIZE, lens(WRIST_SIZE, 1040.0), np.zeros(5), mount)

        rng = np.random.default_rng(1)
        target = board_pose[:3, :3] @ MIDDLE + board_pose[:3, 3]
        camera_poses = [look_at(target + [*rng.uniform(-0.12, 0.12, 2), rng.uniform(0.18, 0.3)],
                                target + rng.uniform(-0.03, 0.03, 3), up=(*rng.uniform(-1, 1, 2), 0.0))
                        for _ in range(15)]
        gripper_poses = [pose @ np.linalg.inv(mount) for pose in camera_poses]

        model, fit = solve_wrist(views_of(board_pose, camera_poses, gripper_poses, camera), WRIST_SIZE,
                                 "gripper_frame_link")

        self.assertLess(fit.reprojection_px, 0.1)
        self.assertLess(fit.spread_mm, 0.5)
        np.testing.assert_allclose(model.mount[:3, 3], mount[:3, 3], atol=0.001)
        self.assertLess(angle_between(model.mount, mount), 0.2)


class ShoulderTests(unittest.TestCase):
    def test_recovers_the_mount_and_corrects_the_boards_tilt(self):
        # The board propped up at a slant, facing a camera low on the shoulder.
        board_pose = transform(rotation((0, 1, 0), -60) @ rotation((0, 0, 1), 90), [0.30, -0.10, 0.06])
        board_middle = board_pose[:3, :3] @ MIDDLE + board_pose[:3, 3]
        axis_point = np.array([0.0388, 0.0, 0.06])
        first_camera = look_at(axis_point + [-0.04, 0.014, 0.02], board_middle + [0, 0.04, 0])
        mount = transform(np.eye(3), -axis_point) @ first_camera
        camera = CameraModel("shoulder_link", SHOULDER_SIZE, lens(SHOULDER_SIZE, 750.0), np.zeros(5), mount)

        shoulder_poses = [transform(rotation((0, 0, 1), pan), axis_point) for pan in range(-18, 19, 3)]
        camera_poses = [pose @ mount for pose in shoulder_poses]
        views = views_of(board_pose, camera_poses, shoulder_poses, camera)

        # The wrist camera's estimate of the board: tipped 3 degrees and 1 cm off the pan axis.
        tipped = transform(rotation((1, 0, 0), 3), [0, 0, 0])
        about_middle = transform(np.eye(3), board_middle + [0.01, 0, 0]) @ tipped @ transform(np.eye(3), -board_middle)
        wrist_estimate = about_middle @ board_pose

        model, fit = solve_shoulder(views, wrist_estimate, SHOULDER_SIZE, "shoulder_link")

        self.assertGreaterEqual(fit.views, 10)
        self.assertLess(fit.reprojection_px, 0.5)
        np.testing.assert_allclose(model.matrix[0, 0], 750.0, rtol=0.01)
        np.testing.assert_allclose(model.mount[:3, 3], mount[:3, 3], atol=0.002)
        self.assertLess(angle_between(model.mount, mount), 0.3)


if __name__ == "__main__":
    unittest.main()
