"""A synthetic two-camera scene for the pick's geometry tests: known cameras, links, and objects."""

import cv2
import numpy as np

from pick.rig import CameraModel, Rig, transform


WRIST_SIZE = (1280, 720)
SHOULDER_SIZE = (640, 480)


def rotation(axis, degrees):
    return cv2.Rodrigues(np.asarray(axis, float) / np.linalg.norm(axis) * np.radians(degrees))[0]


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """A camera pose at ``eye`` looking at ``target``, with picture-up roughly along ``up``."""
    forward = np.asarray(target, float) - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    return transform(np.column_stack([right, np.cross(forward, right), forward]), eye)


def lens(size, focal):
    return np.array([[focal, 0, size[0] / 2], [0, focal, size[1] / 2], [0, 0, 1.0]])


def project(camera_pose, camera, points):
    """Pixels where ``camera`` at ``camera_pose`` sees base-frame ``points``."""
    world_to_camera = np.linalg.inv(camera_pose)
    pixels, _ = cv2.projectPoints(np.asarray(points, float), cv2.Rodrigues(world_to_camera[:3, :3])[0],
                                  world_to_camera[:3, 3], camera.matrix, camera.distortion)
    return pixels.reshape(-1, 2)


def scene_rig():
    """A wrist camera above and behind an object near (0.3, 0, 0), and a low shoulder camera beside the base.

    Both links sit at the base origin, so each camera's mount is its base-frame pose.
    """
    wrist_pose = look_at(np.array([0.15, -0.05, 0.25]), [0.30, 0.0, 0.05])
    # The shoulder camera's picture is turned on its side, as the w200's is.
    turned = transform(rotation((0, 0, 1), 90), [0, 0, 0])
    shoulder_pose = look_at(np.array([-0.10, 0.03, 0.10]), [0.30, 0.0, 0.05]) @ turned
    cameras = {
        "wrist": CameraModel("gripper_frame_link", WRIST_SIZE, lens(WRIST_SIZE, 1040.0), np.array([-0.3, 0.1, 0, 0, 0]),
                             wrist_pose),
        "shoulder": CameraModel("shoulder_link", SHOULDER_SIZE, lens(SHOULDER_SIZE, 750.0), np.zeros(5),
                                shoulder_pose),
    }
    return Rig(cameras, lambda link, ticks: np.eye(4))


def cylinder_points(center_xy, radius, bottom_z, top_z, count=72):
    angles = np.linspace(0, 2 * np.pi, count, endpoint=False)
    rims = [np.column_stack([center_xy[0] + radius * np.cos(angles), center_xy[1] + radius * np.sin(angles),
                             np.full(count, z)]) for z in (bottom_z, top_z)]
    return np.vstack(rims)
