"""Locating an object from both cameras' boxes, in a synthetic scene with known geometry."""

import unittest

import numpy as np

from pick.detector import Box
from pick.locate import NotLocated, closest_approach, locate
from pick_scene import cylinder_points, project, scene_rig


CENTER_XY = (0.30, 0.0)
RADIUS = 0.033
BOTTOM_Z, TOP_Z = -0.02, 0.10


class BoxingDetector:
    """Boxes the projection of each camera's points; the "pictures" are the camera names."""

    def __init__(self, rig, points_by_camera):
        self.boxes = {}
        for camera, points in points_by_camera.items():
            pixels = project(rig.camera_pose(camera, ()), rig.cameras[camera], points)
            self.boxes[camera] = Box(*pixels.min(axis=0), *pixels.max(axis=0), score=0.8)

    def find(self, jpeg, name):
        return self.boxes.get(jpeg.decode())


PICTURES = {"wrist": b"wrist", "shoulder": b"shoulder"}


class LocateTests(unittest.TestCase):
    def setUp(self):
        self.rig = scene_rig()
        self.can = cylinder_points(CENTER_XY, RADIUS, BOTTOM_Z, TOP_Z)

    def test_finds_the_object_and_its_size(self):
        detector = BoxingDetector(self.rig, {"wrist": self.can, "shoulder": self.can})
        target = locate("can", (), PICTURES, detector, self.rig)

        np.testing.assert_allclose(target.center, [*CENTER_XY, (BOTTOM_Z + TOP_Z) / 2], atol=0.002)
        self.assertAlmostEqual(target.width_m, 2 * RADIUS, delta=0.002)
        self.assertAlmostEqual(target.bottom_z_m, BOTTOM_Z, delta=0.002)
        self.assertAlmostEqual(target.top_z_m, TOP_Z, delta=0.002)

    def test_an_object_one_camera_misses_is_not_located(self):
        detector = BoxingDetector(self.rig, {"wrist": self.can})
        with self.assertRaisesRegex(NotLocated, "shoulder camera doesn't see 'can'"):
            locate("can", (), PICTURES, detector, self.rig)

    def test_cameras_matching_different_objects_disagree(self):
        elsewhere = cylinder_points((0.30, 0.08), RADIUS, BOTTOM_Z, TOP_Z)
        detector = BoxingDetector(self.rig, {"wrist": self.can, "shoulder": elsewhere})
        with self.assertRaisesRegex(NotLocated, "different objects"):
            locate("can", (), PICTURES, detector, self.rig)

    def test_closest_approach_of_two_skew_lines(self):
        along, gap = closest_approach(np.zeros(3), np.array([1.0, 0, 0]),
                                      np.array([2.0, -1, 0.5]), np.array([0, 1.0, 0]))
        self.assertAlmostEqual(along, 2.0)
        self.assertAlmostEqual(gap, 0.5)


if __name__ == "__main__":
    unittest.main()
