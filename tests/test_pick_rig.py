"""Camera models: lines of sight through pixels, and saving and loading calibrations."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from pick.rig import load_cameras, save_camera
from pick_scene import project, scene_rig


class RigTests(unittest.TestCase):
    def test_a_pixel_looks_back_along_the_line_to_what_it_shows(self):
        rig = scene_rig()
        point = np.array([0.32, 0.03, 0.02])
        for camera in ("wrist", "shoulder"):
            pixel = project(rig.camera_pose(camera, ()), rig.cameras[camera], [point])[0]
            origin, direction = rig.ray(camera, (), pixel)
            offset = point - origin
            miss = np.linalg.norm(offset - (offset @ direction) * direction)
            self.assertLess(miss, 1e-6, camera)

    def test_saving_one_camera_keeps_the_other(self):
        cameras = scene_rig().cameras
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "state" / "cameras.json"
            save_camera("wrist", cameras["wrist"], path)
            save_camera("shoulder", cameras["shoulder"], path)
            loaded = load_cameras(path)

        self.assertEqual(set(loaded), {"wrist", "shoulder"})
        for name, model in cameras.items():
            self.assertEqual(loaded[name].link, model.link)
            self.assertEqual(loaded[name].image_size, model.image_size)
            np.testing.assert_allclose(loaded[name].mount, model.mount)
            np.testing.assert_allclose(loaded[name].distortion, model.distortion)


if __name__ == "__main__":
    unittest.main()
