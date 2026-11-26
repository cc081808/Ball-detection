import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
import camera_input


def camera(name, index, backend):
    return SimpleNamespace(name=name, index=index, backend=backend)


class CameraTests(unittest.TestCase):
    def test_blank_frame_is_not_a_usable_image(self):
        self.assertFalse(camera_input.has_image(None))
        self.assertFalse(camera_input.has_image(np.zeros((0, 0, 3), dtype=np.uint8)))
        self.assertFalse(camera_input.has_image(np.zeros((8, 8, 3), dtype=np.uint8)))
        self.assertTrue(camera_input.has_image(np.full((8, 8, 3), 60, dtype=np.uint8)))

    def test_backend_switch_resolves_camo_index_by_name(self):
        selected = camera("Camo", 0, cv2.CAP_DSHOW)
        listings = {cv2.CAP_DSHOW: [selected], cv2.CAP_MSMF: [
            camera("FHD Camera", 0, cv2.CAP_MSMF), camera("Camo", 1, cv2.CAP_MSMF)]}
        with patch.object(camera_input, "enumerate_cameras", side_effect=listings.get):
            candidates = list(camera_input.camera_candidates(selected))
        self.assertEqual([(c.index, c.backend) for c in candidates], [(1, cv2.CAP_MSMF), (0, cv2.CAP_DSHOW)])

    def test_missing_camo_never_substitutes_laptop_camera(self):
        selected = camera("Camo", 0, cv2.CAP_DSHOW)
        with patch.object(camera_input, "enumerate_cameras", return_value=[camera("FHD", 0, cv2.CAP_MSMF)]):
            self.assertEqual(list(camera_input.camera_candidates(selected, "msmf")), [])

    def test_blank_backend_released_before_fallback(self):
        selected = camera("Camo", 0, cv2.CAP_DSHOW)
        alternative = camera("Camo", 1, cv2.CAP_MSMF)
        blank, good = Mock(), Mock()
        blank.isOpened.return_value = good.isOpened.return_value = True
        blank.read.return_value = (True, np.zeros((8, 8, 3), dtype=np.uint8))
        frame = np.full((8, 8, 3), 50, dtype=np.uint8)
        good.read.return_value = (True, frame)
        def construct(*args):
            if args[1] == cv2.CAP_DSHOW:
                blank.release.assert_called_once()
                return good
            return blank
        with patch.object(camera_input, "camera_candidates", return_value=iter([alternative, selected])), \
             patch.object(camera_input.cv2, "VideoCapture", side_effect=construct), \
             patch.object(camera_input.time, "monotonic", side_effect=[0, 0, 4, 4, 4]), \
             patch.object(camera_input.time, "sleep"):
            cap, first = camera_input.open_camera(selected)
        self.assertIs(cap, good)
        self.assertIs(first, frame)
        good.release.assert_not_called()

    def test_all_open_failures_close_handles_and_report_problem(self):
        selected = camera("Camo", 0, cv2.CAP_DSHOW)
        capture = Mock()
        capture.isOpened.return_value = False
        with patch.object(camera_input, "camera_candidates", return_value=iter([selected])), \
             patch.object(camera_input.cv2, "VideoCapture", return_value=capture):
            with self.assertRaisesRegex(RuntimeError, "No robot connection was started"):
                camera_input.open_camera(selected)
        capture.release.assert_called_once()
