import cv2
import numpy as np
from go2_piper_perception.detector_core import StabilityFilter, detect_colored_object


def test_green_cube_projects_to_camera_center() -> None:
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(image, (290, 210), (350, 270), (0, 255, 0), -1)
    depth = np.full((480, 640), 1.0, np.float32)
    k = np.array([[400, 0, 320], [0, 400, 240], [0, 0, 1]], np.float32)
    detection = detect_colored_object(image, depth, k)
    assert detection is not None
    np.testing.assert_allclose(detection.point_camera, [0, 0, 1], atol=0.01)


def test_invalid_depth_is_rejected() -> None:
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(image, (290, 210), (350, 270), (0, 255, 0), -1)
    assert detect_colored_object(image, np.zeros((480, 640), np.float32), np.eye(3)) is None


def test_stability_requires_five_consistent_frames() -> None:
    filt = StabilityFilter(count=5, max_std=0.015)
    stable = False
    for offset in [0.0, 0.001, -0.001, 0.002, 0.0]:
        stable, mean, _ = filt.update(np.array([0.5 + offset, 0.0, 0.4]))
    assert stable
    np.testing.assert_allclose(mean, [0.5004, 0.0, 0.4], atol=1e-4)

