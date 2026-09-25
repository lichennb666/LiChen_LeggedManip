from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import cv2
import numpy as np


@dataclass(frozen=True)
class Detection3D:
    point_camera: np.ndarray
    centroid: tuple[int, int]
    pixel_area: int
    confidence: float
    mask: np.ndarray


def detect_colored_object(
    bgr: np.ndarray,
    depth: np.ndarray,
    camera_matrix: np.ndarray,
    lower_hsv=(45, 100, 80),
    upper_hsv=(85, 255, 255),
    min_area: int = 250,
) -> Detection3D | None:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_hsv, np.uint8), np.array(upper_hsv, np.uint8))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(contour))
    if area < min_area:
        return None
    moments = cv2.moments(contour)
    if moments["m00"] <= 0:
        return None
    u = int(moments["m10"] / moments["m00"])
    v = int(moments["m01"] / moments["m00"])
    if u < 3 or v < 3 or u >= bgr.shape[1] - 3 or v >= bgr.shape[0] - 3:
        return None
    inner = np.zeros_like(mask)
    cv2.drawContours(inner, [contour], -1, 255, thickness=-1)
    inner = cv2.erode(inner, np.ones((7, 7), np.uint8))
    values = np.asarray(depth)[inner > 0].astype(np.float32)
    if np.issubdtype(depth.dtype, np.integer):
        values *= 0.001
    values = values[np.isfinite(values) & (values > 0.1) & (values < 5.0)]
    if values.size < 20:
        return None
    z = float(np.median(values))
    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
    point = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], np.float32)
    fill_ratio = area / max(float(cv2.contourArea(cv2.convexHull(contour))), 1.0)
    confidence = float(np.clip(fill_ratio * min(1.0, area / (4.0 * min_area)), 0.0, 1.0))
    return Detection3D(point, (u, v), area, confidence, mask)


class StabilityFilter:
    def __init__(self, count: int = 5, max_std: float = 0.015):
        self.points = deque(maxlen=count)
        self.max_std = max_std

    def update(self, point: np.ndarray) -> tuple[bool, np.ndarray, np.ndarray]:
        self.points.append(np.asarray(point, np.float32))
        array = np.stack(tuple(self.points))
        mean = np.mean(array, axis=0)
        std = np.std(array, axis=0)
        return len(self.points) == self.points.maxlen and float(np.max(std)) < self.max_std, mean, std

