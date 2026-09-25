from __future__ import annotations

import math
import numpy as np


STATES = (
    "INIT", "WBC_STAND", "DRIVE_PICK", "DETECT", "PREGRASP", "DESCEND",
    "CLOSE", "VERIFY", "LIFT_STOW", "DRIVE_DROP", "PLACE", "RELEASE", "DONE", "ABORT",
)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ], np.float32)


def rotation_matrix_xyzw(quaternion) -> np.ndarray:
    """Return the 3x3 rotation matrix for an xyzw quaternion."""
    x, y, z, w = map(float, quaternion)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], np.float32)


def transformed_point(translation, quaternion, local_point) -> np.ndarray:
    """Transform a link-local point into the parent frame."""
    return (np.asarray(translation, np.float32)
            + rotation_matrix_xyzw(quaternion)
            @ np.asarray(local_point, np.float32))


def visual_servo_tcp_target(tcp_position, pad_midpoint, desired_pad_position,
                            max_step: float) -> tuple[np.ndarray, np.ndarray]:
    """Convert finger-pad alignment error into a bounded world TCP command."""
    tcp = np.asarray(tcp_position, np.float32)
    error = (np.asarray(desired_pad_position, np.float32)
             - np.asarray(pad_midpoint, np.float32))
    distance = float(np.linalg.norm(error))
    if max_step <= 0.0:
        raise ValueError("max_step must be positive")
    correction = error
    if distance > max_step:
        correction = error * (max_step / distance)
    return tcp + correction, error


def surface_point_to_object_center(surface_point, camera_position,
                                   depth_offset: float) -> np.ndarray:
    """Move a visible surface point away from the camera along its view ray."""
    surface = np.asarray(surface_point, np.float32)
    ray = surface - np.asarray(camera_position, np.float32)
    distance = float(np.linalg.norm(ray))
    if distance <= 1.0e-6:
        raise ValueError("surface point and camera position must differ")
    return surface + float(depth_offset) * ray / distance


def grasp_retention(cube_position, pad_midpoint, finger_positions,
                    maximum_offset: float, minimum_aperture: float,
                    minimum_height: float) -> tuple[bool, float, float]:
    """Check that a physically pinched object is still between the fingers."""
    cube = np.asarray(cube_position, np.float32)
    pad = np.asarray(pad_midpoint, np.float32)
    fingers = np.asarray(finger_positions, np.float32)
    offset = float(np.linalg.norm(cube - pad))
    aperture = float(abs(fingers[0] - fingers[1]))
    retained = bool(
        np.all(np.isfinite(cube))
        and np.all(np.isfinite(pad))
        and np.all(np.isfinite(fingers))
        and offset <= maximum_offset
        and aperture >= minimum_aperture
        and cube[2] >= minimum_height)
    return retained, offset, aperture


def waypoint_command(current_xy: np.ndarray, yaw: float, goal_xy: np.ndarray,
                     max_speed: float = 0.1) -> tuple[np.ndarray, float]:
    delta_world = np.asarray(goal_xy, np.float32) - np.asarray(current_xy, np.float32)
    distance = float(np.linalg.norm(delta_world))
    c, s = math.cos(yaw), math.sin(yaw)
    delta_body = np.array([c * delta_world[0] + s * delta_world[1],
                           -s * delta_world[0] + c * delta_world[1]], np.float32)
    if distance > 1.0e-6:
        velocity = delta_body / distance * min(max_speed, 0.8 * distance)
    else:
        velocity = np.zeros(2, np.float32)
    return velocity, distance


def wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
