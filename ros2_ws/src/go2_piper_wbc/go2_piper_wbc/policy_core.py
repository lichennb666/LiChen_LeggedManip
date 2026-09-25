"""Pure numerical helpers shared by the ROS node and unit tests."""

from __future__ import annotations

from collections import deque
import math
import numpy as np

from .policy_contract import load_policy_contract

CONTRACT = load_policy_contract()
JOINT_NAMES = list(CONTRACT.joint_names)


def normalize_quaternion_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float32)
    norm = float(np.linalg.norm(q))
    return q / norm if norm > 1.0e-8 else np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)


def quat_conjugate_xyzw(q: np.ndarray) -> np.ndarray:
    q = normalize_quaternion_xyzw(q)
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float32)


def quat_multiply_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = normalize_quaternion_xyzw(a)
    bx, by, bz, bw = normalize_quaternion_xyzw(b)
    return normalize_quaternion_xyzw(np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=np.float32))


def rotation_matrix_xyzw(q: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


def projected_gravity(q_base_world_xyzw: np.ndarray) -> np.ndarray:
    return rotation_matrix_xyzw(q_base_world_xyzw).T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)


def world_pose_to_mixed_command(
    target_position_world: np.ndarray,
    target_orientation_world_xyzw: np.ndarray,
    base_position_world: np.ndarray,
    base_orientation_world_xyzw: np.ndarray,
) -> np.ndarray:
    rot_world_base = rotation_matrix_xyzw(base_orientation_world_xyzw)
    delta_base = rot_world_base.T @ (np.asarray(target_position_world) - np.asarray(base_position_world))
    q_target_base = quat_multiply_xyzw(
        quat_conjugate_xyzw(base_orientation_world_xyzw), target_orientation_world_xyzw
    )
    return np.array([
        delta_base[0], delta_base[1], target_position_world[2],
        q_target_base[3], q_target_base[0], q_target_base[1], q_target_base[2],
    ], dtype=np.float32)


def pointing_pose_command(
    x_reference: float,
    y_reference: float,
    z_world: float,
    reference_z_world: float,
) -> np.ndarray:
    """Build the training command whose tool x-axis points at the target."""
    pitch = -math.atan2(z_world - reference_z_world, math.hypot(x_reference, y_reference))
    yaw = math.atan2(y_reference, x_reference)
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    # q = q_y(pitch) * q_z(yaw), returned in policy wxyz order.
    return np.array([
        x_reference,
        y_reference,
        z_world,
        math.cos(half_pitch) * math.cos(half_yaw),
        -math.sin(half_pitch) * math.sin(half_yaw),
        math.sin(half_pitch) * math.cos(half_yaw),
        math.cos(half_pitch) * math.sin(half_yaw),
    ], dtype=np.float32)


def slew_pose_command(
    current: np.ndarray,
    target: np.ndarray,
    position_step: float,
    angular_step: float,
) -> np.ndarray:
    """Rate-limit a mixed-frame pose command with shortest-path quaternion interpolation."""
    current = np.asarray(current, dtype=np.float32).copy()
    target = np.asarray(target, dtype=np.float32)
    delta = target[:3] - current[:3]
    distance = float(np.linalg.norm(delta))
    if distance > position_step > 0.0:
        current[:3] += delta * (position_step / distance)
    else:
        current[:3] = target[:3]

    q0 = current[3:7] / max(float(np.linalg.norm(current[3:7])), 1.0e-8)
    q1 = target[3:7] / max(float(np.linalg.norm(target[3:7])), 1.0e-8)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    angle = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
    fraction = 1.0 if angle <= angular_step or angle < 1.0e-8 else angular_step / angle
    quaternion = (1.0 - fraction) * q0 + fraction * q1
    current[3:7] = quaternion / max(float(np.linalg.norm(quaternion)), 1.0e-8)
    return current


class ObservationHistory:
    """Maintain the policy's per-field three-frame history layout."""

    FIELD_SIZES = (3, 3, 18, 18, 18, 3, 7)

    def __init__(self, depth: int = CONTRACT.history_depth):
        self.depth = depth
        self.fields = [deque(maxlen=depth) for _ in self.FIELD_SIZES]

    def append(self, values: tuple[np.ndarray, ...]) -> np.ndarray:
        if len(values) != len(self.fields):
            raise ValueError("expected seven observation fields")
        chunks = []
        for value, size, history in zip(values, self.FIELD_SIZES, self.fields):
            array = np.asarray(value, dtype=np.float32).reshape(-1)
            if array.size != size:
                raise ValueError(f"field has {array.size} elements, expected {size}")
            # Isaac Lab's CircularBuffer repeats the first sample across the
            # complete history window.  Matching that reset behavior avoids
            # feeding an out-of-distribution zero/zero/current observation to
            # the policy on activation.
            if not history:
                history.extend(array.copy() for _ in range(self.depth))
            else:
                history.append(array.copy())
            chunks.append(np.concatenate(tuple(history)))
        result = np.concatenate(chunks).astype(np.float32)
        if result.size != 210:
            raise RuntimeError(f"invalid policy observation size {result.size}")
        return np.clip(result, -CONTRACT.observation_clip, CONTRACT.observation_clip)
