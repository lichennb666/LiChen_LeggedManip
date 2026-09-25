from __future__ import annotations

import numpy as np


JOINT_POSITIONS = np.array([
    [0.0, 0.0, 0.123], [0.0, 0.0, 0.0], [0.28503, 0.0, 0.0],
    [-0.021984, -0.25075, 0.0], [0.0, 0.0, 0.0],
    [8.8259e-5, -0.091, 0.0],
], np.float64)
JOINT_QUATERNIONS_WXYZ = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0356735, -0.0356786, -0.706207, -0.706205],
    [0.637536, 0.0, 0.0, -0.77042],
    [0.707105, 0.707108, 0.0, 0.0],
    [0.707105, -0.707108, 0.0, 0.0],
    [0.707105, 0.707108, 0.0, 0.0],
], np.float64)
JOINT_LIMITS = np.array([
    [-2.618, 2.168], [0.0, 3.14], [-2.967, 0.0],
    [-1.745, 1.745], [-1.22, 1.22], [-2.0944, 2.0944],
], np.float64)


def _quaternion_matrix_wxyz(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], np.float64)


def piper_tcp_transform(joint_positions: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    for angle, position, quaternion in zip(
            joint_positions, JOINT_POSITIONS, JOINT_QUATERNIONS_WXYZ):
        origin = np.eye(4, dtype=np.float64)
        origin[:3, :3] = _quaternion_matrix_wxyz(quaternion)
        origin[:3, 3] = position
        transform = transform @ origin
        cosine, sine = np.cos(angle), np.sin(angle)
        rotation = np.eye(4, dtype=np.float64)
        rotation[:3, :3] = [
            [cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
        transform = transform @ rotation
    tcp = np.eye(4, dtype=np.float64)
    tcp[:3, 3] = [0.0, 0.0, 0.13]
    tcp[:3, :3] = [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]
    return transform @ tcp


def piper_tcp_position(joint_positions: np.ndarray) -> np.ndarray:
    return piper_tcp_transform(joint_positions)[:3, 3].astype(np.float32)


def piper_tcp_rotation(joint_positions: np.ndarray) -> np.ndarray:
    return piper_tcp_transform(joint_positions)[:3, :3].astype(np.float32)


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    cosine = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    angle = float(np.arccos(cosine))
    if angle < 1.0e-8:
        return np.zeros(3, np.float64)
    skew = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ], np.float64)
    return angle * skew / max(2.0 * float(np.sin(angle)), 1.0e-8)


def solve_piper_pose_ik(
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    seed: np.ndarray,
    max_iterations: int = 60,
    orientation_weight: float = 0.035,
) -> tuple[np.ndarray, float, float]:
    """Damped least-squares pose IK for Piper TCP.

    The position residual is measured in metres.  The orientation residual is a
    rotation vector in radians, scaled by ``orientation_weight`` so it can be
    solved together with position without letting wrist rotation dominate the
    small tabletop Cartesian task.
    """
    target_position = np.asarray(target_position, np.float64)
    target_rotation = np.asarray(target_rotation, np.float64)
    joints = np.clip(np.asarray(seed, np.float64), JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    finite_difference = 1.0e-5
    damping = 3.0e-4
    for _ in range(max_iterations):
        transform = piper_tcp_transform(joints)
        current_position = transform[:3, 3]
        current_rotation = transform[:3, :3]
        position_error = target_position - current_position
        rotation_error = _rotation_vector(target_rotation @ current_rotation.T)
        error = np.concatenate([position_error, orientation_weight * rotation_error])
        if np.linalg.norm(position_error) < 7.5e-4 and np.linalg.norm(rotation_error) < 0.06:
            break
        jacobian = np.empty((6, 6), np.float64)
        for index in range(6):
            offset = joints.copy()
            offset[index] += finite_difference
            offset_transform = piper_tcp_transform(offset)
            jacobian[:3, index] = (
                offset_transform[:3, 3] - current_position) / finite_difference
            delta_rotation = offset_transform[:3, :3] @ current_rotation.T
            jacobian[3:, index] = (
                orientation_weight * _rotation_vector(delta_rotation) / finite_difference)
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping * np.eye(6), error)
        joints += np.clip(delta, -0.06, 0.06)
        joints = np.clip(joints, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    transform = piper_tcp_transform(joints)
    position_residual = float(np.linalg.norm(target_position - transform[:3, 3]))
    orientation_residual = float(np.linalg.norm(
        _rotation_vector(target_rotation @ transform[:3, :3].T)))
    return joints.astype(np.float32), position_residual, orientation_residual


def solve_piper_position_ik(
    target_position: np.ndarray,
    seed: np.ndarray,
    max_iterations: int = 40,
) -> tuple[np.ndarray, float]:
    """Damped least-squares position IK for the training-model Piper chain."""
    target = np.asarray(target_position, np.float64)
    joints = np.clip(np.asarray(seed, np.float64), JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    finite_difference = 1.0e-5
    for _ in range(max_iterations):
        current = piper_tcp_position(joints).astype(np.float64)
        error = target - current
        if np.linalg.norm(error) < 5.0e-4:
            break
        jacobian = np.empty((3, 6), np.float64)
        for index in range(6):
            offset = joints.copy()
            offset[index] += finite_difference
            jacobian[:, index] = (
                piper_tcp_position(offset).astype(np.float64) - current
            ) / finite_difference
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + 1.0e-4 * np.eye(3), error)
        joints += np.clip(delta, -0.08, 0.08)
        joints = np.clip(joints, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    residual = float(np.linalg.norm(
        target - piper_tcp_position(joints).astype(np.float64)))
    return joints.astype(np.float32), residual
