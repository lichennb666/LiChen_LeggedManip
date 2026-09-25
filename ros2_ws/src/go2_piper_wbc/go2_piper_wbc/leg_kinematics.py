from __future__ import annotations

import numpy as np


LEG_ORDER = ("FR", "FL", "RR", "RL")
HIP_X = {"FR": 0.1934, "FL": 0.1934, "RR": -0.1934, "RL": -0.1934}
SIDE = {"FR": -1.0, "FL": 1.0, "RR": -1.0, "RL": 1.0}
JOINT_LIMITS = np.array([
    [-1.0472, 1.0472], [-1.5708, 4.5379], [-2.7227, -0.83776]
], np.float64)


def _rotation_x(angle: float) -> np.ndarray:
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]], np.float64)


def _rotation_y(angle: float) -> np.ndarray:
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]], np.float64)


def leg_foot_position(leg: str, joints: np.ndarray) -> np.ndarray:
    side = SIDE[leg]
    position = np.array([HIP_X[leg], side * 0.0465, 0.0], np.float64)
    rotation = _rotation_x(float(joints[0]))
    position += rotation @ np.array([0.0, side * 0.0955, 0.0])
    rotation = rotation @ _rotation_y(float(joints[1]))
    position += rotation @ np.array([0.0, 0.0, -0.213])
    rotation = rotation @ _rotation_y(float(joints[2]))
    position += rotation @ np.array([0.0, 0.0, -0.213])
    return position.astype(np.float32)


def solve_leg_position_ik(
    leg: str, target_position: np.ndarray, seed: np.ndarray,
    max_iterations: int = 20,
) -> tuple[np.ndarray, float]:
    target = np.asarray(target_position, np.float64)
    joints = np.clip(np.asarray(seed, np.float64), JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    finite_difference = 1.0e-5
    for _ in range(max_iterations):
        current = leg_foot_position(leg, joints).astype(np.float64)
        error = target - current
        if np.linalg.norm(error) < 2.0e-4:
            break
        jacobian = np.empty((3, 3), np.float64)
        for index in range(3):
            offset = joints.copy()
            offset[index] += finite_difference
            jacobian[:, index] = (
                leg_foot_position(leg, offset).astype(np.float64) - current
            ) / finite_difference
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + 2.0e-4 * np.eye(3), error)
        joints += np.clip(delta, -0.06, 0.06)
        joints = np.clip(joints, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    residual = float(np.linalg.norm(
        target - leg_foot_position(leg, joints).astype(np.float64)))
    return joints.astype(np.float32), residual
