# Copyright (c) 2025-2026, Junjie Zhu.
#
# SPDX-License-Identifier: Apache-2.0
"""Batched torch implementation of the Piper arm kinematics and damped-least-squares IK.

The Denavit-Hartenberg-like chain matches ``piper_kinematics.py`` (the model used
on the ROS side) exactly.  It was cross-checked against the IsaacLab simulation:
``piper_tcp_position(q)`` and the simulated ``end_effector`` body position in the
``link0`` frame agree to 0.0 mm.

Running the solver as a batched tensor operation lets the VBC action term solve
the arm IK *to convergence* for thousands of parallel environments -- unlike the
built-in single-step differential IK, which is a local method seeded from the
current joint pose and therefore stalls ~5-10 cm short on the tabletop targets.
It can track the end-effector position only, or position and orientation jointly.
"""

from __future__ import annotations

import torch

# --- Piper chain constants (identical to the ROS piper_kinematics.py) ---------
_JOINT_POSITIONS = [
    [0.0, 0.0, 0.123],
    [0.0, 0.0, 0.0],
    [0.28503, 0.0, 0.0],
    [-0.021984, -0.25075, 0.0],
    [0.0, 0.0, 0.0],
    [8.8259e-5, -0.091, 0.0],
]
_JOINT_QUATERNIONS_WXYZ = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0356735, -0.0356786, -0.706207, -0.706205],
    [0.637536, 0.0, 0.0, -0.77042],
    [0.707105, 0.707108, 0.0, 0.0],
    [0.707105, -0.707108, 0.0, 0.0],
    [0.707105, 0.707108, 0.0, 0.0],
]
_JOINT_LIMITS = [
    [-2.618, 2.168],
    [0.0, 3.14],
    [-2.967, 0.0],
    [-1.745, 1.745],
    [-1.22, 1.22],
    [-2.0944, 2.0944],
]
_TCP_OFFSET = [0.0, 0.0, 0.13]


def _quat_wxyz_to_matrix(quat: torch.Tensor) -> torch.Tensor:
    """(..., 4) wxyz quaternion -> (..., 3, 3) rotation matrix."""
    w, x, y, z = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]
    row0 = torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], dim=-1)
    row1 = torch.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], dim=-1)
    row2 = torch.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], dim=-1)
    return torch.stack([row0, row1, row2], dim=-2)


def _rotation_matrix_to_rotvec(rotation: torch.Tensor) -> torch.Tensor:
    """(..., 3, 3) rotation matrix -> (..., 3) rotation vector (axis * angle)."""
    trace = rotation[..., 0, 0] + rotation[..., 1, 1] + rotation[..., 2, 2]
    cos_angle = torch.clamp(0.5 * (trace - 1.0), -1.0, 1.0)
    angle = torch.acos(cos_angle)
    skew = torch.stack(
        [
            rotation[..., 2, 1] - rotation[..., 1, 2],
            rotation[..., 0, 2] - rotation[..., 2, 0],
            rotation[..., 1, 0] - rotation[..., 0, 1],
        ],
        dim=-1,
    )
    sin_angle = torch.sin(angle)
    # scale -> 0.5 as angle -> 0, else angle / (2 sin angle)
    scale = torch.where(
        angle < 1.0e-6,
        torch.full_like(angle, 0.5),
        angle / torch.clamp(2.0 * sin_angle, min=1.0e-8),
    )
    return skew * scale.unsqueeze(-1)


class PiperKinematicsTorch:
    """Batched Piper FK + damped-least-squares IK on a torch device."""

    def __init__(self, device: torch.device | str, dtype: torch.dtype = torch.float32) -> None:
        self.device = torch.device(device)
        self.dtype = dtype
        self.num_joints = 6

        positions = torch.tensor(_JOINT_POSITIONS, dtype=dtype, device=self.device)
        quats = torch.tensor(_JOINT_QUATERNIONS_WXYZ, dtype=dtype, device=self.device)
        rotations = _quat_wxyz_to_matrix(quats)  # (6,3,3)
        origins = torch.zeros(6, 4, 4, dtype=dtype, device=self.device)
        origins[:, :3, :3] = rotations
        origins[:, :3, 3] = positions
        origins[:, 3, 3] = 1.0
        self._origins = origins  # (6,4,4)

        tcp = torch.eye(4, dtype=dtype, device=self.device)
        tcp[:3, 3] = torch.tensor(_TCP_OFFSET, dtype=dtype, device=self.device)
        self._tcp = tcp

        limits = torch.tensor(_JOINT_LIMITS, dtype=dtype, device=self.device)
        self.joint_lower = limits[:, 0]
        self.joint_upper = limits[:, 1]

    def fk(self, joint_pos: torch.Tensor) -> torch.Tensor:
        """Batched forward kinematics.  ``joint_pos`` (N, 6) -> transforms (N, 4, 4)."""
        n = joint_pos.shape[0]
        transform = torch.eye(4, dtype=self.dtype, device=self.device).expand(n, 4, 4).clone()
        for index in range(self.num_joints):
            cosine = torch.cos(joint_pos[:, index])
            sine = torch.sin(joint_pos[:, index])
            rotation = torch.zeros(n, 4, 4, dtype=self.dtype, device=self.device)
            rotation[:, 0, 0] = cosine
            rotation[:, 0, 1] = -sine
            rotation[:, 1, 0] = sine
            rotation[:, 1, 1] = cosine
            rotation[:, 2, 2] = 1.0
            rotation[:, 3, 3] = 1.0
            transform = transform @ self._origins[index] @ rotation
        return transform @ self._tcp

    def tcp_position(self, joint_pos: torch.Tensor) -> torch.Tensor:
        return self.fk(joint_pos)[:, :3, 3]

    def solve(
        self,
        target_position: torch.Tensor,
        seed: torch.Tensor,
        target_quaternion: torch.Tensor | None = None,
        max_iterations: int = 8,
        damping: float = 1.0e-4,
        step_clip: float = 0.15,
        finite_difference: float = 1.0e-4,
        orientation_weight: float = 0.05,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Damped-least-squares IK, seeded from ``seed`` and iterated to convergence.

        Returns ``(joint_pos, position_residual)``; both are (N,).
        """
        target_position = target_position.to(self.dtype)
        joints = torch.clamp(seed.to(self.dtype), self.joint_lower, self.joint_upper)
        use_orientation = target_quaternion is not None
        error_dim = 6 if use_orientation else 3
        eye = torch.eye(error_dim, dtype=self.dtype, device=self.device).expand(
            joints.shape[0], error_dim, error_dim
        )

        for _ in range(max_iterations):
            transform = self.fk(joints)
            current_position = transform[:, :3, 3]
            current_rotation = transform[:, :3, :3]
            position_error = target_position - current_position
            error = position_error
            if use_orientation:
                target_rotation = _quat_wxyz_to_matrix(target_quaternion.to(self.dtype))
                rotation_error = _rotation_matrix_to_rotvec(target_rotation @ current_rotation.transpose(1, 2))
                error = torch.cat([position_error, orientation_weight * rotation_error], dim=-1)

            jacobian = torch.empty(
                joints.shape[0], 6 if use_orientation else 3, self.num_joints,
                dtype=self.dtype, device=self.device,
            )
            for index in range(self.num_joints):
                offset = joints.clone()
                offset[:, index] += finite_difference
                offset_transform = self.fk(offset)
                jacobian[:, :3, index] = (offset_transform[:, :3, 3] - current_position) / finite_difference
                if use_orientation:
                    delta_rotation = offset_transform[:, :3, :3] @ current_rotation.transpose(1, 2)
                    jacobian[:, 3:, index] = (
                        orientation_weight * _rotation_matrix_to_rotvec(delta_rotation) / finite_difference
                    )
            # delta = J^T (J J^T + damping I)^-1 error
            jjt = jacobian @ jacobian.transpose(1, 2) + damping * eye
            delta = jacobian.transpose(1, 2) @ torch.linalg.solve(jjt, error.unsqueeze(-1))
            joints = joints + torch.clamp(delta.squeeze(-1), -step_clip, step_clip)
            joints = torch.clamp(joints, self.joint_lower, self.joint_upper)

        residual = (target_position - self.tcp_position(joints)).norm(dim=-1)
        return joints, residual
