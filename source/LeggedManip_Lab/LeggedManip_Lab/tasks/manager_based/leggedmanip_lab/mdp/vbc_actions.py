# Copyright (c) 2025-2026, Junjie Zhu.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""[VBC-PHYSICAL-GRIPPER] Hierarchical action term for a VBC teacher.

The trainable policy emits a normalized 10-D Go2 adapter command:

    [vx, vy, wz, ee_dx, ee_dy, ee_dz, ee_droll, ee_dpitch, ee_dyaw, gripper]

The action term converts the six EE pose increments to the existing 7-D
mixed-frame ``[position, quaternion]`` command and runs the frozen 18-D WBC
TorchScript policy.  The last scalar is a binary high-level open/close command
which is mirrored to physical Piper prismatic ``joint7`` and ``joint8``.

This is deliberately not a 20-D low-level policy.  The frozen WBC still owns
12 leg targets and the six arm targets; the physical gripper is a separate
high-level actuator, matching the paper's decomposition.
"""

from __future__ import annotations

import os
from collections.abc import Sequence
from dataclasses import MISSING, field
from pathlib import Path
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.controllers import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.envs.mdp.actions.task_space_actions import DifferentialInverseKinematicsAction
from isaaclab.managers import ActionTerm, ActionTermCfg, ObservationGroupCfg, ObservationManager
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_from_euler_xyz, quat_mul, subtract_frame_transforms
from isaaclab.utils import configclass
from isaaclab.utils.assets import check_file_path, read_file

from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab.mdp.piper_ik_torch import PiperKinematicsTorch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _default_wbc_policy_path() -> str:
    """[VBC-NEW] Resolve the frozen WBC checkpoint in any host layout."""
    override = os.environ.get("LEGGEDMANIP_WBC_POLICY")
    if override:
        return override

    relative = Path("mujoco") / "deploy" / "policy" / "go2_piper" / "wbc" / "policy.pt"
    # Self-locating: walk up from this installed file to the project root, so no
    # /workspace or author-specific absolute path is required.
    current = Path(__file__).resolve().parent
    for _ in range(12):
        candidate = current / relative
        if candidate.is_file():
            return str(candidate)
        parent = current.parent
        if parent == current:
            break
        current = parent
    # Only reached when the checkpoint is genuinely missing.
    return str(Path.home() / "LeggedManip_Lab" / relative)


class VBCPreTrainedPolicyAction(ActionTerm):
    """[VBC-PHYSICAL-GRIPPER] Apply high-level commands through frozen WBC + gripper PD."""

    cfg: VBCPreTrainedPolicyActionCfg
    _asset: Articulation

    def __init__(self, cfg: VBCPreTrainedPolicyActionCfg, env: ManagerBasedRLEnv) -> None:
        super().__init__(cfg, env)
        self.robot: Articulation = env.scene[cfg.asset_name]

        if not check_file_path(cfg.policy_path):
            raise FileNotFoundError(
                f"[VBC-NEW] Frozen WBC policy not found: {cfg.policy_path}. "
                "Set LEGGEDMANIP_WBC_POLICY to an absolute .pt path."
            )
        self.policy = torch.jit.load(read_file(cfg.policy_path)).to(env.device).eval()

        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._processed_actions = torch.zeros_like(self._raw_actions)
        self._processed_pose_command = torch.zeros(self.num_envs, 7, device=self.device)
        self._gripper_target = torch.zeros(self.num_envs, 2, device=self.device)
        self._gripper_joint_ids, gripper_joint_names = self.robot.find_joints(
            cfg.gripper_joint_names, preserve_order=True
        )
        if len(self._gripper_joint_ids) != 2:
            raise RuntimeError(
                "[VBC-PHYSICAL-GRIPPER] Expected exactly two physical gripper joints "
                f"{cfg.gripper_joint_names}, found {gripper_joint_names}. "
                "The old fixed-gripper USD cannot be used for this task; generate "
                "go2_piper_vbc.urdf/USD first."
            )
        # [VBC-NEW] The low-level term reuses the existing 18-D joint-position
        # action implementation. It is created exactly once to avoid duplicate
        # action-manager registrations and duplicate debug callbacks.
        self._low_level_action_term: ActionTerm = cfg.low_level_actions.class_type(cfg.low_level_actions, env)
        self._low_level_actions = torch.zeros(
            self.num_envs, self._low_level_action_term.action_dim, device=self.device
        )

        # Replace the placeholder command/action functions before ObservationManager
        # deep-copies the configuration. This preserves the exact 210-D WBC input.
        cfg.low_level_observations.actions.func = lambda _env: self._low_level_actions
        cfg.low_level_observations.actions.params = {}
        cfg.low_level_observations.velocity_commands.func = lambda _env: self._processed_actions[:, :3]
        cfg.low_level_observations.velocity_commands.params = {}
        cfg.low_level_observations.pos_commands.func = lambda _env: self._processed_pose_command
        cfg.low_level_observations.pos_commands.params = {}
        self._low_level_obs_manager = ObservationManager({"ll_policy": cfg.low_level_observations}, env)

        self._velocity_scale = torch.tensor(cfg.velocity_scale, device=self.device)
        self._position_scale = torch.tensor(cfg.position_scale, device=self.device)
        self._pose_anchor = torch.tensor(cfg.pose_anchor, device=self.device)
        self._pose_range = torch.tensor(cfg.pose_range, device=self.device)
        self._pose_min = self._pose_anchor - self._pose_range
        self._pose_max = self._pose_anchor + self._pose_range
        self._orientation_scale = torch.tensor(cfg.orientation_scale, device=self.device)
        # [VBC-DELTA] Accumulated end-effector target (position + quaternion).
        self._ee_target_pos = torch.zeros(self.num_envs, 3, device=self.device)
        self._ee_target_quat = torch.zeros(self.num_envs, 4, device=self.device)
        self._gripper_open_positions = torch.tensor(cfg.gripper_open_positions, device=self.device)
        self._gripper_close_positions = torch.tensor(cfg.gripper_close_positions, device=self.device)

        # [VBC-ARM-IK] The end-effector is driven by IK from the high-level pose
        # command (paper-aligned); the frozen WBC only keeps the 12 legs.
        self._link0_id = self.robot.find_bodies("link0")[0][0]
        self._arm_joint_ids, _arm_joint_names = self.robot.find_joints(
            list(cfg.arm_ik_joint_names), preserve_order=True
        )
        self._arm_ik_term: ActionTerm | None = None
        self._piper_kin: PiperKinematicsTorch | None = None
        if cfg.arm_ik_enabled:
            if cfg.arm_ik_backend == "piper_torch":
                # [VBC-ARM-IK] Converged batched IK (matches piper_kinematics.py).
                # The built-in differential IK is a local single-step method and
                # stalls ~5-10 cm short on tabletop targets.
                self._piper_kin = PiperKinematicsTorch(self.device)
            else:
                arm_ik_cfg = DifferentialInverseKinematicsActionCfg(
                    asset_name=cfg.asset_name,
                    joint_names=list(cfg.arm_ik_joint_names),
                    body_name=cfg.arm_ik_body_name,
                    controller=DifferentialIKControllerCfg(
                        command_type=cfg.arm_ik_command_type,
                        use_relative_mode=False,
                        ik_method=cfg.arm_ik_ik_method,
                    ),
                )
                self._arm_ik_term = DifferentialInverseKinematicsAction(arm_ik_cfg, env)

        self._counter = 0
        self._reset_command(slice(None))

    @property
    def action_dim(self) -> int:
        return 10

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    @property
    def pose_command(self) -> torch.Tensor:
        """Physical 7-D mixed-frame pose command consumed by the frozen WBC."""
        return self._processed_pose_command

    @property
    def gripper_command(self) -> torch.Tensor:
        """Binary command: 1 means closed, 0 means open."""
        return (self._processed_actions[:, 9] > self.cfg.gripper_close_threshold).float()

    def _ee_command_pose_root(self) -> torch.Tensor:
        """High-level mixed-frame pose command expressed in the robot root frame.

        The stored command is xy in the ``link0`` frame, z in the world frame and
        an orientation relative to ``link0``.  The IK controller works in the
        articulation root frame, so convert position and orientation there.
        """
        cmd = self._processed_pose_command
        link0_pos_w = self.robot.data.body_pos_w[:, self._link0_id]
        link0_quat_w = self.robot.data.body_quat_w[:, self._link0_id]
        pos_b = torch.zeros(self.num_envs, 3, device=self.device)
        pos_b[:, :2] = cmd[:, :2]
        pos_w = link0_pos_w + quat_apply(link0_quat_w, pos_b)
        pos_w = torch.cat([pos_w[:, :2], cmd[:, 2:3]], dim=-1)
        quat_w = quat_mul(link0_quat_w, cmd[:, 3:7])
        pos_root, quat_root = subtract_frame_transforms(
            self.robot.data.root_pos_w, self.robot.data.root_quat_w, pos_w, quat_w
        )
        if self.cfg.arm_ik_command_type == "position":
            return pos_root
        return torch.cat([pos_root, quat_root], dim=-1)

    def _ee_command_pose_link0(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Return the commanded EE target as (position, quaternion) in the ``link0`` frame.

        The batched Piper IK works in the arm base (``link0``) frame, which is the
        frame the kinematics model was verified against.
        """
        cmd = self._processed_pose_command
        link0_pos_w = self.robot.data.body_pos_w[:, self._link0_id]
        link0_quat_w = self.robot.data.body_quat_w[:, self._link0_id]
        pos_b = torch.zeros(self.num_envs, 3, device=self.device)
        pos_b[:, :2] = cmd[:, :2]
        pos_w = link0_pos_w + quat_apply(link0_quat_w, pos_b)
        pos_w = torch.cat([pos_w[:, :2], cmd[:, 2:3]], dim=-1)
        pos_link0 = quat_apply_inverse(link0_quat_w, pos_w - link0_pos_w)
        return pos_link0, cmd[:, 3:7]

    def process_actions(self, actions: torch.Tensor):
        """Convert PPO output to [base velocity, EE pose delta, binary gripper].

        The six pose dims are *increments* (paper-style delta action): the
        accumulated end-effector target moves by ``delta * scale`` every
        high-level step and is clamped to a workspace box around ``pose_anchor``.
        The frozen WBC still receives the same 7-D mixed-frame pose command.
        """
        self._raw_actions[:] = torch.clamp(actions.to(self.device), -1.0, 1.0)
        self._processed_actions[:, :3] = self._raw_actions[:, :3] * self._velocity_scale

        # Accumulate the end-effector target.
        pos_delta = self._raw_actions[:, 3:6] * self._position_scale
        self._ee_target_pos[:] = torch.clamp(
            self._ee_target_pos + pos_delta, min=self._pose_min, max=self._pose_max
        )
        ori_delta = quat_from_euler_xyz(
            self._raw_actions[:, 6] * self._orientation_scale[0],
            self._raw_actions[:, 7] * self._orientation_scale[1],
            self._raw_actions[:, 8] * self._orientation_scale[2],
        )
        self._ee_target_quat[:] = torch.nn.functional.normalize(
            quat_mul(self._ee_target_quat, ori_delta), dim=-1
        )

        self._processed_actions[:, 3:6] = self._ee_target_pos
        self._processed_actions[:, 6:9] = 0.0
        self._processed_pose_command[:, :3] = self._ee_target_pos
        self._processed_pose_command[:, 3:] = self._ee_target_quat
        self._processed_actions[:, 9] = self._raw_actions[:, 9]
        closed = self._raw_actions[:, 9] > self.cfg.gripper_close_threshold
        self._gripper_target[:] = torch.where(
            closed.unsqueeze(-1),
            self._gripper_close_positions.unsqueeze(0),
            self._gripper_open_positions.unsqueeze(0),
        )

    def apply_actions(self):
        # [VBC-NEW] 200 Hz simulation / 50 Hz frozen WBC policy, matching the
        # existing LeggedManipLab policy contract.
        if self._counter % self.cfg.low_level_decimation == 0:
            low_level_obs = self._low_level_obs_manager.compute_group("ll_policy", update_history=True)
            with torch.inference_mode():
                low_level_actions = self.policy(low_level_obs)
            # [VBC-TUNING] Bound the frozen WBC action before it becomes a joint
            # position target.  The policy was trained unclipped, so the default
            # limit is generous (only the extreme tail is cut); it removes the
            # rare frames whose raw action produced huge joint accelerations and
            # the resulting value-loss outliers.
            if self.cfg.low_level_action_clip > 0.0:
                low_level_actions = torch.clamp(
                    low_level_actions,
                    -self.cfg.low_level_action_clip,
                    self.cfg.low_level_action_clip,
                )
            if low_level_actions.shape[-1] != self._low_level_action_term.action_dim:
                raise RuntimeError(
                    "[VBC-NEW] Frozen WBC action dimension mismatch: "
                    f"policy={low_level_actions.shape[-1]}, "
                    f"joint_term={self._low_level_action_term.action_dim}"
                )
            self._low_level_actions[:] = low_level_actions
            self._low_level_action_term.process_actions(self._low_level_actions)
            self._counter = 0

        self._low_level_action_term.apply_actions()
        # [VBC-ARM-IK] Override the arm joints with the IK solution so the
        # high-level pose command is executed precisely (paper-aligned).  The
        # frozen WBC therefore only effectively controls the 12 legs.
        if self._piper_kin is not None:
            target_pos, target_quat = self._ee_command_pose_link0()
            # Seed from the arm's default (home) posture rather than the current
            # one: the object target needs a large reconfiguration from home, and
            # a warm start from the *moving* arm can fall into a different local
            # minimum (measured ~80-225 mm residual).  Seeding from home converged
            # to 0 mm for every reachable tabletop target.
            q_seed = self.robot.data.default_joint_pos[:, self._arm_joint_ids]
            use_orientation = self.cfg.arm_ik_command_type == "pose"
            q_sol, _residual = self._piper_kin.solve(
                target_pos,
                q_seed,
                target_quaternion=target_quat if use_orientation else None,
                max_iterations=self.cfg.arm_ik_iterations,
                damping=self.cfg.arm_ik_damping,
                step_clip=self.cfg.arm_ik_step_clip,
                orientation_weight=self.cfg.arm_ik_orientation_weight,
            )
            # stored for diagnostics (scripted tests / play)
            self._last_ik_target = q_sol
            self._last_ik_residual = _residual
            self.robot.set_joint_position_target(q_sol, joint_ids=self._arm_joint_ids)
        elif self._arm_ik_term is not None:
            self._arm_ik_term.process_actions(self._ee_command_pose_root())
            self._arm_ik_term.apply_actions()
        # [VBC-PHYSICAL-GRIPPER] Set the two real prismatic joint targets every
        # simulation step.  No attachment/weld is used; object motion is solely
        # produced by PhysX contacts and the gripper actuator.
        self.robot.set_joint_position_target(self._gripper_target, joint_ids=self._gripper_joint_ids)
        self._counter += 1

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            env_ids = slice(None)
        # [VBC-NEW] Restart the nested WBC decimation phase for reset
        # environments so a reset never waits for a stale counter.
        self._counter = 0
        self._raw_actions[env_ids] = 0.0
        self._reset_command(env_ids)
        self._low_level_actions[env_ids] = 0.0
        self._low_level_action_term.reset(env_ids)
        if self._arm_ik_term is not None:
            self._arm_ik_term.reset(env_ids)
        self._low_level_obs_manager.reset(env_ids)

    def _reset_command(self, env_ids: Sequence[int] | slice) -> None:
        self._processed_actions[env_ids, :3] = 0.0
        self._processed_actions[env_ids, 3:9] = 0.0
        self._processed_actions[env_ids, 9] = -1.0
        self._ee_target_pos[env_ids] = self._pose_anchor
        self._ee_target_quat[env_ids] = torch.tensor(
            [1.0, 0.0, 0.0, 0.0], device=self.device
        )
        self._processed_pose_command[env_ids, :3] = self._pose_anchor
        self._processed_pose_command[env_ids, 3:] = torch.tensor(
            [1.0, 0.0, 0.0, 0.0], device=self.device
        )
        self._gripper_target[env_ids] = self._gripper_open_positions


@configclass
class VBCPreTrainedPolicyActionCfg(ActionTermCfg):
    """[VBC-PHYSICAL-GRIPPER] Configuration for hierarchical VBC."""

    class_type: type = VBCPreTrainedPolicyAction
    asset_name: str = MISSING
    policy_path: str = _default_wbc_policy_path()
    low_level_decimation: int = 4
    low_level_action_clip: float = 10.0
    """Symmetric bound on the frozen WBC action (0 disables).  Matches the WBC's
    own training-time action clip of +-10.0, so normal outputs pass through and
    only extreme outliers are cut."""
    low_level_actions: ActionTermCfg = MISSING
    low_level_observations: ObservationGroupCfg = MISSING
    # [VBC-ARM-IK] Drive the 6 arm joints with IK from the high-level pose
    # command; the frozen WBC then only effectively controls the 12 legs.
    arm_ik_enabled: bool = True
    arm_ik_joint_names: list[str] = field(
        default_factory=lambda: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    )
    arm_ik_body_name: str = "end_effector"
    arm_ik_command_type: str = "position"
    arm_ik_ik_method: str = "dls"
    # [VBC-ARM-IK] Backend: "piper_torch" = converged batched IK (default),
    # "isaaclab" = the built-in single-step differential IK.
    arm_ik_backend: str = "piper_torch"
    arm_ik_iterations: int = 10
    arm_ik_damping: float = 1.0e-4
    arm_ik_step_clip: float = 0.40
    arm_ik_orientation_weight: float = 0.05
    velocity_scale: tuple[float, float, float] = (0.40, 0.30, 0.50)
    # [VBC-DELTA] Per-step end-effector position increment (metres). At 50 Hz
    # this is up to ~1.0 m/s of commanded target motion.
    position_scale: tuple[float, float, float] = (0.020, 0.020, 0.020)
    pose_anchor: tuple[float, float, float] = (0.50, 0.00, 0.55)
    # [VBC-DELTA] Workspace box half-extents around pose_anchor (metres).
    pose_range: tuple[float, float, float] = (0.30, 0.25, 0.25)
    # [VBC-DELTA] Per-step orientation increment (radians), ~2.5/4 rad/s.
    orientation_scale: tuple[float, float, float] = (0.05, 0.05, 0.08)
    gripper_joint_names: list[str] = field(default_factory=lambda: ["joint7", "joint8"])
    gripper_open_positions: tuple[float, float] = (0.04, -0.04)
    gripper_close_positions: tuple[float, float] = (0.002, -0.002)
    gripper_close_threshold: float = 0.0
