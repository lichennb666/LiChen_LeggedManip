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

"""[VBC-PHYSICAL-GRIPPER] MDP terms for the Go2-Piper VBC branch.

The first branch is deliberately a privileged teacher.  It uses the simulated
cube pose as a stand-in for a perfect RGB-D detector while the frozen
LeggedManipLab WBC policy remains responsible for the 18 leg/arm targets.  The
two Piper finger joints are driven physically by the high-level gripper action.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.envs.mdp import (
    ang_vel_xy_l2 as _ang_vel_xy_l2,
    joint_acc_l2 as _joint_acc_l2,
    lin_vel_z_l2 as _lin_vel_z_l2,
)
from isaaclab.utils import math as math_utils
from isaaclab.utils.math import (
    euler_xyz_from_quat,
    quat_apply,
    quat_apply_inverse,
    quat_inv,
    quat_mul,
)


def camera_rgbd_history_chw(
    env: ManagerBasedRLEnv,
    body_camera_cfg: SceneEntityCfg,
    wrist_camera_cfg: SceneEntityCfg,
    history_length: int = 4,
    depth_max: float = 3.0,
) -> torch.Tensor:
    """Return paper-style two-view RGB-D history as ``(B, C, H, W)``.

    The paper feeds the visual student recent images from a head camera and a
    gripper-near camera.  IsaacLab stores camera outputs as ``(B,H,W,C)``;
    RSL-RL's CNN expects ``(B,C,H,W)``, so this term performs that conversion
    and stacks four frames along the channel axis.  This first Go2+Piper
    student uses RGB-D rather than the paper's real-world TrackingSAM mask;
    a segmentation/mask channel can be added without changing the action
    contract later.
    """

    def _read_rgbd(cfg: SceneEntityCfg) -> torch.Tensor:
        sensor = env.scene.sensors[cfg.name]
        rgb = sensor.data.output["rgb"].float() / 255.0
        depth = sensor.data.output["distance_to_image_plane"].float()
        depth = torch.nan_to_num(depth, nan=0.0, posinf=depth_max, neginf=0.0)
        depth = torch.clamp(depth, min=0.0, max=depth_max) / depth_max
        return torch.cat((rgb, depth), dim=-1)

    frame = torch.cat((_read_rgbd(body_camera_cfg), _read_rgbd(wrist_camera_cfg)), dim=-1)
    frame = frame.permute(0, 3, 1, 2).contiguous()

    image_history = getattr(env, "_vbc_student_image_history", None)
    if image_history is None or image_history.shape != (
        env.num_envs,
        history_length,
        frame.shape[1],
        frame.shape[2],
        frame.shape[3],
    ):
        image_history = frame.unsqueeze(1).repeat(1, history_length, 1, 1, 1)
    else:
        reset_buf = getattr(env, "episode_length_buf", None)
        if reset_buf is None:
            image_history[:, :-1] = image_history[:, 1:]
            image_history[:, -1] = frame
        else:
            reset_mask = reset_buf == 0
            active_mask = ~reset_mask
            image_history[active_mask, :-1] = image_history[active_mask, 1:]
            image_history[active_mask, -1] = frame[active_mask]
            image_history[reset_mask] = frame[reset_mask].unsqueeze(1).repeat(1, history_length, 1, 1, 1)

    env._vbc_student_image_history = image_history
    return image_history.flatten(1, 2).contiguous()


def camera_masked_depth_history_chw(
    env: ManagerBasedRLEnv,
    body_camera_cfg: SceneEntityCfg,
    wrist_camera_cfg: SceneEntityCfg,
    history_length: int = 4,
    depth_min: float = 0.15,
    depth_max: float = 2.0,
) -> torch.Tensor:
    """Return the paper-aligned mask/segmented-depth history.

    Each camera contributes a binary target mask and normalized target-only
    depth.  Both cameras use a semantic filter that exposes only the
    ``target_object`` class, therefore every non-zero semantic id belongs to
    the manipulation target and no simulator-specific id is hard-coded.

    Channel order per frame is
    ``[body_mask, wrist_mask, body_segmented_depth, wrist_segmented_depth]``.
    Four frames consequently produce a ``(B, 16, H, W)`` tensor.
    """

    def _read_mask_depth(cfg: SceneEntityCfg) -> tuple[torch.Tensor, torch.Tensor]:
        sensor = env.scene.sensors[cfg.name]
        semantic = sensor.data.output["semantic_segmentation"]
        depth = sensor.data.output["distance_to_image_plane"].float()

        if semantic.shape[-1] != 1 or depth.shape[-1] != 1:
            raise RuntimeError(
                "[VBC-MASK-DEPTH] Cameras must return uncolorized semantic ids "
                "and single-channel distance_to_image_plane."
            )

        mask = (semantic > 0).to(dtype=depth.dtype)
        depth = torch.nan_to_num(depth, nan=0.0, posinf=depth_max, neginf=0.0)
        valid = (depth >= depth_min) & (depth <= depth_max)
        normalized_depth = torch.where(valid, depth / depth_max, torch.zeros_like(depth))
        return mask, normalized_depth * mask

    body_mask, body_depth = _read_mask_depth(body_camera_cfg)
    wrist_mask, wrist_depth = _read_mask_depth(wrist_camera_cfg)
    frame = torch.cat((body_mask, wrist_mask, body_depth, wrist_depth), dim=-1)
    frame = frame.permute(0, 3, 1, 2).contiguous()

    image_history = getattr(env, "_vbc_mask_depth_history", None)
    expected_shape = (
        env.num_envs,
        history_length,
        frame.shape[1],
        frame.shape[2],
        frame.shape[3],
    )
    if image_history is None or image_history.shape != expected_shape:
        image_history = frame.unsqueeze(1).repeat(1, history_length, 1, 1, 1)
    else:
        reset_buf = getattr(env, "episode_length_buf", None)
        if reset_buf is None:
            image_history[:, :-1] = image_history[:, 1:].clone()
            image_history[:, -1] = frame
        else:
            reset_mask = reset_buf == 0
            active_mask = ~reset_mask
            image_history[active_mask, :-1] = image_history[active_mask, 1:].clone()
            image_history[active_mask, -1] = frame[active_mask]
            image_history[reset_mask] = frame[reset_mask].unsqueeze(1).repeat(
                1, history_length, 1, 1, 1
            )

    env._vbc_mask_depth_history = image_history
    return image_history.flatten(1, 2).contiguous()


def object_position_link0(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """[VBC-NEW] Return the cube position in the robot ``link0`` frame."""
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]

    link0_ids, _ = robot.find_bodies("link0")
    link0_id = link0_ids[0]
    link0_pos_w = robot.data.body_pos_w[:, link0_id]
    link0_quat_w = robot.data.body_quat_w[:, link0_id]
    object_pos_w = obj.data.root_pos_w
    return quat_apply_inverse(link0_quat_w, object_pos_w - link0_pos_w)


def object_orientation_link0_rpy(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Return privileged object orientation relative to ``link0`` as XYZ Euler angles."""
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]
    link0_ids, _ = robot.find_bodies("link0")
    link0_quat_w = robot.data.body_quat_w[:, link0_ids[0]]
    object_quat_link0 = quat_mul(quat_inv(link0_quat_w), obj.data.root_quat_w)
    roll, pitch, yaw = euler_xyz_from_quat(object_quat_link0)
    return torch.stack((roll, pitch, yaw), dim=-1)


def object_shape_feature(
    env: ManagerBasedRLEnv,
    feature_paths: tuple[str, ...] | list[str],
) -> torch.Tensor:
    """Look up the frozen 1024-D PointNet++ code for each environment's object.

    The paired multi-asset spawner uses ``random_choice=False`` and cycles
    through assets in list order.  The same ``env_id % num_objects`` mapping
    selects the feature row here, matching the official VBC implementation's
    offline feature-table design.  PointNet++ is never run in the simulation
    or deployment control loop.
    """
    cache_key = tuple(str(Path(path).expanduser().resolve()) for path in feature_paths)
    cached_key = getattr(env, "_vbc_shape_feature_key", None)
    if cached_key != cache_key:
        features = []
        for path in cache_key:
            feature_path = Path(path)
            if not feature_path.is_file():
                raise FileNotFoundError(
                    f"[VBC-SHAPE] Missing PointNet++ feature: {feature_path}. "
                    "Run scripts/prepare_vbc_object_assets.py first."
                )
            feature = np.load(feature_path)
            feature = np.asarray(feature, dtype=np.float32).reshape(-1)
            if feature.shape != (1024,):
                raise ValueError(
                    f"[VBC-SHAPE] Expected a 1024-D feature in {feature_path}, got {feature.shape}."
                )
            features.append(torch.from_numpy(feature))
        env._vbc_shape_feature_table = torch.stack(features, dim=0).to(env.device)
        env._vbc_shape_feature_key = cache_key

    object_ids = torch.arange(env.num_envs, device=env.device) % len(cache_key)
    return env._vbc_shape_feature_table[object_ids]


def reset_vbc_object_uniform(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    pose_range: dict[str, tuple[float, float]],
    velocity_range: dict[str, tuple[float, float]],
    object_half_heights: tuple[float, ...] | list[float],
    table_top_z: float = 0.51,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> None:
    """Reset deterministic multi-assets at the correct per-object table height."""
    asset: RigidObject = env.scene[asset_cfg.name]
    root_states = asset.data.default_root_state[env_ids].clone()

    ranges = torch.tensor(
        [pose_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]],
        device=asset.device,
    )
    pose_samples = math_utils.sample_uniform(
        ranges[:, 0], ranges[:, 1], (len(env_ids), 6), device=asset.device
    )
    positions = root_states[:, :3] + env.scene.env_origins[env_ids] + pose_samples[:, :3]
    heights = torch.tensor(object_half_heights, device=asset.device)
    positions[:, 2] = table_top_z + heights[env_ids % len(object_half_heights)] + pose_samples[:, 2]
    orientation_delta = math_utils.quat_from_euler_xyz(
        pose_samples[:, 3], pose_samples[:, 4], pose_samples[:, 5]
    )
    orientations = quat_mul(root_states[:, 3:7], orientation_delta)

    ranges = torch.tensor(
        [velocity_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]],
        device=asset.device,
    )
    velocity_samples = math_utils.sample_uniform(
        ranges[:, 0], ranges[:, 1], (len(env_ids), 6), device=asset.device
    )
    velocities = root_states[:, 7:13] + velocity_samples
    asset.write_root_pose_to_sim(torch.cat((positions, orientations), dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(velocities, env_ids=env_ids)


def object_ee_distance_exp(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg,
    std: float = 0.12,
) -> torch.Tensor:
    """[VBC-NEW] Dense reach reward from the end-effector to the physical cube."""
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]
    ee_ids, _ = robot.find_bodies("end_effector")
    ee_pos_w = robot.data.body_pos_w[:, ee_ids[0]]
    distance = torch.linalg.norm(ee_pos_w - obj.data.root_pos_w, dim=-1)
    return torch.exp(-distance / std)


# [VBC-CLAMP] Upper-clamped penalty terms.  The raw squared terms (joint
# acceleration, vertical / angular body velocity) occasionally blow up on
# impacts and dominate the return, which produced value-loss outliers.  These
# wrappers only cut the extreme tail; normal gradients are unchanged.
def vbc_joint_acc_l2(env: ManagerBasedRLEnv, max_value: float = 2.0e6) -> torch.Tensor:
    """``joint_acc_l2`` clamped so a single impact frame cannot dominate."""
    return torch.clamp(_joint_acc_l2(env), max=max_value)


def vbc_lin_vel_z_l2(env: ManagerBasedRLEnv, max_value: float = 0.25) -> torch.Tensor:
    """``lin_vel_z_l2`` (vertical body velocity) clamped."""
    return torch.clamp(_lin_vel_z_l2(env), max=max_value)


def vbc_ang_vel_xy_l2(env: ManagerBasedRLEnv, max_value: float = 5.0) -> torch.Tensor:
    """``ang_vel_xy_l2`` (roll/pitch rate) clamped."""
    return torch.clamp(_ang_vel_xy_l2(env), max=max_value)


def object_ee_distance_linear(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg,
    max_distance: float = 1.0,
) -> torch.Tensor:
    """Linear reach shaping ``-clamp(distance, 0, max_distance)``.

    The exponential kernel above is almost flat when the end-effector starts far
    from the object, so it provides almost no gradient early in training.  This
    linear term keeps a usable signal at long range; the exponential term still
    sharpens the reward near the object.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]
    ee_ids, _ = robot.find_bodies("end_effector")
    distance = torch.linalg.norm(robot.data.body_pos_w[:, ee_ids[0]] - obj.data.root_pos_w, dim=-1)
    return -torch.clamp(distance, max=max_distance)


def object_ee_near_bonus(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg,
    threshold: float = 0.06,
) -> torch.Tensor:
    """[VBC-NEW] Sparse bonus for entering the pre-grasp distance band."""
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]
    ee_ids, _ = robot.find_bodies("end_effector")
    distance = torch.linalg.norm(robot.data.body_pos_w[:, ee_ids[0]] - obj.data.root_pos_w, dim=-1)
    return (distance < threshold).float()


def gripper_joint_positions(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", joint_names=["joint7", "joint8"]),
) -> torch.Tensor:
    """Return the two physical Piper finger positions for the teacher."""
    robot: Articulation = env.scene[asset_cfg.name]
    joint_ids, _ = robot.find_joints(["joint7", "joint8"], preserve_order=True)
    if len(joint_ids) != 2:
        raise RuntimeError(
            "[VBC-PHYSICAL-GRIPPER] joint7/joint8 are not active articulation joints; "
            "the fixed-gripper USD must not be used for the physical task."
        )
    return robot.data.joint_pos[:, joint_ids]


def gripper_command(
    env: ManagerBasedRLEnv,
    action_name: str = "vbc_command",
) -> torch.Tensor:
    """Expose the binary high-level gripper command to observations/rewards."""
    return env.action_manager.get_term(action_name).gripper_command.unsqueeze(-1)


def gripper_contact_force(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    threshold: float = 0.5,
) -> torch.Tensor:
    """Detect a likely object-finger contact from PhysX force and proximity.

    The proximity/closed-gate prevents a finger touching the tabletop during
    approach from being treated as a grasp reward.  It is still only a
    success signal: the cube remains a separate dynamic body.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]
    sensor = env.scene.sensors[sensor_cfg.name]
    force = sensor.data.net_forces_w[:, sensor_cfg.body_ids, :].norm(dim=-1)
    ee_ids, _ = robot.find_bodies("end_effector")
    if len(ee_ids) == 0:
        raise RuntimeError("[VBC-PHYSICAL-GRIPPER] end_effector helper body is missing")
    near_object = torch.linalg.norm(
        robot.data.body_pos_w[:, ee_ids[0]] - obj.data.root_pos_w, dim=-1
    ) < 0.14
    closed = env.action_manager.get_term("vbc_command").gripper_command > 0.5
    return ((torch.sum(force, dim=-1) > threshold) & near_object & closed).float()


def object_lift_progress(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    table_top_z: float = 0.51,
    start_lift: float = 0.03,
    target_lift: float = 0.10,
    contact_threshold: float = 0.5,
    gate_distance: float = 0.14,
) -> torch.Tensor:
    """Staged physical lift progress, gated on a closed-finger contact.

    No reward below ``start_lift`` and full reward at ``target_lift`` (the grasp
    success threshold), so the policy is not paid for tiny nudges.  Without the
    contact/closed gate it could farm the term by pushing or bouncing the object.
    """
    obj: RigidObject = env.scene[object_cfg.name]
    robot: Articulation = env.scene[robot_cfg.name]
    sensor = env.scene.sensors[sensor_cfg.name]
    finger_force = sensor.data.net_forces_w[:, sensor_cfg.body_ids, :].norm(dim=-1).sum(dim=-1)
    ee_ids, _ = robot.find_bodies("end_effector")
    ee_distance = torch.linalg.norm(
        robot.data.body_pos_w[:, ee_ids[0]] - obj.data.root_pos_w, dim=-1
    )
    closed = env.action_manager.get_term("vbc_command").gripper_command > 0.5
    gate = (finger_force > contact_threshold) & (ee_distance < gate_distance) & closed
    lift = obj.data.root_pos_w[:, 2] - table_top_z
    span = max(target_lift - start_lift, 1e-6)
    progress = torch.clamp((lift - start_lift) / span, min=0.0, max=1.0)
    return progress * gate.float()


def physical_grasp_success(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    table_top_z: float = 0.51,
    lift_threshold: float = 0.10,
    contact_threshold: float = 0.5,
    hold_steps: int = 25,
) -> torch.Tensor:
    """Return 1 only after contact + closed fingers + lifted object are held.

    This is a training success signal, not an attachment mechanism.  The
    object remains a separate dynamic RigidObject and must move through
    PhysX contacts.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[object_cfg.name]
    action_term = env.action_manager.get_term("vbc_command")
    ee_ids, _ = robot.find_bodies("end_effector")
    if len(ee_ids) == 0:
        raise RuntimeError("[VBC-PHYSICAL-GRIPPER] end_effector helper body is missing")

    sensor = env.scene.sensors[sensor_cfg.name]
    finger_force = sensor.data.net_forces_w[:, sensor_cfg.body_ids, :].norm(dim=-1).sum(dim=-1)
    ee_distance = torch.linalg.norm(
        robot.data.body_pos_w[:, ee_ids[0]] - obj.data.root_pos_w, dim=-1
    )
    lifted = obj.data.root_pos_w[:, 2] > table_top_z + lift_threshold
    closed = action_term.gripper_command > 0.5
    candidate = (finger_force > contact_threshold) & (ee_distance < 0.14) & lifted & closed

    if not hasattr(env, "_vbc_grasp_hold_counter"):
        env._vbc_grasp_hold_counter = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)
    env._vbc_grasp_hold_counter = torch.where(
        candidate,
        env._vbc_grasp_hold_counter + 1,
        torch.zeros_like(env._vbc_grasp_hold_counter),
    )
    return (env._vbc_grasp_hold_counter >= hold_steps).float()


def ee_vbc_command_tracking_exp(
    env: ManagerBasedRLEnv,
    action_name: str = "vbc_command",
    std: float = 0.08,
) -> torch.Tensor:
    """[VBC-NEW] Reward the frozen low-level WBC for tracking the high-level EE command."""
    robot: Articulation = env.scene["robot"]
    action_term = env.action_manager.get_term(action_name)
    command = action_term.pose_command

    link0_ids, _ = robot.find_bodies("link0")
    ee_ids, _ = robot.find_bodies("end_effector")
    link0_id = link0_ids[0]
    link0_pos_w = robot.data.body_pos_w[:, link0_id]
    link0_quat_w = robot.data.body_quat_w[:, link0_id]
    command_pos_b = torch.zeros_like(link0_pos_w)
    command_pos_b[:, :2] = command[:, :2]
    command_pos_w = link0_pos_w + quat_apply(link0_quat_w, command_pos_b)
    command_pos_w[:, 2] = command[:, 2]

    ee_pos_w = robot.data.body_pos_w[:, ee_ids[0]]
    error = torch.linalg.norm(ee_pos_w - command_pos_w, dim=-1)
    return torch.exp(-error / std)


def vbc_base_command_l2(
    env: ManagerBasedRLEnv,
    action_name: str = "vbc_command",
) -> torch.Tensor:
    """[VBC-NEW] Penalize unnecessary base velocity commands during tabletop reach."""
    command = env.action_manager.get_term(action_name).processed_actions[:, :3]
    return torch.sum(torch.square(command), dim=-1)


def vbc_command_rate_l2(
    env: ManagerBasedRLEnv,
    action_name: str = "vbc_command",
) -> torch.Tensor:
    """[VBC-NEW] Penalize abrupt high-level command changes."""
    del action_name  # The first/only action term is the VBC command term.
    delta = env.action_manager.action - env.action_manager.prev_action
    return torch.sum(torch.square(delta), dim=-1)


def vbc_object_position_curriculum(
    env: ManagerBasedRLEnv,
    env_ids,
    reward_term_name: str = "physical_grasp_success",
    success_reward_ratio: float = 0.05,
    range_step: float = 0.025,
    max_x_half_range: float = 0.30,
    max_y_half_range: float = 0.30,
) -> dict[str, float]:
    """[VBC-NEW] Spread the cube only after the teacher actually grasps it.

    Expansion is driven by the physical grasp-success term (contact + closed
    gripper + lift + hold), not by mere reaching, so the workspace only grows
    once the current difficulty is solved.
    """
    del env_ids
    event_cfg = env.event_manager.get_term_cfg("reset_cube")
    pose_range = event_cfg.params["pose_range"]

    if (
        env.common_step_counter > 0
        and env.common_step_counter % env.max_episode_length == 0
    ):
        reward_sums = env.reward_manager._episode_sums.get(reward_term_name)
        reward_cfg = env.reward_manager.get_term_cfg(reward_term_name)
        if reward_sums is not None:
            mean_reward = torch.mean(reward_sums) / env.max_episode_length_s
            if mean_reward > reward_cfg.weight * success_reward_ratio:
                x_half = min(abs(pose_range["x"][1]), max_x_half_range) + range_step
                y_half = min(abs(pose_range["y"][1]), max_y_half_range) + range_step
                x_half = min(x_half, max_x_half_range)
                y_half = min(y_half, max_y_half_range)
                pose_range["x"] = (-x_half, x_half)
                pose_range["y"] = (-y_half, y_half)

    return {
        "cube_x_half_range": float(abs(pose_range["x"][1])),
        "cube_y_half_range": float(abs(pose_range["y"][1])),
    }
