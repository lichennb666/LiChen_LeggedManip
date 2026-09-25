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

"""Reward functions for the legged manipulation environment."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
import isaaclab.utils.math as math_utils

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import (
    combine_frame_transforms,
    quat_error_magnitude,
    quat_mul,
    quat_apply_inverse,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# ---------------------------------------------------------------------------
# Task rewards — end-effector pose tracking
# ---------------------------------------------------------------------------


def position_command_b_error_exp(
    env: ManagerBasedRLEnv, command_name: str, std: float, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Reward end-effector position tracking in the robot's body frame (link0 frame)
    using an exponential kernel.

    The position error is computed in the body frame so the policy is invariant to
    the robot's world-frame orientation.
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    root_idx = asset.find_bodies("link0")[0][0]
    end_effector_curr_pos_b = (
        asset.data.body_pos_w[:, asset_cfg.body_ids[0]]
        - asset.data.body_pos_w[:, root_idx]
    )
    end_effector_curr_pos_b = quat_apply_inverse(
        asset.data.body_quat_w[:, root_idx], end_effector_curr_pos_b
    )
    ee_pos_err = torch.abs(end_effector_curr_pos_b[:, :3] - command[:, :3])
    return torch.exp(-torch.sum(torch.square(ee_pos_err) / (std**2), dim=1))


def position_command_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    asset_cfg: SceneEntityCfg,
    link0_name: str = "link0",
) -> torch.Tensor:
    """Reward end-effector position tracking in a mixed coordinate frame.

    XY error is computed in the link0 frame, Z error is computed in the world frame.
    This is used for WBC (Whole-Body Control) where z commands are absolute heights.
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    ee_id = asset_cfg.body_ids[0]
    link0_id, _ = asset.find_bodies(link0_name)
    link0_id = link0_id[0]
    command = env.command_manager.get_command(command_name)

    ee_pos_w = asset.data.body_pos_w[:, ee_id]
    link0_pos_w = asset.data.body_pos_w[:, link0_id]
    link0_quat_w = asset.data.body_quat_w[:, link0_id]

    # XY error in link0 frame
    end_effector_curr_pos_link0 = quat_apply_inverse(
        link0_quat_w, ee_pos_w - link0_pos_w
    )
    pos_error_xy = end_effector_curr_pos_link0[:, :2] - command[:, :2]
    # Z error in world frame
    pos_error_z = (ee_pos_w[:, 2] - command[:, 2]).unsqueeze(1)
    pos_error = torch.cat([pos_error_xy, pos_error_z], dim=-1)

    error_sq = torch.sum(torch.square(pos_error), dim=1)
    return torch.exp(-error_sq / (std**2))


def orientation_command_error(
    env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Penalize end-effector orientation error using the shortest-path quaternion distance."""
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_quat_b = command[:, 3:7]
    des_quat_w = quat_mul(asset.data.root_quat_w, des_quat_b)
    curr_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids[0]]
    return quat_error_magnitude(curr_quat_w, des_quat_w)


# ---------------------------------------------------------------------------
# Task rewards — base velocity / height tracking
# ---------------------------------------------------------------------------


def track_lin_vel_xy_exp(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward base linear velocity tracking (xy axes) using an exponential kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    lin_vel_error = torch.sum(
        torch.square(
            env.command_manager.get_command(command_name)[:, :2]
            - asset.data.root_lin_vel_b[:, :2]
        ),
        dim=1,
    )
    return torch.exp(-lin_vel_error / std**2)


def track_ang_vel_z_exp(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward base yaw-rate tracking using an exponential kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    ang_vel_error = torch.square(
        env.command_manager.get_command(command_name)[:, 2]
        - asset.data.root_ang_vel_b[:, 2]
    )
    return torch.exp(-ang_vel_error / std**2)


def base_height_tracking(
    env: ManagerBasedRLEnv,
    target_height: float,
    std: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward maintaining the base at a target height above the ground."""
    asset: RigidObject = env.scene[asset_cfg.name]
    curr_height = asset.data.root_pos_w[:, 2]
    height_error = torch.abs(target_height - curr_height)
    return torch.exp(-height_error / std)


def stand_still_bonus(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    deadband: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward holding the base still when the velocity command is near zero.

    The base-velocity tracking kernel is intentionally flat at small errors, so a
    policy can accumulate centimetre-scale drift while earning near-full tracking
    reward.  This term uses a sharp kernel and is active only below ``deadband``,
    teaching the policy to actually stop (zero base velocity) when commanded to.
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    lin_cmd = torch.norm(command[:, :2], dim=1)
    active = ((lin_cmd < deadband) & (torch.abs(command[:, 2]) < deadband)).float()
    motion = (
        torch.sum(torch.square(asset.data.root_lin_vel_b[:, :2]), dim=1)
        + torch.square(asset.data.root_ang_vel_b[:, 2])
    )
    return active * torch.exp(-motion / std**2)


def track_lin_vel_low_speed_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    low_speed_threshold: float = 0.2,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Sharpen base linear-velocity tracking for low-speed commands.

    Navigation stacks spend most of their time near a goal issuing small velocity
    corrections, where the standard flat kernel barely discriminates.  This term
    applies a sharp exponential kernel (small ``std``) only when the commanded
    planar speed is below ``low_speed_threshold``.
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    cmd_mag = torch.norm(command[:, :2], dim=1)
    active = (cmd_mag < low_speed_threshold).float()
    error = torch.sum(
        torch.square(command[:, :2] - asset.data.root_lin_vel_b[:, :2]), dim=1
    )
    return active * torch.exp(-error / std**2)


def track_ang_vel_low_speed_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    low_speed_threshold: float = 0.35,
    yaw_deadband: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Sharpen yaw-rate tracking when the base is commanded to turn in place.

    The standard angular-velocity reward is broad enough that under-rotating can
    still score well.  This term focuses on low-linear-speed commands with a
    meaningful yaw request, matching the Gazebo Classic failure mode.
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    lin_cmd = torch.norm(command[:, :2], dim=1)
    yaw_cmd = command[:, 2]
    active = ((lin_cmd < low_speed_threshold) & (torch.abs(yaw_cmd) > yaw_deadband)).float()
    error = torch.square(yaw_cmd - asset.data.root_ang_vel_b[:, 2])
    return active * torch.exp(-error / std**2)


# ---------------------------------------------------------------------------
# Root penalties
# ---------------------------------------------------------------------------


def lin_vel_z_l2(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize z-axis base linear velocity (undesired vertical motion)."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.square(asset.data.root_lin_vel_b[:, 2])


def ang_vel_xy_l2(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize xy-axis base angular velocity (undesired roll/pitch rotation)."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.root_ang_vel_b[:, :2]), dim=1)


def flat_orientation_l2(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize non-flat base orientation by penalizing the xy-components of projected gravity."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.projected_gravity_b[:, :2]), dim=1)


# ---------------------------------------------------------------------------
# Joint penalties
# ---------------------------------------------------------------------------


def joint_torques_l2(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize summed joint torques using L2 squared kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(
        torch.square(asset.data.applied_torque[:, asset_cfg.joint_ids]), dim=1
    )


def joint_torques_max(
    env: ManagerBasedRLEnv,
    joint_names: list[str],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize the maximum absolute torque within a joint group.

    Useful for applying different penalty strengths per joint group (hip, thigh, calf, arm).
    """
    asset: Articulation = env.scene[asset_cfg.name]
    joint_indices, _ = asset.find_joints(joint_names)
    torques = asset.data.applied_torque[:, joint_indices]
    max_abs_torque = torch.max(torch.abs(torques), dim=1).values
    return torch.square(max_abs_torque)


def joint_acc_l2(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize joint accelerations using L2 squared kernel for smooth motion."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.joint_acc[:, asset_cfg.joint_ids]), dim=1)


def joint_power(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize mechanical power (|torque * velocity|) to encourage energy efficiency."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(
        torch.abs(
            asset.data.joint_vel[:, asset_cfg.joint_ids]
            * asset.data.applied_torque[:, asset_cfg.joint_ids]
        ),
        dim=1,
    )


def joint_deviation_l1(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize joints deviating from their default positions using L1 kernel.

    Active only when the robot is upright (projected_gravity_z < -0.7).
    """
    asset: Articulation = env.scene[asset_cfg.name]
    angle = (
        asset.data.joint_pos[:, asset_cfg.joint_ids]
        - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    )
    reward = torch.sum(torch.abs(angle), dim=1)
    reward *= (-env.scene["robot"].data.projected_gravity_b[:, 2]) > 0.7
    return reward


def joint_mirror(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, mirror_joints: list[list[str]]
) -> torch.Tensor:
    """Penalize asymmetry between mirrored joint pairs (e.g. front-right vs rear-left).

    Uses cached joint indices for efficiency.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    if (
        not hasattr(env, "joint_mirror_joints_cache")
        or env.joint_mirror_joints_cache is None
    ):
        env.joint_mirror_joints_cache = [
            [asset.find_joints(joint_name) for joint_name in joint_pair]
            for joint_pair in mirror_joints
        ]
    reward = torch.zeros(env.num_envs, device=env.device)
    for joint_pair in env.joint_mirror_joints_cache:
        diff = torch.sum(
            torch.square(
                asset.data.joint_pos[:, joint_pair[0][0]]
                - asset.data.joint_pos[:, joint_pair[1][0]]
            ),
            dim=-1,
        )
        reward += diff
    reward *= 1 / len(mirror_joints) if len(mirror_joints) > 0 else 0
    reward *= (
        torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    )
    return reward


def joint_pos_limits(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize joints that exceed their soft position limits."""
    asset: Articulation = env.scene[asset_cfg.name]
    out_of_limits = -(
        asset.data.joint_pos[:, asset_cfg.joint_ids]
        - asset.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 0]
    ).clip(max=0.0)
    out_of_limits += (
        asset.data.joint_pos[:, asset_cfg.joint_ids]
        - asset.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 1]
    ).clip(min=0.0)
    return torch.sum(out_of_limits, dim=1)


# ---------------------------------------------------------------------------
# Action penalties
# ---------------------------------------------------------------------------


def action_rate_l2(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize the rate of change of actions for smooth control."""
    return torch.sum(
        torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1
    )


# ---------------------------------------------------------------------------
# Feet / gait rewards
# ---------------------------------------------------------------------------


def feet_slide(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize lateral foot sliding when feet are in contact with the ground."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contacts = (
        contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :]
        .norm(dim=-1)
        .max(dim=1)[0]
        > 1.0
    )
    asset: RigidObject = env.scene[asset_cfg.name]
    cur_footvel_translated = asset.data.body_lin_vel_w[
        :, asset_cfg.body_ids, :
    ] - asset.data.root_lin_vel_w[:, :].unsqueeze(1)
    footvel_in_body_frame = torch.zeros(
        env.num_envs, len(asset_cfg.body_ids), 3, device=env.device
    )
    for i in range(len(asset_cfg.body_ids)):
        footvel_in_body_frame[:, i, :] = math_utils.quat_apply_inverse(
            asset.data.root_quat_w, cur_footvel_translated[:, i, :]
        )
    foot_leteral_vel = torch.sqrt(
        torch.sum(torch.square(footvel_in_body_frame[:, :, :2]), dim=2)
    ).view(env.num_envs, -1)
    reward = torch.sum(foot_leteral_vel * contacts, dim=1)
    reward *= (
        torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    )
    return reward


def feet_air_time(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    threshold: float,
) -> torch.Tensor:
    """Reward feet spending time in the air above a threshold (encourages stepping).

    No reward when the velocity command is near zero.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[
        :, sensor_cfg.body_ids
    ]
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    reward *= (
        torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    )
    return reward


def feet_long_air_penalty(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    max_air_time: float = 0.5,
) -> torch.Tensor:
    """Penalize feet that stay airborne longer than max_air_time."""
    contact_sensor = env.scene.sensors[sensor_cfg.name]
    if contact_sensor.data.current_air_time is None:
        return torch.zeros(env.num_envs, device=env.device)
    current_air = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    excess = torch.clamp(current_air - max_air_time, min=0.0)
    return torch.sum(torch.square(excess), dim=1)


def air_time_variance_penalty(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Penalize variance in foot air/contact times for symmetric gait."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    if contact_sensor.cfg.track_air_time is False:
        raise RuntimeError("Activate ContactSensor's track_air_time!")
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    last_contact_time = contact_sensor.data.last_contact_time[:, sensor_cfg.body_ids]
    return torch.var(torch.clip(last_air_time, max=0.5), dim=1) + torch.var(
        torch.clip(last_contact_time, max=0.5), dim=1
    )


# ---------------------------------------------------------------------------
# Stairs / gait rewards (added for stair locomotion)
# ---------------------------------------------------------------------------


def feet_vertical_surface_contacts(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Penalize feet hitting vertical surfaces (e.g. stair risers).

    A horizontal contact force much larger than the vertical force means the
    foot kicked a wall/riser instead of landing on top of the step.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :]
    forces_z = torch.abs(forces[..., 2])
    forces_xy = torch.norm(forces[..., :2], dim=-1)
    hit_vertical = torch.any(forces_xy > 4.0 * forces_z, dim=1).float()
    # only penalize when the robot is upright
    upright = (-env.scene["robot"].data.projected_gravity_b[:, 2] > 0.7).float()
    return hit_vertical * upright


def feet_to_base_distance(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Penalize feet drifting too far from the base (prevents over-extended legs)."""
    asset: Articulation = env.scene[asset_cfg.name]
    feet_pos = asset.data.body_pos_w[:, asset_cfg.body_ids, :3]
    base_pos = asset.data.root_pos_w[:, :3].unsqueeze(1)
    dist = torch.norm(feet_pos[:, :, :2] - base_pos[:, :, :2], dim=-1)
    return torch.mean(dist, dim=1)


def phase_signal(
    env: ManagerBasedRLEnv,
    step_freq: float = 1.4,
    duty_factor: float = 0.65,
    phase_offset: tuple[float, ...] = (0.0, 0.5, 0.5, 0.0),
) -> torch.Tensor:
    """Per-foot gait phase in [0, 1) driven by the episode clock.

    phase < duty_factor marks the contact (stance) window. Offset is a
    trot gait (FL, FR, RL, RR) = (0, 0.5, 0.5, 0). Adjust the order to match
    the ``.*_foot`` body ordering of the contact sensor.
    """
    t = env.episode_length_buf.float() * env.step_dt
    offsets = torch.tensor(phase_offset, device=env.device).float()
    phase = torch.remainder(t.unsqueeze(1) * step_freq + offsets.unsqueeze(0), 1.0)
    return phase


def periodic_contact_suggestion(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    command_name: str = "base_velocity",
    step_freq: float = 1.4,
    duty_factor: float = 0.65,
) -> torch.Tensor:
    """Reward a periodic trot gait: feet touch during stance, lift during swing."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contacts = (
        torch.abs(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, 2]) > 1.0
    ).float()
    should_move = (
        torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    ).float()
    phase = phase_signal(env, step_freq, duty_factor)
    contact_on = (phase < duty_factor).float()
    correct = contact_on * contacts + (1.0 - contact_on) * (1.0 - contacts)
    return torch.mean(correct, dim=1) * should_move


def feet_height_clearance(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    target_height: float = 0.05,
    command_name: str = "base_velocity",
) -> torch.Tensor:
    """Reward swing feet lifting above a target height relative to the base."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    asset: Articulation = env.scene[asset_cfg.name]
    contacts = (
        torch.abs(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, 2]) > 1.0
    ).float()
    swing = 1.0 - contacts
    foot_height = (
        asset.data.body_pos_w[:, asset_cfg.body_ids, 2]
        - asset.data.root_pos_w[:, 2].unsqueeze(1)
    )
    clearance = torch.exp(-torch.clamp(target_height - foot_height, min=0.0) / 0.02)
    should_move = (
        torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    ).float()
    return torch.mean(clearance * swing, dim=1) * should_move
