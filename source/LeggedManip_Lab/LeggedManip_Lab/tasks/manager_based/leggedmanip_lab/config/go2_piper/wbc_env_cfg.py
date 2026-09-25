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

from isaaclab.utils import configclass
from ...leggedmanip_lab_env_cfg import LeggedManipLabEnvCfg
from LeggedManip_Lab.assets.go2_piper.go2_piper_articulation_cfg import GO2_PIPER_CFG
from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab.leggedmanip_lab_env_cfg import *


@configclass
class WBCCommandsCfg:
    """Command specifications for the MDP."""

    ee_pose = mdp.command_cfg.UniformPoseWBCCommandCfg(
        asset_name="robot",
        body_name="end_effector",
        resampling_time_range=(8.0, 10.0),
        debug_vis=True,
        ranges=mdp.command_cfg.UniformPoseWBCCommandCfg.Ranges(
            # [WBC-WIDER-EE] Widened so the low-level WBC actually learns the
            # workspace the VBC grasp/lift needs.  The old range was only
            # x in (0.40, 0.45), y in (-0.05, 0.05), z == 0.50, which made the
            # tabletop cube (x ~= 0.62, lift to z ~= 0.65) unreachable.
            pos_x=(0.35, 0.65),
            pos_y=(-0.20, 0.20),
            pos_z=(0.40, 0.70),  # World frame
            roll=(-0.0, 0.0),
            pitch=(-0.0, -0.0),  # depends on end-effector axis
            yaw=(-0.0, -0.0),
        ),
        limit_ranges=mdp.command_cfg.UniformPoseWBCCommandCfg.Ranges(
            pos_x=(0.45, 0.7),
            pos_y=(-0.35, 0.35),
            pos_z=(0.1, 0.8),  # World frame
            roll=(-3.14 / 3, 3.14 / 3),
            pitch=(-3.14 / 4, 3.14 / 4),  # depends on end-effector axis
            yaw=(-3.14 / 6, 3.14 / 6),
        ),
    )

    base_velocity = mdp.command_cfg.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.1,
        debug_vis=True,
        heading_command=False,
        ranges=mdp.command_cfg.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.2, 0.2),
            lin_vel_y=(-0.2, 0.2),
            ang_vel_z=(-0.2, 0.2),
            heading=(-0.0, 0.0),
        ),
        limit_ranges=mdp.command_cfg.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0),
            lin_vel_y=(-0.6, 0.6),
            ang_vel_z=(-1.0, 1.0),
            heading=(-0.0, 0.0),
        ),
    )

@configclass
class Go2PiperWBCEnvCfg(LeggedManipLabEnvCfg):

    commands: WBCCommandsCfg = WBCCommandsCfg()
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # scene
        self.scene.robot: ArticulationCfg = GO2_PIPER_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot"
        )

        # observations

        # curriculum

        # events
        self.events.push_robot = None

        # commands
        # self.commands.ee_pose.curriculum_enabled = True
        self.events.reset_base.params["pose_range"] = {
            "x": (-0.8, 0.8),
            "y": (-0.8, 0.8),
            "z": (-0.05, 0.08),
            "roll": (-0.15, 0.15),
            "pitch": (-0.15, 0.15),
            "yaw": (-1.57, 1.57),
        }
        
        # actions
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-10.0, 10.0)}
        
        # rewards
        self.rewards.track_base_height_exp.params["target_height"] = 0.28
        self.rewards.end_effector_position_tracking_exp.func = mdp.position_command_error_exp
        self.rewards.end_effector_position_tracking_exp.weight = 6.0
        # [WBC-PRECISION] Sharpen the position tracking kernel from ~0.316 to
        # 0.06 so the WBC is actually rewarded for centimetre-level accuracy
        # (the old kernel barely distinguished 5 cm from 15 cm).
        self.rewards.end_effector_position_tracking_exp.params["std"] = 0.06
        # [WBC-PRECISION] Orientation is a linear penalty, so it needs a larger
        # weight to balance the now-sharp position reward (position got 6x
        # sharper; orientation would otherwise be sacrificed).
        self.rewards.end_effector_orientation_tracking.weight = -8.0
        self.rewards.track_lin_vel_xy_exp.weight = 3.5
        self.rewards.track_ang_vel_z_exp.weight = 4.0
        # Gazebo sim2sim / navigation readiness: sharp stand-still bonus when
        # the velocity command is near zero (prevents zero-command drift), and
        # a sharp low-speed tracking kernel for precise navigation corrections.
        self.rewards.stand_still_bonus = RewTerm(
            func=mdp.stand_still_bonus,
            weight=2.0,
            params={
                "command_name": "base_velocity",
                "std": 0.05,
                "deadband": 0.05,
            },
        )
        self.rewards.track_lin_vel_low_speed_exp = RewTerm(
            func=mdp.track_lin_vel_low_speed_exp,
            weight=1.5,
            params={
                "command_name": "base_velocity",
                "std": 0.05,
                "low_speed_threshold": 0.2,
            },
        )
        self.rewards.track_ang_vel_low_speed_exp = RewTerm(
            func=mdp.track_ang_vel_low_speed_exp,
            weight=2.0,
            params={
                "command_name": "base_velocity",
                "std": 0.35,
                "low_speed_threshold": 0.35,
                "yaw_deadband": 0.05,
            },
        )

        self.rewards.track_base_height_exp.weight = 0.25
        self.rewards.flat_orientation_l2.weight = -0.5
        self.rewards.feet_long_air.weight = -1.0
        self.rewards.air_time_variance.weight = -1.5
        # terminals

        self.disable_zero_weight_rewards()


class Go2PiperWBCEnvCfg_PLAY(Go2PiperWBCEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.0
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event

        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges

        self.commands.ee_pose.ranges = self.commands.ee_pose.limit_ranges
        self.commands.ee_pose.limit_ranges.pos_y = (0.0,0.0)
        # self.commands.ee_pose.limit_ranges.roll = (0.0,0.0)
        # self.commands.ee_pose.limit_ranges.pitch = (0.0,0.0)
        # self.commands.ee_pose.limit_ranges.yaw = (0.0,0.0)
