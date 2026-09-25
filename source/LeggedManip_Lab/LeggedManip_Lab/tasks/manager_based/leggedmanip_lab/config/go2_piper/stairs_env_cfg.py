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

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from ...leggedmanip_lab_env_cfg import (
    ActionsCfg,
    CommandsCfg,
    EventCfg,
    LeggedManipLabEnvCfg,
    ObservationsCfg,
    RewardsCfg,
    TerminationsCfg,
)
from ... import mdp
from LeggedManip_Lab.assets.go2_piper.go2_piper_articulation_cfg import GO2_PIPER_CFG
from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab.leggedmanip_lab_env_cfg import *
from .stairs_terrain import STAIRS_TERRAINS_CFG


@configclass
class StairsSceneCfg(InteractiveSceneCfg):
    """Scene with 8-subterrain mix (incl. normal+inverted stairs) and height scanner."""

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=STAIRS_TERRAINS_CFG,
        max_init_terrain_level=9,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )

    robot: ArticulationCfg = GO2_PIPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.0, 0.8]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )

    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


@configclass
class StairsObservationsCfg(ObservationsCfg):
    """Add height scan + gait phase to policy observations, longer history."""

    @configclass
    class PolicyCfg(ObservationsCfg.PolicyCfg):
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )
        phase = ObsTerm(func=mdp.phase_signal)

        def __post_init__(self):
            super().__post_init__()
            self.history_length = 5

    policy: PolicyCfg = PolicyCfg()


@configclass
class StairsRewardsCfg(RewardsCfg):
    """Stair-specific rewards on top of the base locomotion rewards."""

    feet_height_clearance = RewTerm(
        func=mdp.feet_height_clearance,
        weight=0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
        },
    )
    periodic_contact_suggestion = RewTerm(
        func=mdp.periodic_contact_suggestion,
        weight=0.25,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
    )
    feet_vertical_surface_contacts = RewTerm(
        func=mdp.feet_vertical_surface_contacts,
        weight=-0.25,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
    )
    feet_to_base_distance = RewTerm(
        func=mdp.feet_to_base_distance,
        weight=-1.5,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=".*_foot")},
    )


@configclass
class StairsCurriculumCfg:
    """Curriculum for command difficulty (terrain mix is fixed, not levelled)."""

    lin_vel_cmd_levels = CurrTerm(func=mdp.lin_vel_cmd_levels)
    ang_vel_cmd_levels = CurrTerm(func=mdp.ang_vel_cmd_levels)
    pos_cmd_levels = CurrTerm(func=mdp.pos_cmd_levels)


@configclass
class Go2PiperStairsEnvCfg(LeggedManipLabEnvCfg):
    """Stairs training environment for Go2+Piper."""

    scene: StairsSceneCfg = StairsSceneCfg(num_envs=2048, env_spacing=2.5)
    observations: StairsObservationsCfg = StairsObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: StairsRewardsCfg = StairsRewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: StairsCurriculumCfg = StairsCurriculumCfg()

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = GO2_PIPER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt

        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.curriculum = False

        self.events.push_robot = None

        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-10.0, 10.0)}

        self.rewards.track_base_height_exp.params["target_height"] = 0.28

        # Down-weight end-effector tracking so stair locomotion is prioritised
        # over the arm pose task. The arm is still controlled (18-dim action),
        # but it no longer dominates the reward and drags down locomotion.
        self.rewards.end_effector_position_tracking_exp.weight = 0.5
        self.rewards.end_effector_orientation_tracking.weight = -0.3

        self.disable_zero_weight_rewards()


@configclass
class Go2PiperStairsEnvCfg_PLAY(Go2PiperStairsEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
        self.commands.ee_pose.ranges = self.commands.ee_pose.limit_ranges
        self.terminations.base_contact = None
        self.terminations.bad_orientation = None
        self.viewer.origin_type = "asset_root"
        self.viewer.asset_name = "robot"
        self.viewer.env_index = 0
        self.viewer.eye = (0.0, -3.0, 2.0)
        self.viewer.lookat = (0.0, 0.0, 0.5)
        self.viewer.resolution = (1280, 720)
