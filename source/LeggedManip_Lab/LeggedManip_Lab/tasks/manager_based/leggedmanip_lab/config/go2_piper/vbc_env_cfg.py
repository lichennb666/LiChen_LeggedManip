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

"""[VBC-PHYSICAL-GRIPPER] Flat tabletop VBC teacher/student for Go2 + Piper.

This is the first incremental VBC branch. The policy trained here is a
privileged high-level teacher. It outputs a 10-D Go2-adapted command and the
custom action term feeds the body/arm part into the frozen 18-D WBC policy and
the last binary value into the two physical Piper finger joints.

The RGB-D student is a separate registered environment in this file.  It is
only an observation/training branch: it keeps the same physical cube,
gripper actuators, and frozen 18-D WBC action term as the teacher.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import (
    CurriculumTermCfg as CurrTerm,
    EventTermCfg as EventTerm,
    ObservationGroupCfg as ObsGroup,
    ObservationTermCfg as ObsTerm,
    RewardTermCfg as RewTerm,
    SceneEntityCfg,
)
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from LeggedManip_Lab.assets.go2_piper.go2_piper_articulation_cfg import GO2_PIPER_VBC_CFG
from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab import mdp
from LeggedManip_Lab.tasks.manager_based.leggedmanip_lab.leggedmanip_lab_env_cfg import (
    EventCfg,
    LeggedManipLabEnvCfg,
    SceneCfg,
    TerminationsCfg,
)


# [VBC-STAGE] Base-command scale for the 3 high-level velocity dims.
#   "0,0,0"               -> fixed-base grasp (stage 1, current default)
#   "0.4,0.3,0.5"         -> mobile grasp (stage 2; set VBC_VELOCITY_SCALE)
# The 3 dims stay in the action vector, so switching needs no code change.
def _velocity_scale_from_env() -> tuple[float, float, float]:
    raw = os.environ.get("VBC_VELOCITY_SCALE", "0,0,0")
    parts = [float(value) for value in raw.replace("(", "").replace(")", "").split(",")]
    if len(parts) != 3:
        raise ValueError(
            "VBC_VELOCITY_SCALE must be three comma-separated numbers, e.g. '0.4,0.3,0.5'"
        )
    return (parts[0], parts[1], parts[2])


# Must stay identical to the frozen WBC policy contract.  The physical
# gripper joints are deliberately excluded from this 18-D low-level vector.
WBC_JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
]

# [VBC-SHAPE] A deliberately small reproduction set from the official VBC
# release.  Assets and precomputed PointNet++ features use the same order;
# the multi-asset spawner cycles through this list deterministically.
VBC_OBJECT_NAMES = ("plate_holder", "glue_1", "blue_cup", "clear_box")
VBC_OBJECT_HALF_HEIGHTS = (0.052, 0.058, 0.031, 0.051)
VBC_OBJECT_ASSET_DIR = Path(__file__).resolve().parents[5] / "assets" / "vbc_objects"
VBC_OBJECT_FEATURE_PATHS = tuple(
    str(VBC_OBJECT_ASSET_DIR / name / "features.npy") for name in VBC_OBJECT_NAMES
)


def _vbc_object_asset_cfg(name: str) -> sim_utils.UrdfFileCfg:
    return sim_utils.UrdfFileCfg(
        asset_path=str(VBC_OBJECT_ASSET_DIR / name / "model.urdf"),
        fix_base=False,
        merge_fixed_joints=True,
        joint_drive=None,
        force_usd_conversion=False,
    )


def _vbc_multi_object_cfg() -> RigidObjectCfg:
    return RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.MultiAssetSpawnerCfg(
            assets_cfg=[_vbc_object_asset_cfg(name) for name in VBC_OBJECT_NAMES],
            random_choice=False,
            semantic_tags=[("class", "target_object")],
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                linear_damping=0.02,
                angular_damping=0.02,
                max_depenetration_velocity=1.0,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.003, rest_offset=0.0
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.50, 0.0, 0.568)),
    )


@configclass
class VBCSceneCfg(SceneCfg):
    """[VBC-NEW] Flat scene containing a fixed table and a dynamic cube."""

    table: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.CuboidCfg(
            size=(0.85, 0.75, 0.06),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.22, 0.10), metallic=0.0),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.50, 0.0, 0.48)),
    )

    cube: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.06, 0.06, 0.06),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                linear_damping=0.02,
                angular_damping=0.02,
                max_depenetration_velocity=1.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.08),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.003, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.8,
                dynamic_friction=0.6,
                restitution=0.0,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.05, 0.75, 0.15), metallic=0.0),
        ),
        # [VBC-NEW] Table top is z=0.51 m; cube center is half its size above it.
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.50, 0.0, 0.54)),
    )


@configclass
class VBCStudentSceneCfg(VBCSceneCfg):
    """[VBC-VISION-STUDENT] Same physical task plus the two RGB-D views.

    The camera prims are attached to the same robot frames used by the
    physical model: the body camera is centered and slightly above ``base``;
    the wrist camera reuses the existing Piper camera extrinsic relative to
    ``link6``.  They are sensors only in IsaacLab; the old Gazebo camera
    plugins remain untouched.
    """

    body_camera: CameraCfg = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base/body_camera",
        update_period=0.0,
        height=84,
        width=84,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=1.5,
            horizontal_aperture=20.955,
            clipping_range=(0.05, 3.0),
        ),
        # Camera optical +Z looks forward and 0.35 rad downward in base.
        offset=CameraCfg.OffsetCfg(
            pos=(0.12, 0.0, 0.60),
            rot=(-0.4053092, 0.5794173, -0.5794173, 0.4053092),
            convention="ros",
        ),
    )

    wrist_camera: CameraCfg = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/link6/wrist_camera",
        update_period=0.0,
        height=84,
        width=84,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=0.6,
            horizontal_aperture=20.955,
            clipping_range=(0.03, 2.0),
        ),
        # Link6 -> existing camera stand/d435/optical chain, collapsed into
        # one calibrated Isaac CameraCfg extrinsic.
        offset=CameraCfg.OffsetCfg(
            pos=(-0.0554, -0.0005, 0.0355),
            rot=(0.6962577, -0.1233903, 0.1233903, -0.6962577),
            convention="ros",
        ),
    )


@configclass
class VBCMultiObjectSceneCfg(VBCSceneCfg):
    """Physical multi-object scene paired with frozen PointNet++ features."""

    cube: RigidObjectCfg = _vbc_multi_object_cfg()


@configclass
class VBCMaskDepthStudentSceneCfg(VBCStudentSceneCfg):
    """Multi-object scene with target-only semantic mask/depth cameras."""

    cube: RigidObjectCfg = _vbc_multi_object_cfg()

    def __post_init__(self):
        self.body_camera.data_types = ["semantic_segmentation", "distance_to_image_plane"]
        self.wrist_camera.data_types = ["semantic_segmentation", "distance_to_image_plane"]
        self.body_camera.semantic_filter = "class:target_object"
        self.wrist_camera.semantic_filter = "class:target_object"
        self.body_camera.colorize_semantic_segmentation = False
        self.wrist_camera.colorize_semantic_segmentation = False

@configclass
class VBCWBCObservationCfg(ObsGroup):
    """[VBC-NEW] Exact observation layout required by the frozen 210-D WBC."""

    base_ang_vel = ObsTerm(
        func=mdp.base_ang_vel,
        noise=Unoise(n_min=-0.2, n_max=0.2),
        scale=0.2,
    )
    projected_gravity = ObsTerm(
        func=mdp.projected_gravity,
        noise=Unoise(n_min=-0.05, n_max=0.05),
    )
    joint_pos = ObsTerm(
        func=mdp.joint_pos_rel,
        params={"joint_names": WBC_JOINT_NAMES},
        noise=Unoise(n_min=-0.01, n_max=0.01),
    )
    joint_vel = ObsTerm(
        func=mdp.joint_vel_rel,
        params={"joint_names": WBC_JOINT_NAMES},
        noise=Unoise(n_min=-1.5, n_max=1.5),
        scale=0.05,
    )
    actions = ObsTerm(func=mdp.last_action)
    # These two functions are replaced by VBCPreTrainedPolicyAction before the
    # internal ObservationManager is constructed.
    velocity_commands = ObsTerm(
        func=mdp.generated_commands,
        params={"command_name": "base_velocity"},
    )
    pos_commands = ObsTerm(
        func=mdp.generated_commands,
        params={"command_name": "ee_pose"},
    )

    def __post_init__(self):
        self.enable_corruption = True
        self.concatenate_terms = True
        self.history_length = 3


@configclass
class VBCActionsCfg:
    """[VBC-PHYSICAL-GRIPPER] 10-D high-level action over frozen WBC + gripper."""

    vbc_command = mdp.VBCPreTrainedPolicyActionCfg(
        asset_name="robot",
        low_level_actions=mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "FR_hip_joint",
                "FR_thigh_joint",
                "FR_calf_joint",
                "FL_hip_joint",
                "FL_thigh_joint",
                "FL_calf_joint",
                "RR_hip_joint",
                "RR_thigh_joint",
                "RR_calf_joint",
                "RL_hip_joint",
                "RL_thigh_joint",
                "RL_calf_joint",
                "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
            ],
            scale=0.25,
            use_default_offset=True,
            preserve_order=True,
        ),
        low_level_observations=VBCWBCObservationCfg(),
        low_level_decimation=4,
        # [VBC-TUNING] Stage-1 teacher freezes the base: the ground-truth object
        # sits within arm reach on a fixed table, and the frozen WBC has a ~0.05
        # m/s command deadband, so tiny base commands are useless.  The 3 base
        # dims are kept in the action vector (inert) so a later stage can simply
        # set a non-zero velocity_scale and fine-tune the same network for mobile
        # grasping.
        velocity_scale=_velocity_scale_from_env(),
        # [VBC-DELTA] Workspace anchor plus per-step increment scale (reverted to
        # the configuration that produced the best grasp rate so far).
        pose_anchor=(0.50, 0.00, 0.55),
        pose_range=(0.15, 0.15, 0.12),
        # [VBC-DELTA] Per-step increment.  Kept at 0.02: halving to 0.01 (to
        # reduce lag) slowed early reaching too much and regressed learning.
        position_scale=(0.020, 0.020, 0.020),
    )


@configclass
class VBCObservationsCfg:
    """[VBC-PHYSICAL-GRIPPER] Privileged teacher observation; no camera yet."""

    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.15, n_max=0.15),
            scale=0.2,
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.03, n_max=0.03),
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"joint_names": WBC_JOINT_NAMES},
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"joint_names": WBC_JOINT_NAMES},
            noise=Unoise(n_min=-1.0, n_max=1.0),
            scale=0.05,
        )
        actions = ObsTerm(func=mdp.last_action)
        # [VBC-NEW] Privileged object pose replaces the future visual student input.
        object_position_link0 = ObsTerm(
            func=mdp.object_position_link0,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "object_cfg": SceneEntityCfg("cube"),
            },
        )
        ee_pose_link0 = ObsTerm(
            func=mdp.end_effector_link0_relative_pose,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        gripper_position = ObsTerm(
            func=mdp.gripper_joint_positions,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["joint7", "joint8"])},
        )
        gripper_command = ObsTerm(
            func=mdp.gripper_command,
            params={"action_name": "vbc_command"},
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 1

    @configclass
    class CriticCfg(PolicyCfg):
        """Privileged critic gets velocity and contact information."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, scale=2.0)
        joint_torques = ObsTerm(
            func=mdp.joint_effort,
            params={"asset_cfg": SceneEntityCfg("robot")},
            clip=(-100.0, 100.0),
            scale=0.1,
        )
        feet_contact = ObsTerm(
            func=mdp.contact,
            params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
            clip=(0.0, 1.0),
            scale=0.1,
        )

        def __post_init__(self):
            super().__post_init__()
            self.enable_corruption = False

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class VBCStudentObservationsCfg:
    """[VBC-VISION-STUDENT] Student proprio/image and privileged teacher views.

    DistillationRunner receives ``policy + images`` as the student input and
    ``teacher`` as the privileged input.  The teacher group intentionally
    mirrors the 65-D PPO teacher observation so its saved actor checkpoint can
    be loaded without a shape adapter.
    """

    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.2)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"joint_names": WBC_JOINT_NAMES},
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"joint_names": WBC_JOINT_NAMES},
            scale=0.05,
        )
        actions = ObsTerm(func=mdp.last_action)
        ee_pose_link0 = ObsTerm(
            func=mdp.end_effector_link0_relative_pose,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        gripper_position = ObsTerm(
            func=mdp.gripper_joint_positions,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["joint7", "joint8"])},
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 1

    @configclass
    class ImagesCfg(ObsGroup):
        rgbd_history = ObsTerm(
            func=mdp.camera_rgbd_history_chw,
            params={
                "body_camera_cfg": SceneEntityCfg("body_camera"),
                "wrist_camera_cfg": SceneEntityCfg("wrist_camera"),
                "history_length": 4,
                "depth_max": 3.0,
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class TeacherCfg(VBCObservationsCfg.PolicyCfg):
        """Privileged state teacher input, kept identical to PPO teacher."""

    policy: PolicyCfg = PolicyCfg()
    images: ImagesCfg = ImagesCfg()
    teacher: TeacherCfg = TeacherCfg()


@configclass
class VBCShapeTeacherObservationsCfg:
    """Privileged state and a separate frozen 1024-D object-shape group."""

    @configclass
    class PolicyCfg(VBCObservationsCfg.PolicyCfg):
        object_orientation_link0 = ObsTerm(
            func=mdp.object_orientation_link0_rpy,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "object_cfg": SceneEntityCfg("cube"),
            },
        )

    @configclass
    class CriticCfg(VBCObservationsCfg.CriticCfg):
        object_orientation_link0 = ObsTerm(
            func=mdp.object_orientation_link0_rpy,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "object_cfg": SceneEntityCfg("cube"),
            },
        )

    @configclass
    class ShapeCfg(ObsGroup):
        pointnet_feature = ObsTerm(
            func=mdp.object_shape_feature,
            params={"feature_paths": VBC_OBJECT_FEATURE_PATHS},
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 1

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()
    shape: ShapeCfg = ShapeCfg()


@configclass
class VBCMaskDepthStudentObservationsCfg:
    """Visual student plus the shape-aware privileged teacher observations."""

    @configclass
    class PolicyCfg(VBCStudentObservationsCfg.PolicyCfg):
        pass

    @configclass
    class ImagesCfg(ObsGroup):
        mask_depth_history = ObsTerm(
            func=mdp.camera_masked_depth_history_chw,
            params={
                "body_camera_cfg": SceneEntityCfg("body_camera"),
                "wrist_camera_cfg": SceneEntityCfg("wrist_camera"),
                "history_length": 4,
                "depth_min": 0.15,
                "depth_max": 2.0,
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class TeacherCfg(VBCShapeTeacherObservationsCfg.PolicyCfg):
        pass

    @configclass
    class ShapeCfg(VBCShapeTeacherObservationsCfg.ShapeCfg):
        pass

    policy: PolicyCfg = PolicyCfg()
    images: ImagesCfg = ImagesCfg()
    teacher: TeacherCfg = TeacherCfg()
    shape: ShapeCfg = ShapeCfg()


@configclass
class VBCRewardsCfg:
    """[VBC-NEW] Reach-first rewards for a physical tabletop cube."""

    object_reach = RewTerm(
        func=mdp.object_ee_distance_exp,
        weight=3.0,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "object_cfg": SceneEntityCfg("cube"),
            "std": 0.12,
        },
    )
    object_reach_linear = RewTerm(
        func=mdp.object_ee_distance_linear,
        # [VBC-TUNING] Linear distance shaping gives a usable gradient while the
        # EE is still far away; the exponential term sharpens the last few cm.
        weight=1.0,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "object_cfg": SceneEntityCfg("cube"),
            "max_distance": 1.0,
        },
    )
    object_near_ee = RewTerm(
        func=mdp.object_ee_near_bonus,
        weight=2.0,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "object_cfg": SceneEntityCfg("cube"),
            "threshold": 0.06,
        },
    )
    gripper_contact = RewTerm(
        func=mdp.gripper_contact_force,
        weight=3.0,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "object_cfg": SceneEntityCfg("cube"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*link7.*", ".*link8.*"]),
            "threshold": 0.5,
        },
    )
    object_lift_progress = RewTerm(
        func=mdp.object_lift_progress,
        # [VBC-TUNING] Gated on a closed-finger contact so pushing/bouncing the
        # cube cannot farm lift reward; only a real hold counts.  Linear again
        # (staged start_lift hurt early learning).
        weight=6.0,
        params={
            "object_cfg": SceneEntityCfg("cube"),
            "robot_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=[".*link7.*", ".*link8.*"]
            ),
            "table_top_z": 0.51,
            "start_lift": 0.0,
            "target_lift": 0.20,
            "contact_threshold": 0.5,
            "gate_distance": 0.14,
        },
    )
    physical_grasp_success = RewTerm(
        func=mdp.physical_grasp_success,
        # [VBC-TUNING] Stable physical grasp is the primary terminal reward.
        weight=40.0,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "object_cfg": SceneEntityCfg("cube"),
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=[".*link7.*", ".*link8.*"]
            ),
            "table_top_z": 0.51,
            "lift_threshold": 0.10,
            "contact_threshold": 0.5,
            "hold_steps": 25,
        },
    )
    base_height = RewTerm(
        func=mdp.base_height_tracking,
        weight=1.0,
        params={"target_height": 0.28, "std": 0.025},
    )
    base_orientation = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)
    vertical_velocity = RewTerm(
        func=mdp.vbc_lin_vel_z_l2, weight=-2.0, params={"max_value": 0.25})
    angular_velocity = RewTerm(
        func=mdp.vbc_ang_vel_xy_l2, weight=-0.10, params={"max_value": 5.0})
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-5)
    joint_acceleration = RewTerm(
        func=mdp.vbc_joint_acc_l2, weight=-2.5e-7, params={"max_value": 2.0e6})
    joint_power = RewTerm(func=mdp.joint_power, weight=-2.0e-5)
    action_rate = RewTerm(func=mdp.vbc_command_rate_l2, weight=-0.02)
    base_command = RewTerm(func=mdp.vbc_base_command_l2, weight=-0.01)
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.10,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
        },
    )
    joint_limits = RewTerm(func=mdp.joint_pos_limits, weight=-1.0)


@configclass
class VBCEventCfg(EventCfg):
    """[VBC-PHYSICAL-GRIPPER] Reset the dynamic cube and open the gripper."""

    reset_cube = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cube"),
            "pose_range": {
                "x": (-0.06, 0.06),
                "y": (-0.06, 0.06),
                "z": (0.0, 0.0),
                # [VBC-NEW] Keep all cube corners on the tabletop at reset;
                # randomize yaw only so physical initialization stays stable.
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (-math.pi, math.pi),
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )


@configclass
class VBCMultiObjectEventCfg(VBCEventCfg):
    """Reset each object at its own half-height while randomizing XY/yaw."""

    reset_cube = EventTerm(
        func=mdp.reset_vbc_object_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cube"),
            "object_half_heights": VBC_OBJECT_HALF_HEIGHTS,
            "table_top_z": 0.51,
            "pose_range": {
                "x": (-0.06, 0.06),
                "y": (-0.06, 0.06),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (-math.pi, math.pi),
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )


@configclass
class VBCCurriculumCfg:
    """[VBC-PHYSICAL-GRIPPER] Expand cube XY range after reach improves."""

    cube_position_range = CurrTerm(
        func=mdp.vbc_object_position_curriculum,
        params={
            "reward_term_name": "physical_grasp_success",
            "success_reward_ratio": 0.05,
            "range_step": 0.025,
            "max_x_half_range": 0.30,
            "max_y_half_range": 0.30,
        },
    )


@configclass
class Go2PiperVBCTeacherEnvCfg(LeggedManipLabEnvCfg):
    """[VBC-NEW] Go2-Piper flat tabletop privileged VBC teacher."""

    scene: VBCSceneCfg = VBCSceneCfg(num_envs=2048, env_spacing=2.5)
    observations: VBCObservationsCfg = VBCObservationsCfg()
    actions: VBCActionsCfg = VBCActionsCfg()
    # [VBC-NEW] The low-level WBC receives commands through the action term;
    # the teacher does not use the old command manager's random pose targets.
    commands = None
    rewards: VBCRewardsCfg = VBCRewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: VBCEventCfg = VBCEventCfg()
    curriculum: VBCCurriculumCfg = VBCCurriculumCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.robot: ArticulationCfg = GO2_PIPER_VBC_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot"
        )
        self.events.push_robot = None
        # Keep the pretrained WBC's nominal joint posture at reset. The cube,
        # rather than the arm reset scale, should provide task variation.
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        # [VBC-FIX-STAGE1] The inherited locomotion reset drops the robot at a
        # random pose inside x/y +/-0.5 m with a random yaw, while the tabletop
        # object is fixed in front of the env origin.  Measured effect: only
        # ~50-60% of spawns left the object inside the arm's kinematic reach,
        # which caps success no matter how long the policy is trained, and
        # contradicts the "stage-1 object sits within arm reach" design note.
        # Stage 1 therefore keeps the robot standing at the table.  A later
        # mobile stage should widen this range (curriculum) together with
        # re-enabling the base velocity command dims.
        self.events.reset_base.params["pose_range"] = {
            "x": (-0.05, 0.05),
            "y": (-0.05, 0.05),
            "z": (-0.02, 0.02),
            "roll": (-0.02, 0.02),
            "pitch": (-0.02, 0.02),
            "yaw": (-0.10, 0.10),
        }
        self.actions.vbc_command.debug_vis = False
        self.disable_zero_weight_rewards()


@configclass
class Go2PiperVBCStudentEnvCfg(Go2PiperVBCTeacherEnvCfg):
    """[VBC-VISION-STUDENT] RGB-D student rollout environment.

    This environment keeps the same physical gripper and frozen WBC action
    term as the teacher.  Only the observation split changes: the student
    sees proprioception plus two camera streams, while DistillationRunner's
    teacher sees the privileged state group.
    """

    scene: VBCStudentSceneCfg = VBCStudentSceneCfg(num_envs=128, env_spacing=2.5)
    observations: VBCStudentObservationsCfg = VBCStudentObservationsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.robot: ArticulationCfg = GO2_PIPER_VBC_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot"
        )
        self.events.push_robot = None
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.actions.vbc_command.debug_vis = False
        self.disable_zero_weight_rewards()


@configclass
class Go2PiperVBCShapeTeacherEnvCfg(Go2PiperVBCTeacherEnvCfg):
    """Paper-aligned privileged teacher with multi-object shape conditioning."""

    scene: VBCMultiObjectSceneCfg = VBCMultiObjectSceneCfg(num_envs=2048, env_spacing=2.5)
    observations: VBCShapeTeacherObservationsCfg = VBCShapeTeacherObservationsCfg()
    events: VBCMultiObjectEventCfg = VBCMultiObjectEventCfg()

    def __post_init__(self):
        super().__post_init__()
        # MultiAssetSpawner intentionally creates different prototypes across
        # environments, so homogeneous-physics replication must be disabled.
        self.scene.replicate_physics = False


@configclass
class Go2PiperVBCMaskDepthStudentEnvCfg(Go2PiperVBCShapeTeacherEnvCfg):
    """Deployable visual student trained from mask + segmented depth."""

    scene: VBCMaskDepthStudentSceneCfg = VBCMaskDepthStudentSceneCfg(
        num_envs=128, env_spacing=2.5
    )
    observations: VBCMaskDepthStudentObservationsCfg = VBCMaskDepthStudentObservationsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.replicate_physics = False


@configclass
class Go2PiperVBCTeacherEnvCfg_PLAY(Go2PiperVBCTeacherEnvCfg):
    """[VBC-NEW] Small deterministic scene for inspecting the teacher."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.observations.critic.enable_corruption = False
        self.terminations.base_contact = None
        self.terminations.bad_orientation = None
        self.viewer.origin_type = "asset_root"
        self.viewer.asset_name = "robot"
        self.viewer.env_index = 0
        self.viewer.eye = (2.2, -2.2, 1.6)
        self.viewer.lookat = (0.45, 0.0, 0.45)
