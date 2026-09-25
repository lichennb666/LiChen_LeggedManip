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

import copy
import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

current_dir = os.path.dirname(os.path.abspath(__file__))

GO2_PIPER_USD = os.path.join(current_dir, "go2_piper.usd")
GO2_PIPER_VBC_URDF = os.environ.get(
    "GO2_PIPER_VBC_URDF",
    os.path.join(current_dir, "go2_piper_vbc.urdf"),
)

##
# Configuration
##


GO2_PIPER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=GO2_PIPER_USD,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(
        #     collision_enabled=True,
        #     contact_offset=0.02,
        #     rest_offset=0.005 ,
        # ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.35),
        joint_pos={
            # leg
            "FL_hip_joint": 0.1,
            "FR_hip_joint": -0.1,
            "RL_hip_joint": 0.1,
            "RR_hip_joint": -0.1,
            "FL_thigh_joint": 0.8,
            "FR_thigh_joint": 0.8,
            "RL_thigh_joint": 1.0,
            "RR_thigh_joint": 1.0,
            "FL_calf_joint": -1.5,
            "FR_calf_joint": -1.5,
            "RL_calf_joint": -1.5,
            "RR_calf_joint": -1.5,
            # arm
            "joint.*": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "base_legs": DelayedPDActuatorCfg(
            joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
            stiffness=30.0,
            damping=0.6,
            armature=0.01,
            min_delay=0,
            max_delay=4,
            friction=0.01,
        ),
        # DO NOT MOVE IF POSSIBLE
        "joint1": DelayedPDActuatorCfg(
            joint_names_expr=["joint1"],
            # effort_limit=20.0,
            # effort_limit_sim=20.0,
            stiffness=200.0,
            damping=8.0,
            armature=0.01,
            min_delay=0,
            max_delay=4,
            friction=0.01,
        ),
        "joint2": DelayedPDActuatorCfg(
            joint_names_expr=["joint2"],
            # effort_limit=20.0,
            # effort_limit_sim=20.0,
            stiffness=200.0,
            damping=6.0,
            armature=0.01,
            min_delay=0,
            max_delay=4,
            friction=0.01,
        ),
        "joint3": DelayedPDActuatorCfg(
            joint_names_expr=["joint3"],
            # effort_limit=15.0,
            # effort_limit_sim=15.0,
            stiffness=320.0,
            damping=8.0,
            armature=0.01,
            min_delay=0,
            max_delay=4,
            friction=0.01,
        ),
        "joint4": DelayedPDActuatorCfg(
            joint_names_expr=["joint4"],
            # effort_limit=7.0,
            # effort_limit_sim=7.0,
            stiffness=120.0,
            damping=8.0,
            armature=0.01,
            min_delay=0,
            max_delay=4,
            friction=0.01,
        ),
        "joint5": DelayedPDActuatorCfg(
            joint_names_expr=["joint5"],
            # effort_limit=5.0,
            # effort_limit_sim=5.0,
            stiffness=120.0,
            damping=7.0,
            armature=0.01,
            min_delay=0,
            max_delay=4,
            friction=0.01,
        ),
        "joint6": DelayedPDActuatorCfg(
            joint_names_expr=["joint6"],
            # effort_limit=5.0,
            # effort_limit_sim=5.0,
            stiffness=80.0,
            damping=4.0,
            armature=0.01,
            min_delay=0,
            max_delay=4,
            friction=0.01,
        ),
    },
)


# [VBC-PHYSICAL-GRIPPER] This is intentionally a separate config.  The old
# GO2_PIPER_CFG keeps using the checked-in USD and therefore keeps the old
# tasks reproducible.  The old USD has fixed gripper visuals only; this asset
# is generated from the Gazebo URDF after removing ROS/Gazebo-only sections.
GO2_PIPER_VBC_CFG = copy.deepcopy(GO2_PIPER_CFG)
GO2_PIPER_VBC_CFG.spawn = sim_utils.UrdfFileCfg(
    asset_path=GO2_PIPER_VBC_URDF,
    # Isaac's URDF importer creates its own ``configuration/`` subdirectory
    # beside the wrapper USD.  Keep usd_dir at the asset root; putting it at
    # ``.../configuration`` would produce the broken configuration/configuration
    # references observed during the first runtime smoke test.
    usd_dir=current_dir,
    usd_file_name="go2_piper_vbc.usd",
    force_usd_conversion=True,
    make_instanceable=True,
    fix_base=False,
    activate_contact_sensors=True,
    # Keep link0 and the explicit end_effector helper body available to the
    # WBC frame/reward lookups.  Fixed sensor links were already removed by
    # prepare_go2_piper_vbc_urdf.py.
    merge_fixed_joints=False,
    root_link_name="base",
    self_collision=False,
    joint_drive=None,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        retain_accelerations=False,
        linear_damping=0.0,
        angular_damping=0.0,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        max_depenetration_velocity=1.0,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False,
        solver_position_iteration_count=8,
        solver_velocity_iteration_count=2,
    ),
)

_vbc_joint_pos = dict(GO2_PIPER_CFG.init_state.joint_pos)
_vbc_joint_pos.pop("joint.*", None)
for _joint_name in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"):
    _vbc_joint_pos[_joint_name] = 0.0
# [VBC-PHYSICAL-GRIPPER] Open reset pose.  The two prismatic joints move in
# opposite directions but share one high-level binary command.
_vbc_joint_pos["joint7"] = 0.04
_vbc_joint_pos["joint8"] = -0.04
GO2_PIPER_VBC_CFG.init_state.joint_pos = _vbc_joint_pos
GO2_PIPER_VBC_CFG.actuators = {
    **GO2_PIPER_CFG.actuators,
    "gripper": DelayedPDActuatorCfg(
        joint_names_expr=["joint7", "joint8"],
        # [VBC-GRIP] Reverted to the original gains: a stiffer gripper
        # (stiffness 1000 / 100 N) knocked the object away and regressed the
        # grasp rate from ~22% to ~1%.  The weak gripper already held 80 g.
        effort_limit=20.0,
        effort_limit_sim=20.0,
        velocity_limit=1.0,
        velocity_limit_sim=1.0,
        stiffness=160.0,
        damping=8.0,
        armature=0.001,
        min_delay=0,
        max_delay=0,
        friction=0.05,
    ),
}
