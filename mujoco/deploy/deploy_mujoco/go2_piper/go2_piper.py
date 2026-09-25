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

"""
MuJoCo deployment script for legged robot policy rollout.

Loads a TorchScript policy and runs it in a MuJoCo simulation with
keyboard-driven velocity / position commands.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import torch
import yaml

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
DEPLOY_MUJOCO_PATH = Path(__file__).resolve().parent.parent.parent
if str(DEPLOY_MUJOCO_PATH) not in sys.path:
    sys.path.append(str(DEPLOY_MUJOCO_PATH))

from deploy_mujoco.keyboard_controller import KeyboardController  # noqa: E402

CURRENT_FILE_DIR = Path(__file__).parent
CURRENT_ROOT_DIR = CURRENT_FILE_DIR.parent.parent.parent
REPOSITORY_ROOT = CURRENT_ROOT_DIR.parent
POLICY_CONTRACT_PATH = (
    REPOSITORY_ROOT / "ros2_ws" / "src" / "go2_piper_wbc"
    / "config" / "policy_contract.yaml"
)
WBC_PYTHON_ROOT = REPOSITORY_ROOT / "ros2_ws" / "src" / "go2_piper_wbc"
if str(WBC_PYTHON_ROOT) not in sys.path:
    sys.path.append(str(WBC_PYTHON_ROOT))

from go2_piper_wbc.policy_contract import indices_for_names, load_policy_contract  # noqa: E402

# ---------------------------------------------------------------------------
# Joint index remapping
# ---------------------------------------------------------------------------
# MuJoCo qpos order is explicit in the MJCF and differs from policy order.
MUJOCO_LEG_JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def get_gravity_orientation(quaternion: torch.Tensor) -> torch.Tensor:
    """Return the projected gravity vector from a (qw, qx, qy, qz) quaternion."""
    qw, qx, qy, qz = quaternion
    return torch.tensor([
        2.0 * (-qz * qx + qw * qy),
        -2.0 * (qz * qy + qw * qx),
        1.0 - 2.0 * (qw * qw + qz * qz),
    ])


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    target_q = torch.tensor(target_q)
    q = torch.tensor(q)
    kp = torch.tensor(kp)
    target_dq = torch.tensor(target_dq)
    dq = torch.tensor(dq)
    kd = torch.tensor(kd)

    return (target_q - q) * kp + (target_dq - dq) * kd


# ---------------------------------------------------------------------------
# Command conversion
# ---------------------------------------------------------------------------

def parse_keyboard_command(cmd: dict) -> torch.Tensor:
    """
    Convert raw keyboard command dict to a (1, 10) command tensor.

    Layout: [vx, vy, yaw | px, py, pz | qw, qx, qy, qz]
    """
    lin = cmd["velocity"]   # (3,)
    pos = cmd["pos"]      # (7,)  px py pz qw qx qy qz

    values = [*lin[:3], *pos[:3], *pos[3:7]]
    command = torch.tensor([values], dtype=torch.float32)

    return command


# ---------------------------------------------------------------------------
# Observation buffer helpers
# ---------------------------------------------------------------------------

def _roll_append(buf: torch.Tensor, new: torch.Tensor, chunk: int) -> torch.Tensor:
    """Shift *buf* left by *chunk* columns and append *new* on the right."""
    return torch.cat([buf, new], dim=-1)[:, chunk:]


# ---------------------------------------------------------------------------
# Camera setup
# ---------------------------------------------------------------------------

def setup_tracking_camera(viewer: mujoco.viewer.Handle, model: mujoco.MjModel,
                           body_name: str = "base_link") -> None:
    """Configure the passive viewer to track *body_name*."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id == -1:
        print(f"[Warning] Body '{body_name}' not found — camera tracking disabled.")
        return
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = body_id
    viewer.cam.distance = 3.0
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 90


# ---------------------------------------------------------------------------
# Command visualization (markers in the MuJoCo viewer)
# ---------------------------------------------------------------------------

def _quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _quat_to_mat_wxyz(q: np.ndarray) -> np.ndarray:
    """Rotation matrix from a (w, x, y, z) quaternion."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def draw_command_viz(viewer, target_world, target_quat_world, ee_world, vel_from, vel_to) -> None:
    """Draw the EE goal marker, the current EE, and the velocity command arrow.

    - green sphere + RGB axes : commanded EE pose (xy body-relative, z world,
      orientation relative to the base, converted to world here)
    - blue sphere             : actual EE position
    - white line              : goal -> actual tracking error
    - yellow arrow            : base-velocity command (world direction)
    """
    scn = viewer.user_scn
    scn.ngeom = 0

    def _sphere(pos, radius, rgba):
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius, 0.0, 0.0]),
            np.asarray(pos, dtype=float),
            np.eye(3).reshape(-1),
            np.asarray(rgba, dtype=np.float32),
        )
        scn.ngeom += 1

    def _arrow(start, end, width, rgba):
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_connector(
            geom, mujoco.mjtGeom.mjGEOM_ARROW, width,
            np.asarray(start, dtype=float), np.asarray(end, dtype=float))
        geom.rgba[:] = rgba
        scn.ngeom += 1

    def _line(start, end, width, rgba):
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_connector(
            geom, mujoco.mjtGeom.mjGEOM_LINE, width,
            np.asarray(start, dtype=float), np.asarray(end, dtype=float))
        geom.rgba[:] = rgba
        scn.ngeom += 1

    target_world = np.asarray(target_world, dtype=float)
    ee_world = np.asarray(ee_world, dtype=float)

    _sphere(target_world, 0.030, (0.0, 1.0, 0.0, 0.9))   # commanded EE pose
    _sphere(ee_world, 0.018, (0.0, 0.4, 1.0, 0.9))        # actual EE pose

    # Orientation axes of the commanded pose (R/G/B = body X/Y/Z).
    rot = _quat_to_mat_wxyz(np.asarray(target_quat_world, dtype=float))
    for axis, rgba in enumerate(((1.0, 0.15, 0.15, 0.95),
                                 (0.15, 1.0, 0.15, 0.95),
                                 (0.15, 0.4, 1.0, 0.95))):
        _arrow(target_world, target_world + rot[:, axis] * 0.08, 0.006, rgba)

    if hasattr(mujoco, "mjv_connector"):
        _line(target_world, ee_world, 2, (1.0, 1.0, 1.0, 0.6))
        _arrow(vel_from, vel_to, 0.012, (1.0, 0.8, 0.0, 0.95))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a TorchScript locomotion policy in MuJoCo."
    )
    parser.add_argument(
        "config_file",
        type=str,
        help="YAML config filename located in the same directory as this script.",
    )
    return parser


def load_config(config_path: Path) -> dict:
    with config_path.open("r") as f:
        return yaml.safe_load(f)


def main() -> None:
    args = build_arg_parser().parse_args()
    config_path = CURRENT_FILE_DIR / args.config_file
    cfg = load_config(config_path)
    contract = load_policy_contract(POLICY_CONTRACT_PATH)
    policy_leg_names = list(contract.joint_names[:12])
    policy_to_mujoco = indices_for_names(policy_leg_names, MUJOCO_LEG_JOINT_NAMES)
    mujoco_to_policy = indices_for_names(MUJOCO_LEG_JOINT_NAMES, policy_leg_names)

    # ── resolve paths ────────────────────────────────────────────────────────
    root = str(CURRENT_ROOT_DIR)
    policy_path = cfg["policy_path"].replace("{CURRENT_ROOT_DIR}", root)
    xml_path    = cfg["xml_path"].replace("{CURRENT_ROOT_DIR}", root)

    # ── simulation parameters ────────────────────────────────────────────────
    simulation_duration  = cfg["simulation_duration"]
    simulation_dt = 1.0 / contract.physics_rate_hz
    control_decimation = round(
        contract.physics_rate_hz / contract.policy_rate_hz)

    # ── control parameters ───────────────────────────────────────────────────
    policy_kps = torch.from_numpy(contract.stiffness)
    policy_kds = torch.from_numpy(contract.damping)
    policy_defaults = torch.from_numpy(contract.default_angles)
    kps = torch.cat([policy_kps[:12][policy_to_mujoco], policy_kps[12:]])
    kds = torch.cat([policy_kds[:12][policy_to_mujoco], policy_kds[12:]])
    default_angles = torch.cat([
        policy_defaults[:12][policy_to_mujoco], policy_defaults[12:]])
    action_scale = contract.action_scale
    action_clip = contract.action_clip

    # ── observation dimensions ───────────────────────────────────────────────
    num_actions  = len(contract.joint_names)
    num_hist     = contract.history_depth
    num_env      = cfg["num_env"]

    # ── scaling factors ──────────────────────────────────────────────────────
    base_ang_vel_scale = contract.base_angular_velocity_scale
    joint_vel_scale    = contract.joint_velocity_scale

    # ── policy & I/O buffers ─────────────────────────────────────────────────
    policy = torch.jit.load(policy_path)
    action = torch.zeros(num_env, num_actions, dtype=torch.float32)

    target_dof_pos = default_angles.clone()

    # history observation buffers  (shape: [num_env, feature_dim * num_hist])
    base_ang_vel_obs      = torch.zeros(num_env, 3  * num_hist)
    joint_pos_obs         = torch.zeros(num_env, 18 * num_hist)
    joint_vel_obs         = torch.zeros(num_env, 18 * num_hist)
    actions_obs           = torch.zeros(num_env, 18 * num_hist)
    projected_gravity_obs = torch.zeros(num_env, 3  * num_hist)
    vel_command_obs       = torch.zeros(num_env, 3  * num_hist)
    pos_command_obs       = torch.zeros(num_env, 7  * num_hist)

    # ── MuJoCo model ─────────────────────────────────────────────────────────
    model = mujoco.MjModel.from_xml_path(xml_path)
    data  = mujoco.MjData(model)
    model.opt.timestep = simulation_dt
    mujoco.mj_forward(model, data)

    kb_controller = KeyboardController()
    kb_controller.start_display(fps=10.0)

    # ── simulation loop ───────────────────────────────────────────────────────
    #
    # MuJoCo qpos layout  (7 + num_joints):
    #   [0:3]  base position (x, y, z)
    #   [3:7]  base quaternion (qw, qx, qy, qz)
    #   [7:]   joint positions
    #
    # MuJoCo qvel layout  (6 + num_joints):
    #   [0:3]  base linear velocity
    #   [3:6]  base angular velocity
    #   [6:]   joint velocities

    step_counter = 0
    last_policy_time = time.time()

    # ── visualization state (world-frame markers) ─────────────────────────────
    viz_target = np.zeros(3)
    viz_target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    viz_ee = np.zeros(3)
    viz_vel_from = np.zeros(3)
    viz_vel_to = np.zeros(3)
    viz_ready = False

    with mujoco.viewer.launch_passive(model, data) as viewer:
        setup_tracking_camera(viewer, model)

        sim_start = time.time()
        while viewer.is_running() and (time.time() - sim_start) < simulation_duration:
            step_start = time.time()

            # ── PD control ───────────────────────────────────────────────────
            tau = pd_control(
                target_dof_pos,
                torch.tensor(data.qpos[7:]),
                kps,
                torch.zeros_like(kds),
                torch.tensor(data.qvel[6:]),
                kds,
            )
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)

            step_counter += 1
            if step_counter % control_decimation == 0:

                # ── read sensors ─────────────────────────────────────────────
                qj   = torch.tensor(data.qpos[7:], dtype=torch.float32).unsqueeze(0)
                dqj  = torch.tensor(data.qvel[6:], dtype=torch.float32).unsqueeze(0) * joint_vel_scale
                quat = torch.tensor(data.qpos[3:7], dtype=torch.float32)
                omega = torch.tensor(data.qvel[3:6], dtype=torch.float32).unsqueeze(0) * base_ang_vel_scale

                # ── command ───────────────────────────────────────────────────
                full_command = parse_keyboard_command(kb_controller.get_command())
                vel_command  = full_command[:, :3]
                pos_command  = full_command[:, 3:10]

                # ── derived quantities ────────────────────────────────────────
                qj_rel           = qj - default_angles
                gravity_vec      = get_gravity_orientation(quat).unsqueeze(0)

                # leg joints: remap Isaac → MuJoCo; arm joints: pass through
                leg_pos_reordered = qj_rel[:, :12][:, mujoco_to_policy]
                leg_vel_reordered = dqj[:, :12][:, mujoco_to_policy]

                # ── update history buffers ────────────────────────────────────
                base_ang_vel_obs      = _roll_append(base_ang_vel_obs,      omega,             3)
                projected_gravity_obs = _roll_append(projected_gravity_obs, gravity_vec,       3)
                joint_pos_obs         = _roll_append(joint_pos_obs,  torch.cat([leg_pos_reordered, qj_rel[:, 12:]], dim=-1), 18)
                joint_vel_obs         = _roll_append(joint_vel_obs,  torch.cat([leg_vel_reordered, dqj[:, 12:]],   dim=-1), 18)
                actions_obs           = _roll_append(actions_obs,          action,            18)
                vel_command_obs       = _roll_append(vel_command_obs,      vel_command,        3)
                pos_command_obs       = _roll_append(pos_command_obs,      pos_command,        7)

                hist_obs = torch.cat([
                    base_ang_vel_obs,
                    projected_gravity_obs,
                    joint_pos_obs,
                    joint_vel_obs,
                    actions_obs,
                    vel_command_obs,
                    pos_command_obs,
                ], dim=-1).float().clamp(-100.0, 100.0)

                # ── policy inference ──────────────────────────────────────────
                if time.time() - last_policy_time > 3.0:
                    action = policy(hist_obs).clamp(-action_clip, action_clip)

                # remap leg actions back to MuJoCo joint order
                leg_action  = action[:, :12][:, policy_to_mujoco]
                arm_action  = action[:, 12:]
                action_out  = torch.cat([leg_action, arm_action], dim=-1)

                target_dof_pos = action_out * action_scale + default_angles

                # ── command visualization ─────────────────────────────────────
                # pos_command is the mixed frame: xy in the body frame, z is the
                # world height.  Convert the xy part to world for the marker.
                base_pos = np.array(data.qpos[0:3], dtype=float)
                qw, qx, qy, qz = (float(v) for v in data.qpos[3:7])
                yaw = float(np.arctan2(
                    2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
                cy, sy = np.cos(yaw), np.sin(yaw)
                px, py, pz = (float(pos_command[0, 0]),
                              float(pos_command[0, 1]),
                              float(pos_command[0, 2]))
                viz_target[:] = (base_pos[0] + cy * px - sy * py,
                                 base_pos[1] + sy * px + cy * py,
                                 pz)
                # target orientation: mixed command is relative to the base, so
                # the world orientation is base_quat * cmd_quat.
                viz_target_quat[:] = _quat_mul_wxyz(
                    np.array([qw, qx, qy, qz], dtype=float),
                    np.asarray(pos_command[0, 3:7], dtype=float))
                viz_ee[:] = np.asarray(data.body("end_effector").xpos, dtype=float)
                vx_cmd, vy_cmd = float(vel_command[0, 0]), float(vel_command[0, 1])
                viz_vel_from[:] = (base_pos[0], base_pos[1], base_pos[2] + 0.08)
                viz_vel_to[:] = (base_pos[0] + (cy * vx_cmd - sy * vy_cmd) * 0.6,
                                 base_pos[1] + (sy * vx_cmd + cy * vy_cmd) * 0.6,
                                 base_pos[2] + 0.08)
                viz_ready = True

            if viz_ready:
                draw_command_viz(viewer, viz_target, viz_target_quat,
                                 viz_ee, viz_vel_from, viz_vel_to)
            viewer.sync()

            # ── real-time pacing ──────────────────────────────────────────────
            elapsed = time.time() - step_start
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time * 2)


if __name__ == "__main__":
    main()
