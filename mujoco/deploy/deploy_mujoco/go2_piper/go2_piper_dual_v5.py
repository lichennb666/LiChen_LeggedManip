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

# ---------------------------------------------------------------------------
# Joint index remapping
# ---------------------------------------------------------------------------
# IsaacLab order  → MuJoCo order
# FL_hip/thigh/calf (0-2), FR_hip/thigh/calf (3-5),
# RL_hip/thigh/calf (6-8), RR_hip/thigh/calf (9-11)
# ↓
# FR (3-5), FL (0-2), RR (9-11), RL (6-8)
ISAAC_TO_MUJOCO: list[int] = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]


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

    # ── resolve paths ────────────────────────────────────────────────────────
    root = str(CURRENT_ROOT_DIR)
    policy_path = cfg["policy_path"].replace("{CURRENT_ROOT_DIR}", root)
    xml_path    = cfg["xml_path"].replace("{CURRENT_ROOT_DIR}", root)

    # ── simulation parameters ────────────────────────────────────────────────
    simulation_duration  = cfg["simulation_duration"]
    simulation_dt        = cfg["simulation_dt"]
    control_decimation   = cfg["control_decimation"]

    # ── control parameters ───────────────────────────────────────────────────
    kps            = torch.tensor(cfg["kps"],            dtype=torch.float32)
    kds            = torch.tensor(cfg["kds"],            dtype=torch.float32)
    default_angles = torch.tensor(cfg["default_angles"], dtype=torch.float32)
    action_scale   = torch.tensor(cfg["action_scale"],   dtype=torch.float32)

    # ── observation dimensions ───────────────────────────────────────────────
    num_actions  = cfg["num_actions"]
    num_hist     = cfg["num_hist"]
    num_env      = cfg["num_env"]

    # ── scaling factors ──────────────────────────────────────────────────────
    base_ang_vel_scale = cfg["base_ang_vel_scale"]
    joint_vel_scale    = cfg["joint_vel_scale"]

    # ── policies ──────────────────────────────────────────────────
    root = str(Path(__file__).resolve().parent.parent.parent.parent)
    policy = torch.jit.load(root + "/deploy/policy/go2_piper/policy_flat.pt")
    policy_stairs = torch.jit.load(root + "/deploy/policy/go2_piper/policy_stairs_v2.pt")
    height_scan_obs = torch.zeros(num_env, 187 * num_hist)
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

    # key listener for toggling stairs mode
    use_stairs_state = [False]
    from pynput import keyboard as pynput_kb
    def on_press(key):
        try:
            if hasattr(key, 'char') and key.char == 't':
                use_stairs_state[0] = not use_stairs_state[0]
                print(f"\n[MODE: {'STAIRS' if use_stairs_state[0] else 'FLAT'}]")
        except: pass
    kb_listener = pynput_kb.Listener(on_press=on_press)
    kb_listener.start()

    step_counter = 0
    control_step = 0
    last_policy_time = time.time()

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
                control_step += 1

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
                leg_pos_reordered = qj_rel[:, :12][:, ISAAC_TO_MUJOCO]
                leg_vel_reordered = dqj[:, :12][:, ISAAC_TO_MUJOCO]

                # ── update history buffers ────────────────────────────────────
                base_ang_vel_obs      = _roll_append(base_ang_vel_obs,      omega,             3)
                projected_gravity_obs = _roll_append(projected_gravity_obs, gravity_vec,       3)
                joint_pos_obs         = _roll_append(joint_pos_obs,  torch.cat([leg_pos_reordered, qj_rel[:, 12:]], dim=-1), 18)
                joint_vel_obs         = _roll_append(joint_vel_obs,  torch.cat([leg_vel_reordered, dqj[:, 12:]],   dim=-1), 18)
                actions_obs           = _roll_append(actions_obs,          action,            18)
                vel_command_obs       = _roll_append(vel_command_obs,      vel_command,        3)
                pos_command_obs       = _roll_append(pos_command_obs,      pos_command,        7)

                if use_stairs_state[0]:
                    # compute real height scan
                    heights = []
                    bp = data.qpos[0:3].copy()
                    R9 = np.zeros(9); mujoco.mju_quat2Mat(R9, data.qpos[3:7].copy())
                    y = np.arctan2(R9[3], R9[0]); sz = bp[2] + 0.2
                    for i in range(17):
                        for j in range(11):
                            dx = (i-8)*0.1; dy = (j-5)*0.1
                            rx = dx*np.cos(y)-dy*np.sin(y); ry = dx*np.sin(y)+dy*np.cos(y)
                            pnt = np.array([bp[0]+rx,bp[1]+ry,sz],dtype=np.float64).reshape(3,1)
                            gid = np.zeros(1,dtype=np.int32)
                            d = mujoco.mj_ray(model,data,pnt,np.array([0.,0.,-1.],dtype=np.float64).reshape(3,1),None,False,-1,gid)
                            heights.append(d if d > 0 else sz)
                    h_t = torch.tensor(heights).unsqueeze(0)
                    height_scan_obs = _roll_append(height_scan_obs, h_t, 187)
                    if control_step % 30 == 0:
                        print(f"[HS] step={control_step} min={min(heights):.2f} max={max(heights):.2f} var={np.var(heights):.4f}")
                    hist_obs = torch.cat([
                        base_ang_vel_obs, projected_gravity_obs, joint_pos_obs,
                        joint_vel_obs, actions_obs, vel_command_obs, pos_command_obs,
                        height_scan_obs,
                    ], dim=-1).float().clamp(-100.0, 100.0)
                    action = policy_stairs(hist_obs).clamp(-20.0, 20.0)
                else:
                    hist_obs = torch.cat([
                        base_ang_vel_obs, projected_gravity_obs, joint_pos_obs,
                        joint_vel_obs, actions_obs, vel_command_obs, pos_command_obs,
                    ], dim=-1).float().clamp(-100.0, 100.0)
                    if time.time() - last_policy_time > 3.0:
                        action = policy(hist_obs).clamp(-20.0, 20.0)

                # remap leg actions back to MuJoCo joint order
                leg_action  = action[:, :12][:, ISAAC_TO_MUJOCO]
                arm_action  = action[:, 12:]
                action_out  = torch.cat([leg_action, arm_action], dim=-1)

                target_dof_pos = action_out * action_scale + default_angles

            viewer.sync()

            # ── real-time pacing ──────────────────────────────────────────────
            elapsed = time.time() - step_start
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time * 2)


if __name__ == "__main__":
    main()