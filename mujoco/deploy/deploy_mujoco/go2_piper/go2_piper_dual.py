#!/usr/bin/env python3
"""
Go2+Piper 双策略混合部署: 平地用 Flat 策略(210D), 台阶用 Stairs 策略(771D).
自动检测地形类型切换策略.
"""

import sys, os, yaml, time
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import mujoco
import torch


def load_cfg(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _roll_append(buf, new, chunk):
    return torch.cat([buf, new], dim=-1)[:, chunk:]


def compute_height_scan(model, data, base_pos, base_quat, num_rays=187):
    """Compute real height scan using MuJoCo ray casting."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, base_quat)
    yaw = np.arctan2(R[3], R[0])

    heights = np.zeros(num_rays)
    grid_x = 17  # 1.6m / 0.1m resolution
    grid_y = 11  # 1.0m / 0.1m resolution
    sensor_z = base_pos[2] + 0.2

    idx = 0
    for i in range(grid_x):
        for j in range(grid_y):
            dx = (i - 8) * 0.1
            dy = (j - 5) * 0.1
            # Rotate by yaw
            rx = dx * np.cos(yaw) - dy * np.sin(yaw)
            ry = dx * np.sin(yaw) + dy * np.cos(yaw)
            origin = np.array([base_pos[0] + rx, base_pos[1] + ry, sensor_z])
            pnt = origin.reshape(3, 1)
            vec = np.array([0.0, 0.0, -1.0]).reshape(3, 1)
            
            geomid = np.zeros(1, dtype=np.int32)
            dist = mujoco.mj_ray(model, data, pnt, vec, None, False, -1, geomid)
            if dist < 0:
                dist = sensor_z  # no hit = assume flat
            heights[idx] = dist
            idx += 1

    return heights


def main():
    if len(sys.argv) < 2:
        config_path = os.path.join(os.path.dirname(__file__), "config_stairs.yaml")
    else:
        config_path = sys.argv[1]

    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    cfg = load_cfg(config_path)

    # Load policies
    flat_policy = torch.jit.load(os.path.join(root, "deploy/policy/go2_piper/policy_flat.pt"))
    stairs_policy = torch.jit.load(os.path.join(root, "deploy/policy/go2_piper/policy_stairs_v5.pt"))
    flat_policy.eval()
    stairs_policy.eval()

    # Load model
    xml_path = cfg["xml_path"].replace("{CURRENT_ROOT_DIR}", root)
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Config
    kps = cfg["kps"]
    kds = cfg["kds"]
    action_scale = cfg["action_scale"]
    default_angles = np.array(cfg["default_angles"])
    dt = cfg["simulation_dt"]
    decimation = cfg["control_decimation"]
    num_hist = cfg["num_hist"]

    # Shared observation buffers (flat policy only needs 210D)
    base_ang_vel_obs = torch.zeros(1, 3 * num_hist)
    joint_pos_obs = torch.zeros(1, 18 * num_hist)
    joint_vel_obs = torch.zeros(1, 18 * num_hist)
    actions_obs = torch.zeros(1, 18 * num_hist)
    projected_gravity_obs = torch.zeros(1, 3 * num_hist)
    vel_command_obs = torch.zeros(1, 3 * num_hist)
    pos_command_obs = torch.zeros(1, 7 * num_hist)
    height_scan_obs = torch.zeros(1, 187 * num_hist)

    # Init robot pose
    data.qpos[7:] = default_angles

    # Viewer
    try:
        from mujoco import viewer as mujoco_viewer
        viewer = mujoco_viewer.launch_passive(model, data)
    except Exception as e:
        print(f"[WARN] Viewer failed: {e}")
        viewer = None

    # Keyboard controller
    from keyboard_controller import KeyboardController
    kc = KeyboardController()
    cmd = kc.get_command()

    # State
    terrain_mode = "flat"  # flat or stairs
    last_height_check = 0
    terrain_var = 0.0

    print(f"[INFO] Dual-policy loaded. Flat: 210D, Stairs: 771D")
    print(f"[INFO] Keys: WASD=move, IJKLUO=arm, T=switch terrain")

    step = 0
    max_steps = 5000
    while True:
        if viewer is not None:
            if not viewer.is_running():
                break
        elif step >= max_steps:
            break
        for _ in range(decimation):
            mujoco.mj_step(model, data)

        step += 1

        # Get state
        qj = torch.tensor(data.qpos[7:].copy()).unsqueeze(0)
        dqj = torch.tensor(data.qvel[6:].copy()).unsqueeze(0)
        omega = torch.tensor(data.qvel[3:6].copy()).unsqueeze(0) * 0.2

        # Gravity projection
        quat = data.qpos[3:7].copy()
        R_mat = np.zeros(9)
        mujoco.mju_quat2Mat(R_mat, quat)
        gravity_vec = -torch.tensor([R_mat[6], R_mat[7], R_mat[8]]).unsqueeze(0)

        # Reorder legs
        leg_idx = [1, 5, 10, 0, 4, 9, 3, 7, 12, 2, 6, 11]
        leg_pos = qj[:, leg_idx]
        leg_vel = dqj[:, leg_idx]
        qj_rel = qj - torch.tensor(default_angles).unsqueeze(0)

        # Terrain detection every 50 steps
        if step - last_height_check >= 50:
            heights = compute_height_scan(model, data, data.qpos[0:3], data.qpos[3:7])
            terrain_var = np.var(heights)
            if terrain_var > 0.01:
                terrain_mode = "stairs"
            else:
                terrain_mode = "flat"
            last_height_check = step
            print(f"\r  [{step}] terrain={terrain_mode} var={terrain_var:.4f}  ", end="")

        # Get commands
        cmd = kc.get_command()
        vel_command = torch.tensor([[cmd["velocity"][0], cmd["velocity"][1], cmd["velocity"][2]]])
        pos_command = torch.tensor([[cmd["pos"][0], cmd["pos"][1], cmd["pos"][2], cmd["pos"][3], cmd["pos"][4], cmd["pos"][5], cmd["pos"][6]]])

        # Update shared buffers
        base_ang_vel_obs = _roll_append(base_ang_vel_obs, omega, 3)
        projected_gravity_obs = _roll_append(projected_gravity_obs, gravity_vec, 3)
        joint_pos_obs = _roll_append(joint_pos_obs, torch.cat([leg_pos, qj_rel[:, 12:]], dim=-1), 18)
        joint_vel_obs = _roll_append(joint_vel_obs, torch.cat([leg_vel, dqj[:, 12:]], dim=-1), 18)
        vel_command_obs = _roll_append(vel_command_obs, vel_command, 3)
        pos_command_obs = _roll_append(pos_command_obs, pos_command, 7)

        # Select policy
        if terrain_mode == "flat":
            hist_obs = torch.cat([
                base_ang_vel_obs, projected_gravity_obs, joint_pos_obs,
                joint_vel_obs, actions_obs, vel_command_obs, pos_command_obs,
            ], dim=-1).float().clamp(-100, 100)
            action = flat_policy(hist_obs).clamp(-20, 20)
        else:
            # Compute real height_scan and update buffer
            heights = compute_height_scan(model, data, data.qpos[0:3], data.qpos[3:7])
            h_tensor = torch.tensor(heights).unsqueeze(0)
            height_scan_obs = _roll_append(height_scan_obs, h_tensor, 187)

            hist_obs = torch.cat([
                base_ang_vel_obs, projected_gravity_obs, joint_pos_obs,
                joint_vel_obs, actions_obs, vel_command_obs, pos_command_obs,
                height_scan_obs,
            ], dim=-1).float().clamp(-100, 100)
            action = stairs_policy(hist_obs).clamp(-20, 20)

        actions_obs = _roll_append(actions_obs, action, 18)

        # Reorder and apply
        rev_leg = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
        leg_action = action[:, rev_leg]
        arm_action = action[:, 12:]
        action_reordered = torch.cat([leg_action, arm_action], dim=-1)
        action_out = action_reordered[0].detach().numpy() * np.array(action_scale)

        target_q = default_angles + action_out
        for i in range(18):
            data.ctrl[i] = target_q[i]
            data.qfrc_applied[6 + i] = kps[i] * (target_q[i] - qj[0, i].item()) - kds[i] * dqj[0, i].item()

        if viewer is not None:
            viewer.sync()

    kc.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
