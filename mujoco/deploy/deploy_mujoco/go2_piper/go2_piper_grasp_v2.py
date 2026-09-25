#!/usr/bin/env python3
"""
Go2+Piper 抓取演示 (基于原始 go2_piper.py)

与原始脚本完全相同的 PD 控制逻辑。
按 G 键: 手臂伸向桌面方块 → 闭合夹爪 → 收回。

文件位置: mujoco/deploy/deploy_mujoco/go2_piper/go2_piper_grasp_v2.py
"""

import sys, os, yaml, time
import numpy as np
import mujoco
import mujoco.viewer
import torch
from pathlib import Path


def load_cfg(path):
    with open(path) as f:
        return yaml.safe_load(f)


def pd_control(target_q, q, kp, target_dq, dq, kd):
    target_q = torch.tensor(target_q)
    q = torch.tensor(q)
    kp = torch.tensor(kp)
    target_dq = torch.tensor(target_dq)
    dq = torch.tensor(dq)
    kd = torch.tensor(kd)
    return (target_q - q) * kp + (target_dq - dq) * kd


def parse_keyboard_command(cmd: dict) -> torch.Tensor:
    vel = cmd["velocity"]
    pos = cmd["pos"]
    return torch.tensor([[vel[0],vel[1],vel[2], pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6]]], dtype=torch.float32)


def get_gravity_orientation(quat: torch.Tensor) -> torch.Tensor:
    qw, qx, qy, qz = quat
    return torch.tensor([2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy)])


def _roll_append(buf, new, chunk):
    return torch.cat([buf, new], dim=-1)[:, chunk:]


# Preset arm poses
HOME  = np.array([0.0, 0.3, -1.5, 0.0, -0.5, 0.0])
REACH = np.array([0.0, 0.6, -2.0, 0.8,  0.2, 0.0])
GRASP = np.array([0.0, 0.7, -2.2, 0.9,  0.1, 0.0])


def main():
    if len(sys.argv) < 2:
        config_path = os.path.join(os.path.dirname(__file__), "config_table.yaml")
    else:
        config_path = sys.argv[1]

    root = str(Path(__file__).resolve().parent.parent.parent.parent)
    cfg = load_cfg(config_path)

    xml_path = cfg["xml_path"].replace("{CURRENT_ROOT_DIR}", root)
    policy_path = root + "/deploy/policy/go2_piper/policy_flat.pt"

    policy = torch.jit.load(policy_path)
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    kps = cfg["kps"]
    kds = cfg["kds"]
    action_scale = torch.tensor(cfg["action_scale"], dtype=torch.float32)
    default_angles = torch.tensor(cfg["default_angles"], dtype=torch.float32)
    dt = cfg["simulation_dt"]
    decimation = cfg["control_decimation"]
    num_env = cfg["num_env"]
    num_hist = cfg["num_hist"]
    num_actions = cfg["num_actions"]

    model.opt.timestep = dt
    data.qpos[7:] = default_angles

    # State buffers
    action = torch.zeros(num_env, num_actions)
    target_dof_pos = default_angles.clone()

    base_ang_vel_obs = torch.zeros(num_env, 3*num_hist)
    joint_pos_obs = torch.zeros(num_env, 18*num_hist)
    joint_vel_obs = torch.zeros(num_env, 18*num_hist)
    actions_obs = torch.zeros(num_env, 18*num_hist)
    projected_gravity_obs = torch.zeros(num_env, 3*num_hist)
    vel_command_obs = torch.zeros(num_env, 3*num_hist)
    pos_command_obs = torch.zeros(num_env, 7*num_hist)

    ISAAC_TO_MUJOCO = [3,4,5, 0,1,2, 9,10,11, 6,7,8]
    joint_vel_scale = 0.05
    base_ang_vel_scale = 0.2
    step_counter = 0
    last_policy_time = time.time()

    # ── Grasp state ─────────────────────────────────────────────
    GRAB_HOME = 0
    GRAB_REACH = 1
    GRAB_DESCEND = 2
    GRAB_CLOSE = 3
    GRAB_LIFT = 4
    GRAB_DONE = 5

    grab_phase = GRAB_HOME
    grab_timer = 0

    def lerp(a, b, alpha):
        return a + (b - a) * alpha

    # ── Keyboard & viewer ───────────────────────────────────────
    sys.path.insert(0, "/home/lili/LeggedManip_Lab/mujoco/deploy")
    from deploy_mujoco.keyboard_controller import KeyboardController
    kb = KeyboardController()
    g_count = [0]

    def key_cb(keycode):
        if keycode == 71:  # G key (GLFW)
            g_count[0] += 1

    kb.start_display(fps=10.0)

    print("=" * 50)
    print("  Go2+Piper 抓取演示")
    print("  WASD: 移动   G: 触发抓取   ESC: 退出")
    print("  目标: 走到桌子前 (X≈1.5) 按 G 抓取绿色方块")
    print("=" * 50)

    with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as viewer:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if body_id != -1:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = body_id
            viewer.cam.distance = 3.0
            viewer.cam.elevation = -20
            viewer.cam.azimuth = 90

        while viewer.is_running():
            step_start = time.time()

            # ── PD control EVERY step ───────────────────────────
            tau = pd_control(target_dof_pos, data.qpos[7:], kps,
                            torch.zeros(num_actions), data.qvel[6:], kds)
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)

            step_counter += 1
            if step_counter % decimation != 0:
                viewer.sync()
                elapsed = time.time() - step_start
                if elapsed < model.opt.timestep:
                    time.sleep(model.opt.timestep - elapsed)
                continue

            # ── Policy / sensory update (every 4th step) ────────
            qj = torch.tensor(data.qpos[7:], dtype=torch.float32).unsqueeze(0)
            dqj = torch.tensor(data.qvel[6:], dtype=torch.float32).unsqueeze(0)*joint_vel_scale
            quat = torch.tensor(data.qpos[3:7], dtype=torch.float32)
            omega = torch.tensor(data.qvel[3:6], dtype=torch.float32).unsqueeze(0)*base_ang_vel_scale

            cmd = kb.get_command()
            full_cmd = parse_keyboard_command(cmd)
            vel_command = full_cmd[:, :3]
            pos_command = full_cmd[:, 3:10]

            qj_rel = qj - default_angles
            gravity_vec = get_gravity_orientation(quat).unsqueeze(0)
            leg_pos_reordered = qj_rel[:, :12][:, ISAAC_TO_MUJOCO]
            leg_vel_reordered = dqj[:, :12][:, ISAAC_TO_MUJOCO]

            base_ang_vel_obs = _roll_append(base_ang_vel_obs, omega, 3)
            projected_gravity_obs = _roll_append(projected_gravity_obs, gravity_vec, 3)
            joint_pos_obs = _roll_append(joint_pos_obs, torch.cat([leg_pos_reordered, qj_rel[:,12:]], dim=-1), 18)
            joint_vel_obs = _roll_append(joint_vel_obs, torch.cat([leg_vel_reordered, dqj[:,12:]], dim=-1), 18)

            # ── G key → start grasp ─────────────────────────────
            if g_count[0] > 0 and grab_phase == GRAB_HOME:
                grab_phase = GRAB_REACH
                grab_timer = 0
                g_count[0] = 0
                print(f"\n[GRASP] 开始抓取! 机器人位置 x={data.qpos[0]:.2f}")

            # ── Grasp or Walk ────────────────────────────────────
            if grab_phase == GRAB_HOME:
                # Walk mode: policy controls all 18 joints
                actions_obs = _roll_append(actions_obs, action, 18)
                vel_command_obs = _roll_append(vel_command_obs, vel_command, 3)
                pos_command_obs = _roll_append(pos_command_obs, pos_command, 7)

                hist_obs = torch.cat([
                    base_ang_vel_obs, projected_gravity_obs, joint_pos_obs,
                    joint_vel_obs, actions_obs, vel_command_obs, pos_command_obs,
                ], dim=-1).float().clamp(-100, 100)

                if time.time() - last_policy_time > 3.0:
                    action = policy(hist_obs).clamp(-20, 20)

                leg_act = action[:, :12][:, ISAAC_TO_MUJOCO]
                arm_act = action[:, 12:]
                action_out = torch.cat([leg_act, arm_act], dim=-1)
                target_dof_pos = (action_out * action_scale + default_angles).squeeze(0)

            else:
                # Grab mode: legs hold position, arm controlled manually
                grab_timer += 1

                if grab_phase == GRAB_REACH:
                    alpha = min(grab_timer / 120.0, 1.0)
                    arm_angles = lerp(HOME, REACH, alpha)
                    if grab_timer >= 120:
                        grab_phase = GRAB_DESCEND
                        grab_timer = 0

                elif grab_phase == GRAB_DESCEND:
                    alpha = min(grab_timer / 100.0, 1.0)
                    arm_angles = lerp(REACH, GRASP, alpha)
                    if grab_timer >= 100:
                        grab_phase = GRAB_CLOSE
                        grab_timer = 0
                        print("[GRASP] 闭合夹爪")

                elif grab_phase == GRAB_CLOSE:
                    arm_angles = GRASP.copy()
                    # Simulate gripper: bring joint7/joint8 together
                    data.ctrl[16] = 0.0   # joint7 close
                    data.ctrl[17] = -0.05 # joint8 close
                    if grab_timer >= 60:
                        grab_phase = GRAB_LIFT
                        grab_timer = 0
                        print("[GRASP] 抬起手臂")

                elif grab_phase == GRAB_LIFT:
                    alpha = min(grab_timer / 150.0, 1.0)
                    arm_angles = lerp(GRASP, HOME, alpha)
                    if grab_timer >= 150:
                        grab_phase = GRAB_DONE
                        print("[GRASP] 完成!")
                        grab_phase = GRAB_HOME

                # Keep legs at current position
                current_legs = torch.tensor(data.qpos[7:7+12])
                leg_target = current_legs.clone()

                target_dof_pos = torch.cat([
                    leg_target,
                    torch.tensor(arm_angles, dtype=torch.float32)
                ])

                # Track action for observation history (zeros = no policy action)
                actions_obs = _roll_append(actions_obs, torch.zeros(1,18), 18)
                vel_command_obs = _roll_append(vel_command_obs, torch.zeros(1,3), 3)
                pos_command_obs = _roll_append(pos_command_obs, torch.zeros(1,7), 7)

            # ── Status display ────────────────────────────────
            if step_counter % 200 == 0:
                rx, ry, rz = data.qpos[0], data.qpos[1], data.qpos[2]
                dist = np.sqrt((rx-2.0)**2 + (ry+10)**2)
                phase_names = {0:"WALK", 1:"REACH", 2:"DESCEND", 3:"GRASP", 4:"LIFT", 5:"DONE"}
                pn = phase_names.get(grab_phase, "?")
                print(f"\r  x={rx:.2f} y={ry:.2f}  | cube dist={dist:.2f}m | {pn}   ", end="")

            viewer.sync()
            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)

    kb.close()


if __name__ == "__main__":
    main()
