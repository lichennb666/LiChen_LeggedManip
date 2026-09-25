#!/usr/bin/env python3
"""
Go2+Piper 抓取演示: 走到桌子前 → 机械臂抓取方块.

按键:
  WASD  移动 (Flat 策略)
  G     触发抓取 (停在当前位置, 臂伸向桌子)
  R     停止
  ESC   退出
"""

import sys, os, yaml, time
import numpy as np
import threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
import mujoco
import torch
from pathlib import Path


def load_cfg(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _roll_append(buf, new, chunk):
    return torch.cat([buf, new], dim=-1)[:, chunk:]


def get_gravity_orientation(quat: torch.Tensor) -> torch.Tensor:
    qw, qx, qy, qz = quat
    return torch.tensor([
        2*(qx*qz - qw*qy),
        2*(qy*qz + qw*qx),
        1 - 2*(qx*qx + qy*qy),
    ])


# Pre-computed grasp poses (arm joint angles: joint1..joint6)
# "home": arm folded (resting)
# "reach": arm reaching forward toward table
HOME_ANGLES = [0.0, 0.3, -1.5, 0.0, -0.5, 0.0]    # rad
REACH_ANGLES = [0.0, 0.6, -2.0, 0.8, 0.2, 0.0]      # rad (reaching forward-down)
GRASP_ANGLES = [0.0, 0.7, -2.2, 0.9, 0.1, 0.0]      # rad (closer to cube)


def main():
    if len(sys.argv) >= 2:
        config_path = sys.argv[1]
    else:
        config_path = os.path.join(os.path.dirname(__file__), "config_table.yaml")
    
    root = str(Path(__file__).resolve().parent.parent.parent.parent)
    cfg = load_cfg(config_path)
    
    # Load policy
    policy = torch.jit.load(root + "/deploy/policy/go2_piper/policy_flat.pt")
    policy.eval()
    
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

    ISAAC_TO_MUJOCO = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
    
    num_env = 1
    num_hist = cfg["num_hist"]
    
    # Observation buffers
    base_ang_vel_obs = torch.zeros(num_env, 3*num_hist)
    joint_pos_obs = torch.zeros(num_env, 18*num_hist)
    joint_vel_obs = torch.zeros(num_env, 18*num_hist)
    actions_obs = torch.zeros(num_env, 18*num_hist)
    projected_gravity_obs = torch.zeros(num_env, 3*num_hist)
    vel_command_obs = torch.zeros(num_env, 3*num_hist)
    pos_command_obs = torch.zeros(num_env, 7*num_hist)
    
    action = torch.zeros(num_env, 18)
    data.qpos[7:] = default_angles
    
    # State
    mode = "walk"  # walk / grab
    grab_phase = 0  # 0=reach, 1=grasp, 2=lift, 3=done
    grab_timer = 0
    leg_target = default_angles[:12].copy()
    arm_target = default_angles[12:].copy()
    
    # Keyboard listener
    from keyboard_controller import KeyboardController
    kb = KeyboardController()
    cmd = kb.get_command()
    
    # Additional key states
    g_pressed = [False]
    from pynput import keyboard as pynput_kb
    def on_press(key):
        try:
            if hasattr(key, 'char') and key.char == 'g':
                g_pressed[0] = True
        except: pass
    kb_listener = pynput_kb.Listener(on_press=on_press)
    kb_listener.start()
    
    print("[INFO] WASD=walk  G=grasp  ESC=exit    TARGET=CUBE at (2.0, -10, 0.36)")
    print("[INFO] Walk forward to the table (X≈1.4), press G to grab the cube")
    print(f"[INFO] Cube world pos: (2.0, -10, 0.36)  Table top: z=0.32m")
    
    # Viewer
    try:
        from mujoco import viewer
        v = viewer.launch_passive(model, data)
        # Track camera
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if body_id != -1:
            v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            v.cam.trackbodyid = body_id
            v.cam.distance = 3.0
            v.cam.elevation = -20
            v.cam.azimuth = 90
    except:
        v = None
        print("[INFO] Headless mode")
    
    step_counter = 0
    last_policy_time = time.time()
    joint_vel_scale = 0.05
    base_ang_vel_scale = 0.2
    
    target_dof_pos = default_angles.copy()
    kps_t = torch.tensor(kps).float()
    kds_t = torch.tensor(kds).float()
    
    while v is None or v.is_running():
        step_start = time.time()
        
        # ── PD control EVERY simulation step (like original) ──────
        tau = kps_t * (torch.tensor(target_dof_pos).float() - torch.tensor(data.qpos[7:]).float()) \
            + kds_t * (torch.zeros_like(kds_t) - torch.tensor(data.qvel[6:]).float())
        data.ctrl[:] = tau.detach().numpy()
        mujoco.mj_step(model, data)
        
        step_counter += 1
        if step_counter % decimation != 0:
            if v: v.sync()
            continue
        
        # ── policy runs here (every 4th step = 50Hz) ──────────
        # Get state
        qj = torch.tensor(data.qpos[7:]).unsqueeze(0)
        dqj = torch.tensor(data.qvel[6:]).unsqueeze(0) * joint_vel_scale
        quat = torch.tensor(data.qpos[3:7])
        omega = torch.tensor(data.qvel[3:6]).unsqueeze(0) * base_ang_vel_scale
        
        qj_rel = qj - torch.tensor(default_angles).unsqueeze(0)
        gravity_vec = get_gravity_orientation(quat).unsqueeze(0)
        
        leg_pos = qj_rel[:, :12][:, ISAAC_TO_MUJOCO]
        leg_vel = dqj[:, :12][:, ISAAC_TO_MUJOCO]
        
        # Commands
        cmd = kb.get_command()
        vel_cmd = cmd["velocity"]
        pos_cmd = cmd["pos"]
        vel_command = torch.tensor([[vel_cmd[0], vel_cmd[1], vel_cmd[2]]])
        pos_command = torch.tensor([[pos_cmd[0],pos_cmd[1],pos_cmd[2],pos_cmd[3],pos_cmd[4],pos_cmd[5],pos_cmd[6]]])
        
        # G key trigger
        if g_pressed[0] and mode == "walk":
            mode = "grab"
            grab_phase = 0
            grab_timer = 0
            g_pressed[0] = False
            print(f"\n[GRASP START] Robot at X={data.qpos[0]:.2f}")
        
        # Update buffers
        base_ang_vel_obs = _roll_append(base_ang_vel_obs, omega, 3)
        projected_gravity_obs = _roll_append(projected_gravity_obs, gravity_vec, 3)
        joint_pos_obs = _roll_append(joint_pos_obs, torch.cat([leg_pos, qj_rel[:,12:]], dim=-1), 18)
        joint_vel_obs = _roll_append(joint_vel_obs, torch.cat([leg_vel, dqj[:,12:]], dim=-1), 18)
        vel_command_obs = _roll_append(vel_command_obs, vel_command, 3)
        pos_command_obs = _roll_append(pos_command_obs, pos_command, 7)
        
        if mode == "walk":
            hist_obs = torch.cat([
                base_ang_vel_obs, projected_gravity_obs, joint_pos_obs,
                joint_vel_obs, actions_obs, vel_command_obs, pos_command_obs,
            ], dim=-1).float().clamp(-100, 100)
            action = policy(hist_obs).clamp(-20, 20)
            actions_obs = _roll_append(actions_obs, action, 18)
        
        elif mode == "grab":
            grab_timer += 1
            
            if grab_phase == 0:  # Move arm to reach position
                alpha = min(grab_timer / 100.0, 1.0)
                arm_target = np.array([alpha * r + (1-alpha) * h for r, h in zip(REACH_ANGLES, HOME_ANGLES)])
                if grab_timer >= 100:
                    grab_phase = 1
                    grab_timer = 0
            
            elif grab_phase == 1:  # Descend to grasp
                alpha = min(grab_timer / 80.0, 1.0)
                arm_target = np.array([alpha * g + (1-alpha) * r for g, r in zip(GRASP_ANGLES, REACH_ANGLES)])
                if grab_timer >= 80:
                    grab_phase = 2
                    grab_timer = 0
                    print("[CLOSE GRIPPER]")
            
            elif grab_phase == 2:  # Close gripper + lift
                arm_target = np.array(GRASP_ANGLES)
                if grab_timer >= 50:  # hold grip for a moment
                    grab_phase = 3
                    grab_timer = 0
            
            else:  # Done, return to home
                alpha = min(grab_timer / 100.0, 1.0)
                arm_target = np.array([alpha * h + (1-alpha) * g for h, g in zip(HOME_ANGLES, GRASP_ANGLES)])
                if grab_timer >= 200:
                    mode = "walk"
                    print("[GRASP DONE]")
            
            # Keep legs stationary, move only arm
            leg_action = torch.zeros(1, 12)
            arm_action_rad = torch.tensor(arm_target - default_angles[12:]).unsqueeze(0) / torch.tensor(action_scale[12:])
            arm_action_rad = arm_action_rad.clamp(-20, 20)
            action = torch.cat([leg_action, arm_action_rad], dim=-1)
            
            # Track action for history
            leg_pos_reorder = torch.zeros(1, 12)
            actions_track = torch.cat([leg_pos_reorder, arm_action_rad], dim=-1)
            actions_obs = _roll_append(actions_obs, actions_track, 18)
        
        # Apply action: remap legs Isaac → MuJoCo
        leg_act = action[:, :12][:, ISAAC_TO_MUJOCO]
        arm_act = action[:, 12:]
        action_out = torch.cat([leg_act, arm_act], dim=-1)
        action_out_np = action_out[0].detach().numpy() * np.array(action_scale)
        target_dof_pos = default_angles + action_out_np
        
        if v:
            v.sync()
            if step_counter % 50 == 0:
                rx, ry, rz = data.qpos[0], data.qpos[1], data.qpos[2]
                dist = np.sqrt((rx-2.0)**2 + (ry+10)**2)
                mode_str = f"GRAB-{['REACH','DESCEND','GRASP','LIFT'][min(grab_phase,3)]}" if mode=="grab" else "WALK"
                print(f"\r  Robot: x={rx:.2f} y={ry:.2f} z={rz:.2f} | dist to cube={dist:.2f}m | {mode_str}   ", end="")
    
    kb_listener.stop()
    print("Done.")


if __name__ == "__main__":
    main()
