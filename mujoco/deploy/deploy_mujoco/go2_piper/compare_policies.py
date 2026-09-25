#!/usr/bin/env python3
"""Compare flat vs stairs policy joint trajectories on flat ground in MuJoCo."""
import sys, os, yaml, time, numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import torch

def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)

def _roll_append(buf, new, chunk):
    return torch.cat([buf, new], dim=-1)[:, chunk:]

def run_policy(policy_path, config_path, label, steps=200, render=False, mujoco_root=None):
    """Run one policy on flat ground and return joint trajectory."""
    import mujoco
    cfg = load_yaml(config_path)
    xml_path = cfg["xml_path"].replace("{CURRENT_ROOT_DIR}", mujoco_root)
    # Force flat scene for fair comparison
    xml_path = xml_path.replace("scene_stairs.xml", "scene.xml")
    
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    renderer = None
    if render:
        renderer = mujoco.Renderer(model, 480, 640)
    
    policy = torch.jit.load(policy_path)
    policy.eval()
    
    # Config
    num_env = cfg["num_env"]
    num_hist = cfg["num_hist"]
    kps = cfg["kps"]
    kds = cfg["kds"]
    action_scale = cfg["action_scale"]
    default_angles = cfg["default_angles"]
    num_actions = cfg["num_actions"]
    dt = cfg["simulation_dt"]
    decimation = cfg["control_decimation"]
    
    # Detect policy input dimension by checking policy file name
    use_height_scan = "stairs" in policy_path.lower() or "stairs" in config_path.lower()
    print(f"    {'With height_scan' if use_height_scan else 'Flat policy'} ({policy_path.split('/')[-1]})")
    
    # History buffers
    base_ang_vel_obs = torch.zeros(num_env, 3 * num_hist)
    joint_pos_obs = torch.zeros(num_env, 18 * num_hist)
    joint_vel_obs = torch.zeros(num_env, 18 * num_hist)
    actions_obs = torch.zeros(num_env, 18 * num_hist)
    projected_gravity_obs = torch.zeros(num_env, 3 * num_hist)
    vel_command_obs = torch.zeros(num_env, 3 * num_hist)
    pos_command_obs = torch.zeros(num_env, 7 * num_hist)
    height_scan_obs = torch.zeros(num_env, 187 * num_hist)
    
    # Set robot to default pose
    q0 = np.array(default_angles)
    data.qpos[7:] = q0
    
    # Log arrays
    joint_pos_history = []
    base_height_history = []
    base_ang_history = []
    action_history = []
    
    # Command: walk forward slowly
    vel_command = torch.tensor([[0.3, 0.0, 0.0]])
    pos_command = torch.tensor([[0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])
    
    for step in range(steps):
        for _ in range(decimation):
            mujoco.mj_step(model, data)
        
        # Get state
        qj = torch.tensor(data.qpos[7:].copy()).unsqueeze(0)  # 18D
        dqj = torch.tensor(data.qvel[6:].copy()).unsqueeze(0)
        omega = torch.tensor(data.qvel[3:6].copy()).unsqueeze(0) * 0.2
        
        # Gravity projection
        quat = data.qpos[3:7].copy()
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        gravity_vec = -torch.tensor([R[6], R[7], R[8]]).unsqueeze(0)  # R.T * [0,0,-1]
        
        # Reorder leg joints (FL<->FR swap per deployment convention)
        leg_idx = [1, 5, 10, 0, 4, 9, 3, 7, 12, 2, 6, 11]
        leg_pos = qj[:, leg_idx]
        leg_vel = dqj[:, leg_idx]
        
        # Default joint offset
        default_angles_t = torch.tensor(default_angles).unsqueeze(0)
        qj_rel = qj - default_angles_t
        
        # Update history buffers
        base_ang_vel_obs = _roll_append(base_ang_vel_obs, omega, 3)
        projected_gravity_obs = _roll_append(projected_gravity_obs, gravity_vec, 3)
        joint_pos_obs = _roll_append(joint_pos_obs, torch.cat([leg_pos, qj_rel[:, 12:]], dim=-1), 18)
        joint_vel_obs = _roll_append(joint_vel_obs, torch.cat([leg_vel, dqj[:, 12:]], dim=-1), 18)
        
        hist_obs_list = [
            base_ang_vel_obs, projected_gravity_obs, joint_pos_obs,
            joint_vel_obs, actions_obs, vel_command_obs, pos_command_obs,
        ]
        if use_height_scan:
            hist_obs_list.append(height_scan_obs)
        hist_obs = torch.cat(hist_obs_list, dim=-1).float().clamp(-100.0, 100.0)
        
        # Policy inference
        action = policy(hist_obs).clip(-20.0, 20.0)
        actions_obs = _roll_append(actions_obs, action, 18)
        
        # Reorder back
        rev_leg = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
        leg_action = action[:, rev_leg]
        arm_action = action[:, 12:]
        action_reordered = torch.cat([leg_action, arm_action], dim=-1)
        action_out = action_reordered[0].detach().numpy() * np.array(action_scale)
        
        # Apply action
        target_q = q0 + action_out
        for i in range(18):
            actuator_id = i
            data.ctrl[actuator_id] = target_q[i]
            data.qfrc_applied[6 + i] = kps[i] * (target_q[i] - qj[0, i].item()) - kds[i] * dqj[0, i].item()
        
        joint_pos_history.append(qj[0].numpy().copy())
        base_height_history.append(data.qpos[2])
        base_ang_history.append(data.qpos[3:7].copy())
        action_history.append(action_out.copy())
        
        if renderer is not None and step % 10 == 0:
            renderer.update_scene(data, camera="track")
            pixels = renderer.render()
    
    return {
        'joint_pos': np.array(joint_pos_history),
        'base_height': np.array(base_height_history),
        'base_quat': np.array(base_ang_history),
        'actions': np.array(action_history),
        'label': label,
    }

if __name__ == "__main__":
    root = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(os.path.dirname(os.path.dirname(root)))  # go up to mujoco/
    
    flat_policy = os.path.join(root, "deploy/policy/go2_piper/policy.pt")
    stairs_policy = os.path.join(root, "deploy/policy/go2_piper/policy_stairs.pt")
    cfg = os.path.join(root, "deploy/deploy_mujoco/go2_piper/config.yaml")
    
    print("=" * 60)
    print("Running Flat policy on flat ground...")
    flat_data = run_policy(flat_policy, cfg, "Flat", steps=200, mujoco_root=root)
    
    print("Running Stairs policy on flat ground...")
    stairs_data = run_policy(stairs_policy, cfg, "Stairs", steps=200, mujoco_root=root)
    
    # Analysis
    print("\n" + "=" * 60)
    print("                POLICY COMPARISON ON FLAT GROUND")
    print("=" * 60)
    
    fj = flat_data['joint_pos']
    sj = stairs_data['joint_pos']
    fa = flat_data['actions']
    sa = stairs_data['actions']
    
    # 1. Joint positions (final state)
    print("\n--- Final Joint Positions (rad) ---")
    joint_names = ['FL_hip','FL_thi','FL_cal','FR_hip','FR_thi','FR_cal',
                   'RL_hip','RL_thi','RL_cal','RR_hip','RR_thi','RR_cal',
                   'j1','j2','j3','j4','j5','j6']
    for i in range(18):
        print(f"  {joint_names[i]:8s}: flat={fj[-1,i]:+.3f}  stairs={sj[-1,i]:+.3f}  Δ={sj[-1,i]-fj[-1,i]:+.3f}")
    
    # 2. Action magnitudes
    print(f"\n--- Action Magnitude (mean |a|) ---")
    print(f"  Flat:   {np.abs(fa).mean():.4f}")
    print(f"  Stairs: {np.abs(sa).mean():.4f}")
    
    # 3. Base height stability
    fb = flat_data['base_height']
    sb = stairs_data['base_height']
    print(f"\n--- Base Height (m) ---")
    print(f"  Flat:   mean={fb.mean():.3f}  std={fb.std():.4f}  min={fb.min():.3f}")
    print(f"  Stairs: mean={sb.mean():.3f}  std={sb.std():.4f}  min={sb.min():.3f}")
    
    # 4. Early collapse check
    print(f"\n--- Collapse Detection ---")
    flat_collapse_step = np.argmax(fb < 0.1) if any(fb < 0.1) else -1
    stairs_collapse_step = np.argmax(sb < 0.1) if any(sb < 0.1) else -1
    print(f"  Flat collapse at step:   {flat_collapse_step if flat_collapse_step >= 0 else 'None (stable)'}")
    print(f"  Stairs collapse at step: {stairs_collapse_step if stairs_collapse_step >= 0 else 'None (stable)'}")
    
    if stairs_collapse_step >= 0 and flat_collapse_step < 0:
        print(f"\n  ❌ Stairs policy collapses at step {stairs_collapse_step}, Flat stays stable")
        print(f"  Stairs joint angles at step {stairs_collapse_step-1}:")
        for i in range(18):
            print(f"    {joint_names[i]:8s}: {sj[stairs_collapse_step-1, i]:+.3f}")
    elif stairs_collapse_step < 0 and flat_collapse_step >= 0:
        print(f"  ⚠️ Flat policy collapses at step {flat_collapse_step}, Stairs stays stable")
    else:
        print(f"  Both policies {'stable' if stairs_collapse_step < 0 else 'collapse'}")
