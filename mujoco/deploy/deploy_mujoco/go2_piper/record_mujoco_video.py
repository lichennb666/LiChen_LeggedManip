#!/usr/bin/env python3
"""Record MuJoCo video of Go2+Piper policy execution (headless, no window needed)."""
import sys, os, yaml, subprocess

sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
import torch
import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def load_cfg(path):
    with open(path) as f:
        return yaml.safe_load(f)

def _roll_append(buf, new, chunk):
    return torch.cat([buf, new], dim=-1)[:, chunk:]

def record(policy_path, xml_path, cfg_path, output, steps=600, fps=30, height_cmds=None):
    """Run policy headless, save MP4 video."""
    cfg = load_cfg(cfg_path)
    xml = xml_path.replace("{CURRENT_ROOT_DIR}", ROOT) if "{CURRENT_ROOT_DIR}" in xml_path else xml_path
    
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, 480, 640)
    
    policy = torch.jit.load(policy_path)
    policy.eval()
    
    num_hist = cfg["num_hist"]
    kps = cfg["kps"]
    kds = cfg["kds"]
    action_scale = cfg["action_scale"]
    default_angles = np.array(cfg["default_angles"])
    decimation = cfg["control_decimation"]
    
    use_height = "stairs" in policy_path
    
    # Buffers
    base_ang_vel_obs = torch.zeros(1, 3 * num_hist)
    joint_pos_obs = torch.zeros(1, 18 * num_hist)
    joint_vel_obs = torch.zeros(1, 18 * num_hist)
    actions_obs = torch.zeros(1, 18 * num_hist)
    projected_gravity_obs = torch.zeros(1, 3 * num_hist)
    vel_command_obs = torch.zeros(1, 3 * num_hist)
    pos_command_obs = torch.zeros(1, 7 * num_hist)
    height_scan_obs = torch.zeros(1, 187 * num_hist)
    
    data.qpos[7:] = default_angles
    
    vel_cmd = torch.tensor([[0.3, 0.0, 0.0]])
    pos_cmd = torch.tensor([[0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])
    
    frames = []
    
    for step in range(steps):
        for _ in range(decimation):
            mujoco.mj_step(model, data)
        
        qj = torch.tensor(data.qpos[7:].copy()).unsqueeze(0)
        dqj = torch.tensor(data.qvel[6:].copy()).unsqueeze(0)
        omega = torch.tensor(data.qvel[3:6].copy()).unsqueeze(0) * 0.2
        
        quat = data.qpos[3:7].copy()
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        gravity_vec = -torch.tensor([R[6], R[7], R[8]]).unsqueeze(0)
        
        leg_idx = [1, 5, 10, 0, 4, 9, 3, 7, 12, 2, 6, 11]
        leg_pos = qj[:, leg_idx]
        leg_vel = dqj[:, leg_idx]
        qj_rel = qj - torch.tensor(default_angles).unsqueeze(0)
        
        base_ang_vel_obs = _roll_append(base_ang_vel_obs, omega, 3)
        projected_gravity_obs = _roll_append(projected_gravity_obs, gravity_vec, 3)
        joint_pos_obs = _roll_append(joint_pos_obs, torch.cat([leg_pos, qj_rel[:, 12:]], dim=-1), 18)
        joint_vel_obs = _roll_append(joint_vel_obs, torch.cat([leg_vel, dqj[:, 12:]], dim=-1), 18)
        
        hist_obs_list = [base_ang_vel_obs, projected_gravity_obs, joint_pos_obs, joint_vel_obs, actions_obs, vel_command_obs, pos_command_obs]
        if use_height:
            hist_obs_list.append(height_scan_obs)
        hist_obs = torch.cat(hist_obs_list, dim=-1).float().clamp(-100, 100)
        
        action = policy(hist_obs).clip(-20, 20)
        actions_obs = _roll_append(actions_obs, action, 18)
        
        rev_leg = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]
        leg_action = action[:, rev_leg]
        arm_action = action[:, 12:]
        action_reordered = torch.cat([leg_action, arm_action], dim=-1)
        action_out = action_reordered[0].detach().numpy() * np.array(action_scale)
        
        target_q = default_angles + action_out
        for i in range(18):
            data.ctrl[i] = target_q[i]
            data.qfrc_applied[6 + i] = kps[i] * (target_q[i] - qj[0, i].item()) - kds[i] * dqj[0, i].item()
        
        if step % (decimation) == 0:
            renderer.update_scene(data, camera=-1)
            frame = renderer.render()
            frames.append(frame.copy())
    
    renderer.close()
    
    # Save as MP4 using ffmpeg
    import tempfile
    with tempfile.NamedTemporaryFile(suffix='.mp4', delete=False) as tf:
        tmpname = tf.name
    
    fourcc = 'mp4v'  # or 'avc1' for H264
    try:
        import cv2
        h, w = frames[0].shape[:2]
        writer = cv2.VideoWriter(tmpname, cv2.VideoWriter_fourcc(*fourcc), fps, (w, h))
        for f in frames:
            writer.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
        writer.release()
        os.rename(tmpname, output)
    except ImportError:
        # Fallback: save frames as raw image and use ffmpeg
        frame_dir = output.replace('.mp4', '_frames')
        os.makedirs(frame_dir, exist_ok=True)
        for i, f in enumerate(frames):
            import cv2
            cv2.imwrite(f"{frame_dir}/frame_{i:04d}.png", cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
        subprocess.run([
            'ffmpeg', '-y', '-framerate', str(fps), '-i', f"{frame_dir}/frame_%04d.png",
            '-c:v', 'libx264', '-preset', 'fast', '-crf', '23', output
        ], check=True)
        import shutil
        shutil.rmtree(frame_dir)
    
    print(f"Video saved: {output} ({len(frames)} frames, {fps} fps)")

if __name__ == "__main__":
    # Default paths
    flat_policy = os.path.join(ROOT, "deploy/policy/go2_piper/policy.pt")
    stairs_policy = os.path.join(ROOT, "deploy/policy/go2_piper/policy_stairs.pt")
    cfg = os.path.join(ROOT, "deploy/deploy_mujoco/go2_piper/config.yaml")
    xml = os.path.join(ROOT, "robots/go2_piper/scene.xml")
    
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default=flat_policy, help="Policy .pt file")
    ap.add_argument("--scene", default=xml, help="MuJoCo scene XML")
    ap.add_argument("--output", default="/tmp/go2_piper_mujoco.mp4", help="Output video")
    ap.add_argument("--steps", type=int, default=600)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--flat", action="store_true", help="Use flat policy")
    ap.add_argument("--stairs", action="store_true", help="Use stairs policy")
    args = ap.parse_args()
    
    if args.stairs:
        args.policy = stairs_policy
    elif args.flat:
        args.policy = flat_policy
    
    print(f"Policy: {args.policy}")
    print(f"Scene: {args.scene}")
    record(args.policy, args.scene, cfg, args.output, steps=args.steps, fps=args.fps)
