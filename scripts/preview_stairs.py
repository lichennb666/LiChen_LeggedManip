"""Preview stairs terrain by running a single environment with zero actions."""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Preview stairs terrain")
parser.add_argument("--video", action="store_true", default=True, help="Record video")
parser.add_argument("--video_length", type=int, default=300, help="Video length (steps)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import gymnasium as gym
import LeggedManip_Lab.tasks  # noqa: F401

env = gym.make(
    "GO2-PIPER-Stairs-Play",
    render_interval=4,
)

if args_cli.video:
    from isaaclab.utils.camera import CameraRecorder

    env.reset()
    env.unwrapped.sim.render()
    recorder = CameraRecorder(
        env=env.unwrapped,
        output_dir="logs/stairs_preview",
        camera_name="camera",
        num_frames=args_cli.video_length,
        fps=25,
        disable_logger=True,
    )
    recorder.start_recording()

obs, _ = env.reset()
print(f"Observation shape: {obs['policy'].shape}")
print(f"Height scan values: min={obs['policy'][:,-160:].min():.3f} max={obs['policy'][:,-160:].max():.3f}")

for step in range(args_cli.video_length):
    actions = torch.zeros(env.unwrapped.num_envs, env.unwrapped.action_manager.total_action_dim, device="cuda:0")
    obs, _, terminated, truncated, _ = env.step(actions)

if args_cli.video:
    recorder.stop_recording()
    print(f"Video saved to: logs/stairs_preview/")

env.close()
simulation_app.close()
