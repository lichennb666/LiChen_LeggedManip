#!/usr/bin/env python3
"""One-shot diagnostic for the VBC mask/segmented-depth observation."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg

import LeggedManip_Lab.tasks  # noqa: F401


def main() -> None:
    task = "GO2-PIPER-VBC-Student-MaskDepth"
    cfg = parse_env_cfg(task, device=args.device, num_envs=1)
    env = gym.make(task, cfg=cfg)
    obs, _ = env.reset()
    for _ in range(3):
        actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
        obs, _, _, _, _ = env.step(actions)

    images = obs["images"]
    mask_channels = images[:, [0, 1, 4, 5, 8, 9, 12, 13]]
    depth_channels = images[:, [2, 3, 6, 7, 10, 11, 14, 15]]
    print(f"MASK_CHECK shape={tuple(images.shape)}")
    print(f"MASK_CHECK mask_nonzero={int(torch.count_nonzero(mask_channels))}")
    print(f"MASK_CHECK mask_mean={float(mask_channels.mean()):.8f}")
    print(f"MASK_CHECK depth_nonzero={int(torch.count_nonzero(depth_channels))}")
    print(f"MASK_CHECK depth_max={float(depth_channels.max()):.8f}")
    if torch.count_nonzero(mask_channels) == 0:
        raise RuntimeError("[VBC-MASK-DEPTH] Target mask is empty in both cameras.")
    if torch.count_nonzero(depth_channels) == 0:
        raise RuntimeError("[VBC-MASK-DEPTH] Segmented depth is empty in both cameras.")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
