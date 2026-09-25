# Copyright (c) 2025-2026, Junjie Zhu.
#
# SPDX-License-Identifier: Apache-2.0
"""Check the torch Piper IK against the *actual* env seed and object targets.

No stepping: builds the env, reads the reset arm pose and the object position in
the link0 frame, then compares the single-seed torch IK (warm started from the
reset pose) against a multi-start reference solve.
"""

from __future__ import annotations

import argparse
import os
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Env-seeded torch IK check.")
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        "source/LeggedManip_Lab/LeggedManip_Lab/tasks/manager_based/leggedmanip_lab/mdp",
    ),
)

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import piper_kinematics as pk  # noqa: E402
from piper_ik_torch import PiperKinematicsTorch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnvCfg  # noqa: E402
from isaaclab.utils.math import quat_apply_inverse  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
from isaaclab_tasks.utils.hydra import hydra_task_config  # noqa: E402

import LeggedManip_Lab.tasks  # noqa: F401,E402


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    env = gym.make(args_cli.task, cfg=env_cfg)
    uenv = env.unwrapped
    robot = uenv.scene["robot"]
    cube = uenv.scene["cube"]
    link0_id = robot.find_bodies("link0")[0][0]
    arm_ids, _ = robot.find_joints(
        ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], preserve_order=True
    )
    N = env_cfg.scene.num_envs
    env.reset()

    link0_pos = robot.data.body_pos_w[:, link0_id]
    link0_quat = robot.data.body_quat_w[:, link0_id]
    obj_rel = quat_apply_inverse(link0_quat, cube.data.root_pos_w - link0_pos)
    seed = robot.data.joint_pos[:, arm_ids]
    print(f"[ikcheck] reset arm pose (env0) = {np.round(seed[0].tolist(),3)}", flush=True)
    print(f"[ikcheck] obj_rel_link0 (env0)  = {np.round(obj_rel[0].tolist(),3)}", flush=True)

    kin = PiperKinematicsTorch("cpu", torch.float32)
    q_sol, res = kin.solve(obj_rel.cpu(), seed.cpu(), max_iterations=10, step_clip=0.40)
    r = res.numpy()
    print(f"[ikcheck] torch IK from reset seed: mean={r.mean()*1000:.1f}mm "
          f"p50={np.percentile(r,50)*1000:.1f} p90={np.percentile(r,90)*1000:.1f} "
          f"max={r.max()*1000:.1f}  frac<5mm={(r<0.005).mean()*100:.1f}%", flush=True)

    # multi-start reference (numpy)
    rng = np.random.default_rng(0)
    seeds = [np.zeros(6)] + [rng.uniform(pk.JOINT_LIMITS[:, 0], pk.JOINT_LIMITS[:, 1]) for _ in range(24)]
    ob = obj_rel.cpu().numpy()
    ref = np.zeros(N)
    for i in range(N):
        best = 1e9
        for s in seeds:
            _, rr = pk.solve_piper_position_ik(ob[i].astype(np.float64), s, max_iterations=120)
            best = min(best, float(rr))
        ref[i] = best
    print(f"[ikcheck] multi-start reference: mean={ref.mean()*1000:.1f}mm "
          f"frac<5mm={(ref<0.005).mean()*100:.1f}%", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
