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

"""Diagnose a VBC teacher checkpoint: is the arm failing to track the commanded
end-effector pose, or is the grasp slipping during the lift?

It rolls out the policy and prints, per logging window:
  - mean EE position tracking error (actual vs commanded, in world)
  - mean commanded EE z and mean actual EE z
  - mean object height above the table and the max reached
  - fraction of envs with the object lifted > 10 cm (grasp success height)
  - fraction of envs with the gripper commanded closed
"""

from __future__ import annotations

import argparse
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Diagnose a VBC teacher checkpoint.")
parser.add_argument("--task", type=str, required=True, help="Task name.")
parser.add_argument("--num_envs", type=int, default=64, help="Number of environments.")
parser.add_argument("--steps", type=int, default=600, help="Number of policy steps to roll out.")
parser.add_argument("--print_every", type=int, default=100, help="Print every N steps.")
parser.add_argument("--table_top_z", type=float, default=0.51, help="Table height (m).")
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import importlib.metadata as metadata  # noqa: E402

from packaging import version  # noqa: E402

installed_version = metadata.version("rsl-rl-lib")

import os  # noqa: E402

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.envs import ManagerBasedRLEnvCfg  # noqa: E402
from isaaclab.utils.assets import retrieve_file_path  # noqa: E402
from isaaclab.utils.math import quat_apply  # noqa: E402
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlBaseRunnerCfg,
    RslRlVecEnvWrapper,
    handle_deprecated_rsl_rl_cfg,
)

import isaaclab_tasks  # noqa: F401,E402
from isaaclab_tasks.utils.hydra import hydra_task_config  # noqa: E402

import LeggedManip_Lab.tasks  # noqa: F401,E402


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)
    env_cfg.seed = agent_cfg.seed
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    resume_path = retrieve_file_path(args_cli.checkpoint)
    env_cfg.log_dir = os.path.dirname(resume_path)

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    uenv = env.unwrapped
    term = uenv.action_manager.get_term("vbc_command")
    robot = uenv.scene["robot"]
    cube = uenv.scene["cube"]
    print("[diag] robot.joint_names:", list(robot.joint_names), flush=True)
    print("[diag] low_level joint ids:", getattr(term._low_level_action_term, "_joint_ids", None), flush=True)
    print("[diag] arm_ik_term:", getattr(term, "_arm_ik_term", None), flush=True)
    if getattr(term, "_arm_ik_term", None) is not None:
        print("[diag] ik joint ids:", term._arm_ik_term._joint_ids, flush=True)
        print("[diag] ik body idx:", getattr(term._arm_ik_term, "_body_idx", None), flush=True)
    link0_ids, _ = robot.find_bodies("link0")
    ee_ids, _ = robot.find_bodies("end_effector")
    link0_id, ee_id = link0_ids[0], ee_ids[0]
    finger_ids, _ = robot.find_joints(["joint7", "joint8"], preserve_order=True)

    obs = env.get_observations()
    rows = []
    with torch.inference_mode():
        for step in range(1, args_cli.steps + 1):
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy.reset(dones)

            cmd = term.pose_command  # (N,7): xy link0 frame, z world, quat link0
            base_pos = robot.data.body_pos_w[:, link0_id]
            base_quat = robot.data.body_quat_w[:, link0_id]
            cmd_pos_b = torch.zeros_like(cmd[:, :3])
            cmd_pos_b[:, :2] = cmd[:, :2]
            cmd_w = base_pos + quat_apply(base_quat, cmd_pos_b)
            cmd_w[:, 2] = cmd[:, 2]
            ee_w = robot.data.body_pos_w[:, ee_id]
            pos_err = torch.linalg.norm(ee_w - cmd_w, dim=-1)
            lift = cube.data.root_pos_w[:, 2] - args_cli.table_top_z
            grip = term.gripper_command

            rows.append((
                float(pos_err.mean()), float(cmd_w[:, 2].mean()), float(ee_w[:, 2].mean()),
                float(lift.mean()), float(lift.max()),
                float((lift > 0.10).float().mean()), float((grip > 0.5).float().mean()),
                float(robot.data.joint_pos[:, finger_ids[0]].mean()),
                float(robot.data.applied_torque[:, finger_ids].abs().mean()),
                float(torch.linalg.norm(ee_w - cube.data.root_pos_w, dim=-1).mean()),
            ))
            if step % args_cli.print_every == 0 or step == args_cli.steps:
                mean = torch.tensor(rows[-args_cli.print_every:], dtype=torch.float32).mean(dim=0)
                print(
                    f"[diag] step {step:4d} | EE pos_err={mean[0]:.4f} m | "
                    f"cmd_z={mean[1]:.3f} ee_z={mean[2]:.3f} | "
                    f"lift_mean={mean[3]:.4f} (max {mean[4]:.3f}) | "
                    f"lift>10cm={mean[5]*100:.1f}% | grip_closed={mean[6]*100:.1f}% | "
                    f"finger_pos={mean[7]:.4f} finger_effort={mean[8]:.2f}N | "
                    f"ee_obj_dist={mean[9]:.3f} m",
                    flush=True,
                )
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
