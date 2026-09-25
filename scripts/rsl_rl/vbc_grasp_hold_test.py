# Copyright (c) 2025-2026, Junjie Zhu.
#
# SPDX-License-Identifier: Apache-2.0
"""Scripted grasp *hold* feasibility test with a globally-solved arm IK.

The arm is driven by a fixed joint configuration obtained from a multi-start
analytical IK (so the TCP actually reaches the target, unlike a local IK seeded
from the standing pose).  Phases: above -> grasp -> close -> lift.  If the object
is not held even with a correct, converged arm pose, the blocker is physics /
assets, not the RL method or the IK.
"""

from __future__ import annotations

import argparse
import os
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Scripted VBC grasp-hold feasibility test.")
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--table_top_z", type=float, default=0.51)
parser.add_argument("--above", type=float, default=0.10)
parser.add_argument("--lift", type=float, default=0.15)
parser.add_argument("--reach", type=float, default=0.45, help="teleport object to this reach distance")
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import piper_kinematics as pk  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnvCfg  # noqa: E402
from isaaclab.utils.math import quat_apply_inverse  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
from isaaclab_tasks.utils.hydra import hydra_task_config  # noqa: E402

import LeggedManip_Lab.tasks  # noqa: F401,E402

STEPS = {"above": 50, "grasp": 50, "close": 40, "lift": 150}


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    env_cfg.actions.vbc_command.arm_ik_enabled = False

    env = gym.make(args_cli.task, cfg=env_cfg)
    uenv = env.unwrapped
    term = uenv.action_manager.get_term("vbc_command")
    robot = uenv.scene["robot"]
    cube = uenv.scene["cube"]
    link0_id = robot.find_bodies("link0")[0][0]
    ee_id = robot.find_bodies("end_effector")[0][0]
    arm_ids, _ = robot.find_joints(
        ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], preserve_order=True)
    grip_ids, _ = robot.find_joints(["joint7", "joint8"], preserve_order=True)
    sensor = uenv.scene.sensors.get("contact_forces", None)
    fs_ids = None
    if sensor is not None and hasattr(sensor, "body_names"):
        fs_ids = [i for i, n in enumerate(sensor.body_names) if ("link7" in n or "link8" in n)] or None

    N = term.num_envs
    dev = term.device
    open_g = torch.tensor([0.04, -0.04], device=dev).repeat(N, 1)
    close_g = torch.tensor([0.002, -0.002], device=dev).repeat(N, 1)

    orig_apply = term.apply_actions
    term.process_actions = lambda actions: None
    zeros = torch.zeros(N, term.action_dim, device=dev)
    env.reset()

    rng = np.random.default_rng(3)
    SEEDS = [np.zeros(6)] + [rng.uniform(pk.JOINT_LIMITS[:, 0], pk.JOINT_LIMITS[:, 1])
                             for _ in range(24)]

    def solve(target_rel: np.ndarray) -> tuple[torch.Tensor, np.ndarray]:
        q = np.empty((N, 6), np.float64)
        res = np.zeros(N, np.float64)
        for i in range(N):
            best_q, best_r = np.zeros(6), 1.0e9
            for seed in SEEDS:
                qs, r = pk.solve_piper_position_ik(target_rel[i].astype(np.float64), seed,
                                                   max_iterations=120)
                if float(r) < best_r:
                    best_r, best_q = float(r), qs
            q[i] = best_q
            res[i] = best_r
        return torch.tensor(q, dtype=torch.float32, device=dev), res

    link0_pos_w = robot.data.body_pos_w[:, link0_id]
    link0_quat_w = robot.data.body_quat_w[:, link0_id]
    # Teleport the object in front of the arm base at a comfortable reach so the
    # test isolates grab/lift physics from the reach problem.
    R = quat_apply_inverse(link0_quat_w, torch.tensor([1.0, 0.0, 0.0], device=dev).expand(N, 3))
    obj_now = cube.data.root_pos_w.clone()
    obj_new = obj_now.clone()
    obj_new[:, :2] = link0_pos_w[:, :2] + args_cli.reach * R[:, :2]
    cube.write_root_pose_to_sim(torch.cat([obj_new, cube.data.root_quat_w], dim=-1))
    cube.write_root_velocity_to_sim(torch.zeros(N, 6, device=dev))
    obj_rel = quat_apply_inverse(link0_quat_w, obj_new - link0_pos_w).cpu().numpy()
    up = np.array([0.0, 0.0, 1.0])
    q_above, r_above = solve(obj_rel + args_cli.above * up)
    q_grasp, r_grasp = solve(obj_rel)
    q_lift, r_lift = solve(obj_rel + args_cli.lift * up)
    print(f"[hold-test] |obj_rel_link0|={np.linalg.norm(obj_rel, axis=1).mean():.3f}m  "
          f"IK residual: above={r_above.mean()*1000:.1f}mm grasp={r_grasp.mean()*1000:.1f}mm "
          f"lift={r_lift.mean()*1000:.1f}mm", flush=True)

    TARGET = {"above": q_above, "grasp": q_grasp, "close": q_grasp, "lift": q_lift}
    state = {"phase": "above"}

    def patched_apply():
        orig_apply()
        ph = state["phase"]
        robot.set_joint_position_target(TARGET[ph], arm_ids)
        robot.set_joint_position_target(close_g if ph in ("close", "lift") else open_g, grip_ids)

    term.apply_actions = patched_apply

    for phase, ln in STEPS.items():
        state["phase"] = phase
        rows = []
        for _ in range(ln):
            env.step(zeros)
            obj_h = float((cube.data.root_pos_w[:, 2] - args_cli.table_top_z).mean())
            dz = float((robot.data.body_pos_w[:, ee_id, 2] - cube.data.root_pos_w[:, 2]).mean())
            ff = 0.0
            if sensor is not None and fs_ids is not None:
                ff = float(sensor.data.net_forces_w[:, fs_ids, :].norm(dim=-1).sum(dim=-1).mean())
            fe = float(robot.data.applied_torque[:, grip_ids].abs().mean())
            je = float((robot.data.joint_pos[:, arm_ids] - TARGET[phase]).abs().max(dim=-1).values.mean())
            rows.append((obj_h, dz, ff, fe, je))
        a = np.array(rows)
        print(f"[hold-test] {phase:6s} obj_lift={a[:,0].mean()*1000:6.1f}mm (max {a[:,0].max()*1000:6.1f})  "
              f"ee_z-obj_z={a[:,1].mean()*1000:6.1f}mm  finger_force={a[:,2].mean():.2f}N  "
              f"finger_effort={a[:,3].mean():.2f}N  arm_jnt_err={a[:,4].mean():.3f}rad", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
