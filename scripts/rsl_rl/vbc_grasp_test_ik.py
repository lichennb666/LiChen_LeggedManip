# Copyright (c) 2025-2026, Junjie Zhu.
#
# SPDX-License-Identifier: Apache-2.0
"""Scripted grasp test with an *analytical* (converged) arm IK.

The VBC action term's own arm IK (and the frozen WBC arm output) are bypassed;
the 6 arm joints are driven by ``solve_piper_position_ik`` from the scripted
target.  This isolates whether the arm can reach precisely and hold the object,
i.e. whether the low-level is good enough for any high-level training to work.

Phases: approach above the object -> descend -> close gripper -> lift.
"""

from __future__ import annotations

import argparse
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Scripted grasp test with converged arm IK.")
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--table_top_z", type=float, default=0.51)
parser.add_argument("--above", type=float, default=0.10)
parser.add_argument("--lift", type=float, default=0.15)
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os  # noqa: E402

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


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg):
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    # disable the action term's own arm IK; we drive the arm ourselves
    env_cfg.actions.vbc_command.arm_ik_enabled = False

    env = gym.make(args_cli.task, cfg=env_cfg)
    uenv = env.unwrapped
    term = uenv.action_manager.get_term("vbc_command")
    robot = uenv.scene["robot"]
    cube = uenv.scene["cube"]
    link0_id = robot.find_bodies("link0")[0][0]
    ee_id = robot.find_bodies("end_effector")[0][0]
    arm_ids, arm_names = robot.find_joints(
        ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], preserve_order=True)
    grip_ids, _ = robot.find_joints(["joint7", "joint8"], preserve_order=True)
    sensor = uenv.scene.sensors.get("contact_forces", None)
    fs_ids = None
    if sensor is not None and hasattr(sensor, "body_names"):
        fs_ids = [i for i, n in enumerate(sensor.body_names) if ("link7" in n or "link8" in n)] or None

    PHASES = [("approach_above", 60), ("descend", 50), ("close", 40), ("lift", 120)]
    state = {"step": 0}
    N = term.num_envs
    open_g = torch.tensor([0.04, -0.04], device=term.device).repeat(N, 1)
    close_g = torch.tensor([0.002, -0.002], device=term.device).repeat(N, 1)
    # multi-start seeds for the local IK (the standing pose is far from the grasp
    # configuration, so a single seed gets stuck in a local minimum)
    _rng = np.random.default_rng(7)
    SEEDS = [np.zeros(6)] + [np.clip(_rng.uniform(pk.JOINT_LIMITS[:, 0], pk.JOINT_LIMITS[:, 1]), -2.0, 2.0)
                             for _ in range(5)]

    def current_phase():
        acc = 0
        for name, ln in PHASES:
            if state["step"] < acc + ln:
                return name
            acc += ln
        return PHASES[-1][0]

    orig_apply = term.apply_actions

    def patched_apply():
        phase = current_phase()
        obj_w = cube.data.root_pos_w
        link0_pos_w = robot.data.body_pos_w[:, link0_id]
        link0_quat_w = robot.data.body_quat_w[:, link0_id]
        target_w = obj_w.clone()
        if phase == "approach_above":
            target_w[:, 2] = obj_w[:, 2] + args_cli.above
        elif phase in ("descend", "close"):
            target_w[:, 2] = obj_w[:, 2]
        else:
            target_w[:, 2] = obj_w[:, 2] + args_cli.lift
        target_b = quat_apply_inverse(link0_quat_w, target_w - link0_pos_w)  # (N,3) link0 frame

        # legs + default gripper from the (frozen WBC) action term
        orig_apply()

        # arm: analytical IK, solved to convergence, per env
        q_cur = robot.data.joint_pos[:, arm_ids].detach().cpu().numpy()
        tb = target_b.detach().cpu().numpy()
        q_sol = np.empty_like(q_cur)
        ik_res = np.zeros(N, dtype=np.float64)
        for i in range(N):
            best_q, best_r = q_cur[i], 1.0e9
            for seed in SEEDS:
                qs, r = pk.solve_piper_position_ik(tb[i].astype(np.float64), seed, max_iterations=80)
                if float(r) < best_r:
                    best_r, best_q = float(r), qs
            q_sol[i] = best_q
            ik_res[i] = best_r
        # predicted FK of the IK solution vs requested target (residual in the model)
        fk_sol = np.array([pk.piper_tcp_position(q_sol[i]) for i in range(N)])
        model_res = np.linalg.norm(fk_sol - tb, axis=1)
        arm_t = torch.tensor(np.clip(q_sol, pk.JOINT_LIMITS[:, 0], pk.JOINT_LIMITS[:, 1]),
                             dtype=torch.float32, device=term.device)
        # sim tracking errors (for diagnostics)
        ee_actual_w = robot.data.body_pos_w[:, ee_id]
        ee_err = (ee_actual_w - target_w).norm(dim=-1)
        jnt_err = (robot.data.joint_pos[:, arm_ids] - arm_t).abs().max(dim=-1).values
        term._dbg = (float(model_res.mean()) * 1000,
                     float(np.linalg.norm(target_b.detach().cpu().numpy(), axis=1).mean()) * 1000,
                     float(ee_err.mean()) * 1000,
                     float(jnt_err.mean()))
        robot.set_joint_position_target(arm_t, arm_ids)
        # gripper
        robot.set_joint_position_target(close_g if phase in ("close", "lift") else open_g, grip_ids)
        # keep the WBC leg command consistent
        term._processed_pose_command[:, :2] = target_b[:, :2]
        term._processed_pose_command[:, 2] = target_w[:, 2]
        term._processed_pose_command[:, 3] = 1.0
        state["_last_phase"] = phase

    term.process_actions = lambda actions: None
    term.apply_actions = patched_apply

    zeros = torch.zeros(N, term.action_dim, device=term.device)
    env.reset()

    # freeze the base: zero the velocity command so the robot stands still
    try:
        names = list(uenv.command_manager._terms.keys())
        print("[grasp-test] command terms:", names, flush=True)
        for name in names:
            t = uenv.command_manager.get_term(name)
            for attr in ("vel_command_b", "vel_command_w", "heading_target"):
                if hasattr(t, attr):
                    getattr(t, attr)[:] = 0.0
        uenv.command_manager.compute = lambda dt: None
    except Exception as e:  # pragma: no cover
        print("[grasp-test] WARN could not freeze base:", e, flush=True)

    # standing geometry: where is the object relative to the arm base (link0)?
    link0_pos_w = robot.data.body_pos_w[:, link0_id]
    link0_quat_w = robot.data.body_quat_w[:, link0_id]
    rel = quat_apply_inverse(link0_quat_w, cube.data.root_pos_w - link0_pos_w)
    print(f"[grasp-test] STANDING geometry: link0_w={np.round(link0_pos_w[0].tolist(),3)} "
          f"obj_w={np.round(cube.data.root_pos_w[0].tolist(),3)} "
          f"obj_rel_link0={np.round(rel[0].tolist(),3)} |rel|={rel.norm(dim=-1).mean():.3f}m", flush=True)

    # FK vs simulation sanity check (arm at reset pose)
    q0 = robot.data.joint_pos[0, arm_ids].detach().cpu().numpy().astype(np.float64)
    fk0 = pk.piper_tcp_position(q0)
    ee0 = quat_apply_inverse(robot.data.body_quat_w[0, link0_id].unsqueeze(0),
                             (robot.data.body_pos_w[0, ee_id] - robot.data.body_pos_w[0, link0_id]).unsqueeze(0))
    print(f"[grasp-test] FK-vs-sim check: fk={np.round(fk0,4)} sim={np.round(ee0[0].detach().cpu().numpy(),4)} "
          f"diff={np.linalg.norm(fk0 - ee0[0].detach().cpu().numpy()):.4f} m", flush=True)

    for phase_name, ln in PHASES:
        rows = []
        for _ in range(ln):
            state["step"] += 1
            env.step(zeros)
            obj_h = float((cube.data.root_pos_w[:, 2] - args_cli.table_top_z).mean())
            ee_static = float((robot.data.body_pos_w[:, ee_id, 2] - cube.data.root_pos_w[:, 2]).mean())
            ff = 0.0
            if sensor is not None and fs_ids is not None:
                ff = float(sensor.data.net_forces_w[:, fs_ids, :].norm(dim=-1).sum(dim=-1).mean())
            fe = float(robot.data.applied_torque[:, grip_ids].abs().mean())
            rows.append((obj_h, ee_static, ff, fe))
        a = np.array(rows)
        dbg = getattr(term, "_dbg", (0, 0, 0, 0))
        print(f"[grasp-test] {phase_name:14s} obj_lift={a[:,0].mean()*1000:6.1f}mm (max {a[:,0].max()*1000:6.1f})  "
              f"ee_z-obj_z={a[:,1].mean()*1000:6.1f}mm  finger_force={a[:,2].mean():.2f}N  "
              f"finger_effort={a[:,3].mean():.2f}N  | ik_model_res={dbg[0]:5.1f}mm  "
              f"target_dist={dbg[1]:5.1f}mm  ee_err={dbg[2]:5.1f}mm  jnt_err={dbg[3]:.3f}rad", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
