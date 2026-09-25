# Copyright (c) 2025-2026, Junjie Zhu.
#
# SPDX-License-Identifier: Apache-2.0
"""Scripted grasp feasibility test for the VBC scene.

No RL policy is used: the arm is commanded by a fixed script (approach above the
object -> descend -> close gripper -> lift straight up).  This isolates whether
the physics/assets allow holding and lifting the object at all, which decides
whether any training method could succeed.

Prints, per phase, the object height above the table, the EE height and the
finger contact force / effort.
"""

from __future__ import annotations

import argparse
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Scripted VBC grasp feasibility test.")
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--table_top_z", type=float, default=0.51)
parser.add_argument("--above", type=float, default=0.10, help="approach height above object")
parser.add_argument("--lift", type=float, default=0.15, help="lift height")
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

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
    term = uenv.action_manager.get_term("vbc_command")
    robot = uenv.scene["robot"]
    cube = uenv.scene["cube"]
    link0_ids, _ = robot.find_bodies("link0")
    link0_id = link0_ids[0]
    ee_ids, _ = robot.find_bodies("end_effector")
    ee_id = ee_ids[0]
    finger_ids, _ = robot.find_joints(["joint7", "joint8"], preserve_order=True)
    sensor = uenv.scene.sensors.get("contact_forces", None)
    finger_sensor_ids = None
    if sensor is not None and hasattr(sensor, "body_names"):
        finger_sensor_ids = [i for i, n in enumerate(sensor.body_names)
                             if ("link7" in n or "link8" in n)]
        finger_sensor_ids = finger_sensor_ids or None

    # Script phases (in policy steps, 50 Hz)
    PHASES = [("approach_above", 120), ("descend", 80), ("close", 60), ("lift", 150)]
    total = sum(n for _, n in PHASES)
    state = {"step": 0, "target0": None}

    def script_command_and_gripper():
        # find phase
        acc = 0
        phase = PHASES[-1][0]
        for name, ln in PHASES:
            if state["step"] < acc + ln:
                phase = name
                break
            acc += ln
        # Lock the grasp point to the object's initial pose: following the live
        # object lets the crude script chase (and push) a moving object.
        if state["target0"] is None:
            state["target0"] = cube.data.root_pos_w.clone()
        obj_w = state["target0"]  # (N,3)
        link0_pos_w = robot.data.body_pos_w[:, link0_id]
        link0_quat_w = robot.data.body_quat_w[:, link0_id]
        target_w = obj_w.clone()
        if phase == "approach_above":
            target_w[:, 2] = obj_w[:, 2] + args_cli.above
        elif phase == "descend":
            target_w[:, 2] = obj_w[:, 2]
        elif phase == "close":
            target_w[:, 2] = obj_w[:, 2]
        else:  # lift
            target_w[:, 2] = obj_w[:, 2] + args_cli.lift
        # convert world -> mixed command (xy in link0, z world, quat identity)
        delta_b = quat_apply_inverse(link0_quat_w, target_w - link0_pos_w)
        cmd = torch.zeros(term.num_envs, 7, device=term.device)
        cmd[:, 0] = delta_b[:, 0]
        cmd[:, 1] = delta_b[:, 1]
        cmd[:, 2] = target_w[:, 2]
        cmd[:, 3] = 1.0
        term._processed_pose_command[:] = cmd
        term._ee_target_pos[:] = cmd[:, :3]
        term._ee_target_quat[:] = cmd[:, 3:7]
        closed = phase in ("close", "lift")
        term._processed_actions[:, 9] = 1.0 if closed else -1.0
        term._gripper_target[:] = (
            term._gripper_close_positions if closed else term._gripper_open_positions
        )

    # bypass the policy: the action manager calls process_actions every step
    term.process_actions = lambda actions: script_command_and_gripper()
    term.raw_actions[:] = 0.0

    zeros = torch.zeros(term.num_envs, term.action_dim, device=term.device)
    env.reset()

    import numpy as np

    # [DIAG] hard-lock the base so arm reaction / zero-command drift cannot move
    # the robot while we isolate whether the arm + gripper can grasp at all.
    lock_pose = torch.cat([robot.data.root_pos_w, robot.data.root_quat_w], dim=-1).clone()
    lock_vel = torch.zeros(term.num_envs, 6, device=term.device)

    for phase_name, ln in PHASES:
        rows = []
        for _ in range(ln):
            state["step"] += 1
            env.step(zeros)
            robot.write_root_pose_to_sim(lock_pose)
            robot.write_root_velocity_to_sim(lock_vel)
            obj_h = float((cube.data.root_pos_w[:, 2] - args_cli.table_top_z).mean())
            ee_h = float(robot.data.body_pos_w[:, ee_id, 2].mean())
            ff = 0.0
            if sensor is not None and finger_sensor_ids is not None:
                ff = float(sensor.data.net_forces_w[:, finger_sensor_ids, :].norm(dim=-1).sum(dim=-1).mean())
            fe = float(robot.data.applied_torque[:, finger_ids].abs().mean())
            # link0-frame EE error against the commanded target + IK/PD diagnostics
            tpos, _ = term._ee_command_pose_link0()
            l0p = robot.data.body_pos_w[:, link0_id]
            l0q = robot.data.body_quat_w[:, link0_id]
            ee_l0 = quat_apply_inverse(l0q, robot.data.body_pos_w[:, ee_id] - l0p)
            ee_err = float((ee_l0 - tpos).norm(dim=-1).mean())
            ik_res = float(term._last_ik_residual.mean()) if getattr(term, "_last_ik_residual", None) is not None else -1.0
            arm_err = float(
                (robot.data.joint_pos[:, term._arm_joint_ids] - term._last_ik_target).abs().max(dim=-1).values.mean()
            ) if getattr(term, "_last_ik_target", None) is not None else -1.0
            rows.append((obj_h, ee_h, ff, fe, ee_err, ik_res, arm_err))
        a = np.array(rows)
        t0, _ = term._ee_command_pose_link0()
        l0q = robot.data.body_quat_w[:, link0_id]
        obj0 = quat_apply_inverse(l0q, cube.data.root_pos_w - robot.data.body_pos_w[:, link0_id])
        q_arm = robot.data.joint_pos[0, term._arm_joint_ids]
        print(f"[grasp-test] {phase_name:14s} obj_lift={a[:,0].mean()*1000:6.1f}mm (max {a[:,0].max()*1000:6.1f})  "
              f"ee_z={a[:,1].mean():.3f}  finger_force={a[:,2].mean():.2f}N  "
              f"ee_err={a[:,4].mean()*1000:6.1f}mm  ik_res={a[:,5].mean()*1000:5.1f}mm  "
              f"arm_jnt_err={a[:,6].mean():.3f}rad", flush=True)
        print(f"[grasp-test]   env0: target_link0={np.round(t0[0].cpu().numpy(),3)} "
              f"obj_rel={np.round(obj0[0].cpu().numpy(),3)} "
              f"arm_actual={np.round(q_arm.cpu().numpy(),2)}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
