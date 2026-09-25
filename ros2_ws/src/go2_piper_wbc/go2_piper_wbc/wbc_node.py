#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray, String
import torch

from .policy_core import (
    CONTRACT,
    JOINT_NAMES,
    ObservationHistory,
    projected_gravity,
    rotation_matrix_xyzw,
    slew_pose_command,
    world_pose_to_mixed_command,
)
from .piper_kinematics import (
    piper_tcp_position,
    piper_tcp_rotation,
    solve_piper_pose_ik,
)
from .leg_kinematics import LEG_ORDER, leg_foot_position, solve_leg_position_ik


DEFAULT_ANGLES = CONTRACT.default_angles
KPS = CONTRACT.stiffness
KDS = CONTRACT.damping
EFFORT_LIMITS = CONTRACT.effort_limits


class WbcNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_wbc")
        self.declare_parameter("policy_path", "/workspace/LeggedManip_Lab/mujoco/deploy/policy/go2_piper/wbc/policy.pt")
        self.declare_parameter("startup_delay", 3.0)
        self.declare_parameter("command_timeout", 0.2)
        # /cmd_vel adapter: navigation stacks emit step-shaped velocity goals;
        # clip them to the training command range, slew the executed command
        # toward the goal (acceleration ramp), and stop smoothly instead of
        # hard-cutting on timeout.  The policy observes the slewed command.
        self.declare_parameter("cmd_vel_max_linear", 0.3)
        self.declare_parameter("cmd_vel_max_angular", 1.0)
        self.declare_parameter("cmd_vel_lin_accel", 1.0)
        self.declare_parameter("cmd_vel_ang_accel", 2.0)
        self.declare_parameter("cmd_vel_deadband", 0.01)
        self.declare_parameter("action_scale", CONTRACT.action_scale)
        self.declare_parameter("policy_rate", CONTRACT.policy_rate_hz)
        self.declare_parameter("enable_policy", True)
        self.declare_parameter("trace_path", "")
        self.declare_parameter("trace_max_samples", 1000)
        self.declare_parameter("link0_offset_z", 0.06)
        self.declare_parameter("pose_position_slew_rate", 0.10)
        self.declare_parameter("pose_angular_slew_rate", 0.50)
        self.declare_parameter("enable_arm_ik", True)
        self.declare_parameter("arm_ik_position_slew_rate", 0.10)
        self.declare_parameter("arm_ik_joint_slew_rate", 1.0)
        self.declare_parameter("hold_idle_arm_startup_target", True)
        self.declare_parameter("freeze_arm", False)
        self.declare_parameter("freeze_arm_mode", "current")
        # Target held before the policy takes over.  Launched with the arm in
        # its policy natural pose so no zero-joint startup sweep occurs.
        self.declare_parameter("startup_target", DEFAULT_ANGLES.tolist())
        self.startup_target = np.asarray(
            self.get_parameter("startup_target").value, np.float32)
        self.policy = torch.jit.load(self.get_parameter("policy_path").value, map_location="cpu").eval()
        self.history = ObservationHistory()
        self.action = np.zeros(18, dtype=np.float32)
        self.q = None
        self.dq = None
        self.arm_ik_active = False
        self.arm_ik_position = None
        self.arm_ik_goal = None
        self.arm_ik_world_goal = None
        self.arm_ik_world_rotation_goal = None
        self.arm_ik_rotation_goal = None
        self.arm_ik_target = None
        self.freeze_arm = bool(self.get_parameter("freeze_arm").value)
        self.freeze_arm_mode = str(self.get_parameter("freeze_arm_mode").value)
        self.frozen_arm_target = None
        self.arm_ik_residual = 0.0
        self.arm_ik_orientation_residual = 0.0
        self.imu_q = np.array([0.0, 0.0, 0.0, 1.0], np.float32)
        self.imu_omega = np.zeros(3, np.float32)
        self.base_pos = np.zeros(3, np.float32)
        self.base_q = np.array([0.0, 0.0, 0.0, 1.0], np.float32)
        self.velocity_command = np.zeros(3, np.float32)
        self.velocity_command_goal = np.zeros(3, np.float32)
        # The exported WBC policy was trained around zero roll/pitch/yaw in
        # the link0 frame.  Tool-pointing orientation is not the policy's
        # neutral command and causes a large, unintended wrist transient.
        self.pose_command = np.array(
            [0.425, 0.0, 0.70, 1.0, 0.0, 0.0, 0.0], np.float32)
        self.pose_command_goal = self.pose_command.copy()
        self.last_command_time = self.get_clock().now()
        self.started_at = time.monotonic()
        self.inference_ms = 0.0
        self.action_saturation_fraction = 0.0
        self.effort_saturation_fraction = 0.0
        self.controllers_connected = False
        self.locomotion_enabled = True
        self.stance_foot_world = None
        self.stance_leg_target = None
        self.stance_ik_residual = 0.0
        self.joint_received_at = None
        self.imu_received_at = None
        self.odom_received_at = None
        self.trace_stream = None
        self.trace_samples = 0
        trace_path = str(self.get_parameter("trace_path").value)
        if trace_path:
            path = Path(trace_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.trace_stream = path.open("w", encoding="utf-8")
            self.get_logger().info(f"writing WBC trace to {path}")

        self.target_pub = self.create_publisher(
            Float64MultiArray, "/go2_piper/policy_targets", 10)
        self.status_pub = self.create_publisher(String, "/go2_piper/wbc/status", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 20)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 20)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(
            Bool, "/go2_piper/wbc/locomotion_enabled", self._locomotion_cb, 10)
        self.create_subscription(
            Bool, "/go2_piper/wbc/freeze_arm", self._freeze_arm_cb, 10)
        self.create_subscription(PoseStamped, "/go2_piper/wbc/ee_target", self._pose_cb, 10)
        self.create_timer(1.0 / float(self.get_parameter("policy_rate").value), self._update)
        self.create_timer(0.5, self._publish_status)

    def _joint_cb(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in index for name in JOINT_NAMES):
            return
        self.q = np.array([msg.position[index[name]] for name in JOINT_NAMES], np.float32)
        self.dq = np.array([
            msg.velocity[index[name]] if index[name] < len(msg.velocity) else 0.0
            for name in JOINT_NAMES
        ], np.float32)
        self.joint_received_at = self.get_clock().now()

    def _imu_cb(self, msg: Imu) -> None:
        self.imu_q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], np.float32)
        self.imu_omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], np.float32)
        self.imu_received_at = self.get_clock().now()

    def _odom_cb(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.base_pos = np.array([p.x, p.y, p.z], np.float32)
        self.base_q = np.array([q.x, q.y, q.z, q.w], np.float32)
        self.odom_received_at = self.get_clock().now()

    def _cmd_cb(self, msg: Twist) -> None:
        max_linear = float(self.get_parameter("cmd_vel_max_linear").value)
        max_angular = float(self.get_parameter("cmd_vel_max_angular").value)
        deadband = float(self.get_parameter("cmd_vel_deadband").value)
        goal = np.array([
            np.clip(msg.linear.x, -max_linear, max_linear),
            np.clip(msg.linear.y, -max_linear, max_linear),
            np.clip(msg.angular.z, -max_angular, max_angular),
        ], np.float32)
        if float(np.max(np.abs(goal))) < deadband:
            goal.fill(0.0)
        self.velocity_command_goal = goal
        self.last_command_time = self.get_clock().now()

    def _locomotion_cb(self, msg: Bool) -> None:
        self.locomotion_enabled = bool(msg.data)
        if not self.locomotion_enabled:
            self.velocity_command.fill(0.0)
            if self.q is not None:
                base_rotation = rotation_matrix_xyzw(self.base_q)
                self.stance_foot_world = np.stack([
                    self.base_pos + base_rotation @ leg_foot_position(
                        leg, self.q[index:index + 3])
                    for leg, index in zip(LEG_ORDER, (0, 3, 6, 9))
                ])
                self.stance_leg_target = self.q[:12].copy()

    def _freeze_arm_cb(self, msg: Bool) -> None:
        self.freeze_arm = bool(msg.data)
        if self.freeze_arm and self.q is not None:
            self.frozen_arm_target = self._freeze_arm_target()
            self.arm_ik_active = False
            self.get_logger().info(
                f"freezing Piper arm joints with mode={self.freeze_arm_mode}")
        elif not self.freeze_arm:
            self.frozen_arm_target = None
            self.get_logger().info("unfreezing Piper arm joints")

    def _freeze_arm_target(self) -> np.ndarray:
        if self.freeze_arm_mode == "startup":
            return self.startup_target[12:18].copy()
        return self.q[12:18].copy()

    def _pose_cb(self, msg: PoseStamped) -> None:
        if self.freeze_arm:
            return
        p, q = msg.pose.position, msg.pose.orientation
        base_rotation = rotation_matrix_xyzw(self.base_q)
        link0_offset = base_rotation @ np.array(
            [0.0, 0.0, float(self.get_parameter("link0_offset_z").value)],
            np.float32,
        )
        self.pose_command_goal = world_pose_to_mixed_command(
            np.array([p.x, p.y, p.z], np.float32),
            np.array([q.x, q.y, q.z, q.w], np.float32),
            self.base_pos + link0_offset,
            self.base_q,
        )
        self.arm_ik_world_goal = np.array([p.x, p.y, p.z], np.float32)
        self.arm_ik_world_rotation_goal = rotation_matrix_xyzw(
            np.array([q.x, q.y, q.z, q.w], np.float32))
        self.arm_ik_goal = base_rotation.T @ (
            self.arm_ik_world_goal - (self.base_pos + link0_offset))
        self.arm_ik_rotation_goal = base_rotation.T @ self.arm_ik_world_rotation_goal
        if self.q is not None and not self.arm_ik_active:
            self.arm_ik_position = piper_tcp_position(self.q[12:18])
            self.arm_ik_target = self.q[12:18].copy()
        self.arm_ik_active = True
        self.last_command_time = self.get_clock().now()

    def _update(self) -> None:
        if self.q is None or self.dq is None:
            return
        connected = self.target_pub.get_subscription_count() > 0
        if connected and not self.controllers_connected:
            self.started_at = time.monotonic()
            self.history = ObservationHistory()
        self.controllers_connected = connected
        if not connected:
            return
        if not self._sensors_fresh()[0]:
            self.target_pub.publish(
                Float64MultiArray(data=self.startup_target.astype(float).tolist()))
            return
        elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds * 1.0e-9
        timeout = float(self.get_parameter("command_timeout").value)
        goal = self.velocity_command_goal if elapsed <= timeout else np.zeros(3, np.float32)
        policy_rate = float(self.get_parameter("policy_rate").value)
        lin_step = float(self.get_parameter("cmd_vel_lin_accel").value) / policy_rate
        ang_step = float(self.get_parameter("cmd_vel_ang_accel").value) / policy_rate
        step = np.array([lin_step, lin_step, ang_step], np.float32)
        self.velocity_command = self.velocity_command + np.clip(
            goal - self.velocity_command, -step, step)
        velocity = self.velocity_command
        self.pose_command = slew_pose_command(
            self.pose_command,
            self.pose_command_goal,
            float(self.get_parameter("pose_position_slew_rate").value) / policy_rate,
            float(self.get_parameter("pose_angular_slew_rate").value) / policy_rate,
        )
        policy_enabled = bool(self.get_parameter("enable_policy").value)
        observation = self.history.append((
            self.imu_omega * CONTRACT.base_angular_velocity_scale,
            projected_gravity(self.imu_q),
            self.q - DEFAULT_ANGLES,
            self.dq * CONTRACT.joint_velocity_scale,
            self.action,
            velocity,
            self.pose_command,
        ))
        raw_action_array = np.zeros(18, dtype=np.float32)
        policy_active = False
        if (not policy_enabled or
                time.monotonic() - self.started_at < float(self.get_parameter("startup_delay").value)):
            target = self.startup_target
        else:
            started = time.perf_counter()
            with torch.inference_mode():
                output = self.policy(torch.from_numpy(observation).unsqueeze(0))
                raw_action = output.squeeze(0)
                raw_action_array = raw_action.cpu().numpy().copy()
                self.action_saturation_fraction = float(
                    (raw_action.abs() >= CONTRACT.action_clip).float().mean().cpu())
                self.action = raw_action.clamp(
                    -CONTRACT.action_clip, CONTRACT.action_clip).cpu().numpy()
            policy_active = True
            self.inference_ms = (time.perf_counter() - started) * 1000.0
            target = DEFAULT_ANGLES + float(self.get_parameter("action_scale").value) * self.action
        if (self.arm_ik_active and self.arm_ik_goal is not None
                and bool(self.get_parameter("enable_arm_ik").value)):
            base_rotation = rotation_matrix_xyzw(self.base_q)
            link0_offset = base_rotation @ np.array(
                [0.0, 0.0, float(self.get_parameter("link0_offset_z").value)],
                np.float32,
            )
            self.arm_ik_goal = base_rotation.T @ (
                self.arm_ik_world_goal - (self.base_pos + link0_offset))
            self.arm_ik_rotation_goal = base_rotation.T @ self.arm_ik_world_rotation_goal
            cartesian_step = (
                float(self.get_parameter("arm_ik_position_slew_rate").value) / policy_rate)
            delta = self.arm_ik_goal - self.arm_ik_position
            distance = float(np.linalg.norm(delta))
            if distance > cartesian_step:
                delta *= cartesian_step / distance
            self.arm_ik_position = self.arm_ik_position + delta
            solution, self.arm_ik_residual, self.arm_ik_orientation_residual = (
                solve_piper_pose_ik(
                    self.arm_ik_position, self.arm_ik_rotation_goal,
                    self.arm_ik_target))
            joint_step = (
                float(self.get_parameter("arm_ik_joint_slew_rate").value) / policy_rate)
            self.arm_ik_target = self.arm_ik_target + np.clip(
                solution - self.arm_ik_target, -joint_step, joint_step)
            target = target.copy()
            target[12:18] = self.arm_ik_target
            self.action[12:18] = np.clip(
                (self.arm_ik_target - DEFAULT_ANGLES[12:18])
                / float(self.get_parameter("action_scale").value),
                -CONTRACT.action_clip,
                CONTRACT.action_clip,
            )
        elif bool(self.get_parameter("hold_idle_arm_startup_target").value):
            target = target.copy()
            target[12:18] = self.startup_target[12:18]
            self.action[12:18] = np.clip(
                (self.startup_target[12:18] - DEFAULT_ANGLES[12:18])
                / float(self.get_parameter("action_scale").value),
                -CONTRACT.action_clip,
                CONTRACT.action_clip,
            )
        if self.freeze_arm:
            if self.frozen_arm_target is None:
                self.frozen_arm_target = self._freeze_arm_target()
            target = target.copy()
            target[12:18] = self.frozen_arm_target
            self.action[12:18] = np.clip(
                (self.frozen_arm_target - DEFAULT_ANGLES[12:18])
                / float(self.get_parameter("action_scale").value),
                -CONTRACT.action_clip,
                CONTRACT.action_clip,
            )
        if not self.locomotion_enabled:
            target = target.copy()
            if self.stance_foot_world is not None:
                base_rotation = rotation_matrix_xyzw(self.base_q)
                joint_step = 2.0 / policy_rate
                residuals = []
                for leg_index, (leg, index) in enumerate(
                        zip(LEG_ORDER, (0, 3, 6, 9))):
                    foot_relative = base_rotation.T @ (
                        self.stance_foot_world[leg_index] - self.base_pos)
                    solution, residual = solve_leg_position_ik(
                        leg, foot_relative, self.stance_leg_target[index:index + 3])
                    self.stance_leg_target[index:index + 3] += np.clip(
                        solution - self.stance_leg_target[index:index + 3],
                        -joint_step, joint_step)
                    residuals.append(residual)
                self.stance_ik_residual = max(residuals)
                target[:12] = self.stance_leg_target
            else:
                target[:12] = DEFAULT_ANGLES[:12]
        raw_effort = KPS * (target - self.q) - KDS * self.dq
        self.effort_saturation_fraction = float(
            np.mean(np.abs(raw_effort) >= EFFORT_LIMITS))
        self.target_pub.publish(Float64MultiArray(data=target.astype(float).tolist()))
        self._write_trace(
            observation, raw_action_array, target, raw_effort, policy_active)

    def _publish_status(self) -> None:
        controllers_connected = self.target_pub.get_subscription_count() > 0
        sensors_fresh, sensor_ages = self._sensors_fresh()
        ready = (self.q is not None and controllers_connected and sensors_fresh
                 and time.monotonic() - self.started_at >= float(self.get_parameter("startup_delay").value))
        arm_model_tcp = (
            piper_tcp_position(self.q[12:18]).astype(float).round(4).tolist()
            if self.q is not None else None)
        arm_model_tcp_x = (
            piper_tcp_rotation(self.q[12:18])[:, 0].astype(float).round(4).tolist()
            if self.q is not None else None)
        arm_ik_position = (
            self.arm_ik_position.astype(float).round(4).tolist()
            if self.arm_ik_position is not None else None)
        arm_ik_goal = (
            self.arm_ik_goal.astype(float).round(4).tolist()
            if self.arm_ik_goal is not None else None)
        self.status_pub.publish(String(data=json.dumps({
            "ready": ready,
            "policy_observation_dim": 210,
            "policy_action_dim": 18,
            "inference_ms": round(self.inference_ms, 3),
            "action_saturation_fraction": round(self.action_saturation_fraction, 4),
            "effort_saturation_fraction": round(self.effort_saturation_fraction, 4),
            "contract_version": CONTRACT.version,
            "policy_enabled": bool(self.get_parameter("enable_policy").value),
            "controllers_connected": controllers_connected,
            "freeze_arm": self.freeze_arm,
            "freeze_arm_mode": self.freeze_arm_mode,
            "arm_ik_active": self.arm_ik_active,
            "arm_ik_residual": round(self.arm_ik_residual, 5),
            "arm_ik_orientation_residual": round(
                self.arm_ik_orientation_residual, 5),
            "arm_model_tcp": arm_model_tcp,
            "arm_model_tcp_x": arm_model_tcp_x,
            "arm_ik_position": arm_ik_position,
            "arm_ik_goal": arm_ik_goal,
            "locomotion_enabled": self.locomotion_enabled,
            "stance_ik_residual": round(self.stance_ik_residual, 5),
            "sensors_fresh": sensors_fresh,
            "sensor_ages": sensor_ages,
        })))

    def _sensors_fresh(self) -> tuple[bool, dict[str, float | None]]:
        now = self.get_clock().now()
        stamps = {
            "joint_state": self.joint_received_at,
            "imu": self.imu_received_at,
            "odom": self.odom_received_at,
        }
        ages = {
            name: (None if stamp is None else
                   round((now - stamp).nanoseconds * 1.0e-9, 4))
            for name, stamp in stamps.items()
        }
        return all(age is not None and 0.0 <= age <= 0.1 for age in ages.values()), ages

    def _write_trace(
        self,
        observation: np.ndarray,
        raw_action: np.ndarray,
        target: np.ndarray,
        estimated_effort: np.ndarray,
        policy_active: bool,
    ) -> None:
        if self.trace_stream is None:
            return
        if self.trace_samples >= int(self.get_parameter("trace_max_samples").value):
            return
        record = {
            "sim_time": self.get_clock().now().nanoseconds * 1.0e-9,
            "joint_position": self.q.astype(float).tolist(),
            "joint_velocity": self.dq.astype(float).tolist(),
            "imu_xyzw": self.imu_q.astype(float).tolist(),
            "angular_velocity": self.imu_omega.astype(float).tolist(),
            "base_position": self.base_pos.astype(float).tolist(),
            "base_xyzw": self.base_q.astype(float).tolist(),
            "velocity_command": self.velocity_command.astype(float).tolist(),
            "pose_command_wxyz": self.pose_command.astype(float).tolist(),
            "observation": observation.astype(float).tolist(),
            "raw_action": raw_action.astype(float).tolist(),
            "clipped_action": self.action.astype(float).tolist(),
            "target_position": target.astype(float).tolist(),
            "estimated_effort": estimated_effort.astype(float).tolist(),
            "policy_active": policy_active,
        }
        self.trace_stream.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.trace_stream.flush()
        self.trace_samples += 1

    def destroy_node(self):
        if self.trace_stream is not None:
            self.trace_stream.close()
            self.trace_stream = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WbcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if "Unable to convert call argument to Python object" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
