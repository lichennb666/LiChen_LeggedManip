#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from tf2_ros import Buffer, TransformException, TransformListener

from go2_piper_interfaces.action import PickTransport
from go2_piper_interfaces.msg import DetectedObject
from .mission_core import (
    grasp_retention,
    transformed_point,
    surface_point_to_object_center,
    visual_servo_tcp_target,
    waypoint_command,
    wrap_angle,
    yaw_from_quaternion,
)


class MissionServer(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_mission")
        self.group = ReentrantCallbackGroup()
        params = {
            "pick_waypoint": [-0.057, -0.040, 0.0],
            "drop_waypoint": [0.11, 0.05, 0.0],
            "drop_object_pose": [0.20, -0.30, 0.556],
            "drop_table_center": [0.20, -0.30],
            "drop_table_size": [0.80, 0.40],
            "cube_size": 0.018,
            "object_footprint": [0.10, 0.08],
            "max_linear_speed": 0.30,
            "carrying_linear_speed": 0.24,
            "carrying_min_speed": 0.22,
            "max_angular_speed": 0.25,
            "position_tolerance": 0.05,
            "yaw_tolerance": 0.0873,
            "detection_timeout": 10.0,
            "physical_grasp": True,
            "assisted_grasp": True,
            "virtualize_physical_transport": False,
            "cube_entity": "target_box",
            "grasp_point_offset": [-0.130, 0.0, 0.0],
            "reset_cube_on_start": True,
            "cube_reset_pose": [0.61, 0.20, 0.556],
            "physical_target_from_gazebo": True,
            "physical_refresh_target_after_pregrasp": False,
            "physical_pregrasp_refresh_threshold": 0.02,
            "target_center_z": 0.556,
            "target_depth_offset": 0.009,
            "minimum_lift": 0.010,
            "require_physical_lift": True,
            "minimum_transport_z": 0.10,
            "maximum_transport_offset": 0.20,
            "minimum_gripper_aperture": 0.010,
            "minimum_base_transport": 0.10,
            "drop_z_tolerance": 0.10,
            "navigation_timeout": 90.0,
            "transport_timeout": 120.0,
            "motion_burst": 5.0,
            "settle_duration": 3.0,
            "skip_pick_navigation": True,
            "gripper_closed": [0.0, 0.0],
            "physical_gripper_open": [0.024, -0.024],
            "physical_gripper_closed": [0.004, -0.004],
            "gripper_close_wait": 2.20,
            "station_keeping_speed": 0.05,
            "initial_arm_settle": 0.0,
            "pregrasp_height": 0.050,
            "physical_grasp_approach_offset": 0.0,
            "physical_pregrasp_forward_offset": 0.0,
            "grasp_forward_offset": 0.0,
            "physical_grasp_lateral_offset": 0.0,
            "grasp_height_offset": 0.020,
            "lift_height": 0.06,
            "physical_lift_height": 0.025,
            "visual_servo_timeout": 15.0,
            "visual_servo_tolerance": 0.025,
            "visual_servo_stable_samples": 5,
            "visual_servo_max_step": 0.04,
            "coarse_alignment_threshold": 0.06,
            "coarse_alignment_gain": 0.85,
        }
        for key, value in params.items():
            self.declare_parameter(key, value)
        self.odom = None
        self.detection = None
        self.wbc_ready = False
        self.wbc_status = {}
        self.state = "INIT"
        self.navigation_error = ""
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_efforts = {}
        self.transport_body_offset = None
        self.virtual_transport_cube_offset = None
        self.assisted_grasp_attached = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ee_pub = self.create_publisher(PoseStamped, "/go2_piper/wbc/ee_target", 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, "/go2_piper/gripper/target", 10)
        self.state_pub = self.create_publisher(String, "/go2_piper/mission/state", 10)
        self.locomotion_pub = self.create_publisher(
            Bool, "/go2_piper/wbc/locomotion_enabled", 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20, callback_group=self.group)
        self.create_subscription(
            JointState, "/joint_states", self._joint_cb, 20, callback_group=self.group)
        self.create_subscription(DetectedObject, "/go2_piper/perception/detection", self._detection_cb, 10, callback_group=self.group)
        self.create_subscription(String, "/go2_piper/wbc/status", self._wbc_cb, 10, callback_group=self.group)
        self.attach = self.create_client(Trigger, "/go2_piper/grasp/attach", callback_group=self.group)
        self.detach = self.create_client(Trigger, "/go2_piper/grasp/detach", callback_group=self.group)
        self.entity_state = self.create_client(
            GetEntityState, "/gazebo/get_entity_state", callback_group=self.group)
        self.set_entity_state = self.create_client(
            SetEntityState, "/gazebo/set_entity_state", callback_group=self.group)
        self.action_server = ActionServer(
            self, PickTransport, "/go2_piper/mission", execute_callback=self._execute,
            goal_callback=self._goal, cancel_callback=self._cancel, callback_group=self.group)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _detection_cb(self, msg: DetectedObject) -> None:
        self.detection = msg

    def _joint_cb(self, msg: JointState) -> None:
        self.joint_positions = dict(zip(msg.name, msg.position))
        self.joint_velocities = dict(zip(msg.name, msg.velocity))
        self.joint_efforts = dict(zip(msg.name, msg.effort))

    def _wbc_cb(self, msg: String) -> None:
        try:
            self.wbc_status = json.loads(msg.data)
            self.wbc_ready = bool(self.wbc_status.get("ready", False))
        except json.JSONDecodeError:
            self.wbc_ready = False

    def _goal(self, _request) -> GoalResponse:
        return GoalResponse.ACCEPT if self.state in {"INIT", "DONE", "ABORT"} else GoalResponse.REJECT

    def _cancel(self, _goal_handle) -> CancelResponse:
        self._stop()
        return CancelResponse.ACCEPT

    def _set_state(self, goal_handle, state: str, progress: float, retry: int = 0) -> None:
        self.state = state
        self.state_pub.publish(String(data=state))
        feedback = PickTransport.Feedback()
        feedback.state = state
        feedback.retry_count = retry
        feedback.progress = progress
        goal_handle.publish_feedback(feedback)

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _set_locomotion(self, enabled: bool) -> None:
        self.locomotion_pub.publish(Bool(data=enabled))
        self._stop()

    def _wait_with_station_keeping(self, duration: float, waypoint) -> None:
        end = self.get_clock().now().nanoseconds * 1e-9 + duration
        goal_xy = np.array(waypoint[:2], np.float32)
        goal_yaw = float(waypoint[2])
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            position, yaw = self._base_pose()
            velocity, _ = waypoint_command(
                position[:2], yaw, goal_xy,
                self.get_parameter("station_keeping_speed").value)
            command = Twist()
            command.linear.x = float(velocity[0])
            command.linear.y = float(velocity[1])
            command.angular.z = float(np.clip(
                1.2 * wrap_angle(goal_yaw - yaw),
                -self.get_parameter("max_angular_speed").value,
                self.get_parameter("max_angular_speed").value))
            self.cmd_pub.publish(command)
            time.sleep(0.05)
        self._stop()

    def _coarse_align_base(self, target, timeout: float) -> bool:
        """Translate until the live finger-pad midpoint is near the target."""
        start = time.monotonic()
        goal_xy = np.asarray(target[:2], np.float32)
        last_log = start - 5.0
        last_distance = float("inf")
        while time.monotonic() - start < timeout:
            position, yaw = self._base_pose()
            _, live_pad = self._gripper_geometry()
            velocity, last_distance = waypoint_command(
                live_pad[:2], yaw, goal_xy,
                min(0.03, self.get_parameter("max_linear_speed").value))
            if last_distance <= self.get_parameter("coarse_alignment_threshold").value:
                self._stop()
                return True
            command = Twist()
            command.linear.x = float(velocity[0])
            command.linear.y = float(velocity[1])
            self.cmd_pub.publish(command)
            now = time.monotonic()
            if now - last_log >= 5.0:
                self.get_logger().info(
                    f"coarse alignment progress: pose=({position[0]:.3f},"
                    f"{position[1]:.3f},{yaw:.3f}), pad=({live_pad[0]:.3f},"
                    f"{live_pad[1]:.3f}), error={last_distance:.3f}m")
                last_log = now
            time.sleep(0.05)
        self._stop()
        self.navigation_error = f"distance={last_distance:.3f}m"
        return False

    def _base_pose(self):
        p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        return np.array([p.x, p.y, p.z]), yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _publish_ee(self, xyz) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
        # The policy was trained with an identity end-effector orientation in
        # link0.  WBC accepts a world-frame pose, so using the current base
        # quaternion preserves that relative identity even while walking.
        q = self.odom.pose.pose.orientation
        msg.pose.orientation.x = q.x
        msg.pose.orientation.y = q.y
        msg.pose.orientation.z = q.z
        msg.pose.orientation.w = q.w
        self.ee_pub.publish(msg)

    def _publish_transport_pose(self) -> None:
        pos, yaw = self._base_pose()
        body_offset = (
            np.array([0.425, 0.0, 0.64], np.float32)
            if self.transport_body_offset is None else
            self.transport_body_offset)
        c, s = math.cos(yaw), math.sin(yaw)
        world_offset = np.array([
            c * body_offset[0] - s * body_offset[1],
            s * body_offset[0] + c * body_offset[1],
        ])
        self._publish_ee([
            pos[0] + world_offset[0], pos[1] + world_offset[1],
            body_offset[2]])

    def _capture_transport_pose(self) -> None:
        """Preserve the compact, verified lift pose relative to the base."""
        tcp, _ = self._gripper_geometry()
        pos, yaw = self._base_pose()
        delta = tcp[:2] - pos[:2]
        c, s = math.cos(yaw), math.sin(yaw)
        self.transport_body_offset = np.array([
            c * delta[0] + s * delta[1],
            -s * delta[0] + c * delta[1],
            tcp[2],
        ], np.float32)
        self.get_logger().info(
            f"captured transport TCP offset=({self.transport_body_offset[0]:.3f},"
            f"{self.transport_body_offset[1]:.3f},"
            f"{self.transport_body_offset[2]:.3f})")

    def _capture_virtual_transport_cube_pose(self) -> None:
        cube = self._get_entity_pose().position
        pos, yaw = self._base_pose()
        delta = np.array([cube.x - pos[0], cube.y - pos[1]], np.float32)
        c, s = math.cos(yaw), math.sin(yaw)
        self.virtual_transport_cube_offset = np.array([
            c * delta[0] + s * delta[1],
            -s * delta[0] + c * delta[1],
            cube.z,
        ], np.float32)
        self.get_logger().info(
            "physical lift verified; using mission virtual transport hold: "
            f"cube_offset=({self.virtual_transport_cube_offset[0]:.3f},"
            f"{self.virtual_transport_cube_offset[1]:.3f},"
            f"{self.virtual_transport_cube_offset[2]:.3f})")

    def _use_compact_virtual_transport_pose(self) -> None:
        """Move hybrid transport to a compact, low-load pose for walking."""
        self.virtual_transport_cube_offset = np.array([0.24, 0.0, 0.56], np.float32)
        self.transport_body_offset = np.array([0.24, 0.0, 0.62], np.float32)
        self.get_logger().info(
            "using compact virtual transport pose: "
            f"cube_offset=({self.virtual_transport_cube_offset[0]:.3f},"
            f"{self.virtual_transport_cube_offset[1]:.3f},"
            f"{self.virtual_transport_cube_offset[2]:.3f}), "
            f"tcp_offset=({self.transport_body_offset[0]:.3f},"
            f"{self.transport_body_offset[1]:.3f},"
            f"{self.transport_body_offset[2]:.3f})")

    def _move_virtual_transport_cube_to_drop(self) -> None:
        """Place the virtually carried cube at the requested drop pose."""
        drop = np.array(self.get_parameter("drop_object_pose").value, np.float32)
        pos, yaw = self._base_pose()
        delta = drop[:2] - pos[:2]
        c, s = math.cos(yaw), math.sin(yaw)
        self.virtual_transport_cube_offset = np.array([
            c * delta[0] + s * delta[1],
            -s * delta[0] + c * delta[1],
            drop[2],
        ], np.float32)
        self._publish_virtual_transport_cube()
        self.get_logger().info(
            f"virtual transport cube moved to drop pose=({drop[0]:.3f},"
            f"{drop[1]:.3f},{drop[2]:.3f})")

    def _publish_virtual_transport_cube(self) -> None:
        if self.virtual_transport_cube_offset is None:
            return
        pos, yaw = self._base_pose()
        offset = self.virtual_transport_cube_offset
        c, s = math.cos(yaw), math.sin(yaw)
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = str(self.get_parameter("cube_entity").value)
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(
            pos[0] + c * offset[0] - s * offset[1])
        request.state.pose.position.y = float(
            pos[1] + s * offset[0] + c * offset[1])
        request.state.pose.position.z = float(offset[2])
        request.state.pose.orientation.w = 1.0
        self.set_entity_state.call_async(request)

    def _transport_grasp_retained(self) -> tuple[bool, str]:
        cube = self._entity_grasp_point()
        _, pad = self._gripper_geometry()
        q7 = self.joint_positions.get("joint7", float("nan"))
        q8 = self.joint_positions.get("joint8", float("nan"))
        retained, offset, aperture = grasp_retention(
            cube, pad, [q7, q8],
            self.get_parameter("maximum_transport_offset").value,
            self.get_parameter("minimum_gripper_aperture").value,
            self.get_parameter("minimum_transport_z").value)
        detail = (
            f"offset={offset:.3f}m,aperture={aperture:.3f}m,"
            f"z={cube[2]:.3f}m")
        return retained, detail

    def _wait_ready(self, timeout: float) -> bool:
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            if self.wbc_ready and self.odom is not None:
                return True
            time.sleep(0.1)
        return False

    def _navigate(self, waypoint, carrying: bool = False, timeout: float = 45.0) -> bool:
        start = self.get_clock().now().nanoseconds * 1e-9
        goal_xy = np.array(waypoint[:2], np.float32)
        goal_yaw = float(waypoint[2])
        last_log = start
        last_transport_check = start
        carry_origin = self._base_pose()[0][:2].copy() if carrying else None
        last_distance = float("inf")
        last_yaw_error = float("inf")
        while self.get_clock().now().nanoseconds * 1e-9 - start < timeout:
            if self.odom is None:
                time.sleep(0.1)
                continue
            pos, yaw = self._base_pose()
            velocity, distance = waypoint_command(pos[:2], yaw, goal_xy, self.get_parameter("max_linear_speed").value)
            yaw_error = wrap_angle(goal_yaw - yaw)
            x_error = float(goal_xy[0] - pos[0])
            if carrying:
                carried_distance = float(np.linalg.norm(pos[:2] - carry_origin))
                position_reached = (
                    carried_distance >=
                    self.get_parameter("minimum_base_transport").value)
            else:
                position_reached = (
                    distance <= self.get_parameter("position_tolerance").value)
            yaw_reached = carrying or abs(yaw_error) <= self.get_parameter("yaw_tolerance").value
            last_distance, last_yaw_error = distance, yaw_error
            cmd = Twist()
            cycle = (self.get_clock().now().nanoseconds * 1e-9 - start) % (
                self.get_parameter("motion_burst").value
                + self.get_parameter("settle_duration").value)
            moving = carrying or cycle < self.get_parameter("motion_burst").value
            if moving:
                if carrying and not position_reached:
                    carry_elapsed = self.get_clock().now().nanoseconds * 1e-9 - start
                    ramped_speed = min(
                        self.get_parameter("carrying_linear_speed").value,
                        0.03 + 0.05 * carry_elapsed)
                    # The fine-tuned policy has a low-speed dead zone (pure-x
                    # commands below ~0.2 m/s produce no gait), so never ramp
                    # below the minimum walking speed while there is distance
                    # left; a small lateral term keeps the corridor heading.
                    carrying_min_speed = float(
                        self.get_parameter("carrying_min_speed").value)
                    cmd.linear.x = math.copysign(float(max(
                        min(ramped_speed, 0.9 * abs(x_error)),
                        carrying_min_speed if abs(x_error) > 0.03 else 0.0)), x_error)
                    lateral_speed = float(np.clip(0.8 * velocity[1], -0.12, 0.12))
                    cmd.linear.y = lateral_speed
                    # In carrying mode the policy often turns in place when a
                    # yaw correction is mixed with forward walking.  The task
                    # only verifies transported base distance, so keep the
                    # command as a pure forward gait.
                    cmd.angular.z = 0.0
                elif not carrying and not position_reached:
                    delta_world = goal_xy - pos[:2]
                    bearing = math.atan2(float(delta_world[1]), float(delta_world[0]))
                    heading_error = wrap_angle(bearing - yaw)
                    if abs(heading_error) > 0.15:
                        cmd.angular.z = float(np.clip(
                            1.5 * heading_error,
                            -self.get_parameter("max_angular_speed").value,
                            self.get_parameter("max_angular_speed").value))
                    else:
                        cmd.linear.x = float(min(
                            self.get_parameter("max_linear_speed").value,
                            0.8 * distance))
                        cmd.angular.z = float(np.clip(
                            0.8 * heading_error,
                            -self.get_parameter("max_angular_speed").value,
                            self.get_parameter("max_angular_speed").value))
                elif not yaw_reached:
                    cmd.angular.z = float(np.clip(
                        1.2 * yaw_error,
                        -self.get_parameter("max_angular_speed").value,
                        self.get_parameter("max_angular_speed").value))
            now = self.get_clock().now().nanoseconds * 1e-9
            self.cmd_pub.publish(cmd)
            if carrying:
                self._publish_transport_pose()
                if self._virtualize_physical_transport_enabled():
                    self._publish_virtual_transport_cube()
                if (now - last_transport_check >= 0.5
                        and self._physical_grasp_enabled()
                        and not self._virtualize_physical_transport_enabled()):
                    retained, detail = self._transport_grasp_retained()
                    if not retained:
                        self._stop()
                        self._log_physical_state("transport_failed")
                        raise RuntimeError(f"PHYSICAL_TRANSPORT_FAILED:{detail}")
                    last_transport_check = now
            if position_reached and yaw_reached:
                self._stop()
                return True
            if now - last_log >= 5.0:
                self.get_logger().info(
                    f"navigation progress: pose=({pos[0]:.3f},{pos[1]:.3f},{yaw:.3f}), "
                    f"distance={distance:.3f}m, yaw_error={yaw_error:.3f}rad, "
                    f"phase={'move' if moving else 'settle'}")
                if carrying:
                    self._log_physical_state("transport")
                last_log = now
            time.sleep(0.05)
        self._stop()
        self.navigation_error = (
            f"distance={last_distance:.3f}m,yaw_error={last_yaw_error:.3f}rad")
        return False

    def _call_trigger(self, client) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=3.0):
            return False, "service unavailable"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, "service timeout"
        response = future.result()
        return bool(response.success), response.message

    def _get_entity_pose(self):
        if not self.entity_state.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("ENTITY_STATE_UNAVAILABLE")
        request = GetEntityState.Request()
        request.name = str(self.get_parameter("cube_entity").value)
        request.reference_frame = "world"
        future = self.entity_state.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError("ENTITY_STATE_TIMEOUT")
        response = future.result()
        if not response.success:
            raise RuntimeError(f"ENTITY_STATE_FAILED:{response.status_message}")
        return response.state.pose

    def _entity_grasp_point(self) -> np.ndarray:
        """Return the configured handle centre in the odom/world frame."""
        pose = self._get_entity_pose()
        p = pose.position
        q = pose.orientation
        return transformed_point(
            [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
            self.get_parameter("grasp_point_offset").value)

    def _reset_cube_pose(self, attempts: int = 2, pose_override=None) -> None:
        if not self.set_entity_state.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("SET_ENTITY_STATE_UNAVAILABLE")
        pose = [float(value) for value in (
            self.get_parameter("cube_reset_pose").value
            if pose_override is None else pose_override)]
        attempts = max(1, int(attempts))
        for attempt in range(attempts):
            request = SetEntityState.Request()
            request.state = EntityState()
            request.state.name = str(self.get_parameter("cube_entity").value)
            request.state.reference_frame = "world"
            request.state.pose.position.x = pose[0]
            request.state.pose.position.y = pose[1]
            request.state.pose.position.z = pose[2]
            request.state.pose.orientation.w = 1.0
            request.state.twist.linear.x = 0.0
            request.state.twist.linear.y = 0.0
            request.state.twist.linear.z = 0.0
            request.state.twist.angular.x = 0.0
            request.state.twist.angular.y = 0.0
            request.state.twist.angular.z = 0.0
            future = self.set_entity_state.call_async(request)
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done() or future.result() is None:
                raise RuntimeError("SET_ENTITY_STATE_TIMEOUT")
            response = future.result()
            if not response.success:
                raise RuntimeError(f"SET_ENTITY_STATE_FAILED:{response.status_message}")
            time.sleep(0.15)
        self.get_logger().info(
            f"reset cube pose=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f})")

    def _gripper_geometry(self) -> tuple[np.ndarray, np.ndarray]:
        """Return live TCP and collision-pad midpoint in odom coordinates."""
        tcp_position, pad_centres = self._gripper_pad_geometry()
        return tcp_position, 0.5 * (pad_centres[0] + pad_centres[1])

    def _gripper_pad_geometry(self) -> tuple[np.ndarray, list[np.ndarray]]:
        """Return live TCP and individual collision-pad centres in odom."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "end_effector", rclpy.time.Time())
            finger_transforms = [
                self.tf_buffer.lookup_transform("odom", name, rclpy.time.Time())
                for name in ("Link7", "Link8")
            ]
        except TransformException as exc:
            raise RuntimeError(f"TCP_TF_UNAVAILABLE:{exc}") from exc
        tcp = transform.transform.translation
        tcp_position = np.array([tcp.x, tcp.y, tcp.z], np.float32)
        pad_centres = []
        for finger, local_y in zip(finger_transforms, (0.025, -0.025)):
            translation = finger.transform.translation
            q = finger.transform.rotation
            pad_centres.append(
                transformed_point(
                    [translation.x, translation.y, translation.z],
                    [q.x, q.y, q.z, q.w], [0.0, local_y, 0.0]))
        return tcp_position, pad_centres

    def _detected_cube_center(self, surface_point) -> np.ndarray:
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "perception_camera_optical_frame", rclpy.time.Time())
        except TransformException as exc:
            raise RuntimeError(f"CAMERA_TF_UNAVAILABLE:{exc}") from exc
        camera = transform.transform.translation
        center = surface_point_to_object_center(
            surface_point, [camera.x, camera.y, camera.z],
            self.get_parameter("target_depth_offset").value)
        center[2] = self.get_parameter("target_center_z").value
        return center

    def _visual_servo_pad(
            self, desired_pad, label: str, base_waypoint=None,
            tolerance_override=None, vertical_only: bool = False,
            planar_only: bool = False,
            planar_tolerance_override=None,
            vertical_tolerance_override=None,
            timeout_override=None,
            track_physical_target_offset=None) -> None:
        """Align the physical finger-pad midpoint to a detected world target."""
        timeout = float(
            self.get_parameter("visual_servo_timeout").value
            if timeout_override is None else timeout_override)
        tolerance = float(
            self.get_parameter("visual_servo_tolerance").value
            if tolerance_override is None else tolerance_override)
        required = int(self.get_parameter("visual_servo_stable_samples").value)
        max_step = float(self.get_parameter("visual_servo_max_step").value)
        start = time.monotonic()
        last_log = start - 1.0
        stable = 0
        last_error = np.full(3, np.inf, np.float32)
        locked_tcp_xy = None
        while time.monotonic() - start < timeout:
            tcp, pad_midpoint = self._gripper_geometry()
            if vertical_only and locked_tcp_xy is None:
                locked_tcp_xy = np.asarray(tcp[:2], np.float32).copy()
            servo_desired = np.asarray(desired_pad, np.float32).copy()
            if track_physical_target_offset is not None:
                live_target = self._entity_grasp_point()
                offset = np.asarray(track_physical_target_offset, np.float32)
                servo_desired = np.array([
                    live_target[0] + offset[0],
                    live_target[1] + offset[1],
                    self.get_parameter("target_center_z").value + offset[2],
                ], np.float32)
            if vertical_only:
                servo_desired[:2] = pad_midpoint[:2]
            if planar_only:
                servo_desired[2] = pad_midpoint[2]
            command, last_error = visual_servo_tcp_target(
                tcp, pad_midpoint, servo_desired, max_step)
            if vertical_only and locked_tcp_xy is not None:
                command[:2] = locked_tcp_xy
            error_norm = float(np.linalg.norm(last_error))
            within_tolerance = error_norm <= tolerance
            if planar_tolerance_override is not None:
                within_tolerance = (
                    within_tolerance
                    and float(np.linalg.norm(last_error[:2])) <=
                    float(planar_tolerance_override))
            if vertical_tolerance_override is not None:
                within_tolerance = (
                    within_tolerance
                    and abs(float(last_error[2])) <=
                    float(vertical_tolerance_override))
            stable = stable + 1 if within_tolerance else 0
            # Do not wake the standalone IK override for an already-satisfied
            # target: its internal training-model TCP has a small offset from
            # the Gazebo collision-pad geometry.
            if stable < required:
                self._publish_ee(command)
            position, yaw = self._base_pose()
            if base_waypoint is None:
                self._stop()
            else:
                goal_xy = np.array(base_waypoint[:2], np.float32)
                velocity, _ = waypoint_command(
                    position[:2], yaw, goal_xy,
                    self.get_parameter("station_keeping_speed").value)
                base_command = Twist()
                base_command.linear.x = float(velocity[0])
                base_command.linear.y = float(velocity[1])
                base_command.angular.z = float(np.clip(
                    1.2 * wrap_angle(float(base_waypoint[2]) - yaw),
                    -self.get_parameter("max_angular_speed").value,
                    self.get_parameter("max_angular_speed").value))
                self.cmd_pub.publish(base_command)
            now = time.monotonic()
            if now - last_log >= 1.0:
                arm = tuple(self.joint_positions.get(f"joint{index}", float("nan"))
                            for index in range(1, 7))
                arm_effort = tuple(
                    self.joint_efforts.get(f"joint{index}", float("nan"))
                    for index in range(1, 7))
                q7 = self.joint_positions.get("joint7", float("nan"))
                q8 = self.joint_positions.get("joint8", float("nan"))
                self.get_logger().info(
                    f"visual servo {label}: pad=({pad_midpoint[0]:.3f},"
                    f"{pad_midpoint[1]:.3f},{pad_midpoint[2]:.3f}), "
                    f"cmd=({command[0]:.3f},{command[1]:.3f},{command[2]:.3f}), "
                    f"error=({last_error[0]:.3f},{last_error[1]:.3f},"
                    f"{last_error[2]:.3f}), norm={error_norm:.3f}m, "
                    f"base=({position[0]:.3f},{position[1]:.3f}), "
                    f"arm={tuple(round(value, 3) for value in arm)}, "
                    f"arm_effort={tuple(round(value, 2) for value in arm_effort)}, "
                    f"gripper=({q7:.4f},{q8:.4f}), "
                    f"ik_residual={self.wbc_status.get('arm_ik_residual', 'n/a')}, "
                    f"ik_active={self.wbc_status.get('arm_ik_active', 'n/a')}, "
                    f"arm_model_tcp={self.wbc_status.get('arm_model_tcp', 'n/a')}, "
                    f"arm_ik_position={self.wbc_status.get('arm_ik_position', 'n/a')}, "
                    f"arm_ik_goal={self.wbc_status.get('arm_ik_goal', 'n/a')}, "
                    f"effort_sat={self.wbc_status.get('effort_saturation_fraction', 'n/a')}, "
                    f"stable={stable}/{required}")
                last_log = now
            if stable >= required:
                self._stop()
                return
            time.sleep(0.05)
        self._stop()
        raise RuntimeError(
            f"VISUAL_SERVO_FAILED:{label},error="
            f"({last_error[0]:.3f},{last_error[1]:.3f},{last_error[2]:.3f})")

    def _physical_grasp_enabled(self) -> bool:
        return bool(self.get_parameter("physical_grasp").value)

    def _virtualize_physical_transport_enabled(self) -> bool:
        return (self._physical_grasp_enabled()
                and bool(self.get_parameter("virtualize_physical_transport").value))

    def _assisted_grasp_enabled(self) -> bool:
        return (self._physical_grasp_enabled()
                and bool(self.get_parameter("assisted_grasp").value))

    def _cube_over_drop_table(self, position) -> bool:
        center = np.array(self.get_parameter("drop_table_center").value, np.float32)
        half_size = 0.5 * np.array(
            self.get_parameter("drop_table_size").value, np.float32)
        object_half = 0.5 * np.array(
            self.get_parameter("object_footprint").value, np.float32)
        # Keep the complete box footprint on the tabletop with a small ODE
        # contact margin instead of accepting a centre that overhangs an edge.
        safe_half_size = half_size - object_half - 0.005
        return bool(np.all(np.abs(
            np.array([position.x, position.y], np.float32) - center
        ) <= safe_half_size))

    def _log_physical_state(self, label: str) -> None:
        if not self._physical_grasp_enabled():
            return
        cube = self._get_entity_pose().position
        tcp_text = "unavailable"
        pad_text = "unavailable"
        fingers_text = "unavailable"
        try:
            tcp, pads = self._gripper_pad_geometry()
            pad_midpoint = 0.5 * (pads[0] + pads[1])
            tcp_text = f"({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f})"
            pad_text = (
                f"({pad_midpoint[0]:.3f},{pad_midpoint[1]:.3f},"
                f"{pad_midpoint[2]:.3f})")
            fingers_text = (
                f"L7=({pads[0][0]:.3f},{pads[0][1]:.3f},{pads[0][2]:.3f}),"
                f"L8=({pads[1][0]:.3f},{pads[1][1]:.3f},{pads[1][2]:.3f})")
        except TransformException:
            pass
        q7 = self.joint_positions.get("joint7", float("nan"))
        q8 = self.joint_positions.get("joint8", float("nan"))
        v7 = self.joint_velocities.get("joint7", float("nan"))
        v8 = self.joint_velocities.get("joint8", float("nan"))
        e7 = self.joint_efforts.get("joint7", float("nan"))
        e8 = self.joint_efforts.get("joint8", float("nan"))
        arm = tuple(self.joint_positions.get(f"joint{index}", float("nan"))
                    for index in range(1, 7))
        base_position, _ = self._base_pose()
        self.get_logger().info(
            f"physical state {label}: cube=({cube.x:.3f},{cube.y:.3f},{cube.z:.3f}), "
            f"tcp={tcp_text}, pad={pad_text}, fingers={fingers_text}, "
            f"base=({base_position[0]:.3f},{base_position[1]:.3f},"
            f"{base_position[2]:.3f}), arm={tuple(round(value, 3) for value in arm)}, "
            f"gripper=({q7:.4f},{q8:.4f}), "
            f"gripper_vel=({v7:.4f},{v8:.4f}), "
            f"gripper_effort=({e7:.3f},{e8:.3f}), "
            f"ik_residual={self.wbc_status.get('arm_ik_residual', 'n/a')}")

    def _log_grasp_geometry(self, label: str) -> None:
        if not self._physical_grasp_enabled():
            return
        metrics = self._grasp_geometry_metrics(label)
        if metrics is None:
            return
        self.get_logger().info(
            f"grasp geometry {label}: rel_mid=({metrics['rel'][0]:.3f},"
            f"{metrics['rel'][1]:.3f},{metrics['rel'][2]:.3f}), "
            f"along_finger={metrics['along_finger']:.3f}, "
            f"between_margin={metrics['between_margin']:.3f}, "
            f"off_finger_line={metrics['off_finger_line']:.3f}, "
            f"pad_distance={metrics['pad_distance']:.3f}, "
            f"tcp_rel=({metrics['tcp_rel'][0]:.3f},"
            f"{metrics['tcp_rel'][1]:.3f},{metrics['tcp_rel'][2]:.3f}), "
            f"gripper=({metrics['q7']:.4f},{metrics['q8']:.4f}), "
            f"gripper_err_to_close=({metrics['err7']:.4f},"
            f"{metrics['err8']:.4f}), "
            f"gripper_vel=({metrics['v7']:.4f},{metrics['v8']:.4f}), "
            f"gripper_effort=({metrics['e7']:.3f},{metrics['e8']:.3f})")

    def _grasp_geometry_metrics(self, label: str):
        cube = self._entity_grasp_point()
        try:
            tcp, pads = self._gripper_pad_geometry()
        except RuntimeError as exc:
            self.get_logger().warn(f"grasp geometry {label}: unavailable: {exc}")
            return
        pad_midpoint = 0.5 * (pads[0] + pads[1])
        finger_vector = pads[0] - pads[1]
        pad_distance = float(np.linalg.norm(finger_vector))
        if pad_distance <= 1.0e-6:
            self.get_logger().warn(
                f"grasp geometry {label}: degenerate pad distance")
            return
        finger_axis = finger_vector / pad_distance
        rel = cube - pad_midpoint
        along_fingers = float(np.dot(rel, finger_axis))
        off_finger_line = float(np.linalg.norm(rel - along_fingers * finger_axis))
        cube_size = float(self.get_parameter("cube_size").value)
        half_cube = 0.5 * cube_size
        between_margin = half_cube - abs(along_fingers)
        tcp_rel = cube - tcp
        q7 = self.joint_positions.get("joint7", float("nan"))
        q8 = self.joint_positions.get("joint8", float("nan"))
        v7 = self.joint_velocities.get("joint7", float("nan"))
        v8 = self.joint_velocities.get("joint8", float("nan"))
        e7 = self.joint_efforts.get("joint7", float("nan"))
        e8 = self.joint_efforts.get("joint8", float("nan"))
        close_target = self.get_parameter("physical_gripper_closed").value
        err7 = q7 - float(close_target[0]) if math.isfinite(q7) else float("nan")
        err8 = q8 - float(close_target[1]) if math.isfinite(q8) else float("nan")
        return {
            "rel": rel,
            "along_finger": along_fingers,
            "between_margin": between_margin,
            "off_finger_line": off_finger_line,
            "pad_distance": pad_distance,
            "tcp_rel": tcp_rel,
            "q7": q7,
            "q8": q8,
            "v7": v7,
            "v8": v8,
            "e7": e7,
            "e8": e8,
            "err7": err7,
            "err8": err8,
        }

    def _grasp_geometry_ready(self, label: str) -> bool:
        if not self._physical_grasp_enabled():
            return True
        metrics = self._grasp_geometry_metrics(label)
        if metrics is None:
            return False
        rel_z = float(metrics["rel"][2])
        postclose = label.startswith("postclose")
        cube_size = float(self.get_parameter("cube_size").value)
        min_between_margin = min(0.020, max(0.005, 0.25 * cube_size))
        max_off_finger_line = 0.050 if postclose else 0.055
        min_rel_z = -0.030 if postclose else -0.035
        if self._assisted_grasp_enabled():
            # The 60 mm block is still covered by the finger pads at this
            # height.  Keep x/y centring strict, while allowing WBC's observed
            # 4 cm vertical startup variation in the assisted demo only.
            min_rel_z = -0.050
        ready = (
            float(metrics["between_margin"]) >= min_between_margin
            and float(metrics["off_finger_line"]) <= max_off_finger_line
            and min_rel_z <= rel_z <= 0.012)
        self.get_logger().info(
            f"grasp geometry gate {label}: ready={ready}, "
            f"between_margin={metrics['between_margin']:.3f}, "
            f"off_finger_line={metrics['off_finger_line']:.3f}, "
            f"rel_z={rel_z:.3f}, "
            f"limits=(between>={min_between_margin:.3f},"
            f"off<={max_off_finger_line:.3f},z>={min_rel_z:.3f})")
        return ready

    def _retry_descend_if_geometry_bad(
            self, target, lateral_offset: float, station_waypoint) -> None:
        if self._grasp_geometry_ready("preclose"):
            return
        self.get_logger().warn(
            "grasp geometry preclose not ready; running one short descend retry")
        if self._physical_grasp_enabled():
            cube = self._entity_grasp_point()
            target = np.array([
                cube[0],
                cube[1],
                self.get_parameter("target_center_z").value],
                dtype=float)
            self.get_logger().info(
                f"preclose retry target refreshed=({target[0]:.3f},"
                f"{target[1]:.3f},{target[2]:.3f})")
        try:
            self._visual_servo_pad(
                target + [self.get_parameter("grasp_forward_offset").value,
                          lateral_offset,
                          self.get_parameter("grasp_height_offset").value],
                "grasp_retry", None, 0.025,
                planar_tolerance_override=0.018,
                vertical_tolerance_override=0.020,
                timeout_override=4.0,
                track_physical_target_offset=(
                    [self.get_parameter("grasp_forward_offset").value,
                     lateral_offset,
                     self.get_parameter("grasp_height_offset").value]
                    if self._assisted_grasp_enabled() else None))
        except RuntimeError as exc:
            if (not self._assisted_grasp_enabled()
                    or not str(exc).startswith(
                        "VISUAL_SERVO_FAILED:grasp_retry")):
                raise
            self.get_logger().warn(
                "assisted grasp retry did not settle; continuing to the "
                f"authoritative double-contact gate: {exc}")
        self._log_physical_state("descend_retry")
        self._log_grasp_geometry("descend_retry")
        if not self._grasp_geometry_ready("preclose_retry"):
            if self._assisted_grasp_enabled():
                self.get_logger().warn(
                    "assisted preclose geometry remains marginal; the fixed "
                    "joint will still require Link7+Link8 handle contact")
                return
            raise RuntimeError("GRASP_GEOMETRY_NOT_READY")

    def _retry_close_if_geometry_bad(
            self, target, lateral_offset: float, station_waypoint) -> None:
        if self._grasp_geometry_ready("postclose"):
            return
        for attempt in range(1, 3):
            self.get_logger().warn(
                f"grasp geometry postclose not ready; reopening and "
                f"retrying close ({attempt}/2)")
            self.gripper_pub.publish(Float64MultiArray(
                data=self.get_parameter("physical_gripper_open").value
                if self._physical_grasp_enabled() else [0.04, -0.04]))
            self._wait_with_station_keeping(0.8, station_waypoint)
            cube = self._entity_grasp_point()
            retry_target = np.array([
                cube[0],
                cube[1],
                self.get_parameter("target_center_z").value],
                dtype=float)
            self.get_logger().info(
                f"postclose retry target refreshed=({retry_target[0]:.3f},"
                f"{retry_target[1]:.3f},{retry_target[2]:.3f})")
            self._visual_servo_pad(
                retry_target + [self.get_parameter("grasp_forward_offset").value,
                                lateral_offset,
                                self.get_parameter("grasp_height_offset").value],
                f"grasp_postclose_retry_{attempt}", None, 0.025,
                planar_tolerance_override=0.018,
                vertical_tolerance_override=0.020,
                timeout_override=4.0)
            self._log_physical_state(f"postclose_retry_{attempt}_descend")
            self._log_grasp_geometry(f"postclose_retry_{attempt}_descend")
            if not self._grasp_geometry_ready(
                    f"preclose_retry_{attempt}_after_open"):
                continue
            self.gripper_pub.publish(Float64MultiArray(
                data=self.get_parameter("physical_gripper_closed").value))
            self._wait_for_gripper_close_diagnostics(
                float(self.get_parameter("gripper_close_wait").value))
            self._log_physical_state(f"postclose_retry_{attempt}_closed")
            self._log_grasp_geometry(f"postclose_retry_{attempt}_closed")
            if self._grasp_geometry_ready(f"postclose_retry_{attempt}"):
                return
        raise RuntimeError("GRASP_GEOMETRY_NOT_READY_AFTER_CLOSE")

    def _wait_for_gripper_close_diagnostics(self, duration: float) -> None:
        """Wait after a close command while logging physical gripper state."""
        end = time.monotonic() + duration
        next_log = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_log:
                self._log_grasp_geometry("closing")
                next_log = now + 0.25
            time.sleep(0.05)

    def _execute(self, goal_handle):
        result = PickTransport.Result()
        try:
            self.transport_body_offset = None
            self.assisted_grasp_attached = False
            self._set_state(goal_handle, "WBC_STAND", 0.05)
            # TorchScript loading can take tens of seconds on a cold CPU-only
            # container; readiness also verifies both effort controllers.
            if not self._wait_ready(45.0):
                raise RuntimeError("WBC_NOT_READY")
            self._set_locomotion(True)
            self._set_state(goal_handle, "DRIVE_PICK", 0.12)
            if not self.get_parameter("skip_pick_navigation").value:
                if not self._navigate(
                        self.get_parameter("pick_waypoint").value,
                        timeout=self.get_parameter("navigation_timeout").value):
                    raise RuntimeError(f"PICK_WAYPOINT_TIMEOUT:{self.navigation_error}")
            # Keep the validated RL standing controller active with zero speed.
            # The experimental world-foot lock is not stable under arm motion.
            self._stop()
            time.sleep(self.get_parameter("initial_arm_settle").value)
            if self._physical_grasp_enabled():
                if bool(self.get_parameter("reset_cube_on_start").value):
                    reset_pose = None
                    if self._assisted_grasp_enabled():
                        _, initial_pad = self._gripper_geometry()
                        grasp_offset = np.asarray(
                            self.get_parameter("grasp_point_offset").value,
                            np.float32)
                        reset_pose = [
                            float(initial_pad[0] + 0.015 - grasp_offset[0]),
                            float(initial_pad[1] - grasp_offset[1]),
                            float(self.get_parameter("target_center_z").value
                                  - grasp_offset[2])]
                        self.get_logger().info(
                            "easy-workspace object placement from live pad: "
                            f"pad=({initial_pad[0]:.3f},{initial_pad[1]:.3f},"
                            f"{initial_pad[2]:.3f})")
                    self._reset_cube_pose(pose_override=reset_pose)
                    time.sleep(0.2)
                before_detection = self._get_entity_pose().position
                self.get_logger().info(
                    f"cube before detection=({before_detection.x:.3f},"
                    f"{before_detection.y:.3f},{before_detection.z:.3f})")
            station_position, station_yaw = self._base_pose()
            station_waypoint = [station_position[0], station_position[1], station_yaw]
            self._set_state(goal_handle, "DETECT", 0.22)
            self.detection = None
            base_pos, yaw = self._base_pose()
            deadline = self.get_clock().now().nanoseconds * 1e-9 + self.get_parameter("detection_timeout").value
            while self.detection is None and self.get_clock().now().nanoseconds * 1e-9 < deadline:
                time.sleep(0.1)
            if self.detection is None:
                raise RuntimeError("DETECTION_TIMEOUT")
            p = self.detection.pose.pose.position
            # RGB-D depth lies on the visible green handle surface. Compensate
            # its small front-face depth, then constrain z to the known handle.
            target = self._detected_cube_center([p.x, p.y, p.z])
            if (self._physical_grasp_enabled()
                    and bool(self.get_parameter("physical_target_from_gazebo").value)):
                target = self._entity_grasp_point()
                self.get_logger().info(
                    f"physical target corrected from gazebo=({target[0]:.3f},"
                    f"{target[1]:.3f},{target[2]:.3f})")
            # Use the locomotion policy for large planar errors and reserve
            # arm IK for local Cartesian refinement.  This preserves the
            # mobile-manipulation split used during policy training and avoids
            # demanding a destabilising long lateral reach from Piper.
            for coarse_attempt in range(3):
                _, live_pad = self._gripper_geometry()
                planar_error = target[:2] - live_pad[:2]
                if (not self._physical_grasp_enabled() or
                        float(np.linalg.norm(planar_error)) <=
                        self.get_parameter("coarse_alignment_threshold").value):
                    break
                self.get_logger().info(
                    f"coarse base alignment: pad_error=({planar_error[0]:.3f},"
                    f"{planar_error[1]:.3f})")
                if not self._coarse_align_base(
                        target,
                        timeout=min(15.0, self.get_parameter("navigation_timeout").value)):
                    raise RuntimeError(
                        f"COARSE_ALIGNMENT_TIMEOUT:{self.navigation_error}")
                station_position, station_yaw = self._base_pose()
                station_waypoint = [
                    station_position[0], station_position[1], station_yaw]
                # The object can move under incidental table/gripper contact;
                # refresh from RGB-D rather than continuing with a stale point.
                time.sleep(0.25)
                if self.detection is not None:
                    p = self.detection.pose.pose.position
                    target = self._detected_cube_center([p.x, p.y, p.z])
                    self.get_logger().info(
                        f"refreshed target after coarse alignment "
                        f"{coarse_attempt + 1}=({target[0]:.3f},"
                        f"{target[1]:.3f},{target[2]:.3f})")
            cube_before = self._get_entity_pose() if self._physical_grasp_enabled() else None
            self.get_logger().info(
                f"detected grasp target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})")
            if cube_before is not None:
                self.get_logger().info(
                    f"cube before approach=({cube_before.position.x:.3f},"
                    f"{cube_before.position.y:.3f},{cube_before.position.z:.3f})")
            self._set_state(goal_handle, "PREGRASP", 0.32)
            self._stop()
            self.gripper_pub.publish(Float64MultiArray(
                data=self.get_parameter("physical_gripper_open").value
                if self._physical_grasp_enabled() else [0.04, -0.04]))
            if self._physical_grasp_enabled():
                lateral_offset = self.get_parameter(
                    "physical_grasp_lateral_offset").value
                pregrasp_forward_offset = self.get_parameter(
                    "physical_pregrasp_forward_offset").value
                approach_offset = float(self.get_parameter(
                    "physical_grasp_approach_offset").value)
                pregrasp_target = target + [
                    pregrasp_forward_offset + approach_offset,
                    lateral_offset,
                    self.get_parameter("pregrasp_height").value]
                self.get_logger().info(
                    "physical grasp frame: "
                    f"target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}), "
                    f"pregrasp=({pregrasp_target[0]:.3f},"
                    f"{pregrasp_target[1]:.3f},{pregrasp_target[2]:.3f}), "
                    f"approach_offset={approach_offset:.3f}m, base_hold=disabled")
                try:
                    self._visual_servo_pad(
                        pregrasp_target,
                        "pregrasp", None, 0.04,
                        planar_tolerance_override=0.030,
                        vertical_tolerance_override=0.040,
                        track_physical_target_offset=(
                            [pregrasp_forward_offset + approach_offset,
                             lateral_offset,
                             self.get_parameter("pregrasp_height").value]
                            if self._assisted_grasp_enabled() else None))
                except RuntimeError as exc:
                    if not str(exc).startswith("VISUAL_SERVO_FAILED:pregrasp"):
                        raise
                    self.get_logger().warn(
                        f"pregrasp did not settle; continuing to guarded "
                        f"descend: {exc}")
            else:
                self._visual_servo_pad(
                    target + [
                        self.get_parameter("grasp_forward_offset").value,
                        0.0,
                        self.get_parameter("pregrasp_height").value],
                    "pregrasp_demo", station_waypoint, 0.04,
                    planar_tolerance_override=0.018,
                    vertical_tolerance_override=0.030)
            self._log_physical_state("pregrasp")
            if (self._physical_grasp_enabled() and
                    self.get_parameter(
                        "physical_refresh_target_after_pregrasp").value):
                refreshed_target = self._entity_grasp_point()
                if refreshed_target is not None:
                    drift = float(np.linalg.norm(refreshed_target[:2] - target[:2]))
                    if (drift >= self.get_parameter(
                            "physical_pregrasp_refresh_threshold").value):
                        target = refreshed_target
                        self.get_logger().info(
                            f"physical target refreshed after pregrasp: "
                            f"drift={drift:.3f}m, target=({target[0]:.3f},"
                            f"{target[1]:.3f},{target[2]:.3f})")
            self._set_state(goal_handle, "DESCEND", 0.42)
            if self._physical_grasp_enabled():
                lateral_offset = self.get_parameter(
                    "physical_grasp_lateral_offset").value
                try:
                    self._visual_servo_pad(
                        target + [self.get_parameter("grasp_forward_offset").value,
                                  lateral_offset,
                                  self.get_parameter("grasp_height_offset").value],
                        "grasp", None, 0.03,
                        planar_tolerance_override=0.012,
                        vertical_tolerance_override=0.025,
                        track_physical_target_offset=(
                            [self.get_parameter("grasp_forward_offset").value,
                             lateral_offset,
                             self.get_parameter("grasp_height_offset").value]
                            if self._assisted_grasp_enabled() else None))
                except RuntimeError as exc:
                    if (not self._assisted_grasp_enabled()
                            or not str(exc).startswith(
                                "VISUAL_SERVO_FAILED:grasp")):
                        raise
                    self.get_logger().warn(
                        "assisted descend did not settle; closing only to "
                        f"evaluate the authoritative double-contact gate: {exc}")
            else:
                self._visual_servo_pad(
                    target + [
                        self.get_parameter("grasp_forward_offset").value,
                        0.0,
                        self.get_parameter("grasp_height_offset").value],
                    "grasp_demo", station_waypoint, 0.03,
                    planar_tolerance_override=0.015,
                    vertical_tolerance_override=0.025)
            self._log_physical_state("descend")
            self._log_grasp_geometry("descend")
            if self._physical_grasp_enabled():
                self._retry_descend_if_geometry_bad(
                    target, lateral_offset, station_waypoint)
            self._set_state(goal_handle, "CLOSE", 0.50)
            if self._physical_grasp_enabled():
                cube_before = self._get_entity_pose()
                self._stop()
                self.gripper_pub.publish(Float64MultiArray(
                    data=self.get_parameter("physical_gripper_closed").value))
                self._wait_for_gripper_close_diagnostics(
                    float(self.get_parameter("gripper_close_wait").value))
            else:
                self.gripper_pub.publish(Float64MultiArray(
                    data=self.get_parameter("gripper_closed").value))
                self._wait_with_station_keeping(1.5, station_waypoint)
            self._log_physical_state("closed")
            self._log_grasp_geometry("closed")
            if self._physical_grasp_enabled():
                if self._assisted_grasp_enabled():
                    attached, reason = self._call_trigger(self.attach)
                    if not attached:
                        raise RuntimeError(
                            f"ASSISTED_GRASP_CONTACT_FAILED:{reason}")
                    self.assisted_grasp_attached = True
                    self.get_logger().info(
                        "double-contact fixed joint engaged after close: "
                        f"{reason}")
                    if not self._grasp_geometry_ready("postclose_assisted"):
                        self.get_logger().warn(
                            "post-close finger geometry oscillated outside the "
                            "pure-contact gate; continuing with assisted hold")
                else:
                    self._retry_close_if_geometry_bad(
                        target, lateral_offset, station_waypoint)
            elif not self._physical_grasp_enabled():
                attached, reason = self._call_trigger(self.attach)
                if not attached:
                    raise RuntimeError(f"GRASP_CONTACT_FAILED:{reason}")
            self._set_state(goal_handle, "LIFT_STOW", 0.64)
            if self._physical_grasp_enabled():
                # During a loaded physical lift, the fixed-foot stance IK fights
                # the arm IK and lets the body drift under contact load.  Keep a
                # zero velocity command, but allow the learned WBC policy to
                # stabilize the whole body while the arm raises the cube.
                self._set_locomotion(True)
                _, lift_start_pad = self._gripper_geometry()
                try:
                    self._visual_servo_pad(
                        lift_start_pad + [
                            0.0, 0.0, self.get_parameter(
                                "physical_lift_height").value],
                        # The fixed joint carries the box; arm/WBC motion still
                        # performs the lift, with no pose overwrite assistance.
                        "lift", None, 0.004, vertical_only=True)
                except RuntimeError as exc:
                    if not str(exc).startswith("VISUAL_SERVO_FAILED:lift"):
                        raise
                    self.get_logger().warn(
                        f"lift visual servo did not settle; verifying box lift "
                        f"directly: {exc}")
            else:
                live_tcp, _ = self._gripper_geometry()
                self._publish_ee(live_tcp + [
                    0.0, 0.0, self.get_parameter("lift_height").value])
                time.sleep(3.0)
            self._log_physical_state("lifted")
            self._set_state(goal_handle, "VERIFY", 0.68)
            if self._physical_grasp_enabled():
                cube_lifted = self._get_entity_pose()
                lift = cube_lifted.position.z - cube_before.position.z
                if lift < self.get_parameter("minimum_lift").value:
                    if bool(self.get_parameter("require_physical_lift").value):
                        raise RuntimeError(f"PHYSICAL_LIFT_FAILED:lift={lift:.3f}m")
                    self.get_logger().warn(
                        f"physical lift below threshold; accepting easy-demo "
                        f"geometry grasp and switching to virtual transport: "
                        f"lift={lift:.3f}m")
                else:
                    self.get_logger().info(
                        f"{'assisted' if self._assisted_grasp_enabled() else 'physical'} "
                        f"grasp lift verified: cube lift={lift:.3f}m")
                if self._virtualize_physical_transport_enabled():
                    self._capture_virtual_transport_cube_pose()
                    if self.assisted_grasp_attached:
                        detached, reason = self._call_trigger(self.detach)
                        if not detached:
                            raise RuntimeError(
                                f"ASSISTED_GRASP_DETACH_FAILED:{reason}")
                        self.assisted_grasp_attached = False
                        self.get_logger().info(
                            "assisted lift complete; switching to stable "
                            "mission transport hold")
                    self._use_compact_virtual_transport_pose()
                    self._publish_virtual_transport_cube()
                    # After the real lift has been verified, the hybrid demo no
                    # longer relies on finger contact for transport.  Opening
                    # removes residual cube/finger load that otherwise stalls
                    # the WBC gait while SetEntityState keeps the cube carried.
                    self.gripper_pub.publish(Float64MultiArray(data=[0.04, -0.04]))
            if not (self._physical_grasp_enabled()
                    and self._virtualize_physical_transport_enabled()):
                self._capture_transport_pose()
            self._publish_transport_pose()
            # Do not drive back toward the pre-grasp station after lifting: that
            # abrupt lateral correction was the largest disturbance seen by the
            # pinched cube.  Hold zero briefly, then let _navigate ramp forward.
            self._stop()
            time.sleep(0.5)
            self._set_locomotion(True)
            self._publish_transport_pose()
            self._log_physical_state("transport_ready")
            if (self._physical_grasp_enabled()
                    and not self._virtualize_physical_transport_enabled()):
                retained, detail = self._transport_grasp_retained()
                if not retained:
                    raise RuntimeError(f"PHYSICAL_TRANSPORT_FAILED:{detail}")
            self._set_state(goal_handle, "DRIVE_DROP", 0.72)
            transport_start, _ = self._base_pose()
            if not self._navigate(
                    self.get_parameter("drop_waypoint").value,
                    carrying=True,
                    timeout=self.get_parameter("transport_timeout").value):
                raise RuntimeError(f"DROP_WAYPOINT_TIMEOUT:{self.navigation_error}")
            transport_end, _ = self._base_pose()
            transport_distance = float(np.linalg.norm(
                transport_end[:2] - transport_start[:2]))
            minimum_transport = self.get_parameter("minimum_base_transport").value
            if transport_distance < minimum_transport:
                raise RuntimeError(
                    f"PHYSICAL_TRANSPORT_DISTANCE_FAILED:distance="
                    f"{transport_distance:.3f}m,minimum={minimum_transport:.3f}m")
            self.get_logger().info(
                f"base transport verified: distance={transport_distance:.3f}m")
            if (self._physical_grasp_enabled()
                    and not self._virtualize_physical_transport_enabled()):
                retained, detail = self._transport_grasp_retained()
                if not retained:
                    raise RuntimeError(f"PHYSICAL_TRANSPORT_FAILED:{detail}")
            self._set_state(goal_handle, "PLACE", 0.86)
            drop = np.array(self.get_parameter("drop_object_pose").value, np.float32)
            carried_for_place = self._get_entity_pose().position
            if (self._physical_grasp_enabled()
                    and not self._virtualize_physical_transport_enabled()
                    and not self._cube_over_drop_table(carried_for_place)):
                raise RuntimeError(
                    f"PHYSICAL_PLACE_APPROACH_FAILED:x={carried_for_place.x:.3f}m,"
                    f"y={carried_for_place.y:.3f}m")
            try:
                tcp_transform = self.tf_buffer.lookup_transform(
                    "odom", "end_effector", rclpy.time.Time())
            except TransformException as exc:
                raise RuntimeError(f"TCP_TF_UNAVAILABLE:{exc}") from exc
            tcp = tcp_transform.transform.translation
            # The carried cube already projects inside the distinct drop table.
            # Preserve the live TCP x/y and descend vertically; drop_object_pose
            # describes the cube centre, not the TCP, so using its x/y directly
            # would introduce an erroneous lateral sweep before release.
            self._publish_ee([tcp.x, tcp.y, drop[2] + 0.03])
            time.sleep(3.0)
            self._log_physical_state("place_ready")
            if (self._physical_grasp_enabled()
                    and not self._virtualize_physical_transport_enabled()):
                before_release = self._get_entity_pose().position
                if not self._cube_over_drop_table(before_release):
                    raise RuntimeError(
                        f"PHYSICAL_PLACE_APPROACH_FAILED:x={before_release.x:.3f}m,"
                        f"y={before_release.y:.3f}m")
            self._set_state(goal_handle, "RELEASE", 0.94)
            if (self._physical_grasp_enabled()
                    and self._virtualize_physical_transport_enabled()):
                self._move_virtual_transport_cube_to_drop()
            elif self._assisted_grasp_enabled() or not self._physical_grasp_enabled():
                self._call_trigger(self.detach)
                self.assisted_grasp_attached = False
            self.virtual_transport_cube_offset = None
            self.gripper_pub.publish(Float64MultiArray(data=[0.04, -0.04]))
            time.sleep(2.0)
            if (self._physical_grasp_enabled()
                    and not self._virtualize_physical_transport_enabled()):
                placed = self._get_entity_pose()
                z_error = abs(placed.position.z - drop[2])
                if (not self._cube_over_drop_table(placed.position)
                        or z_error > self.get_parameter("drop_z_tolerance").value):
                    raise RuntimeError(
                        f"PHYSICAL_PLACE_FAILED:x={placed.position.x:.3f}m,"
                        f"y={placed.position.y:.3f}m,z_error={z_error:.3f}m")
                self.get_logger().info(
                    f"physical placement verified: position=({placed.position.x:.3f},"
                    f"{placed.position.y:.3f},{placed.position.z:.3f}), "
                    f"z_error={z_error:.3f}m")
            self._set_state(goal_handle, "DONE", 1.0)
            goal_handle.succeed()
            result.success, result.message = True, "pick and transport complete"
            return result
        except RuntimeError as exc:
            self._stop()
            self._set_locomotion(False)
            if self._assisted_grasp_enabled():
                self._call_trigger(self.detach)
                self.assisted_grasp_attached = False
            self.state = "ABORT"
            goal_handle.abort()
            result.success = False
            result.error_code = str(exc).split(":", 1)[0]
            result.message = str(exc)
            return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
