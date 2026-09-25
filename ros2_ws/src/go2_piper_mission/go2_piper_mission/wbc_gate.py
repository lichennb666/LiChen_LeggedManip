#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class WbcGate(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_wbc_gate")
        self.declare_parameter("duration", 60.0)
        self.declare_parameter("exercise_commands", False)
        self.declare_parameter("response_command_x", 0.0)
        self.declare_parameter("response_start", 2.0)
        self.declare_parameter("response_duration", 5.0)
        self.ready = False
        self.started = None
        self.failures = []
        self.odom_samples = 0
        self.checked_samples = 0
        self.failed_samples = 0
        self.max_abs_roll = 0.0
        self.max_abs_pitch = 0.0
        self.min_height = float("inf")
        self.max_height = float("-inf")
        self.finished = False
        self.failed = False
        self.latest_odom = None
        self.response_start_pose = None
        self.response_end_pose = None
        self.min_x = self.min_y = self.min_yaw = float("inf")
        self.max_x = self.max_y = self.max_yaw = float("-inf")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ee_pub = self.create_publisher(PoseStamped, "/go2_piper/wbc/ee_target", 10)
        self.create_subscription(String, "/go2_piper/wbc/status", self._status, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_timer(0.2, self._check)

    def _status(self, msg: String) -> None:
        try:
            self.ready = bool(json.loads(msg.data).get("ready", False))
        except json.JSONDecodeError:
            self.ready = False

    def _odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_samples += 1
        if self.started is None:
            return
        self.checked_samples += 1
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        sinr = 2 * (q.w * q.x + q.y * q.z)
        cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        self.min_x, self.max_x = min(self.min_x, p.x), max(self.max_x, p.x)
        self.min_y, self.max_y = min(self.min_y, p.y), max(self.max_y, p.y)
        self.min_yaw, self.max_yaw = min(self.min_yaw, yaw), max(self.max_yaw, yaw)
        self.max_abs_roll = max(self.max_abs_roll, abs(roll))
        self.max_abs_pitch = max(self.max_abs_pitch, abs(pitch))
        self.min_height = min(self.min_height, p.z)
        self.max_height = max(self.max_height, p.z)
        reasons = []
        if abs(roll) > math.radians(15) or abs(pitch) > math.radians(15):
            reasons.append(f"tilt roll={roll:.3f} pitch={pitch:.3f}")
        if not 0.23 <= p.z <= 0.33:
            reasons.append(f"base height {p.z:.3f}")
        if reasons:
            self.failed_samples += 1
            if not self.failures:
                elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.started
                self.failures.append(f"t={elapsed:.3f}s {'; '.join(reasons)}")

    def _check(self) -> None:
        if self.finished:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.ready and self.odom_samples > 0 and self.started is None:
            self.started = now
            self.get_logger().info("WBC ready; stability gate timing started")
        if self.started is not None and bool(self.get_parameter("exercise_commands").value):
            self._publish_exercise(now - self.started)
        response_command_x = float(self.get_parameter("response_command_x").value)
        if self.started is not None and abs(response_command_x) > 1.0e-6:
            self._publish_response(now - self.started, response_command_x)
        if self.started is None or now - self.started < self.get_parameter("duration").value:
            return
        metrics = {
            "duration": float(self.get_parameter("duration").value),
            "checked_samples": self.checked_samples,
            "failed_samples": self.failed_samples,
            "max_abs_roll": round(self.max_abs_roll, 6),
            "max_abs_pitch": round(self.max_abs_pitch, 6),
            "min_height": round(self.min_height, 6),
            "max_height": round(self.max_height, 6),
            "planar_span": round(math.hypot(self.max_x - self.min_x, self.max_y - self.min_y), 6),
            "yaw_span": round(self.max_yaw - self.min_yaw, 6),
        }
        if self.response_start_pose is not None and self.response_end_pose is not None:
            start_x, start_y, start_yaw = self.response_start_pose
            end_x, end_y, end_yaw = self.response_end_pose
            metrics["response_command_x"] = response_command_x
            metrics["response_dx"] = round(end_x - start_x, 6)
            metrics["response_dy"] = round(end_y - start_y, 6)
            metrics["response_dyaw"] = round(end_yaw - start_yaw, 6)
            metrics["response_distance"] = round(
                math.hypot(end_x - start_x, end_y - start_y), 6)
        summary = json.dumps(metrics, sort_keys=True)
        if bool(self.get_parameter("exercise_commands").value):
            if math.hypot(self.max_x - self.min_x, self.max_y - self.min_y) < 0.1:
                self.failures.append("velocity exercise produced less than 0.1 m planar motion")
            if self.max_yaw - self.min_yaw < 0.05:
                self.failures.append("yaw exercise produced less than 0.05 rad rotation")
        if self.failures or self.odom_samples == 0:
            reason = self.failures[0] if self.failures else "no odometry samples"
            self.get_logger().error(f"WBC_GATE_FAIL: {reason}; metrics={summary}")
            self.failed = True
        else:
            self.get_logger().info(f"WBC_GATE_PASS: metrics={summary}")
        self.finished = True

    def _publish_response(self, elapsed: float, command_x: float) -> None:
        """Publish one constant body-x command and record its odometry response."""
        start = float(self.get_parameter("response_start").value)
        end = start + float(self.get_parameter("response_duration").value)
        command = Twist()
        if start <= elapsed < end:
            command.linear.x = command_x
            if self.response_start_pose is None and self.latest_odom is not None:
                self.response_start_pose = self._planar_pose(self.latest_odom)
        elif elapsed >= end and self.response_start_pose is not None and self.latest_odom is not None:
            self.response_end_pose = self._planar_pose(self.latest_odom)
        self.cmd_pub.publish(command)

    @staticmethod
    def _planar_pose(msg: Odometry) -> tuple[float, float, float]:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        return p.x, p.y, yaw

    def _publish_exercise(self, elapsed: float) -> None:
        """Exercise in-distribution velocity and mixed-frame EE commands."""
        command = Twist()
        if 4.0 <= elapsed < 10.0:
            command.linear.x = 0.12
        elif 13.0 <= elapsed < 19.0:
            command.linear.y = 0.10
        elif 22.0 <= elapsed < 28.0:
            command.angular.z = 0.12
        self.cmd_pub.publish(command)

        if elapsed < 31.0 or self.latest_odom is None:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        base = self.latest_odom.pose.pose
        q = base.orientation
        x_ref, y_ref, z_world = (
            # Keep the exercise above the 0.506 m work surfaces.  The former
            # z=0.42 command drove the gripper into the pickup table and tested
            # collision recovery rather than free-space WBC stability.
            (0.40, -0.05, 0.55) if elapsed < 36.0 else (0.45, 0.05, 0.60)
        )
        rotation = self._rotation_matrix(q)
        # Commands use link0-frame XY but world-frame Z. Solve the third
        # link0-frame component so the generated PoseStamped is exactly the
        # requested mixed-frame command even while the base is tilted.
        link0 = [
            base.position.x + 0.06 * rotation[0][2],
            base.position.y + 0.06 * rotation[1][2],
            base.position.z + 0.06 * rotation[2][2],
        ]
        dz_ref = (
            z_world - link0[2]
            - rotation[2][0] * x_ref - rotation[2][1] * y_ref
        ) / rotation[2][2]
        pose.pose.position.x = link0[0] + sum(
            rotation[0][i] * value for i, value in enumerate((x_ref, y_ref, dz_ref)))
        pose.pose.position.y = link0[1] + sum(
            rotation[1][i] * value for i, value in enumerate((x_ref, y_ref, dz_ref)))
        pose.pose.position.z = z_world
        # Identity orientation relative to link0/base, expressed in odom.
        pose.pose.orientation = q
        self.ee_pub.publish(pose)

    @staticmethod
    def _rotation_matrix(q) -> list[list[float]]:
        return [
            [1 - 2 * (q.y * q.y + q.z * q.z),
             2 * (q.x * q.y - q.z * q.w),
             2 * (q.x * q.z + q.y * q.w)],
            [2 * (q.x * q.y + q.z * q.w),
             1 - 2 * (q.x * q.x + q.z * q.z),
             2 * (q.y * q.z - q.x * q.w)],
            [2 * (q.x * q.z - q.y * q.w),
             2 * (q.y * q.z + q.x * q.w),
             1 - 2 * (q.x * q.x + q.y * q.y)],
        ]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WbcGate()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.5)
    failed = node.failed
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if failed:
        raise SystemExit(1)
