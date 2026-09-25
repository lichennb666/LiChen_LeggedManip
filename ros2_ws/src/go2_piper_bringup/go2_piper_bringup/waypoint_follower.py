#!/usr/bin/env python3
"""Simple waypoint follower for Gazebo navigation validation.

Publishes /cmd_vel toward a list of [x, y] waypoints using the same
waypoint_command helper as the grasp-transport mission.  Used to baseline
locomotion tracking before/after the Gazebo-domain fine-tune.
"""

from __future__ import annotations

import math
import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from go2_piper_mission.mission_core import waypoint_command, wrap_angle


class WaypointFollower(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_waypoint_follower")
        self.declare_parameter("waypoints", [0.0, 0.0])
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 0.25)
        self.declare_parameter("position_tolerance", 0.08)
        self.declare_parameter("yaw_gain", 1.2)
        flat = [float(v) for v in self.get_parameter("waypoints").value]
        if len(flat) % 2 != 0:
            raise RuntimeError("waypoint_follower: waypoints must be [x0,y0,x1,y1,...]")
        self.waypoints = [
            [flat[i], flat[i + 1]] for i in range(0, len(flat), 2)
        ]
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.target_index = 0
        self.odom = None
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_timer(0.05, self._update)
        if not self.waypoints:
            raise RuntimeError("waypoint_follower: no waypoints given")
        self.get_logger().info(f"waypoints={self.waypoints}")

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _update(self) -> None:
        if self.odom is None:
            return
        if self.target_index >= len(self.waypoints):
            self._stop()
            return
        position = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        current_xy = np.array([position.x, position.y], np.float32)
        goal = np.array(self.waypoints[self.target_index][:2], np.float32)
        velocity, distance = waypoint_command(current_xy, yaw, goal, self.max_linear)
        if distance <= self.position_tolerance:
            self.get_logger().info(
                f"waypoint {self.target_index} reached: "
                f"goal=({goal[0]:.2f},{goal[1]:.2f}) pose=({position.x:.2f},"
                f"{position.y:.2f})")
            self.target_index += 1
            if self.target_index >= len(self.waypoints):
                self.get_logger().info("ALL WAYPOINTS REACHED")
                self._stop()
                return
            goal = np.array(self.waypoints[self.target_index][:2], np.float32)
            velocity, distance = waypoint_command(current_xy, yaw, goal, self.max_linear)
        desired_yaw = math.atan2(goal[1] - current_xy[1], goal[0] - current_xy[0])
        command = Twist()
        command.linear.x = float(velocity[0])
        command.linear.y = float(velocity[1])
        command.angular.z = float(np.clip(
            self.yaw_gain * wrap_angle(desired_yaw - yaw),
            -self.max_angular, self.max_angular))
        self.publisher.publish(command)

    def _stop(self) -> None:
        self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
