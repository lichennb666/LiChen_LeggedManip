#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from go2_piper_interfaces.action import PickTransport
from rclpy.action import ActionClient
from rclpy.node import Node


class Course1GoalGateway(Node):
    def __init__(self) -> None:
        super().__init__("course1_goal_gateway")
        self.declare_parameter("input_goal_topic", "/move_base_simple/goal")
        self.declare_parameter("navigation_goal_topic", "/course1/navigation_goal")
        self.declare_parameter("capture_min_x", 0.05)
        self.declare_parameter("capture_max_x", 1.10)
        self.declare_parameter("capture_min_y", -0.55)
        self.declare_parameter("capture_max_y", 0.45)
        self.declare_parameter("mission_color", "green")
        self.declare_parameter("mission_rearm_delay", 2.5)
        self.declare_parameter("use_mission", True)

        self.input_goal_topic = str(self.get_parameter("input_goal_topic").value)
        self.navigation_goal_topic = str(self.get_parameter("navigation_goal_topic").value)
        self.capture_min_x = float(self.get_parameter("capture_min_x").value)
        self.capture_max_x = float(self.get_parameter("capture_max_x").value)
        self.capture_min_y = float(self.get_parameter("capture_min_y").value)
        self.capture_max_y = float(self.get_parameter("capture_max_y").value)
        self.mission_color = str(self.get_parameter("mission_color").value)
        self.mission_rearm_delay = float(self.get_parameter("mission_rearm_delay").value)
        self.use_mission = bool(self.get_parameter("use_mission").value)

        self.navigation_goal_pub = self.create_publisher(PoseStamped, self.navigation_goal_topic, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.input_goal_topic,
            self._on_goal,
            10,
        )

        self.client = ActionClient(self, PickTransport, "/go2_piper/mission")
        self._mission_active = False
        self._mission_pending = False
        self._mission_completed_at = 0.0

    def _mission_ready(self, now: float) -> bool:
        if self._mission_pending or self._mission_active:
            return False
        return now >= self._mission_completed_at + self.mission_rearm_delay

    def _in_capture_zone(self, msg: PoseStamped) -> bool:
        x, y = msg.pose.position.x, msg.pose.position.y
        return self.capture_min_x <= x <= self.capture_max_x and self.capture_min_y <= y <= self.capture_max_y

    def _send_mission(self) -> bool:
        if self._mission_pending:
            return True
        if not self.client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning("mission action server not ready; forward goal to navigation")
            return False
        if not self.use_mission:
            return False
        goal = PickTransport.Goal()
        goal.target_color = self.mission_color
        self._mission_pending = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._mission_accepted)
        return True

    def _mission_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("mission goal rejected")
            self._mission_pending = False
            return
        self._mission_active = True
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._mission_done)

    def _mission_done(self, future) -> None:
        try:
            _ = future.result()
            self.get_logger().info("mission finished")
        except Exception as exc:
            self.get_logger().error(f"mission failed: {exc}")
        self._mission_pending = False
        self._mission_active = False
        self._mission_completed_at = self.get_clock().now().nanoseconds * 1e-9

    def _on_goal(self, msg: PoseStamped) -> None:
        if self.use_mission and self._in_capture_zone(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._mission_ready(now):
                self.get_logger().info("goal in capture zone; trigger capture mission")
                if self._send_mission():
                    return
            self.get_logger().info("capture-zone goal blocked by mission state/cooldown; forwarding to navigation")
            self.navigation_goal_pub.publish(msg)
            return
        self.navigation_goal_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Course1GoalGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
