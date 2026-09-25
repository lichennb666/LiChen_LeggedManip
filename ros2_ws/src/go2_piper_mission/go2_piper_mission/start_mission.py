#!/usr/bin/env python3
import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from go2_piper_interfaces.action import PickTransport
from std_msgs.msg import String


class StartMission(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_start_mission")
        self.client = ActionClient(self, PickTransport, "/go2_piper/mission")
        self.timer = self.create_timer(5.0, self._send_once)
        self.sent = False
        self.wbc_ready = False
        self.create_subscription(String, "/go2_piper/wbc/status", self._wbc_status, 10)

    def _wbc_status(self, msg: String) -> None:
        try:
            self.wbc_ready = bool(json.loads(msg.data).get("ready", False))
        except json.JSONDecodeError:
            self.wbc_ready = False

    def _send_once(self) -> None:
        if self.sent or not self.wbc_ready or not self.client.wait_for_server(timeout_sec=0.1):
            return
        self.sent = True
        goal = PickTransport.Goal()
        goal.target_color = "green"
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._accepted)

    def _feedback(self, msg) -> None:
        self.get_logger().info(f"mission={msg.feedback.state} progress={msg.feedback.progress:.0%}")

    def _accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("mission goal rejected")
            rclpy.shutdown()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._done)

    def _done(self, future) -> None:
        result = future.result().result
        level = self.get_logger().info if result.success else self.get_logger().error
        level(result.message)
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StartMission()
    rclpy.spin(node)
