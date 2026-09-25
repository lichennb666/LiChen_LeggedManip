#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_odom_relay")
        self.declare_parameter("input_topic", "/Odometry")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("enabled", True)
        self.declare_parameter("use_sim_time", True)

        self._output_topic = str(self.get_parameter("output_topic").value)
        self._enabled = bool(self.get_parameter("enabled").value)
        self._subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._callback,
            20,
        )
        self._publisher = self.create_publisher(Odometry, self._output_topic, 20)

    def _callback(self, msg: Odometry) -> None:
        if not self._enabled:
            return
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomRelay()
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
