#!/usr/bin/env python3
"""Keyboard locomotion and manual fixed-joint grasp for Go2-Piper."""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


BINDINGS = {
    "w": (0.30, 0.00, 0.00),
    "s": (-0.30, 0.00, 0.00),
    "a": (0.00, 0.30, 0.00),
    "d": (0.00, -0.30, 0.00),
    "q": (0.00, 0.00, 0.80),
    "e": (0.00, 0.00, -0.80),
}
OPEN_GRIPPER = [0.040, -0.040]
CLOSED_GRIPPER = [0.004, -0.004]
PUBLISH_PERIOD = 0.05
KEY_HOLD_TIMEOUT = 0.25
ATTACH_DELAY = 1.25
EE_STEP = 0.02

HELP = """\
Go2+Piper manual grasp (Gazebo + WBC)
  W / S    forward / backward
  A / D    strafe left / right
  Q / E    turn left / right
           walk speed: 0.30 m/s, yaw speed: 0.80 rad/s
  X        stop walking
  I / K    arm forward / backward
  J / L    arm left / right
  U / O    arm up / down
  G        close gripper, then attach on double-finger contact
  F        retry attach without changing the gripper
  R        detach object and open gripper
  Ctrl+C   quit
"""


class TeleopKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_teleop_keyboard")
        self.velocity_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, "/go2_piper/gripper/target", 10)
        self.ee_pub = self.create_publisher(
            PoseStamped, "/go2_piper/wbc/ee_target", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.attach_client = self.create_client(
            Trigger, "/go2_piper/grasp/attach")
        self.detach_client = self.create_client(
            Trigger, "/go2_piper/grasp/detach")
        self.command = (0.0, 0.0, 0.0)
        self.gripper_target = OPEN_GRIPPER.copy()
        self.last_key_time = 0.0
        self.attach_at: float | None = None
        self.service_future = None
        self.ee_target: PoseStamped | None = None

    def handle_key(self, character: str) -> None:
        if character in BINDINGS:
            self.command = BINDINGS[character]
            self.last_key_time = time.monotonic()
            # Once walking starts, let the WBC return the arm to its natural
            # body-relative pose instead of pinning the gripper in odom.
            self.ee_target = None
        elif character == "x":
            self.command = (0.0, 0.0, 0.0)
            self.last_key_time = time.monotonic()
        elif character == "g":
            self.gripper_target = CLOSED_GRIPPER.copy()
            self.attach_at = time.monotonic() + ATTACH_DELAY
            self.get_logger().info(
                "closing gripper; fixed-joint request will follow")
        elif character == "r":
            self.attach_at = None
            self.gripper_target = OPEN_GRIPPER.copy()
            self._call_service(self.detach_client, "detach")
        elif character == "f":
            self._call_service(self.attach_client, "attach")
        elif character in {
                "i", "k", "j", "l", "u", "o"}:
            increments = {
                "i": (EE_STEP, 0.0, 0.0),
                "k": (-EE_STEP, 0.0, 0.0),
                "j": (0.0, EE_STEP, 0.0),
                "l": (0.0, -EE_STEP, 0.0),
                "u": (0.0, 0.0, EE_STEP),
                "o": (0.0, 0.0, -EE_STEP),
            }
            self._move_ee(*increments[character])

    def _move_ee(self, dx: float, dy: float, dz: float) -> None:
        if self.ee_target is None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "odom", "end_effector", rclpy.time.Time())
            except TransformException as error:
                self.get_logger().error(f"end-effector transform unavailable: {error}")
                return
            self.ee_target = PoseStamped()
            self.ee_target.header.frame_id = "odom"
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.ee_target.pose.position.x = translation.x
            self.ee_target.pose.position.y = translation.y
            self.ee_target.pose.position.z = translation.z
            self.ee_target.pose.orientation = rotation
        self.ee_target.pose.position.x += dx
        self.ee_target.pose.position.y += dy
        self.ee_target.pose.position.z += dz
        self.get_logger().info(
            "arm target: "
            f"x={self.ee_target.pose.position.x:.3f}, "
            f"y={self.ee_target.pose.position.y:.3f}, "
            f"z={self.ee_target.pose.position.z:.3f}")

    def _call_service(self, client, action: str) -> None:
        if self.service_future is not None and not self.service_future.done():
            self.get_logger().warning("another grasp request is still pending")
            return
        if not client.service_is_ready():
            self.get_logger().error(f"grasp {action} service is not ready")
            return
        future = client.call_async(Trigger.Request())
        self.service_future = future

        def report(result_future) -> None:
            try:
                response = result_future.result()
                log = self.get_logger().info if response.success else self.get_logger().error
                log(f"grasp {action}: {response.message}")
            except Exception as error:  # pragma: no cover - ROS transport failure
                self.get_logger().error(f"grasp {action} failed: {error}")

        future.add_done_callback(report)

    def publish(self) -> None:
        now = time.monotonic()
        if now - self.last_key_time > KEY_HOLD_TIMEOUT:
            self.command = (0.0, 0.0, 0.0)
        if self.attach_at is not None and now >= self.attach_at:
            self.attach_at = None
            self._call_service(self.attach_client, "attach")

        velocity = Twist()
        velocity.linear.x = float(self.command[0])
        velocity.linear.y = float(self.command[1])
        velocity.angular.z = float(self.command[2])
        self.velocity_pub.publish(velocity)
        self.gripper_pub.publish(Float64MultiArray(data=self.gripper_target))
        if self.ee_target is not None:
            self.ee_target.header.stamp = self.get_clock().now().to_msg()
            self.ee_pub.publish(self.ee_target)


def main(args=None) -> int:
    tty_fd = None
    if sys.stdin.isatty():
        tty_fd = sys.stdin.fileno()
    else:
        try:
            tty_fd = os.open("/dev/tty", os.O_RDONLY | os.O_NONBLOCK)
        except OSError as error:
            print(
                f"ERROR: teleop_keyboard requires an interactive terminal: {error}",
                file=sys.stderr)
            return 1
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.get_logger().info(HELP)
    old_settings = termios.tcgetattr(tty_fd)
    try:
        tty.setcbreak(tty_fd)
        while rclpy.ok():
            readable, _, _ = select.select([tty_fd], [], [], 0.0)
            if readable:
                data = os.read(tty_fd, 256).decode(errors="ignore")
                for character in data.lower():
                    node.handle_key(character)
            node.publish()
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(PUBLISH_PERIOD)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_settings)
        if tty_fd != sys.stdin.fileno():
            os.close(tty_fd)
        node.command = (0.0, 0.0, 0.0)
        node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
