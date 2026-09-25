#!/usr/bin/env python3
"""Deterministic RGB-D sensor for the Gazebo Classic demo test path."""

import math
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class SyntheticCamera(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_synthetic_camera")
        self.odom = None
        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, "/demo_camera/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/demo_camera/depth/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/demo_camera/camera_info", 10)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_timer(1.0 / 15.0, self._publish)

    def _odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _publish(self) -> None:
        if self.odom is None:
            return
        height, width = 480, 640
        bgr = np.full((height, width, 3), 170, np.uint8)
        depth = np.full((height, width), np.nan, np.float32)
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        camera = np.array([p.x + 0.25 * math.cos(yaw), p.y + 0.25 * math.sin(yaw), 0.63])
        cube = np.array([0.427, -0.065, 0.531])
        distance = float(np.linalg.norm(cube - camera))
        if distance < 1.2:
            half = int(np.clip(14.0 / max(distance, 0.15), 18, 55))
            cx, cy = width // 2, height // 2
            bgr[cy-half:cy+half, cx-half:cx+half] = (0, 255, 0)
            depth[cy-half:cy+half, cx-half:cx+half] = distance
        stamp = self.get_clock().now().to_msg()
        frame = "perception_camera_optical_frame"
        rgb_msg = self.bridge.cv2_to_imgmsg(bgr, "bgr8")
        rgb_msg.header.stamp, rgb_msg.header.frame_id = stamp, frame
        depth_msg = self.bridge.cv2_to_imgmsg(depth, "32FC1")
        depth_msg.header.stamp, depth_msg.header.frame_id = stamp, frame
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, frame
        info.width, info.height = width, height
        info.k = [400.0, 0.0, 319.5, 0.0, 400.0, 239.5, 0.0, 0.0, 1.0]
        info.p = [400.0, 0.0, 319.5, 0.0, 0.0, 400.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticCamera()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
