#!/usr/bin/env python3
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException

from go2_piper_interfaces.msg import DetectedObject
from .detector_core import StabilityFilter, detect_colored_object


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = max(x*x + y*y + z*z + w*w, 1.0e-12)
    s = 2.0 / n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)],
    ], np.float32)


class ColorDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_piper_color_detector")
        defaults = {
            "rgb_topic": "/d435/camera/image_raw",
            "depth_topic": "/d435/camera/depth/image_raw",
            "info_topic": "/d435/camera/camera_info",
            "output_frame": "odom",
            "min_area": 250,
            "h_min": 45, "h_max": 85, "s_min": 100, "v_min": 80,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.stability = StabilityFilter()
        self.publisher = self.create_publisher(DetectedObject, "/go2_piper/perception/detection", 10)
        self.debug_image_pub = self.create_publisher(Image, "/go2_piper/perception/debug_image", 2)
        self.debug_mask_pub = self.create_publisher(Image, "/go2_piper/perception/debug_mask", 2)
        # gazebo_ros_camera lazily creates its RGB publisher for a reliable
        # subscriber, so use the default reliable QoS for all synchronized inputs.
        rgb = Subscriber(self, Image, self.get_parameter("rgb_topic").value)
        depth = Subscriber(self, Image, self.get_parameter("depth_topic").value)
        info = Subscriber(self, CameraInfo, self.get_parameter("info_topic").value)
        sync = ApproximateTimeSynchronizer([rgb, depth, info], queue_size=10, slop=0.05)
        sync.registerCallback(self._callback)
        self.sync = sync

    def _callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        k = np.asarray(info_msg.k, np.float32).reshape(3, 3)
        result = detect_colored_object(
            bgr, depth, k,
            (self.get_parameter("h_min").value, self.get_parameter("s_min").value, self.get_parameter("v_min").value),
            (self.get_parameter("h_max").value, 255, 255), self.get_parameter("min_area").value)
        if result is None:
            return
        output_frame = self.get_parameter("output_frame").value
        try:
            transform = self.tf_buffer.lookup_transform(output_frame, rgb_msg.header.frame_id, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warning(f"TF unavailable: {exc}", throttle_duration_sec=2.0)
            return
        t, q = transform.transform.translation, transform.transform.rotation
        point_world = quaternion_matrix(q.x, q.y, q.z, q.w) @ result.point_camera + np.array([t.x, t.y, t.z])
        stable, mean, std = self.stability.update(point_world)
        debug = bgr.copy()
        cv2.circle(debug, result.centroid, 6, (0, 0, 255), 2)
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        self.debug_mask_pub.publish(self.bridge.cv2_to_imgmsg(result.mask, "mono8"))
        if not stable:
            return
        msg = DetectedObject()
        msg.header = rgb_msg.header
        msg.header.frame_id = output_frame
        msg.color = "green"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, mean)
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = float(std[0] ** 2)
        msg.pose.covariance[7] = float(std[1] ** 2)
        msg.pose.covariance[14] = float(std[2] ** 2)
        msg.confidence = result.confidence
        msg.pixel_area = result.pixel_area
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorDetectorNode()
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
