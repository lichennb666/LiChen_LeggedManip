#!/usr/bin/env python3
"""
Gazebo D435 相机可视化: 订阅 /d435_rgb/color/image_raw, OpenCV 显示.
用法: python camera_viewer.py
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class D435Viewer:
    def __init__(self):
        rospy.init_node("d435_viewer", anonymous=True)
        self.bridge = CvBridge()
        self.rgb_sub = rospy.Subscriber("/d435_rgb/color/image_raw", Image, self._rgb_cb)
        self.depth_sub = rospy.Subscriber("/d435_depth/depth/image_raw", Image, self._depth_cb)
        self.object_sub = rospy.Subscriber("/detected_object_marker", rospy.AnyMsg, lambda _: None)
        self.latest_rgb = None
        self.latest_depth = None
        rospy.loginfo("D435 Viewer started. Showing camera feed...")

    def _rgb_cb(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.latest_rgb is not None:
                display = self.latest_rgb.copy()
                # Overlay detection info
                h, w = display.shape[:2]
                cv2.putText(display, "D435 RGB", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display, "Press Q to quit", (10, h - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                if self.latest_depth is not None:
                    depth_norm = cv2.normalize(self.latest_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                    depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
                    depth_small = cv2.resize(depth_color, (w // 3, h // 3))
                    display[0:h//3, 0:w//3] = depth_small
                    cv2.putText(display, "Depth", (5, 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                cv2.imshow("Go2+Piper D435 Camera", display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    D435Viewer().run()
