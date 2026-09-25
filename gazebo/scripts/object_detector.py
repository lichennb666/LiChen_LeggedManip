#!/usr/bin/env python3
"""
物体检测节点: 订阅 D435 深度相机, 检测目标物体并发布 pose.
仿真模式: 直接从 Gazebo 获取物体 ground truth 位置.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.srv import GetModelState
from visualization_msgs.msg import Marker
from std_msgs.msg import String


class ObjectDetector:
    def __init__(self):
        rospy.init_node("object_detector", anonymous=True)

        self.target_name = rospy.get_param("~target_name", "target_cube")
        self.frame_id = rospy.get_param("~frame_id", "base")
        self.use_ground_truth = rospy.get_param("~use_ground_truth", True)

        self.pose_pub = rospy.Publisher("/detected_object", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/detected_object_marker", Marker, queue_size=1)
        self.status_pub = rospy.Publisher("/object_status", String, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        if self.use_ground_truth:
            rospy.wait_for_service("/gazebo/get_model_state")
            self.get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            rospy.loginfo(f"ObjectDetector: ground truth mode (model: '{self.target_name}')")
        else:
            rospy.Subscriber("/d435_depth/depth/image_raw", Image, self._depth_cb)
            self.depth_msg = None
            rospy.loginfo("ObjectDetector: depth camera mode (/d435_depth)")

        self.run()

    def get_object_pose(self):
        """Get object ground truth pose from Gazebo."""
        try:
            resp = self.get_model(self.target_name, "world")
            pose = resp.pose
            return pose.position.x, pose.position.y, pose.position.z
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5, f"Object not found: {e}")
            return None

    def transform_to_base(self, wx, wy, wz):
        """Transform world coordinates to base frame."""
        try:
            trans = self.tf_buffer.lookup_transform("base", "world", rospy.Time(0))
            # Simple transform (ignore rotation for now)
            bx = wx - trans.transform.translation.x
            by = wy - trans.transform.translation.y
            bz = wz - trans.transform.translation.z
            return bx, by, bz
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(5, f"TF error: {e}")
            return wx, wy, wz

    def publish_marker(self, wx, wy, wz):
        """Publish RViz marker."""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = wx
        marker.pose.position.y = wy
        marker.pose.position.z = wz
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0)
        self.marker_pub.publish(marker)

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            pos = self.get_object_pose()
            if pos:
                wx, wy, wz = pos
                # Transform to base frame
                bx, by, bz = self.transform_to_base(wx, wy, wz)

                # Publish in world frame for RViz
                self.publish_marker(wx, wy, wz)

                # Publish detected pose
                msg = PoseStamped()
                msg.header.frame_id = "world"
                msg.header.stamp = rospy.Time.now()
                msg.pose.position.x = wx
                msg.pose.position.y = wy
                msg.pose.position.z = wz
                msg.pose.orientation.w = 1.0
                self.pose_pub.publish(msg)

                # Publish base-frame coords as string for grasp pipeline
                self.status_pub.publish(f"x={bx:.3f} y={by:.3f} z={bz:.3f}")
            rate.sleep()


if __name__ == "__main__":
    ObjectDetector()
