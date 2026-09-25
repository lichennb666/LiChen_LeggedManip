#!/usr/bin/env python3
"""
Go2+Piper 完整抓取演示 (Gazebo + MoveIt + OpenCV)
===================================================
启动步骤:
  1. roslaunch go2_piper_description full_stack.launch
  2. python gazebo/scripts/spawn_table.py --x 2.0 --y -1.0
  3. python gazebo/scripts/camera_viewer.py          (可选, 看相机画面)
  4. python gazebo/scripts/grasp_demo.py             (本脚本)

流程:
  检测物体 → 计算抓取位姿 → MoveIt规划 → 执行 → 夹爪闭合
"""

import sys, os, copy, math
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import tf2_ros
from tf.transformations import quaternion_from_euler


class GraspDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("grasp_demo", anonymous=True)

        # MoveIt
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_planner_id("RRTConnect")
        self.arm.set_planning_time(3.0)
        self.arm.set_num_planning_attempts(20)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.allow_replanning(True)
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.3)

        # Gripper
        self.grip7 = rospy.Publisher("/go2_piper_gazebo/joint7_controller/command", Float64MultiArray, queue_size=1)
        self.grip8 = rospy.Publisher("/go2_piper_gazebo/joint8_controller/command", Float64MultiArray, queue_size=1)

        # Object detection
        self.object_pose = None
        rospy.Subscriber("/detected_object", PoseStamped, self._obj_cb)
        rospy.Subscriber("/object_status", String, self._status_cb)

        # Visualization markers
        self.marker_pub = rospy.Publisher("/grasp_target_marker", Marker, queue_size=1)

        print("=" * 60)
        print("  Go2+Piper Grasp Demo (MoveIt + Gazebo)")
        print("  等待物体检测 /detected_object...")
        print("=" * 60)

    def _obj_cb(self, msg):
        self.object_pose = msg

    def _status_cb(self, msg):
        rospy.loginfo(f"Object status: {msg.data}")

    def open_gripper(self):
        self.grip7.publish(Float64MultiArray(data=[0.05]))
        self.grip8.publish(Float64MultiArray(data=[0.0]))
        rospy.sleep(0.4)

    def close_gripper(self):
        self.grip7.publish(Float64MultiArray(data=[0.0]))
        self.grip8.publish(Float64MultiArray(data=[-0.05]))
        rospy.sleep(0.4)

    def publish_marker(self, x, y, z):
        m = Marker()
        m.header.frame_id = "base"
        m.header.stamp = rospy.Time.now()
        m.ns = "grasp"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = geometry_msgs.msg.Point(x, y, z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.03
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0
        m.lifetime = rospy.Duration(5.0)
        self.marker_pub.publish(m)

    def wait_for_object(self, timeout=30):
        start = rospy.Time.now()
        while self.object_pose is None:
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logerr("Timeout waiting for object detection")
                return False
            rospy.loginfo_throttle(2, "Waiting for object...")
            rospy.sleep(0.2)
        return True

    def plan_to_pose(self, x, y, z, roll=0, pitch=0, yaw=0):
        """Plan arm to target pose in base frame."""
        target = geometry_msgs.msg.Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        target.orientation.x = q[0]
        target.orientation.y = q[1]
        target.orientation.z = q[2]
        target.orientation.w = q[3]

        self.publish_marker(x, y, z)
        self.arm.set_pose_target(target)

        rospy.loginfo(f"[PLAN] Target: ({x:.3f}, {y:.3f}, {z:.3f})")
        plan = self.arm.plan()
        if not plan or not plan.joint_trajectory.points:
            rospy.logerr("[PLAN] Failed!")
            return False

        rospy.loginfo(f"[PLAN] OK - {len(plan.joint_trajectory.points)} waypoints, "
                      f"time={plan.joint_trajectory.points[-1].time_from_start.to_sec():.1f}s")
        return self.arm.execute(plan, wait=True)

    def cartesian_step(self, dz):
        """Move arm down by dz meters along Z."""
        current = self.arm.get_current_pose().pose
        waypoints = [copy.deepcopy(current)]
        waypoints[0].position.z += dz

        rospy.loginfo(f"[CARTESIAN] dz={dz:.3f}m")
        plan, fraction = self.arm.compute_cartesian_path(waypoints, eef_step=0.005, jump_threshold=0.0)
        rospy.loginfo(f"[CARTESIAN] path fraction={fraction:.1%}")
        if fraction > 0.8 and plan.joint_trajectory.points:
            return self.arm.execute(plan, wait=True)
        return False

    def go_home(self):
        self.arm.set_named_target("home")
        return self.arm.go(wait=True)

    def run(self):
        """Main grasp sequence."""
        # 1. Go home first
        print("\n━━━ STEP 1: Go Home ━━━")
        self.go_home()
        self.open_gripper()

        # 2. Wait for object
        print("\n━━━ STEP 2: Detect Object ━━━")
        if not self.wait_for_object():
            return

        obj = self.object_pose.pose.position
        print(f"  Object world pos: ({obj.x:.3f}, {obj.y:.3f}, {obj.z:.3f})")

        # 3. Convert world to base frame (approximate)
        # The robot base is at approximately the same as world frame
        bx, by, bz = obj.x, obj.y, obj.z
        print(f"  Object base pos:  ({bx:.3f}, {by:.3f}, {bz:.3f})")

        # 4. Approach from above
        print("\n━━━ STEP 3: Approach ━━━")
        if not self.plan_to_pose(bx, by, bz + 0.12, roll=0, pitch=1.2, yaw=0):
            print("  Approach failed, trying alternative...")
            # Try from a different angle
            self.plan_to_pose(bx + 0.05, by, bz + 0.15, roll=0, pitch=1.0, yaw=0.2)
        rospy.sleep(1.0)

        # 5. Descend
        print("\n━━━ STEP 4: Descend ━━━")
        self.cartesian_step(-0.08)
        rospy.sleep(0.5)

        # 6. Grasp
        print("\n━━━ STEP 5: Grasp! ━━━")
        self.close_gripper()
        rospy.sleep(1.0)

        # 7. Lift
        print("\n━━━ STEP 6: Lift ━━━")
        self.cartesian_step(0.15)
        rospy.sleep(1.0)

        print("\n━━━ DONE ━━━")
        print("  Object grasped! Press Ctrl+C to exit.")


if __name__ == "__main__":
    demo = GraspDemo()
    demo.run()
    rospy.spin()
