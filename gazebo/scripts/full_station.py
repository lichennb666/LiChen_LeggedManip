#!/usr/bin/env python3
"""
全自动导航+抓取主调度节点.

状态机:
  IDLE → NAVIGATE → APPROACH → GRASP → CARRY → RELEASE → IDLE

触发方式:
  # 方式1: 发布 /mission_goal
  rostopic pub /mission_goal geometry_msgs/Point "x: 2.0, y: 1.0, z: 0.3"

  # 方式2: 命令行指定目标
  python full_station.py --x 2.0 --y 1.0
"""

import sys
import time
import threading
import rospy
import actionlib
import numpy as np
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import Float64MultiArray, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class Go2PiperFullStation:
    """完整导航+抓取自主任务."""

    STATE_IDLE = "IDLE"
    STATE_NAVIGATE = "NAVIGATE"
    STATE_APPROACH = "APPROACH"
    STATE_GRASP = "GRASP"
    STATE_CARRY = "CARRY"
    STATE_RELEASE = "RELEASE"

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("go2_piper_station", anonymous=True)

        # ── MoveIt ──
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.arm.set_planner_id("RRTConnect")
        self.arm.set_planning_time(2.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_goal_position_tolerance(0.02)
        self.arm.allow_replanning(True)

        # ── Publishers ──
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.joint7_pub = rospy.Publisher(
            "/go2_piper_gazebo/joint7_controller/command", Float64MultiArray, queue_size=1)
        self.joint8_pub = rospy.Publisher(
            "/go2_piper_gazebo/joint8_controller/command", Float64MultiArray, queue_size=1)
        self.mission_status_pub = rospy.Publisher("/mission_status", String, queue_size=1)

        # ── Subscribers ──
        self.odom_sub = rospy.Subscriber("/Odometry", Odometry, self._odom_cb)
        self.object_sub = rospy.Subscriber("/detected_object", PoseStamped, self._object_cb)
        self.goal_sub = rospy.Subscriber("/mission_goal", Point, self._goal_cb)

        # ── State ──
        self.state = self.STATE_IDLE
        self.current_odom = None
        self.target_goal = None       # Navigation goal (world frame)
        self.object_pose = None       # Detected object pose (world frame)
        self.drop_location = None

        # ── Config ──
        self.nav_tolerance = 0.3      # m, distance to consider "arrived"
        self.grasp_height_offset = 0.15  # Approach above object
        self.carry_drop_point = Point(0, 0, 0)  # base frame origin

        rospy.loginfo("Full Station initialized. Send /mission_goal to start.")
        self._run()

    def _odom_cb(self, msg):
        self.current_odom = msg

    def _object_cb(self, msg):
        self.object_pose = msg

    def _goal_cb(self, msg):
        rospy.loginfo(f"Received mission goal: x={msg.x:.2f} y={msg.y:.2f}")
        self.target_goal = msg
        if self.state == self.STATE_IDLE:
            self.state = self.STATE_NAVIGATE

    def _get_robot_pos(self):
        """Get current robot position in world frame from odometry."""
        if self.current_odom is None:
            return None
        p = self.current_odom.pose.pose.position
        return np.array([p.x, p.y, p.z])

    def _dist_to_goal(self):
        """Distance from robot to navigation goal."""
        pos = self._get_robot_pos()
        if pos is None or self.target_goal is None:
            return float('inf')
        g = np.array([self.target_goal.x, self.target_goal.y, 0.0])
        return np.linalg.norm(pos[:2] - g[:2])

    def _open_gripper(self):
        self.joint7_pub.publish(Float64MultiArray(data=[0.05]))
        self.joint8_pub.publish(Float64MultiArray(data=[0.0]))
        rospy.sleep(0.3)

    def _close_gripper(self):
        self.joint7_pub.publish(Float64MultiArray(data=[0.0]))
        self.joint8_pub.publish(Float64MultiArray(data=[-0.05]))
        rospy.sleep(0.3)

    def _go_to_pose(self, x, y, z, roll=0, pitch=0, yaw=0):
        """Move arm end-effector to pose in base frame."""
        from tf.transformations import quaternion_from_euler
        target = geometry_msgs.msg.Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        target.orientation.x = q[0]
        target.orientation.y = q[1]
        target.orientation.z = q[2]
        target.orientation.w = q[3]
        self.arm.set_pose_target(target)
        return self.arm.go(wait=True)

    def _world_to_base(self, wx, wy, wz):
        """Approximate world-to-base transform using odometry."""
        pos = self._get_robot_pos()
        if pos is None:
            return wx, wy, wz
        return wx - pos[0], wy - pos[1], wz

    # ═══════════════════════════════════════════════════
    #  STATE MACHINE
    # ═══════════════════════════════════════════════════

    def _run(self):
        """Main state machine loop."""
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            # ── IDLE: wait for mission goal ──
            if self.state == self.STATE_IDLE:
                self._do_idle()

            # ── NAVIGATE: drive to target area ──
            elif self.state == self.STATE_NAVIGATE:
                self._do_navigate()

            # ── APPROACH: detect object, move arm to pre-grasp ──
            elif self.state == self.STATE_APPROACH:
                self._do_approach()

            # ── GRASP: descend, close gripper ──
            elif self.state == self.STATE_GRASP:
                self._do_grasp()

            # ── CARRY: navigate to drop point ──
            elif self.state == self.STATE_CARRY:
                self._do_carry()

            # ── RELEASE: open gripper ──
            elif self.state == self.STATE_RELEASE:
                self._do_release()

            rate.sleep()

    def _do_idle(self):
        self.mission_status_pub.publish("IDLE")

    def _do_navigate(self):
        self.mission_status_pub.publish("NAVIGATE")
        dist = self._dist_to_goal()

        if dist < self.nav_tolerance:
            rospy.loginfo(f"Arrived at goal (dist={dist:.2f}m)")
            self.cmd_pub.publish(Twist())  # stop
            self.state = self.STATE_APPROACH
            return

        # Simple P-controller to drive toward goal
        pos = self._get_robot_pos()
        g = np.array([self.target_goal.x, self.target_goal.y])
        direction = g - pos[:2]
        direction = direction / max(np.linalg.norm(direction), 0.01)

        twist = Twist()
        twist.linear.x = min(0.5, dist) * direction[0]
        twist.linear.y = min(0.5, dist) * direction[1]
        self.cmd_pub.publish(twist)

    def _do_approach(self):
        self.mission_status_pub.publish("APPROACH")
        self.cmd_pub.publish(Twist())  # stop walking

        # Wait for object detection
        if self.object_pose is None:
            rospy.loginfo_throttle(2, "Waiting for object detection...")
            return

        # Move arm to pre-grasp position
        obj = self.object_pose.pose.position
        bx, by, bz = self._world_to_base(obj.x, obj.y, obj.z)
        rospy.loginfo(f"Approaching object at base frame: ({bx:.2f}, {by:.2f}, {bz:.2f})")
        self._go_to_pose(bx, by, bz + self.grasp_height_offset)
        rospy.sleep(1.0)
        self.state = self.STATE_GRASP

    def _do_grasp(self):
        self.mission_status_pub.publish("GRASP")

        # Open gripper, descend, close
        self._open_gripper()
        rospy.sleep(0.5)

        # Cartesian descent
        current = self.arm.get_current_pose().pose
        target = geometry_msgs.msg.Pose()
        target.position = current.position
        target.position.z -= 0.1  # descend 10cm
        target.orientation = current.orientation

        self.arm.set_pose_target(target)
        self.arm.go(wait=True)
        rospy.sleep(0.5)

        self._close_gripper()
        rospy.sleep(0.5)

        rospy.loginfo("Object grasped!")
        self.state = self.STATE_CARRY

    def _do_carry(self):
        self.mission_status_pub.publish("CARRY")

        # Navigate to drop position
        # (simplified: just rotate 180 degrees and walk forward)
        if self.drop_location is not None:
            self.target_goal = self.drop_location
            self.state = self.STATE_NAVIGATE
            return

        # Default: no drop, just finish
        self.state = self.STATE_RELEASE

    def _do_release(self):
        self.mission_status_pub.publish("RELEASE")
        self._open_gripper()
        rospy.sleep(1.0)

        # Go home
        self.arm.set_named_target("home")
        self.arm.go(wait=True)

        rospy.loginfo("Mission complete!")
        self.state = self.STATE_IDLE


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, help="Goal X in world frame")
    ap.add_argument("--y", type=float, help="Goal Y in world frame")
    args = ap.parse_args()

    station = Go2PiperFullStation()

    if args.x is not None and args.y is not None:
        rospy.sleep(2.0)  # wait for init
        goal = Point(args.x, args.y, 0.3)
        station._goal_cb(goal)
