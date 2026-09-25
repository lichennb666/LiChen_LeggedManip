#!/usr/bin/env python3
"""
Go2+Piper 抓取管道: MoveIt 规划 + Gazebo 执行

用法:
  # 移动到预设位姿
  python grasp_pipeline.py --target home

  # 抓取位姿 (末端到指定位置 + 闭合夹爪)
  python grasp_pipeline.py --target grasp --x 0.4 --y 0.0 --z 0.2

  # 笛卡尔直线路径
  python grasp_pipeline.py --cartesian --dx 0.2 --dz -0.1
  
必须条件: roslaunch go2_piper_moveit_config gazebo_moveit.launch
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import Float64MultiArray


class Go2PiperGraspPipeline:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("go2_piper_grasp", anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # Configure arm planner
        self.arm_group.set_planner_id("RRTConnect")
        self.arm_group.set_planning_time(2.0)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_goal_position_tolerance(0.01)
        self.arm_group.set_goal_orientation_tolerance(0.05)
        self.arm_group.allow_replanning(True)
        
        # Gripper publisher (direct joint commands, bypass MoveIt for speed)
        self.gripper_pub = rospy.Publisher(
            "/go2_piper_gazebo/joint7_controller/command",
            Float64MultiArray, queue_size=10
        )
        self.gripper_pub2 = rospy.Publisher(
            "/go2_piper_gazebo/joint8_controller/command",
            Float64MultiArray, queue_size=10
        )
        
        rospy.loginfo("Go2+Piper Grasp Pipeline initialized")
        rospy.loginfo(f"  Arm planning frame: {self.arm_group.get_planning_frame()}")
        rospy.loginfo(f"  End effector link: {self.arm_group.get_end_effector_link()}")
    
    def go_home(self):
        """Move arm to home position."""
        self.arm_group.set_named_target("home")
        return self.arm_group.go(wait=True)
    
    def go_to_pose(self, x, y, z, roll=0, pitch=0, yaw=0):
        """Move end effector to target pose (in base frame)."""
        target = geometry_msgs.msg.Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        # Convert RPY to quaternion
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(roll, pitch, yaw)
        target.orientation.x = q[0]
        target.orientation.y = q[1]
        target.orientation.z = q[2]
        target.orientation.w = q[3]
        
        self.arm_group.set_pose_target(target)
        return self.arm_group.go(wait=True)
    
    def cartesian_path(self, waypoints_delta):
        """Move along a Cartesian path.
        
        Args:
            waypoints_delta: list of [dx, dy, dz] deltas from current pose
        """
        current = self.arm_group.get_current_pose().pose
        waypoints = []
        for dx, dy, dz in waypoints_delta:
            wp = copy.deepcopy(current)
            wp.position.x += dx
            wp.position.y += dy
            wp.position.z += dz
            waypoints.append(wp)
            current = wp  # chain subsequent moves
        
        plan, fraction = self.arm_group.compute_cartesian_path(
            waypoints, eef_step=0.01, jump_threshold=0.0
        )
        if fraction < 0.9:
            rospy.logwarn(f"Cartesian path only {fraction*100:.0f}% achievable")
        if plan.joint_trajectory.points:
            self.arm_group.execute(plan, wait=True)
            return True
        return False
    
    def open_gripper(self):
        """Open gripper (50mm)."""
        self.gripper_pub.publish(Float64MultiArray(data=[0.05]))
        self.gripper_pub2.publish(Float64MultiArray(data=[0.0]))
        rospy.sleep(0.5)
    
    def close_gripper(self):
        """Close gripper (0mm)."""
        self.gripper_pub.publish(Float64MultiArray(data=[0.0]))
        self.gripper_pub2.publish(Float64MultiArray(data=[-0.05]))
        rospy.sleep(0.5)
    
    def grasp_sequence(self, x, y, z):
        """Complete grasp sequence: approach → grasp → lift."""
        rospy.loginfo("Phase 1: Pre-grasp approach...")
        self.go_to_pose(x, y, z + 0.1)  # 10cm above target
        rospy.sleep(1.0)
        
        rospy.loginfo("Phase 2: Descent to grasp...")
        self.open_gripper()
        self.cartesian_path([[0, 0, -0.1]])  # move down
        rospy.sleep(0.5)
        
        rospy.loginfo("Phase 3: Grasp!")
        self.close_gripper()
        rospy.sleep(0.5)
        
        rospy.loginfo("Phase 4: Lift...")
        self.cartesian_path([[0, 0, 0.15]])  # lift up
        rospy.sleep(1.0)
        
        rospy.loginfo("Grasp complete!")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="home", choices=["home", "grasp", "vertical"])
    ap.add_argument("--x", type=float, default=0.5)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.2)
    ap.add_argument("--cartesian", action="store_true")
    ap.add_argument("--dx", type=float, default=0.2)
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dz", type=float, default=-0.1)
    ap.add_argument("--grasp", action="store_true", help="Execute full grasp sequence")
    args = ap.parse_args()
    
    pipeline = Go2PiperGraspPipeline()
    
    if args.grasp:
        pipeline.grasp_sequence(args.x, args.y, args.z)
    elif args.cartesian:
        pipeline.open_gripper()
        pipeline.cartesian_path([[args.dx, args.dy, args.dz]])
    elif args.target == "grasp":
        pipeline.go_to_pose(args.x, args.y, args.z)
    else:
        pipeline.go_home()
