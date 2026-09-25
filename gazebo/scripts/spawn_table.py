#!/usr/bin/env python3
"""
Gazebo 场景生成: 桌子 + 目标方块.
用法:
  python spawn_table.py                        # 默认位置
  python spawn_table.py --x 2.0 --y -1.0      # 指定位置
"""

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import sys


def spawn_sdf(name, sdf_text, x, y, z=0):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
    spawn(name, sdf_text, "", pose, "world")
    rospy.loginfo(f"Spawned '{name}' at ({x:.1f}, {y:.1f}, {z:.2f})")


def make_table(x, y):
    """桌子: 0.8m宽 x 0.6m深 x 0.6m高"""
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="table">
    <static>true</static>
    <link name="table_top">
      <collision name="collision">
        <pose>0 0 0.6 0 0 0</pose>
        <geometry><box><size>0.8 0.6 0.03</size></box></geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 0.6 0 0 0</pose>
        <geometry><box><size>0.8 0.6 0.03</size></box></geometry>
        <material><ambient>0.5 0.25 0.1 1</ambient><diffuse>0.5 0.25 0.1 1</diffuse></material>
      </visual>
    </link>
    <!-- Legs -->
    <link name="leg1"><pose>{x-0.35} {y-0.25} 0.3 0 0 0</pose>
      <collision><geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry></collision>
      <visual><geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry></visual>
    </link>
    <link name="leg2"><pose>{x+0.35} {y-0.25} 0.3 0 0 0</pose>
      <collision><geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry></collision>
      <visual><geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry></visual>
    </link>
    <link name="leg3"><pose>{x-0.35} {y+0.25} 0.3 0 0 0</pose>
      <collision><geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry></collision>
      <visual><geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry></visual>
    </link>
    <link name="leg4"><pose>{x+0.35} {y+0.25} 0.3 0 0 0</pose>
      <collision><geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry></collision>
      <visual><geometry><cylinder><radius>0.02</radius><length>0.6</length></cylinder></geometry></visual>
    </link>
  </model>
</sdf>"""


def make_cube(name, x, y, z=0.63, size=0.04):
    """绿色方块"""
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial><mass>0.05</mass>
        <inertia><ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz></inertia>
      </inertial>
      <collision><geometry><box><size>{size} {size} {size}</size></box></geometry></collision>
      <visual>
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=2.0)
    ap.add_argument("--y", type=float, default=-1.0)
    args = ap.parse_args()

    rospy.init_node("spawn_table")

    # Table
    spawn_sdf("table", make_table(args.x, args.y), args.x, args.y)

    # Green cube on table
    spawn_sdf("target_cube", make_cube("target_cube", args.x, args.y, 0.63), args.x, args.y, 0.63)

    rospy.loginfo(f"Table + cube at ({args.x:.1f}, {args.y:.1f})")
    rospy.loginfo("Start: roslaunch go2_piper_description full_stack.launch")
    rospy.loginfo("Camera: python gazebo/scripts/camera_viewer.py")
