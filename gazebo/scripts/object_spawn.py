#!/usr/bin/env python3
"""
在 Gazebo 世界生成目标物体 (cube/cylinder).
"""

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
import sys


def spawn_cube(name, x, y, z=0.5, size=0.05):
    """Spawn a colored cube in Gazebo."""
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia><ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""
    
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
    spawn(name, sdf, "", pose, "world")
    rospy.loginfo(f"Spawned cube '{name}' at ({x:.2f}, {y:.2f}, {z:.2f})")


def spawn_cylinder(name, x, y, z=0.5, radius=0.02, length=0.08):
    """Spawn a colored cylinder in Gazebo."""
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia><ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""
    
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
    spawn(name, sdf, "", pose, "world")
    rospy.loginfo(f"Spawned cylinder '{name}' at ({x:.2f}, {y:.2f}, {z:.2f})")


if __name__ == "__main__":
    rospy.init_node("object_spawner")
    
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="target_cube")
    ap.add_argument("--x", type=float, default=1.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.4)
    ap.add_argument("--size", type=float, default=0.05)
    ap.add_argument("--shape", default="cube", choices=["cube", "cylinder"])
    args = ap.parse_args()
    
    if args.shape == "cube":
        spawn_cube(args.name, args.x, args.y, args.z, args.size)
    else:
        spawn_cylinder(args.name, args.x, args.y, args.z, args.size)
