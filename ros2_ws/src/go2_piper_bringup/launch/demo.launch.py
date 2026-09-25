from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("go2_piper_bringup")
    legacy_world = os.path.join(share, "worlds", "pick_transport.world")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "simulation.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"), "mode": "demo",
            "world": legacy_world,
        }.items())
    workspace = "/workspace/LeggedManip_Lab/ros2_ws"
    perception_cfg = f"{workspace}/src/go2_piper_perception/config/perception.yaml"
    mission_cfg = f"{workspace}/src/go2_piper_mission/config/mission.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"), simulation,
        TimerAction(period=5.0, actions=[Node(package="go2_piper_perception", executable="synthetic_camera", parameters=[{"use_sim_time": True}], output="screen")]),
        TimerAction(period=5.0, actions=[Node(package="go2_piper_perception", executable="color_detector", parameters=[perception_cfg, {"use_sim_time": True}], output="screen")]),
        TimerAction(period=5.0, actions=[Node(package="go2_piper_mission", executable="mission_server", parameters=[mission_cfg, {"use_sim_time": True}], output="screen")]),
        TimerAction(period=5.0, actions=[Node(package="go2_piper_mission", executable="start_mission", parameters=[{"use_sim_time": True}], output="screen")]),
    ])
