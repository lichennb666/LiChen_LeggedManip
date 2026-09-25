from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("go2_piper_bringup")
    world = os.path.join(share, "worlds", "manual_grasp.world")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "simulation.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "mode": "physics",
            "enable_policy": "true",
            "world": world,
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        simulation,
    ])
