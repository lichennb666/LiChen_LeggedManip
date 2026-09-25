from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("go2_piper_bringup")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "simulation.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"), "mode": "physics",
            "enable_policy": LaunchConfiguration("enable_policy"),
            "trace_path": LaunchConfiguration("trace_path"),
            "trace_max_samples": LaunchConfiguration("trace_max_samples"),
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("duration", default_value="60.0"),
        DeclareLaunchArgument("enable_policy", default_value="true"),
        DeclareLaunchArgument("trace_path", default_value=""),
        DeclareLaunchArgument("trace_max_samples", default_value="1000"),
        DeclareLaunchArgument("exercise_commands", default_value="false"),
        DeclareLaunchArgument("response_command_x", default_value="0.0"),
        DeclareLaunchArgument("response_start", default_value="2.0"),
        DeclareLaunchArgument("response_duration", default_value="5.0"),
        simulation,
        TimerAction(period=5.0, actions=[Node(
            package="go2_piper_mission", executable="wbc_gate",
            parameters=[{
                "duration": LaunchConfiguration("duration"),
                "exercise_commands": LaunchConfiguration("exercise_commands"),
                "response_command_x": LaunchConfiguration("response_command_x"),
                "response_start": LaunchConfiguration("response_start"),
                "response_duration": LaunchConfiguration("response_duration"),
                "use_sim_time": True,
            }], output="screen",
            on_exit=Shutdown(reason="WBC stability gate finished"))]),
    ])
