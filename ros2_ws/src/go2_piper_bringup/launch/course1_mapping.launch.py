from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share = get_package_share_directory("go2_piper_bringup")
    default_world = os.path.join(share, "worlds", "floor1_navigation_course_obstacles.world")
    default_fast_lio_config = os.path.join(share, "config", "fastlio_mapping.yaml")
    default_rviz_config = os.path.join(share, "rviz", "course1_mapping.rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "simulation.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "mode": LaunchConfiguration("mode"),
            "world": LaunchConfiguration("world"),
            "enable_policy": LaunchConfiguration("enable_policy"),
            "trace_path": LaunchConfiguration("trace_path"),
            "trace_max_samples": LaunchConfiguration("trace_max_samples"),
            "freeze_arm": LaunchConfiguration("freeze_arm"),
            "freeze_arm_mode": LaunchConfiguration("freeze_arm_mode"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
        }.items(),
    )

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            LaunchConfiguration("fast_lio_config"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_file_path": LaunchConfiguration("map_file_path"),
            },
        ],
    )

    teleop_gui = Node(
        package="go2_piper_bringup",
        executable="teleop_gui",
        name="teleop_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_teleop")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("mode", default_value="physics"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("enable_policy", default_value="true"),
        DeclareLaunchArgument("trace_path", default_value=""),
        DeclareLaunchArgument("trace_max_samples", default_value="1000"),
        DeclareLaunchArgument("freeze_arm", default_value="true"),
        DeclareLaunchArgument("freeze_arm_mode", default_value="startup"),
        DeclareLaunchArgument("spawn_x", default_value="-4.0"),
        DeclareLaunchArgument("spawn_y", default_value="-2.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "fast_lio_config",
            default_value=default_fast_lio_config,
            description="FAST-LIO ROS 2 YAML config file",
        ),
        DeclareLaunchArgument(
            "map_file_path",
            default_value="/tmp/go2_course1_map.pcd",
            description="Output PCD from mapping mode",
        ),
        DeclareLaunchArgument(
            "enable_teleop",
            default_value="true",
            description="Open a separate GUI window for manual keyboard teleop during mapping",
        ),
        simulation,
        teleop_gui,
        TimerAction(period=6.0, actions=[fast_lio]),
        TimerAction(period=8.0, actions=[rviz]),
    ])
