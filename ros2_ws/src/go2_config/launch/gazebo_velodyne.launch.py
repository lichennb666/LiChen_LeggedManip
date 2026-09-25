import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    rviz_path = LaunchConfiguration("rviz_path")
    base_frame = "base_link"

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_config"
    ).find("go2_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        config_pkg_share, "/config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot_VLP.xacro")
    default_world_path = os.path.join(config_pkg_share, "worlds/indoor.world")
    default_rviz_path = os.path.join(config_pkg_share, "rviz/vlp16.rviz")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_description_path = DeclareLaunchArgument(
        "description_path",
        default_value=default_model_path,
        description="Absolute path to the robot xacro file",
    )
    declare_rviz_path = DeclareLaunchArgument(
        "rviz_path",
        default_value=default_rviz_path,
        description="Absolute path to the RViz configuration",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.275")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    declare_use_slam = DeclareLaunchArgument(
        "use_slam", default_value="false",
        description="When true, disable EKF odom and publish body->base_link static TF for SLAM bridge",
    )
    declare_slam_tf_x = DeclareLaunchArgument("slam_tf_x", default_value="0")
    declare_slam_tf_y = DeclareLaunchArgument("slam_tf_y", default_value="0")
    declare_slam_tf_z = DeclareLaunchArgument("slam_tf_z", default_value="0")
    declare_slam_tf_yaw = DeclareLaunchArgument("slam_tf_yaw", default_value="0")
    declare_slam_tf_pitch = DeclareLaunchArgument("slam_tf_pitch", default_value="0")
    declare_slam_tf_roll = DeclareLaunchArgument("slam_tf_roll", default_value="0")

    # Static TF: body (FAST-LIO output frame) -> base_link (robot URDF root)
    # Only active when use_slam:=true
    slam_tf_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="slam_tf_bridge",
        arguments=[
            "--x",
            LaunchConfiguration("slam_tf_x"),
            "--y",
            LaunchConfiguration("slam_tf_y"),
            "--z",
            LaunchConfiguration("slam_tf_z"),
            "--yaw",
            LaunchConfiguration("slam_tf_yaw"),
            "--pitch",
            LaunchConfiguration("slam_tf_pitch"),
            "--roll",
            LaunchConfiguration("slam_tf_roll"),
            "--frame-id",
            "body",
            "--child-frame-id",
            "base_link",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("use_slam"), "' == 'true'"])
        ),
    )

    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": description_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_path": rviz_path,
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "true",
            "close_loop_odom": "true",
            "use_slam": LaunchConfiguration("use_slam"),
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "lite": LaunchConfiguration("lite"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
            "close_loop_odom": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_description_path,
            declare_rviz_path,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_use_slam,
            declare_slam_tf_x,
            declare_slam_tf_y,
            declare_slam_tf_z,
            declare_slam_tf_yaw,
            declare_slam_tf_pitch,
            declare_slam_tf_roll,
            slam_tf_bridge,
            bringup_ld,
            gazebo_ld

        ]
    )
