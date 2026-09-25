from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("open3d_loc")

    map_path = LaunchConfiguration("map_path")
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                description="Absolute path to the PCD or PLY global map",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=package_share + "/config/open3d_loc.yaml",
                description="Open3D localization parameter file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo clock",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start the localization RViz configuration",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=package_share + "/rviz/localization.rviz",
                description="RViz configuration file",
            ),
            Node(
                package="open3d_loc",
                executable="global_localization_node",
                name="open3d_global_localization",
                output="screen",
                parameters=[
                    config_file,
                    {"map_path": map_path, "use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="open3d_localization_rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
