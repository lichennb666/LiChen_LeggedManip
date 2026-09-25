from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory("fast_lio")
    open3d_share = get_package_share_directory("open3d_loc")

    map_path = LaunchConfiguration("map_path")
    fast_lio_config = LaunchConfiguration("fast_lio_config")
    localization_config = LaunchConfiguration("localization_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_share + "/launch/mapping.launch.py"),
        launch_arguments={
            "config_file": fast_lio_config,
            "use_sim_time": use_sim_time,
            "rviz": "false",
        }.items(),
    )

    localization = Node(
        package="open3d_loc",
        executable="global_localization_node",
        name="open3d_global_localization",
        output="screen",
        parameters=[
            localization_config,
            {"map_path": map_path, "use_sim_time": use_sim_time},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="open3d_localization_rviz",
        arguments=["-d", open3d_share + "/rviz/localization.rviz"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                description="Absolute path to the PCD or PLY global map",
            ),
            DeclareLaunchArgument(
                "fast_lio_config",
                default_value="mid360_sim.yaml",
                description="FAST-LIO config file name",
            ),
            DeclareLaunchArgument(
                "localization_config",
                default_value=open3d_share + "/config/open3d_loc.yaml",
                description="Open3D localization parameter file",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use Gazebo clock"
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start one RViz instance for localization",
            ),
            fast_lio,
            localization,
            rviz,
        ]
    )
