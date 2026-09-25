import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_share = get_package_share_directory("go2_config")
    description_share = get_package_share_directory("go2_description")

    common_launch = os.path.join(
        config_share, "launch", "gazebo_velodyne.launch.py"
    )
    mid360_model = os.path.join(
        description_share, "xacro", "robot_mid360.xacro"
    )
    mid360_rviz = os.path.join(config_share, "rviz", "mid360.rviz")

    # Common launch arguments such as use_slam, gui, rviz and world remain
    # available to the included launch. Only the sensor-specific files change.
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(common_launch),
                launch_arguments={
                    "description_path": mid360_model,
                    "rviz_path": mid360_rviz,
                    # FAST-LIO's body frame is the MID-360 IMU frame. These
                    # values invert base_link -> livox_imu_frame from the URDF.
                    "slam_tf_x": "0.01502",
                    "slam_tf_y": "-0.02329",
                    "slam_tf_z": "-0.29987",
                    "slam_tf_pitch": "-0.785",
                }.items(),
            )
        ]
    )
