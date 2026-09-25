from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("go2_piper_bringup")
    legacy_world = os.path.join(share, "worlds", "pick_transport.world")
    physical_grasp = LaunchConfiguration("physical_grasp")
    assisted_grasp = LaunchConfiguration("assisted_grasp")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "simulation.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "mode": PythonExpression([
                "'physics' if '", physical_grasp, "' == 'true' else 'demo'"
            ]),
            "enable_policy": "true",
            "world": legacy_world,
        }.items(),
    )
    workspace = "/workspace/LeggedManip_Lab/ros2_ws"
    perception_cfg = f"{workspace}/src/go2_piper_perception/config/perception.yaml"
    mission_cfg = f"{workspace}/src/go2_piper_mission/config/mission.yaml"
    detector = Node(
        package="go2_piper_perception",
        executable="color_detector",
        parameters=[perception_cfg, {
            "use_sim_time": True,
            "rgb_topic": "/d435/camera/image_raw",
            "depth_topic": "/d435/camera/depth/image_raw",
            "info_topic": "/d435/camera/camera_info",
            "min_area": 50,
        }],
        output="screen",
    )
    mission = Node(
        package="go2_piper_mission",
        executable="mission_server",
        parameters=[mission_cfg, {
            "use_sim_time": True,
            "physical_grasp": ParameterValue(
                physical_grasp, value_type=bool),
            "assisted_grasp": ParameterValue(
                assisted_grasp, value_type=bool),
            "detection_timeout": 20.0,
        }],
        output="screen",
    )
    starter = Node(
        package="go2_piper_mission",
        executable="start_mission",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("physical_grasp", default_value="true"),
        DeclareLaunchArgument("assisted_grasp", default_value="true"),
        simulation,
        TimerAction(period=5.0, actions=[detector]),
        # The handled box is part of the world from startup, and the robot
        # spawns with its arm in the policy natural pose, so there is no
        # startup sweep.  Pure physics grasp should start sooner: otherwise
        # the target can drift laterally before the mission even begins.
        TimerAction(
            period=10.0, actions=[mission, starter],
            condition=IfCondition(physical_grasp)),
        TimerAction(
            period=17.0, actions=[mission, starter],
            condition=UnlessCondition(physical_grasp)),
        RegisterEventHandler(OnProcessExit(target_action=starter, on_exit=[Shutdown()])),
    ])
