from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import json
import os


# The policy contract order for the 18 WBC joints (wbc startup_target).
WBC_JOINT_ORDER = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
]


def generate_launch_description():
    gazebo_share = get_package_share_directory("gazebo_ros")
    bringup_share = get_package_share_directory("go2_piper_bringup")
    description_share = get_package_share_directory("go2_piper_description")
    default_world = os.path.join(bringup_share, "worlds", "manual_grasp.world")
    world = LaunchConfiguration("world")
    physics_urdf = os.path.join(description_share, "urdf", "go2_piper_ros2.urdf")
    demo_urdf = os.path.join(description_share, "urdf", "go2_piper_demo.urdf")
    # The SDF is spawned so the robot starts in its policy natural pose
    # (<initial_position> baked in at build time); the URDF is only used for
    # robot_state_publisher.  The same pose is held by the WBC node during its
    # startup window, so no zero-joint sweep ever reaches the handled box.
    physics_sdf = os.path.join(description_share, "urdf", "go2_piper_ros2.sdf")
    demo_sdf = os.path.join(description_share, "urdf", "go2_piper_demo.sdf")
    startup_pose = json.load(open(
        os.path.join(description_share, "config", "startup_pose.json"),
        encoding="utf-8"))
    startup_target = [startup_pose[name] for name in WBC_JOINT_ORDER]
    physics_description = open(physics_urdf, encoding="utf-8").read()
    demo_description = open(demo_urdf, encoding="utf-8").read()
    wbc_cfg = "/workspace/LeggedManip_Lab/ros2_ws/src/go2_piper_wbc/config/wbc.yaml"
    gui = LaunchConfiguration("gui")
    mode = LaunchConfiguration("mode")
    enable_policy = LaunchConfiguration("enable_policy")
    trace_path = LaunchConfiguration("trace_path")
    trace_max_samples = LaunchConfiguration("trace_max_samples")
    freeze_arm = LaunchConfiguration("freeze_arm")
    freeze_arm_mode = LaunchConfiguration("freeze_arm_mode")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    physics_mode = IfCondition(PythonExpression(["'", mode, "' == 'physics'"]))
    demo_mode = IfCondition(PythonExpression(["'", mode, "' == 'demo'"]))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"world": world, "gui": gui, "pause": "true"}.items())
    physics_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": physics_description, "use_sim_time": True}],
        condition=physics_mode, output="screen")
    demo_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": demo_description, "use_sim_time": True}],
        condition=demo_mode, output="screen")
    # Passing the generated file avoids lxml rejecting the XML encoding declaration
    # when spawn_entity receives robot_description as a Python unicode string.
    physics_spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-entity", "go2_piper", "-file", physics_sdf,
                   "-x", spawn_x, "-y", spawn_y,
                   "-z", "0.34", "-Y", spawn_yaw],
        condition=physics_mode)
    demo_spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-entity", "go2_piper", "-file", demo_sdf,
                   "-x", spawn_x, "-y", spawn_y,
                   "-z", "0.34", "-Y", spawn_yaw],
        condition=demo_mode)
    controllers = [
        Node(package="controller_manager", executable="spawner", arguments=[name, "--controller-manager", "/controller_manager"], output="screen")
        for name in ("joint_state_broadcaster", "go2_piper_pd_controller")
    ]
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "world", default_value=default_world,
            description="Absolute path to the Gazebo world file"),
        DeclareLaunchArgument(
            "mode", default_value="physics", choices=["physics", "demo"],
            description="physics disables all state-overwriting demo plugins"),
        DeclareLaunchArgument("enable_policy", default_value="true"),
        DeclareLaunchArgument("trace_path", default_value=""),
        DeclareLaunchArgument("trace_max_samples", default_value="1000"),
        DeclareLaunchArgument("freeze_arm", default_value="false"),
        DeclareLaunchArgument("freeze_arm_mode", default_value="current"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.20"),
        gazebo, physics_state_publisher, demo_state_publisher, physics_spawn, demo_spawn,
        TimerAction(period=1.5, actions=controllers),
        TimerAction(period=2.0, actions=[Node(package="go2_piper_bringup", executable="odom_tf", parameters=[{"use_sim_time": True}])]),
        TimerAction(period=2.0, actions=[Node(
            package="go2_piper_wbc", executable="wbc_node",
            parameters=[wbc_cfg, {
                "use_sim_time": True,
                "enable_policy": enable_policy,
                "startup_target": startup_target,
                "trace_path": trace_path,
                "trace_max_samples": trace_max_samples,
                "freeze_arm": freeze_arm,
                "freeze_arm_mode": freeze_arm_mode,
            }],
            output="screen")]),
        TimerAction(period=4.0, actions=[ExecuteProcess(
            cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
            output="screen")]),
    ])
