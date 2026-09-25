from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share = get_package_share_directory("go2_piper_bringup")
    go2_config_share = get_package_share_directory("go2_config")
    default_world = os.path.join(share, "worlds", "floor1_navigation_course_obstacles.world")
    default_fast_lio_config = os.path.join(share, "config", "fastlio_localization.yaml")
    default_open3d_config = os.path.join(share, "config", "open3d_loc_sim.yaml")
    default_map_yaml = "/workspace/LeggedManip_Lab/maps/go2_course1_map.yaml"
    mission_cfg = os.path.join(get_package_share_directory("go2_piper_mission"), "config", "mission.yaml")
    perception_cfg = os.path.join(get_package_share_directory("go2_piper_perception"), "config", "perception.yaml")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "simulation.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"), "mode": LaunchConfiguration("mode"),
            "world": LaunchConfiguration("world"), "enable_policy": LaunchConfiguration("enable_policy"),
            "trace_path": LaunchConfiguration("trace_path"),
            "trace_max_samples": LaunchConfiguration("trace_max_samples"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "freeze_arm": LaunchConfiguration("freeze_arm"),
            "freeze_arm_mode": LaunchConfiguration("freeze_arm_mode"),
        }.items(),
    )
    fast_lio = Node(package="fast_lio", executable="fastlio_mapping", name="laserMapping", output="screen",
                    parameters=[LaunchConfiguration("fast_lio_config"), {"use_sim_time": LaunchConfiguration("use_sim_time")}])
    open3d = Node(package="open3d_loc", executable="global_localization_node", name="open3d_global_localization",
                  output="screen", parameters=[LaunchConfiguration("open3d_config"), {
                      "map_path": LaunchConfiguration("map_path"), "use_sim_time": LaunchConfiguration("use_sim_time")}])
    camera_init2odom = Node(package="tf2_ros", executable="static_transform_publisher", name="camera_init2odom",
                            output="screen", arguments=["0", "0", "0", "0", "0", "0", "1", "camera_init", "odom"])
    map_to_base = Node(package="tf2_ros", executable="static_transform_publisher", name="body2base", output="screen",
                       arguments=["0", "0", "0", "0", "0", "0", "1", "base", "body"])
    odom_relay = Node(package="go2_piper_bringup", executable="odom_relay", name="open3d_odom_relay", output="screen",
                      parameters=[{"input_topic": LaunchConfiguration("odom_topic"), "output_topic": "/odom",
                                   "use_sim_time": LaunchConfiguration("use_sim_time"), "enabled": LaunchConfiguration("relay_odom")}])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(go2_config_share, "launch", "nav2_fastlio.launch.py")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time"), "map": LaunchConfiguration("map_yaml"),
                          "params_file": LaunchConfiguration("nav2_params"), "cloud_topic": LaunchConfiguration("scan_input_topic"),
                          "rviz": LaunchConfiguration("rviz")}.items(),
    )
    goal_gateway = Node(package="go2_piper_bringup", executable="course1_goal_gateway", name="course1_goal_gateway",
                        output="screen", parameters=[{"input_goal_topic": "/move_base_simple/goal", "navigation_goal_topic": "/goal_pose",
                            "capture_min_x": 0.05, "capture_max_x": 1.10, "capture_min_y": -0.55, "capture_max_y": 0.45,
                            "mission_color": "green", "mission_rearm_delay": 2.5,
                            "use_mission": LaunchConfiguration("enable_table_capture")}])
    perception_tf = Node(package="go2_piper_perception", executable="synthetic_camera", name="synthetic_camera", output="screen",
                         remappings=[("/demo_camera/image_raw", "/d435/camera/image_raw"),
                                     ("/demo_camera/depth/image_raw", "/d435/camera/depth/image_raw"),
                                     ("/demo_camera/camera_info", "/d435/camera/camera_info")],
                         condition=IfCondition(LaunchConfiguration("enable_mission_perception")))
    detector = Node(package="go2_piper_perception", executable="color_detector", name="color_detector", output="screen",
                    parameters=[perception_cfg, {"use_sim_time": LaunchConfiguration("use_sim_time"),
                        "rgb_topic": "/d435/camera/image_raw", "depth_topic": "/d435/camera/depth/image_raw",
                        "info_topic": "/d435/camera/camera_info", "min_area": 50}],
                    condition=IfCondition(LaunchConfiguration("enable_mission_perception")))
    mission_server = Node(package="go2_piper_mission", executable="mission_server", name="mission_server", output="screen",
                          parameters=[mission_cfg, {"use_sim_time": LaunchConfiguration("use_sim_time"), "skip_pick_navigation": True}],
                          condition=IfCondition(LaunchConfiguration("enable_mission_server")))

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"), DeclareLaunchArgument("mode", default_value="physics"),
        DeclareLaunchArgument("world", default_value=default_world), DeclareLaunchArgument("enable_policy", default_value="true"),
        DeclareLaunchArgument("trace_path", default_value=""), DeclareLaunchArgument("trace_max_samples", default_value="1000"),
        DeclareLaunchArgument("spawn_x", default_value="-4.0"), DeclareLaunchArgument("spawn_y", default_value="-2.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"), DeclareLaunchArgument("freeze_arm", default_value="true"),
        DeclareLaunchArgument("freeze_arm_mode", default_value="startup"),
        DeclareLaunchArgument("use_sim_time", default_value="true"), DeclareLaunchArgument("map_path", default_value="/tmp/go2_course1_map.pcd"),
        DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),
        DeclareLaunchArgument("fast_lio_config", default_value=default_fast_lio_config), DeclareLaunchArgument("open3d_config", default_value=default_open3d_config),
        DeclareLaunchArgument("nav2_params", default_value=os.path.join(go2_config_share, "config/autonomy/navigation.yaml")),
        DeclareLaunchArgument("relay_odom", default_value="true"), DeclareLaunchArgument("scan_input_topic", default_value="/cloud_registered"),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry"), DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("enable_table_capture", default_value="false"), DeclareLaunchArgument("enable_mission_server", default_value="false"),
        DeclareLaunchArgument("enable_mission_perception", default_value="false"),
        mission_server, simulation, fast_lio, open3d, camera_init2odom, map_to_base, odom_relay, nav2,
        goal_gateway, detector, perception_tf,
    ])
