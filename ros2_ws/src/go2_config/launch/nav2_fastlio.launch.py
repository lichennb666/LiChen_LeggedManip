# Nav2 launch for FAST-LIO + SC-PGO localization (no AMCL, no slam_toolbox)
# Phase 1: 2D Nav2 validation with VLP-16 → LaserScan conversion
#
# All Nav2 nodes launched inline (not via navigation_launch.py) so that
# each node gets precise topic remaps — especially behavior_server, whose
# cmd_vel output is routed through twist_mux.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    this_package = FindPackageShare('go2_config')
    twist_mux_params = PathJoinSubstitution([this_package, 'config/autonomy', 'twist_mux.yaml'])

    # Defaults
    default_map = PathJoinSubstitution([this_package, 'maps', 'playground.yaml'])
    default_params = PathJoinSubstitution([this_package, 'config/autonomy', 'navigation.yaml'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    cloud_topic = LaunchConfiguration('cloud_topic')

    # Rewrite params with sim time and map path
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            },
            convert_types=True),
        allow_substs=True)

    rviz_config = PathJoinSubstitution(
        [this_package, 'rviz', 'navigation.rviz']
    )

    # Shared tf remappings (same as nav2_bringup convention)
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Nav2 lifecycle-managed nodes
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/livox/lidar',
            description='PointCloud2 source used for Nav2 local obstacle sensing',
        ),

        # ── twist_mux ────────────────────────────────────────────────
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('cmd_vel_out', '/cmd_vel')],
            output='screen',
        ),

        # ── pointcloud_to_laserscan ──────────────────────────────────
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'use_sim_time': True,
                'min_height': 0.1,
                'max_height': 0.7,
                'range_min': 0.3,
                'range_max': 30.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00349,
                'scan_time': 0.1,
                'inf_is_valid': False,
                'target_frame': 'base',
            }],
            remappings=[
                ('cloud_in', cloud_topic),
                ('scan', '/scan'),
            ],
            output='screen',
        ),

        # ── map_server ───────────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[configured_params],
            output='screen',
        ),

        # ── lifecycle manager for map_server ─────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server'],
            }],
            output='screen',
        ),

        # ══════════════════════════════════════════════════════════════
        # Nav2 core nodes (inline, not via navigation_launch.py)
        # ══════════════════════════════════════════════════════════════

        # ── controller_server ────────────────────────────────────────
        # Publishes to cmd_vel_nav (internal); velocity_smoother reads it
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),

        # ── smoother_server ──────────────────────────────────────────
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings,
        ),

        # ── planner_server ───────────────────────────────────────────
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings,
        ),

        # ── behavior_server ──────────────────────────────────────────
        # Remap cmd_vel → cmd_vel_behavior so spin/backup/wait go through
        # twist_mux (priority 50) instead of writing /cmd_vel directly
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings + [('cmd_vel', 'cmd_vel_behavior')],
        ),

        # ── bt_navigator ─────────────────────────────────────────────
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings,
        ),

        # ── waypoint_follower ────────────────────────────────────────
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings,
        ),

        # ── velocity_smoother ────────────────────────────────────────
        # Reads cmd_vel_nav from controller, publishes to cmd_vel_nav_smooth
        # which feeds into twist_mux (priority 10)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel_nav_smooth'),
            ],
        ),

        # ── lifecycle manager for Nav2 nodes ─────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),

        # ── RViz ─────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
