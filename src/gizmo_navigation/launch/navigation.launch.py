#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params_file = LaunchConfiguration('params_file').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'

    nav2_bt_dir = get_package_share_directory('nav2_bt_navigator')
    bt_search_dirs = [os.path.join(nav2_bt_dir, 'behavior_trees')]

    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    remappings = [
        ('cmd_vel', '/cmd_vel'),
        ('odom', '/odometry/filtered')
    ]

    # Standard standalone Nav2 nodes (matching multiRobot architecture)
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    bt_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time, 'bt_search_directories': bt_search_dirs}
        ]
    )

    waypoint_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': navigation_nodes,
            'bond_timeout': 0.0,
            'service_timeout': 30.0,
            'attempt_respawn_reconnection': True
        }]
    )

    return [
        controller_node,
        planner_node,
        behavior_node,
        bt_node,
        waypoint_node,
        lifecycle_manager_node
    ]


def generate_launch_description():
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    default_nav2_params = os.path.join(
        gizmo_navigation_dir,
        'config',
        'nav2.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to the Nav2 parameters YAML file'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        OpaqueFunction(function=launch_setup)
    ])
