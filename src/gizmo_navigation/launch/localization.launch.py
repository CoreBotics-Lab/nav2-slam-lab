#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'

    lifecycle_nodes = ['map_server', 'amcl']

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_file, 'use_sim_time': use_sim_time}
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
            'bond_timeout': 0.0,
            'service_timeout': 30.0,
            'attempt_respawn_reconnection': True
        }]
    )

    return [map_server_node, amcl_node, lifecycle_manager_node]


def generate_launch_description():
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    default_map_file = os.path.join(
        gizmo_navigation_dir,
        'maps',
        'simple_bigger_world_map.yaml'
    )
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
            'map',
            default_value=default_map_file,
            description='Full path to the map YAML file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to the Nav2 parameters YAML file'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the localization lifecycle stack'
        ),
        OpaqueFunction(function=launch_setup)
    ])
