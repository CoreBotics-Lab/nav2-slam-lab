#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    mask_yaml_file = LaunchConfiguration('mask').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'

    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # 1. Filter Mask Server (Publishes /keepout_filter_mask OccupancyGrid)
    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'yaml_filename': mask_yaml_file,
                'topic_name': '/keepout_filter_mask',
                'frame_id': 'map',
                'use_sim_time': use_sim_time
            }
        ]
    )

    # 2. Costmap Filter Info Server (Publishes /costmap_filter_info metadata)
    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time
            }
        ]
    )

    # 3. Lifecycle Manager (Transitions both servers to ACTIVE)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filter',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
            'bond_timeout': 0.0,
            'service_timeout': 30.0,
            'attempt_respawn_reconnection': True
        }]
    )

    return [filter_mask_server, costmap_filter_info_server, lifecycle_manager]


def generate_launch_description():
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    default_mask = os.path.join(
        gizmo_mapping_dir,
        'maps',
        'simple_bigger_world_map_keepout.yaml'
    )
    default_params = os.path.join(
        gizmo_navigation_dir,
        'config',
        'nav2.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mask',
            default_value=default_mask,
            description='Full path to filter mask YAML file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to Nav2 parameters YAML file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup lifecycle servers'
        ),
        OpaqueFunction(function=launch_setup)
    ])