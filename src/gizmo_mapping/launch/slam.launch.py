#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'

    lifecycle_nodes = ['slam_toolbox']

    # 1. SLAM Toolbox Node (Online Asynchronous)
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 2. Lifecycle Manager for SLAM
    start_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
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

    return [start_async_slam_toolbox_node, start_lifecycle_manager]


def generate_launch_description():
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')

    default_slam_params = os.path.join(
        gizmo_mapping_dir,
        'config',
        'slam_toolbox.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Full path to the parameters file for slam_toolbox'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the SLAM lifecycle stack'
        ),
        OpaqueFunction(function=launch_setup)
    ])
