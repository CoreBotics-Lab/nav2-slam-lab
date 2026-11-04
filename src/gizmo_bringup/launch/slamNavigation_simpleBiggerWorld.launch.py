#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gizmo_bringup_dir = get_package_share_directory('gizmo_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without GUI if true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gizmo_bringup_dir, 'launch', 'navigation_simulation.launch.py')
            ),
            launch_arguments={
                'world': 'simpleBiggerWorld.sdf',
                'slam': 'true',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'headless': LaunchConfiguration('headless'),
            }.items()
        )
    ])
