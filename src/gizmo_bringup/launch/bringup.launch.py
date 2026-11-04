#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    mode = LaunchConfiguration('mode').perform(context).lower()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    slam_launch_path = os.path.join(gizmo_mapping_dir, 'launch', 'slam.launch.py')
    localization_launch_path = os.path.join(gizmo_navigation_dir, 'launch', 'localization.launch.py')
    navigation_launch_path = os.path.join(gizmo_navigation_dir, 'launch', 'navigation.launch.py')

    # Unified RViz configuration for both localization and mapping
    if not rviz_config:
        rviz_config = os.path.join(gizmo_navigation_dir, 'rviz', 'nav2.rviz')

    actions: list[Action] = []

    # 1. SLAM (Mapping) or AMCL (Localization)
    if mode == 'mapping':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_path),
                launch_arguments={
                    'use_sim_time': str(use_sim_time).lower(),
                    'slam_params_file': slam_params_file,
                    'autostart': str(autostart).lower()
                }.items()
            )
        )
    elif mode == 'localization':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch_path),
                launch_arguments={
                    'use_sim_time': str(use_sim_time).lower(),
                    'map': map_file,
                    'params_file': params_file,
                    'autostart': str(autostart).lower()
                }.items()
            )
        )
    else:
        raise RuntimeError(f"Invalid mode '{mode}'. Expected 'localization' or 'mapping'.")

    # 2. Nav2 Navigation Stack (Planner, Controller, BT Navigator, Smoother, Recoveries)
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time).lower(),
                'params_file': params_file,
                'autostart': str(autostart).lower()
            }.items()
        )
    )

    # 3. RViz2 Dashboard
    if use_rviz:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        )

    return actions


def generate_launch_description():
    gizmo_mapping_dir = get_package_share_directory('gizmo_mapping')
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    default_map_file = os.path.join(
        gizmo_mapping_dir,
        'maps',
        'simple_bigger_world_map.yaml'
    )
    default_nav2_params = os.path.join(
        gizmo_navigation_dir,
        'config',
        'nav2.yaml'
    )
    default_slam_params = os.path.join(
        gizmo_mapping_dir,
        'config',
        'slam_toolbox.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='localization',
            choices=['localization', 'mapping'],
            description='Operating mode: "localization" (pre-loaded map + AMCL) or "mapping" (online SLAM Toolbox)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map YAML file (used when mode:=localization)'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to Nav2 parameters file'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Full path to SLAM Toolbox parameters file (used when mode:=mapping)'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the Nav2 lifecycle stack'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz2'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional custom RViz config file path. Defaults to gizmo_navigation/rviz/nav2.rviz.'
        ),
        OpaqueFunction(function=launch_setup)
    ])

