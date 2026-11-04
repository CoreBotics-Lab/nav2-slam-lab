#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_bringup_dir = get_package_share_directory('gizmo_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gazebo_delay = float(LaunchConfiguration('gazebo_delay').perform(context))
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    # Resolve world path
    if os.path.isabs(world):
        world_path = world
    else:
        world_path = os.path.join(gizmo_gazebo_dir, 'worlds', world)

    gazebo_launch_path = os.path.join(gizmo_gazebo_dir, 'launch', 'gazebo.launch.py')
    bringup_launch_path = os.path.join(gizmo_bringup_dir, 'launch', 'slamNavigation_bringup.launch.py')

    # 1. Gazebo Simulation
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'world_file': world_path,
            'headless': str(headless).lower(),
            'use_sim_time': str(use_sim_time).lower(),
            'use_camera': 'false',
        }.items()
    )

    # 2. Nav2 & SLAM / Localization Bringup
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'slam': str(slam).lower(),
            'map': map_file,
            'use_sim_time': str(use_sim_time).lower(),
            'use_rviz': str(use_rviz).lower(),
            'rviz_config': rviz_config,
        }.items()
    )

    # Delay Nav2 / SLAM until Gazebo physics, /clock, and TF stabilize
    delayed_navigation = TimerAction(
        period=gazebo_delay,
        actions=[bringup_launch]
    )

    return [gazebo_simulation, delayed_navigation]


def generate_launch_description():
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    default_map_file = os.path.join(
        gizmo_navigation_dir,
        'maps',
        'simple_bigger_world_map.yaml'
    )

    default_rviz_config = os.path.join(
        gizmo_navigation_dir,
        'rviz',
        'nav2.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='simpleBiggerWorld.sdf',
            description='World file name inside gizmo_gazebo/worlds/ or full absolute path'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Whether to run SLAM mapping (true) or AMCL localization (false)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map YAML file (used when slam:=false)'
        ),
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
        DeclareLaunchArgument(
            'gazebo_delay',
            default_value='5.0',
            description='Delay in seconds before launching Nav2 / SLAM stack'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Full path to RViz configuration file'
        ),
        OpaqueFunction(function=launch_setup)
    ])

