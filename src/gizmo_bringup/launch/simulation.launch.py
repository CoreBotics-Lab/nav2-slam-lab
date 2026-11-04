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

    mode = LaunchConfiguration('mode').perform(context).lower()
    world = LaunchConfiguration('world').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
    keepout = LaunchConfiguration('keepout').perform(context).lower() == 'true'
    mask_file = LaunchConfiguration('mask').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gazebo_delay = float(LaunchConfiguration('gazebo_delay').perform(context))
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    # Resolve world path (either absolute or relative to gizmo_gazebo/worlds/)
    if os.path.isabs(world):
        world_path = world
    else:
        world_path = os.path.join(gizmo_gazebo_dir, 'worlds', world)

    gazebo_launch_path = os.path.join(gizmo_gazebo_dir, 'launch', 'gazebo.launch.py')
    bringup_launch_path = os.path.join(gizmo_bringup_dir, 'launch', 'bringup.launch.py')

    # 1. Gazebo Simulation (World + Robot Spawning + ros_gz_bridge)
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'world_file': world_path,
            'headless': str(headless).lower(),
            'use_sim_time': 'true',
            'use_camera': 'false',
        }.items()
    )

    # 2. Nav2 & SLAM / Localization Bringup
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'mode': mode,
            'map': map_file,
            'params_file': params_file,
            'slam_params_file': slam_params_file,
            'keepout': str(keepout).lower(),
            'mask': mask_file,
            'use_sim_time': 'true',
            'use_rviz': str(use_rviz).lower(),
            'rviz_config': rviz_config,
        }.items()
    )

    # 3. Delay Bringup until Gazebo physics, /clock, /scan, and TF tree stabilize
    delayed_bringup = TimerAction(
        period=gazebo_delay,
        actions=[bringup_launch]
    )

    return [gazebo_simulation, delayed_bringup]


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
    default_mask_file = os.path.join(
        gizmo_mapping_dir,
        'maps',
        'simple_bigger_world_map_keepout.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='localization',
            choices=['localization', 'mapping'],
            description='Operating mode: "localization" (pre-loaded map + AMCL) or "mapping" (online SLAM Toolbox)'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='simpleBiggerWorld.sdf',
            description='World file name inside gizmo_gazebo/worlds/ or full absolute path'
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
            'keepout',
            default_value='true',
            description='Whether to activate keepout zones costmap filter'
        ),
        DeclareLaunchArgument(
            'mask',
            default_value=default_mask_file,
            description='Full path to filter mask YAML file'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without GUI if true'
        ),
        DeclareLaunchArgument(
            'gazebo_delay',
            default_value='5.0',
            description='Seconds to delay Nav2 bringup until Gazebo physics, clock, and TF initialize'
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

