import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_bringup_dir = get_package_share_directory('gizmo_bringup')
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    # 1. Declare Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    default_map_file = os.path.join(
        gizmo_navigation_dir,
        'maps',
        'simple_bigger_world_map.yaml'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to the map YAML file to load for AMCL localization'
    )

    # 2. Gazebo Simulation with simpleBiggerWorld (boots immediately)
    gazebo_bigger_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_gazebo_dir, 'launch', 'gazebo_simpleBiggerWorld.launch.py')
        )
    )

    # 3. Master Bringup in Localization Mode (AMCL + Map Server + Nav2 + RViz2)
    amcl_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_bringup_dir, 'launch', 'slamNavigation_bringup.launch.py')
        ),
        launch_arguments={
            'slam': 'false',
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Delay AMCL Localization & Nav2 by 5.0 seconds so Gazebo physics, clock, and TF can stabilize first
    delayed_amcl_navigation = TimerAction(
        period=5.0,
        actions=[amcl_navigation_launch]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        gazebo_bigger_world_launch,
        delayed_amcl_navigation
    ])
