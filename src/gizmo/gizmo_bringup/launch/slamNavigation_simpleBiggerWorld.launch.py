import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gizmo_gazebo_dir = get_package_share_directory('gizmo_gazebo')
    gizmo_navigation_dir = get_package_share_directory('gizmo_navigation')

    # 1. Declare Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 2. Gazebo Simulation with simpleBiggerWorld (boots immediately)
    gazebo_bigger_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_gazebo_dir, 'launch', 'gazebo_simpleBiggerWorld.launch.py')
        )
    )

    # 3. Online SLAM Toolbox + Nav2 Navigation Stack (always runs RViz2)
    slam_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gizmo_navigation_dir, 'launch', 'slam_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Delay SLAM & Nav2 by 3.0 seconds so Gazebo physics, clock, and TF can stabilize first
    delayed_slam_navigation = TimerAction(
        period=3.0,
        actions=[slam_navigation_launch]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo_bigger_world_launch,
        delayed_slam_navigation
    ])

